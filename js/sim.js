/* Braccio-Simulator – Simulationskern
 *  - Simulationszeit & Ereignis-Scheduler (für delay() im Arduino-Code)
 *  - Servo-Modell (PT1 + Geschwindigkeitsbegrenzung)
 *  - Fertigungsfehler (Kalibrierung) und Kompensation
 *  - Objekte, Greifen, Förderband mit Lichtschranken
 *  - Arbeitsraumüberwachung
 */
(function () {
  'use strict';
  const BS = window.BS;
  const C = BS.config;
  const kin = BS.kin;

  const sim = (BS.sim = {
    t: 0,
    speed: 1,
    paused: false,
    cmd: C.beginPose.slice(), // Soll vom Programm/Bediener
    act: C.beginPose.slice(), // Ist am Servo-Abtrieb
    vel: [0, 0, 0, 0, 0, 0],
    roundCmd: true,
    // Kalibrierung
    errOn: false,
    errOff: [0, 0, 0, 0, 0, 0],
    errMax: 6,
    compOn: false,
    comp: [0, 0, 0, 0, 0, 0],
    // Bewegungsführung: 'manual' | 'program' | 'traj'
    owner: 'manual',
    play: null, // laufende Trajektorie {plan, t0, loop, label}
    vScale: 1,
    estop: false,
    estopReason: '',
    pins: { out: {}, mode: {} },
    objects: [],
    targets: [],
    zones: [],
    markers: [],
    conveyor: null,
    held: null,
    F: null, // Frames (real)
    Fn: null, // Frames (Modell, Soll)
    tcpTrue: null,
    tcpVel: 0,
    scope: [],
    trace: [],
    nextId: 1,
  });

  // ------------------------------------------------------------------
  // Matrix-Hilfen (zeilenweise 4×4)
  // ------------------------------------------------------------------
  function rotZ(a) {
    const c = Math.cos(a),
      s = Math.sin(a);
    return [c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  }
  function setPos(M, p) {
    M[3] = p[0];
    M[7] = p[1];
    M[11] = p[2];
    return M;
  }
  function invRigid(M) {
    // Inverse einer Starrkörpertransformation
    const R = [M[0], M[4], M[8], M[1], M[5], M[9], M[2], M[6], M[10]];
    const t = [M[3], M[7], M[11]];
    const ti = [-(R[0] * t[0] + R[1] * t[1] + R[2] * t[2]), -(R[3] * t[0] + R[4] * t[1] + R[5] * t[2]), -(R[6] * t[0] + R[7] * t[1] + R[8] * t[2])];
    return [R[0], R[1], R[2], ti[0], R[3], R[4], R[5], ti[1], R[6], R[7], R[8], ti[2], 0, 0, 0, 1];
  }
  const pos = (M) => [M[3], M[7], M[11]];
  const col = (M, j) => [M[j], M[4 + j], M[8 + j]];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  sim.mat = { rotZ, setPos, invRigid, pos, col };

  /** Halbe Ausdehnung eines Quaders entlang der Richtung a (Einheitsvektor) */
  function halfExtent(o, a) {
    let s = 0;
    for (let k = 0; k < 3; k++) s += Math.abs(dot(a, col(o.M, k))) * o.size[k] * 0.5;
    return s;
  }
  sim.halfExtent = halfExtent;

  // ------------------------------------------------------------------
  // Servos
  // ------------------------------------------------------------------
  sim.servoTarget = function (i) {
    let v = sim.cmd[i] + (sim.compOn ? sim.comp[i] : 0);
    return BS.clamp(v, 0, 180);
  };
  /** Wahre Gelenkwinkel (inkl. Fertigungsfehler) */
  sim.trueAngles = function () {
    return sim.act.map((a, i) => a + (sim.errOn ? sim.errOff[i] : 0));
  };
  sim.jawFromAngle = (a) => BS.clamp((C.gripper.wMax * (C.gripper.closed - a)) / (C.gripper.closed - C.gripper.open), 0, C.gripper.wMax * 1.1);
  sim.angleFromJaw = (w) => C.gripper.closed - (w / C.gripper.wMax) * (C.gripper.closed - C.gripper.open);
  sim.jawWidth = () => sim.jawFromAngle(sim.act[5]);

  sim.canManual = () => sim.owner === 'manual' || sim.owner === 'traj-manual';

  sim.setCmd = function (i, v) {
    sim.cmd[i] = v;
  };
  sim.setManual = function (m) {
    if (sim.owner === 'program') return false;
    if (sim.play) sim.stopMotion();
    for (let i = 0; i < m.length; i++) if (m[i] != null) sim.cmd[i] = m[i];
    return true;
  };

  // ------------------------------------------------------------------
  // Trajektorien abspielen (Bedienung, IK-Anfahren, Trajektorien-Tab)
  // ------------------------------------------------------------------
  sim.playPlan = function (plan, opts) {
    if (sim.owner === 'program') {
      BS.toast('Programm läuft – bitte zuerst stoppen', 'warn');
      return false;
    }
    if (!plan || !plan.ok) return false;
    opts = opts || {};
    sim.play = { plan, t0: sim.t, loop: !!opts.loop, label: opts.label || 'Bewegung', onEnd: opts.onEnd };
    sim.owner = 'traj';
    BS.emit('owner');
    return true;
  };
  sim.moveTo = function (m, opts) {
    const target = sim.cmd.slice();
    for (let i = 0; i < 6; i++) if (m[i] != null) target[i] = m[i];
    const plan = BS.traj.ptp(sim.cmd.slice(), target, (opts && opts.speed) || 70);
    return sim.playPlan(plan, Object.assign({ label: 'PTP' }, opts));
  };
  sim.stopMotion = function () {
    if (sim.play) {
      sim.play = null;
      if (sim.owner === 'traj') sim.owner = 'manual';
      BS.emit('owner');
    }
  };

  // ------------------------------------------------------------------
  // Scheduler für Arduino-Programme
  // ------------------------------------------------------------------
  const sched = (sim.sched = { pending: null, idleWaiter: null });
  sim.sleep = function (ms) {
    return new Promise((res, rej) => {
      sched.pending = { t: sim.t + Math.max(0, ms) / 1000, res, rej };
      sim.notifyIdle();
    });
  };
  sim.notifyIdle = function () {
    const w = sched.idleWaiter;
    sched.idleWaiter = null;
    if (w) w();
  };
  sim.cancelPending = function (err) {
    const p = sched.pending;
    sched.pending = null;
    if (p) p.rej(err);
    sim.notifyIdle();
  };
  function waitIdle() {
    return new Promise((res) => {
      sched.idleWaiter = res;
      // Sicherheitsnetz, falls das Programm nicht mehr zurückmeldet
      setTimeout(() => {
        if (sched.idleWaiter === res) {
          sched.idleWaiter = null;
          res();
        }
      }, 250);
    });
  }

  /** Simulationszeit um realDt·speed vorwärts bewegen (async wegen Programmausführung) */
  sim.frame = async function (realDt) {
    if (sim.paused) return;
    const tTarget = sim.t + realDt * sim.speed;
    let guard = 0;
    while (guard++ < 20000) {
      const p = sched.pending;
      if (p && p.t <= tTarget) {
        advance(p.t);
        sched.pending = null;
        const idle = waitIdle();
        p.res();
        await idle;
        continue;
      }
      advance(tTarget);
      break;
    }
  };

  function advance(tEnd) {
    let n = 0;
    while (sim.t < tEnd - 1e-9 && n++ < 400) {
      const dt = Math.min(0.002, tEnd - sim.t);
      step(dt);
      sim.t += dt;
    }
    if (sim.t < tEnd) sim.t = tEnd;
  }

  // ------------------------------------------------------------------
  // Ein Simulationsschritt
  // ------------------------------------------------------------------
  let scopeAcc = 0,
    traceAcc = 0,
    lastTcp = null;
  function step(dt) {
    // 1) Trajektorie
    if (sim.play) {
      const P = sim.play;
      let tl = sim.t - P.t0;
      if (tl >= P.plan.duration) {
        if (P.loop) {
          P.t0 = sim.t;
          tl = 0;
        } else {
          const s = P.plan.sample(P.plan.duration);
          for (let i = 0; i < 6; i++) sim.cmd[i] = sim.roundCmd ? Math.round(s.q[i]) : s.q[i];
          const cb = P.onEnd;
          sim.play = null;
          sim.owner = 'manual';
          BS.emit('owner');
          if (cb) cb();
        }
      }
      if (sim.play) {
        const s = P.plan.sample(tl);
        for (let i = 0; i < 6; i++) sim.cmd[i] = sim.roundCmd ? Math.round(s.q[i]) : s.q[i];
        sim.playT = tl;
      }
    }
    // 2) Servos
    const vmax = C.servo.vmax,
      tau = C.servo.tau;
    let gripLimit = Infinity;
    if (sim.held) gripLimit = sim.angleFromJaw(sim.held.gripW);
    for (let i = 0; i < 6; i++) {
      const target = sim.servoTarget(i);
      const e = target - sim.act[i];
      const vm = vmax[i] * sim.vScale;
      let v = BS.clamp(e / tau, -vm, vm);
      let next = sim.act[i] + v * dt;
      if ((target - next) * e < 0) next = target;
      if (i === 5 && next > gripLimit) next = Math.max(sim.act[5], Math.min(next, gripLimit));
      sim.vel[i] = (next - sim.act[i]) / dt;
      sim.act[i] = next;
    }
    // 3) Kinematik
    const qt = sim.trueAngles();
    sim.F = kin.frames(qt);
    const tcp = pos(sim.F[5]);
    if (lastTcp) {
      const v = Math.hypot(tcp[0] - lastTcp[0], tcp[1] - lastTcp[1], tcp[2] - lastTcp[2]) / dt;
      sim.tcpVel += (v - sim.tcpVel) * Math.min(1, dt / 0.03);
    }
    lastTcp = tcp;
    // 4) Greifen & Objekte
    updateGrip();
    updateObjects(dt);
    // 5) Überwachung
    BS.monitor.check(dt);
    // 6) Aufzeichnung
    scopeAcc += dt;
    if (scopeAcc >= 0.02) {
      scopeAcc = 0;
      sim.scope.push({ t: sim.t, cmd: sim.cmd.slice(), act: sim.act.slice(), v: sim.tcpVel });
      if (sim.scope.length > 3000) sim.scope.splice(0, sim.scope.length - 3000);
    }
    traceAcc += dt;
    if (traceAcc >= 0.01) {
      traceAcc = 0;
      const L = sim.trace[sim.trace.length - 1];
      if (!L || Math.hypot(L[0] - tcp[0], L[1] - tcp[1], L[2] - tcp[2]) > 1) {
        sim.trace.push(tcp);
        if (sim.trace.length > 4000) sim.trace.shift();
        sim.traceDirty = true;
      }
    }
  }
  sim.updateKinematics = function () {
    sim.F = kin.frames(sim.trueAngles());
  };

  // ------------------------------------------------------------------
  // Greifen
  // ------------------------------------------------------------------
  function graspFrame() {
    const T5 = sim.F[5];
    return { T: T5, p: pos(T5), x: col(T5, 0), y: col(T5, 1), z: col(T5, 2) };
  }
  function updateGrip() {
    const g = graspFrame();
    const cmdW = sim.jawFromAngle(sim.servoTarget(5));
    if (sim.held) {
      const o = sim.held.obj;
      o.M = kin.mul(g.T, sim.held.rel);
      if (cmdW > Math.min(sim.held.gripW + 2, C.gripper.wMax - 0.5)) {
        // loslassen
        releaseObject(o);
      }
      return;
    }
    const w = sim.jawWidth();
    for (const o of sim.objects) {
      if (o.held) continue;
      const rel = [o.M[3] - g.p[0], o.M[7] - g.p[1], o.M[11] - g.p[2]];
      const hx = halfExtent(o, g.x);
      const along = dot(rel, g.z),
        side = dot(rel, g.y),
        across = dot(rel, g.x);
      const hz = halfExtent(o, g.z),
        hy = halfExtent(o, g.y);
      const inRegion = along > -45 - hz * 0.2 && along < hz + 12 && Math.abs(side) < hy + 6 && Math.abs(across) < hx + 2;
      // Nur greifen, wenn die Backen vorher geöffnet um das Objekt lagen
      if (!inRegion) o.armed = false;
      else if (w > 2 * hx + 1) o.armed = true;
      if (inRegion && o.armed && w <= 2 * hx + 0.5 && cmdW < 2 * hx) {
        o.armed = false;
        // Die schließenden Finger zentrieren das Objekt zwischen den Backen
        o.M[3] -= across * g.x[0];
        o.M[7] -= across * g.x[1];
        o.M[11] -= across * g.x[2];
        // ... und drehen es so, dass eine Seitenfläche plan an den Backen anliegt
        const k = alignToJaw(o, g.x);
        o.held = true;
        o.resting = false;
        o.onBelt = false;
        sim.held = { obj: o, rel: kin.mul(invRigid(g.T), o.M), gripW: o.size[k] };
        BS.emit('grip', { obj: o, held: true });
        break;
      }
    }
  }
  /** Dreht Objekt o um seine Mitte, bis die Achse, die am besten zu a passt, parallel zu a ist. Liefert den Achsindex. */
  function alignToJaw(o, a) {
    let k = 0,
      best = -1;
    for (let j = 0; j < 3; j++) {
      const d = Math.abs(dot(col(o.M, j), a));
      if (d > best) {
        best = d;
        k = j;
      }
    }
    let u = col(o.M, k);
    if (dot(u, a) < 0) u = u.map((v) => -v);
    // Rodrigues: Drehachse n = u × a, Winkel θ
    let n = [u[1] * a[2] - u[2] * a[1], u[2] * a[0] - u[0] * a[2], u[0] * a[1] - u[1] * a[0]];
    const sn = Math.hypot(n[0], n[1], n[2]);
    if (sn < 1e-9) return k;
    n = n.map((v) => v / sn);
    const c = BS.clamp(dot(u, a), -1, 1),
      s = sn,
      t = 1 - c;
    const R = [
      [t * n[0] * n[0] + c, t * n[0] * n[1] - s * n[2], t * n[0] * n[2] + s * n[1]],
      [t * n[0] * n[1] + s * n[2], t * n[1] * n[1] + c, t * n[1] * n[2] - s * n[0]],
      [t * n[0] * n[2] - s * n[1], t * n[1] * n[2] + s * n[0], t * n[2] * n[2] + c],
    ];
    const M = o.M.slice();
    for (let i = 0; i < 3; i++)
      for (let j = 0; j < 3; j++) M[i * 4 + j] = R[i][0] * o.M[j] + R[i][1] * o.M[4 + j] + R[i][2] * o.M[8 + j];
    o.M = M;
    return k;
  }

  function releaseObject(o) {
    o.held = false;
    sim.held = null;
    // auf Gierwinkel reduzieren: senkrechteste Achse als "oben"
    const ax = [0, 1, 2].map((k) => col(o.M, k));
    let up = 0;
    for (let k = 1; k < 3; k++) if (Math.abs(ax[k][2]) > Math.abs(ax[up][2])) up = k;
    const others = [0, 1, 2].filter((k) => k !== up);
    const h = ax[others[0]];
    const yaw = Math.atan2(h[1], h[0]);
    // Größe so umsortieren, dass size[2] die Höhe ist
    const s = o.size;
    o.size = [s[others[0]], s[others[1]], s[up]];
    const p = pos(o.M);
    o.M = setPos(rotZ(yaw), p);
    o.vz = 0;
    o.resting = false;
    BS.emit('grip', { obj: o, held: false });
  }
  sim.releaseAll = function () {
    if (sim.held) releaseObject(sim.held.obj);
  };

  // ------------------------------------------------------------------
  // Objekte: Fallen, Stapeln, Förderband
  // ------------------------------------------------------------------
  function footprintOverlap(a, b) {
    const ra = Math.max(a.size[0], a.size[1]) * 0.5,
      rb = Math.max(b.size[0], b.size[1]) * 0.5;
    return Math.hypot(a.M[3] - b.M[3], a.M[7] - b.M[7]) < (ra + rb) * 0.75;
  }
  function supportHeight(o) {
    let h = 0;
    const cv = sim.conveyor;
    if (cv && onBeltArea(o)) h = cv.height;
    for (const b of sim.objects) {
      if (b === o || b.held) continue;
      // b trägt o, wenn b darunter liegt und sich die Grundflächen überlappen (o stapelt sich auf b)
      const top = b.M[11] + b.size[2] / 2;
      if (b.M[11] < o.M[11] - 0.5 && footprintOverlap(o, b)) h = Math.max(h, top);
    }
    return h;
  }
  function onBeltArea(o) {
    const cv = sim.conveyor;
    if (!cv) return false;
    return Math.abs(o.M[3] - cv.x) < cv.width / 2 && o.M[7] < cv.yStart + 10 && o.M[7] > cv.yEnd - 25;
  }
  function updateObjects(dt) {
    const cv = sim.conveyor;
    for (const o of sim.objects) {
      if (o.held) continue;
      if (!o.resting) {
        o.vz = (o.vz || 0) - 9810 * dt;
        o.M[11] += o.vz * dt;
        const sup = supportHeight(o);
        if (o.M[11] - o.size[2] / 2 <= sup) {
          o.M[11] = sup + o.size[2] / 2;
          o.vz = 0;
          o.resting = true;
        }
      } else {
        // Unterlage weggenommen?
        const sup = supportHeight(o);
        if (o.M[11] - o.size[2] / 2 > sup + 0.5) o.resting = false;
      }
    }
    if (!cv) return;
    cv.running = !!sim.pins.out[cv.pinMotor] || cv.manual;
    // Objekte auf dem Band bewegen (Richtung −y), Stau am Anschlag
    const onBelt = sim.objects.filter((o) => !o.held && o.resting && onBeltArea(o) && Math.abs(o.M[11] - o.size[2] / 2 - cv.height) < 1);
    onBelt.sort((a, b) => a.M[7] - b.M[7]);
    let front = cv.yEnd; // Anschlag
    for (const o of onBelt) {
      const half = o.size[1] / 2;
      if (cv.running) {
        let ny = o.M[7] - cv.speed * dt;
        ny = Math.max(ny, front + half);
        o.M[7] = Math.min(o.M[7], ny);
      }
      front = o.M[7] + half + 2;
    }
    // Lichtschranken
    const beam = (z) =>
      sim.objects.some((o) => Math.abs(o.M[7] - cv.lsY) < o.size[1] / 2 && Math.abs(o.M[3] - cv.x) < cv.width / 2 + 20 && o.M[11] - o.size[2] / 2 < z && o.M[11] + o.size[2] / 2 > z);
    cv.lsLowState = beam(cv.lsLow);
    cv.lsHighState = beam(cv.lsHigh);
    // Neue Boxen
    if (cv.autoSpawn && cv.running && sim.t >= cv.nextSpawn) {
      const free = !sim.objects.some((o) => Math.abs(o.M[3] - cv.x) < cv.width && o.M[7] > cv.yStart - 80);
      if (free) {
        const big = Math.random() < 0.5;
        const s = big ? 46 : 28;
        sim.addObject({
          size: [s, s, s],
          color: big ? '#1c7ed6' : '#f76707',
          mass: big ? 60 : 20,
          pos: [cv.x, cv.yStart - s / 2 - 5, cv.height + s / 2],
          yaw: 0,
          tag: big ? 'groß' : 'klein',
          name: big ? 'Box groß' : 'Box klein',
        });
        cv.nextSpawn = sim.t + cv.interval;
      }
    }
  }

  sim.addObject = function (o) {
    const s = o.size || [30, 30, 30];
    // ohne Angabe: Würfel zum Roboter ausgerichtet (Gierwinkel = Azimut)
    const yaw = o.yaw != null ? o.yaw : (Math.atan2(o.pos[1], o.pos[0]) * 180) / Math.PI;
    const M = setPos(rotZ((yaw * Math.PI) / 180), [o.pos[0], o.pos[1], o.pos[2] != null ? o.pos[2] : s[2] / 2]);
    const obj = {
      id: sim.nextId++,
      name: o.name || 'Würfel',
      size: s.slice(),
      color: o.color || '#e03131',
      mass: o.mass || 30,
      M,
      held: false,
      resting: false,
      vz: 0,
      tag: o.tag || '',
    };
    sim.objects.push(obj);
    BS.emit('objects');
    return obj;
  };
  sim.removeObject = function (o) {
    if (sim.held && sim.held.obj === o) sim.held = null;
    sim.objects = sim.objects.filter((x) => x !== o);
    BS.emit('objects');
  };

  /** Liegt Objekt o im Zielbereich tg? */
  sim.inTarget = function (o, tg) {
    return !o.held && o.resting && Math.abs(o.M[3] - tg.x) < tg.w / 2 && Math.abs(o.M[7] - tg.y) < tg.h / 2;
  };

  // ------------------------------------------------------------------
  // Szenarien
  // ------------------------------------------------------------------
  sim.scenarios = {
    leer: { name: 'Leerer Tisch' },
    einzel: { name: 'Einzelner Würfel' },
    sortieren: { name: 'Farbsortierung (3 Würfel)' },
    stapeln: { name: 'Stapeln' },
    band: { name: 'Förderband mit Lichtschranken' },
  };
  sim.loadScenario = function (key) {
    sim.releaseAll();
    sim.objects = [];
    sim.targets = [];
    sim.conveyor = null;
    sim.scenario = key;
    const T = (name, x, y, w, h, color, accept) => sim.targets.push({ id: sim.nextId++, name, x, y, w, h, color, accept });
    if (key === 'einzel') {
      sim.addObject({ pos: [0, 260], color: '#e03131', name: 'Würfel rot' });
      T('Ziel', 200, 170, 70, 70, '#e03131', '');
    } else if (key === 'sortieren') {
      sim.addObject({ pos: [-120, 230], color: '#e03131', name: 'Würfel rot' });
      sim.addObject({ pos: [0, 260], color: '#2f9e44', name: 'Würfel grün' });
      sim.addObject({ pos: [120, 230], color: '#1c7ed6', name: 'Würfel blau' });
      T('Ziel rot', 230, 60, 70, 70, '#e03131', '#e03131');
      T('Ziel grün', 260, 170, 70, 70, '#2f9e44', '#2f9e44');
      T('Ziel blau', 200, 270, 70, 70, '#1c7ed6', '#1c7ed6');
    } else if (key === 'stapeln') {
      sim.addObject({ pos: [-160, 200], color: '#e03131', name: 'Würfel 1' });
      sim.addObject({ pos: [-60, 260], color: '#f59f00', name: 'Würfel 2' });
      sim.addObject({ pos: [60, 260], color: '#1c7ed6', name: 'Würfel 3' });
      T('Stapelplatz', 200, 200, 60, 60, '#868e96', '');
    } else if (key === 'band') {
      sim.conveyor = {
        x: -230,
        yStart: 440,
        yEnd: 140,
        width: 90,
        height: 30,
        speed: 60,
        lsY: 168,
        lsLow: 30 + 12,
        lsHigh: 30 + 38,
        pinMotor: 7,
        pinLow: 2,
        pinHigh: 4,
        autoSpawn: true,
        interval: 7,
        nextSpawn: 0,
        manual: false,
        running: false,
        lsLowState: false,
        lsHighState: false,
      };
      T('Ablage klein', 230, 110, 100, 100, '#f76707', 'klein');
      T('Ablage groß', 170, 300, 120, 120, '#1c7ed6', 'groß');
    }
    BS.emit('objects');
    BS.emit('scenario');
  };

  // ------------------------------------------------------------------
  // Kalibrierfehler
  // ------------------------------------------------------------------
  sim.randomizeErrors = function () {
    const m = sim.errMax;
    sim.errOff = [0, 1, 2, 3, 4].map(() => +((Math.random() * 2 - 1) * m).toFixed(2)).concat([0]);
    sim.errRevealed = false;
  };
  sim.markers = [
    { id: 1, name: 'K1', p: [150, 150, 60] },
    { id: 2, name: 'K2', p: [-150, 150, 60] },
    { id: 3, name: 'K3', p: [0, 260, 40] },
    { id: 4, name: 'K4', p: [0, 170, 130] },
    { id: 5, name: 'K5', p: [220, 60, 80] },
    { id: 6, name: 'K6', p: [-200, 250, 100] },
  ];

  // ------------------------------------------------------------------
  // Not-Halt / Reset
  // ------------------------------------------------------------------
  sim.emergencyStop = function (reason) {
    if (sim.estop) return;
    sim.estop = true;
    sim.estopReason = reason || 'Not-Halt';
    if (BS.arduino) BS.arduino.stop('NOT-HALT: ' + sim.estopReason);
    sim.play = null;
    sim.owner = 'manual';
    for (let i = 0; i < 6; i++) sim.cmd[i] = sim.act[i] - (sim.compOn ? sim.comp[i] : 0);
    BS.emit('owner');
    BS.emit('estop');
  };
  sim.ackEstop = function () {
    sim.estop = false;
    sim.estopReason = '';
    BS.emit('estop');
  };

  sim.reset = function () {
    if (BS.arduino) BS.arduino.stop();
    sim.play = null;
    sim.owner = 'manual';
    sim.cmd = C.beginPose.slice();
    sim.act = C.beginPose.slice();
    sim.vel = [0, 0, 0, 0, 0, 0];
    sim.pins = { out: {}, mode: {} };
    sim.trace = [];
    sim.traceDirty = true;
    sim.scope = [];
    sim.t = 0;
    sim.estop = false;
    sim.vScale = 1;
    lastTcp = null;
    sim.loadScenario(sim.scenario || 'sortieren');
    sim.updateKinematics();
    BS.monitor.reset();
    BS.emit('owner');
    BS.emit('estop');
    BS.emit('reset');
  };

  // ------------------------------------------------------------------
  // Arbeitsraumüberwachung
  // ------------------------------------------------------------------
  const mon = (BS.monitor = {
    enabled: true,
    reaction: 'warn', // 'warn' | 'slow' | 'stop'
    margin: 20,
    table: true,
    base: true,
    vLimitOn: false,
    vLimit: 250,
    state: 'ok',
    issues: [],
    log: [],
    zoneState: {},
    linkState: {},
  });
  mon.reset = function () {
    mon.issues = [];
    mon.state = 'ok';
    mon.prevKeys = new Set();
    mon.zoneState = {};
    mon.linkState = {};
  };
  mon.prevKeys = new Set();
  mon.addLog = function (level, msg) {
    mon.log.unshift({ t: sim.t, level, msg });
    if (mon.log.length > 200) mon.log.pop();
    BS.emit('monlog');
  };
  function inBox(p, z, m) {
    return (
      p[0] > z.c[0] - z.s[0] / 2 - m &&
      p[0] < z.c[0] + z.s[0] / 2 + m &&
      p[1] > z.c[1] - z.s[1] / 2 - m &&
      p[1] < z.c[1] + z.s[1] / 2 + m &&
      p[2] > z.c[2] - z.s[2] / 2 - m &&
      p[2] < z.c[2] + z.s[2] / 2 + m
    );
  }
  mon.check = function () {
    const issues = [];
    const zoneState = {},
      linkState = {};
    if (mon.enabled && sim.F) {
      const ap = kin.armPoints(sim.F, C.geom, sim.jawWidth());
      for (const z of sim.zones) {
        if (z.type === 'forbidden') {
          let lvl = null,
            link = '';
          for (const q of ap.pts) {
            if (inBox(q.p, z, 0)) {
              lvl = 'viol';
              link = q.link;
              break;
            }
            if (!lvl && inBox(q.p, z, mon.margin)) {
              lvl = 'warn';
              link = q.link;
            }
          }
          if (lvl) {
            zoneState[z.id] = lvl;
            linkState[link] = lvl;
            issues.push({ key: 'z' + z.id + lvl, level: lvl, msg: `${link} ${lvl === 'viol' ? 'in' : 'nahe'} Sperrzone „${z.name}“` });
          }
        } else if (z.type === 'allowed') {
          if (!inBox(ap.tcp, z, 0)) {
            zoneState[z.id] = 'viol';
            issues.push({ key: 'a' + z.id, level: 'viol', msg: `TCP außerhalb des Arbeitsbereichs „${z.name}“` });
          } else if (!inBox(ap.tcp, z, -mon.margin)) {
            zoneState[z.id] = 'warn';
            issues.push({ key: 'aw' + z.id, level: 'warn', msg: `TCP nahe Grenze des Arbeitsbereichs „${z.name}“` });
          }
        }
      }
      if (mon.table) {
        for (const q of ap.pts)
          if (q.p[2] < -1) {
            linkState[q.link] = 'viol';
            issues.push({ key: 'table' + q.link, level: 'viol', msg: `Kollision mit Tisch (${q.link})` });
            break;
          }
      }
      if (mon.base) {
        for (const q of ap.pts) {
          if (q.link === 'Oberarm') continue;
          if (Math.hypot(q.p[0], q.p[1]) < 62 && q.p[2] < 85) {
            linkState[q.link] = 'viol';
            issues.push({ key: 'base' + q.link, level: 'viol', msg: `Selbstkollision mit Basis (${q.link})` });
            break;
          }
        }
      }
      if (mon.vLimitOn && sim.tcpVel > mon.vLimit) issues.push({ key: 'vel', level: 'warn', msg: `TCP-Geschwindigkeit ${sim.tcpVel.toFixed(0)} mm/s > ${mon.vLimit} mm/s` });
      for (let i = 0; i < 6; i++) {
        const L = C.limits[i];
        if (sim.cmd[i] < L[0] - 0.01 || sim.cmd[i] > L[1] + 0.01)
          issues.push({ key: 'lim' + i, level: 'warn', msg: `${C.short[i]} Soll ${sim.cmd[i].toFixed(0)}° außerhalb [${L[0]}, ${L[1]}]` });
      }
    }
    // Flanken protokollieren
    const keys = new Set(issues.map((i) => i.key));
    let newViol = null;
    for (const is of issues)
      if (!mon.prevKeys.has(is.key)) {
        mon.addLog(is.level, is.msg);
        if (is.level === 'viol' && !newViol) newViol = is;
      }
    mon.prevKeys = keys;
    mon.issues = issues;
    mon.zoneState = zoneState;
    mon.linkState = linkState;
    mon.state = issues.some((i) => i.level === 'viol') ? 'viol' : issues.length ? 'warn' : 'ok';
    // Reaktion
    if (mon.enabled && mon.reaction === 'slow') sim.vScale = mon.state === 'ok' ? 1 : 0.25;
    else sim.vScale = 1;
    if (mon.enabled && mon.reaction === 'stop' && newViol) sim.emergencyStop(newViol.msg);
  };
})();
