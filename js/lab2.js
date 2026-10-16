/* Braccio-Simulator – Laborübung 2: Nachbildung des Simulink-Modells Braccio_CtrlSimMdl.slx
 *
 * Signalfluss (feste Abtastzeit dt = 10 ms, wie im Modell):
 *   Bedienpanel → Chart (Pause / Run / Notaus) → Umschalter CtrlMode
 *     1 Manual (Slider φ)  2 PushBtnCtrl (+/−)  3 Replay (TeachIn)  4 PickAndPlace  5 CartesianPath
 *   → JointLimits → simpleDynamics (Ratenbegrenzer) → ForwardKinematics (MATLAB-Code) → Arbeitsraumüberwachung
 * Die MATLAB-Function-Blöcke (ForwardKinematics, StoreWaypoints, inverseKinematics3dof) laufen im
 * MATLAB-Interpreter (js/matlab.js) mit dem vom Benutzer bearbeiteten Code.
 *
 * Koordinaten: Labor-KS0 in m (x nach vorne), Simulator in mm: x_sim = −y_lab, y_sim = x_lab, z_sim = z_lab.
 * Gelenkwinkel q (DH-Konvention der Übung) ↔ Servowinkel M des Simulators:
 *   M1 = q1 + 90°, M2 = 180° − q2, M3 = 90° − q3, M4 = 180° − q4, M5 = q5 + 90°, M6 = q6
 */
(function () {
  'use strict';
  const BS = window.BS;
  const ml = BS.ml;
  const C = BS.config;
  const D2R = Math.PI / 180,
    R2D = 180 / Math.PI;
  const DT = 0.01;
  const clamp = BS.clamp;

  // Offizielle Parameter der Übung (Tabelle in Robotik_Uebung2.pdf)
  const DH_REF = {
    d1: 0.0715, a1: 0, alpha1: Math.PI / 2, theta1: 0,
    d2: 0, a2: 0.125, alpha2: 0, theta2: Math.PI / 2,
    d3: 0, a3: 0.125, alpha3: 0, theta3: 0,
    d4: 0, a4: 0, alpha4: Math.PI / 2, theta4: Math.PI / 2,
    d5: 0.13, a5: 0, alpha5: 0, theta5: 0,
  }; // prettier-ignore
  const LIM_DEF = [
    [-90, 90],
    [15, 165],
    [-90, 90],
    [0, 180],
    [-90, 90],
    [10, 73],
  ];
  const HOME_Q = [0, 90, 0, 90, 0]; // Grad, Default-Pose (alle φ = 0)
  // PushBtnCtrl: Geschwindigkeit je Achse [°/s] und Integratorgrenzen [°]
  const PB_RATE = [50, 15, 30, 30, 30, 30];
  const PB_LIM = [
    [-90, 90],
    [0, 180],
    [-90, 90],
    [0, 180],
    [-90, 90],
    [-90, 90],
  ];
  const MODES = ['Manual (Default Pose)', 'Manual (Push Buttons)', 'Replay', "Pick'n'Place", 'Cartesian Control', 'Demo Mode'];

  const DEF_PARAMS = {
    pnpWaypoints: 'zeros(5,7)',
    pnpTimePoints: '[0 2 3 4 6 7 9]',
    pnpMethod: 'quintic',
    pnpGripT: '[0 2.2 2.8 4 6.2 6.8 9]',
    pnpGripV: '[10 10 50 50 50 10 10]',
    pnpLoopT: '9',
    pnpLoop: true,
    cartMaxIter: '500',
    cartWeights: '[1000;1000;1]',
    cartLoop: true,
    rateLimit: '1',
    autoIni: true,
  };

  const L = (BS.lab2 = {
    MODES,
    DH_REF,
    running: false,
    files: [],
    params: Object.assign({}, DEF_PARAMS, BS.store.get('lab2params', {})),
    hmi: { mode: 1, axis: 2, phi: [0, 0, 0, 0, 0, 10], up: false, down: false, show: { cube1: true, cube2: true, regal: true, efence: true } },
    pulses: {},
    st: null,
    pv: null,
    lastError: null,
    consoleBuf: [],
    skeleton: null,
    trails: [],
  });

  // ------------------------------------------------------------------
  // Dateien (Originale + Änderungen des Benutzers)
  // ------------------------------------------------------------------
  const saved = BS.store.get('lab2files', {});
  L.files = (window.BS_LAB2_FILES || []).map((f) => ({ ...f, orig: f.src, src: saved[f.name] != null ? saved[f.name] : f.src }));
  L.file = (name) => L.files.find((f) => f.name === name);
  L.setSource = function (name, src) {
    const f = L.file(name);
    if (!f || f.src === src) return;
    f.src = src;
    f.parsed = null;
    const all = BS.store.get('lab2files', {});
    if (src === f.orig) delete all[name];
    else all[name] = src;
    BS.store.set('lab2files', all);
  };
  L.saveParams = () => BS.store.set('lab2params', L.params);
  L.resetParams = () => {
    L.params = Object.assign({}, DEF_PARAMS);
    L.saveParams();
  };
  /** Datei parsen (zwischengespeichert); Syntaxfehler mit Zeilennummer */
  L.parsed = function (name) {
    const f = L.file(name);
    if (!f.parsed || f.parsedSrc !== f.src) {
      f.parsed = ml.parse(f.src, f.name);
      f.parsedSrc = f.src;
    }
    return f.parsed;
  };

  // ------------------------------------------------------------------
  // MATLAB-Interpreter & Befehlsfenster
  // ------------------------------------------------------------------
  function print(text, kind) {
    L.consoleBuf.push({ text, kind: kind || '' });
    if (L.consoleBuf.length > 2000) L.consoleBuf.splice(0, L.consoleBuf.length - 1500);
    BS.emit('lab2:print', { text, kind });
  }
  L.print = print;
  L.interp = new ml.Interp({
    print,
    onEvent(ev, data) {
      if (ev === 'clc') {
        L.consoleBuf = [];
        BS.emit('lab2:clc');
      } else if (ev === 'figures') BS.emit('lab2:figures');
      else if (ev === 'workspace') BS.emit('lab2:workspace');
      else if (ev === 'showTree') L.showTree(data);
      else if (ev === 'animatedline') L.trails.push(data);
      else if (ev === 'addpoints') L.trailsDirty = true;
    },
  });
  const ws = () => L.interp.base.vars;

  function errText(e) {
    if (!(e instanceof ml.MLError)) return String((e && e.message) || e);
    return `${e.message}${e.file ? `\n→ Fehler in ${e.file}${e.line ? ', Zeile ' + e.line : ''}` : e.line ? ` (Zeile ${e.line})` : ''}`;
  }
  L.errText = errText;

  /** Skript (ganz oder Abschnitt) im Basis-Workspace ausführen */
  L.runFile = function (name, section) {
    const f = L.file(name);
    let src = f.src,
      label = name;
    if (section) {
      src = section.src;
      label = name;
    }
    const t0 = performance.now();
    try {
      // Abschnitte: Leerzeilen voranstellen, damit Zeilennummern stimmen
      const pad = section ? '\n'.repeat(section.line0) : '';
      L.interp.runScript(pad + src, label);
      L.lastScriptError = null;
      BS.emit('lab2:ran', { name, ms: performance.now() - t0 });
      return true;
    } catch (e) {
      print(errText(e) + '\n', 'err');
      L.lastScriptError = { file: e.file || name, line: e.line, msg: e.message };
      BS.emit('lab2:scriptError', L.lastScriptError);
      return false;
    }
  };
  L.runCommand = function (cmd) {
    print('>> ' + cmd + '\n', 'cmd');
    try {
      L.interp.runScript(cmd, 'Befehlsfenster');
      return true;
    } catch (e) {
      print(errText(e).replace(/\n→ Fehler in Befehlsfenster.*$/, '') + '\n', 'err');
      return false;
    }
  };
  /** Ausdruck im Basis-Workspace auswerten (Blockparameter) */
  L.evalExpr = function (src, what) {
    const I = L.interp;
    const name = '__blockparam__';
    try {
      I.runScript(`${name} = ${src};`, what || 'Blockparameter');
    } catch (e) {
      throw new ml.MLError(`Blockparameter ${what ? '„' + what + '“ ' : ''}= ${src}: ${e.message}`);
    } finally {
      // Blockparameter erscheinen nicht im Workspace
    }
    const v = ws().get(name);
    ws().delete(name);
    return v;
  };

  // ------------------------------------------------------------------
  // Umrechnungen
  // ------------------------------------------------------------------
  const toSim = (p) => [-p[1] * 1000, p[0] * 1000, p[2] * 1000];
  const toLab = (p) => [p[1] / 1000, -p[0] / 1000, p[2] / 1000];
  /** Gelenkwinkel q [rad] → Servowinkel M [°] */
  function qToM(q, q6) {
    return [q[0] * R2D + 90, 180 - q[1] * R2D, 90 - q[2] * R2D, 180 - q[3] * R2D, q[4] * R2D + 90, q6 != null ? q6 * R2D : BS.sim.cmd[5]];
  }
  function mToQ(m) {
    return [(m[0] - 90) * D2R, (180 - m[1]) * D2R, (90 - m[2]) * D2R, (180 - m[3]) * D2R, (m[4] - 90) * D2R];
  }
  L.toSim = toSim;
  L.toLab = toLab;
  L.qToM = qToM;
  L.mToQ = mToQ;

  /** Referenz-Vorwärtskinematik mit den offiziellen DH-Parametern (Labor-KS, m) */
  function mm4(A, B) {
    const R = new Array(16);
    for (let i = 0; i < 4; i++)
      for (let j = 0; j < 4; j++) {
        let s = 0;
        for (let k = 0; k < 4; k++) s += A[i * 4 + k] * B[k * 4 + j];
        R[i * 4 + j] = s;
      }
    return R;
  }
  L.refFK = function (q, P) {
    P = P || DH_REF;
    const T = (i, th) => BS.kin.dhMat(th, P['d' + i] + (i === 5 ? 0.05 : 0), P['a' + i] + (i === 5 ? -0.006 : 0), P['alpha' + i]);
    const T1 = T(1, q[0]),
      T2 = T(2, q[1]),
      T3 = T(3, q[2]),
      T4 = T(4, q[3]),
      T5 = T(5, q[4]);
    const T12 = mm4(T1, T2),
      T13 = mm4(T12, T3),
      T15 = mm4(mm4(T13, T4), T5);
    const o = (M) => [M[3], M[7], M[11]];
    return { T15, tcp: o(T15), B: o(T12), C: o(T13) };
  };

  // ------------------------------------------------------------------
  // Szene (Simulink-3D-Aktoren / collisionBox-Objekte des Ini-Skripts)
  // ------------------------------------------------------------------
  const SCENE = {
    platform: { c: [0.1, 0, -0.01], s: [0.6, 0.3, 0.02], color: 0xb4632e, label: 'Plattform' },
    pickupMarker1: { c: [0.25, 0, -0.005], s: [0.025, 0.025, 0.02], color: 0x5c636e },
    pickupMarker2: { c: [0.35, 0, -0.005], s: [0.025, 0.025, 0.02], color: 0x5c636e },
    regal: { c: [0.4, 0, 0.05], s: [0.04, 0.06, 0.1], color: 0x8d6e63, label: 'Regal' },
    box1_floor: { c: [0, 0.2, -0.015], s: [0.1, 0.1, 0.005], color: 0xd9c7a8 },
    box1_wall1: { c: [0, 0.15, 0.03], s: [0.1, 0.005, 0.1], color: 0xd9c7a8 },
    box1_wall2: { c: [0, 0.25, 0.03], s: [0.1, 0.005, 0.1], color: 0xd9c7a8 },
    box1_wall3: { c: [0.05, 0.2, 0.03], s: [0.005, 0.1, 0.1], color: 0xd9c7a8 },
    box1_wall4: { c: [-0.05, 0.2, 0.03], s: [0.005, 0.1, 0.1], color: 0xd9c7a8 },
  };
  const CUBES = {
    cube1: { c: [0.25, 0, 0.015], color: '#e03131', name: 'Würfel 1' },
    cube2: { c: [0.35, 0, 0.015], color: '#1c7ed6', name: 'Würfel 2' },
  };
  const simBox = (b) => {
    const c = toSim(b.c);
    return { c, s: [b.s[1] * 1000, b.s[0] * 1000, b.s[2] * 1000] };
  };
  function efenceX() {
    try {
      const v = ws().get('efence1_coord');
      if (v instanceof ml.Struct && v.f.has('x')) return ml.scalar(v.f.get('x'));
    } catch (e) {
      /* Standardwert */
    }
    return -0.2;
  }
  L.efenceX = efenceX;

  L.buildScenario = function () {
    const sim = BS.sim;
    sim.statics = [];
    for (const [name, b] of Object.entries(SCENE)) {
      if (name === 'platform') continue;
      if (name === 'regal' && !L.hmi.show.regal) continue;
      const sb = simBox(b);
      sim.statics.push({ name, c: sb.c, s: sb.s });
    }
    // Boden: Plattform (z = 0), Sortierbox (Boden bei −12,5 mm), sonst darunter
    sim.groundAt = (x, y) => {
      if (Math.abs(x) <= 150 && y >= -200 && y <= 400) return 0;
      if (x >= -250 && x <= -150 && Math.abs(y) <= 50) return -12.5;
      return -20;
    };
    // Roboter in die Default-Pose der Übung (alle φ = 0) stellen
    if (!sim.locked()) {
      const M = qToM(HOME_Q.map((v) => v * D2R), 10 * D2R);
      if (sim.play) sim.stopMotion();
      for (let i = 0; i < 6; i++) sim.cmd[i] = sim.act[i] = M[i];
      sim.updateKinematics();
    }
    L.cubeObjs = {};
    for (const [k, cb] of Object.entries(CUBES)) {
      if (!L.hmi.show[k]) continue;
      L.cubeObjs[k] = sim.addObject({ size: [20, 20, 20], pos: toSim(cb.c), yaw: 0, color: cb.color, name: cb.name, tag: k });
    }
  };
  L.setShow = function (key, on) {
    L.hmi.show[key] = on;
    const sim = BS.sim;
    if (sim.scenario !== 'labor2') return;
    if (key === 'cube1' || key === 'cube2') {
      const o = sim.objects.find((x) => x.tag === key);
      if (!on && o) sim.removeObject(o);
      if (on && !o) sim.addObject({ size: [20, 20, 20], pos: toSim(CUBES[key].c), yaw: 0, color: CUBES[key].color, name: CUBES[key].name, tag: key });
    }
    if (key === 'regal') {
      sim.statics = sim.statics.filter((s) => s.name !== 'regal');
      if (on) {
        const sb = simBox(SCENE.regal);
        sim.statics.push({ name: 'regal', c: sb.c, s: sb.s });
      }
      sim.objects.forEach((o) => (o.resting = false));
    }
  };
  L.ensureScene = function () {
    if (BS.sim.scenario !== 'labor2') {
      BS.sim.loadScenario('labor2');
      return true;
    }
    return false;
  };

  // ------------------------------------------------------------------
  // Simulation
  // ------------------------------------------------------------------
  const LAB_GEOM = { d1: 71.5, a2: 125, a3: 125, d5: 130, lTcp: 180, lTip: 186 };

  function num2(v, what) {
    try {
      return ml.scalar(v, what);
    } catch (e) {
      throw new ml.MLError(`${what}: Skalar erwartet.`);
    }
  }
  function fieldOf(st, name, what) {
    if (!(st instanceof ml.Struct)) throw new ml.MLError(`Variable '${what}' muss eine Struktur sein.`);
    if (!st.f.has(name)) throw new ml.MLError(`Feld '${what}.${name}' fehlt.`);
    return st.f.get(name);
  }
  /** Blockparameter auswerten (wie „Update Diagram“ in Simulink) */
  function evalParams() {
    const P = L.params;
    const W = ws();
    const need = (n) => {
      if (!W.has(n)) throw new ml.MLError(`Variable '${n}' fehlt im Workspace – zuerst Braccio_kinematics.m ausführen.`);
      return W.get(n);
    };
    const pv = {};
    pv.braccio = need('braccio');
    if (!(pv.braccio instanceof ml.Struct)) throw new ml.MLError("Variable 'braccio' muss eine Struktur sein.");
    // JointLimits: jnt1..jnt6.PositionLimits
    pv.lim = [];
    for (let i = 1; i <= 6; i++) {
      const v = L.evalExpr(`jnt${i}.PositionLimits`, `JointLimits/Saturation${i > 1 ? i - 1 : ''}`);
      const a = ml.toArr(v);
      if (a.length !== 2) throw new ml.MLError(`jnt${i}.PositionLimits muss [min max] sein.`);
      pv.lim.push(a);
    }
    pv.efenceX = num2(L.evalExpr('efence1_coord.x', 'Compare To Constant1'), 'efence1_coord.x');
    // CartesianPath
    const cp = need('cartesianPath');
    const arr = (n) => ml.toArr(fieldOf(cp, n, 'cartesianPath'));
    pv.cart = { time: arr('time'), x: arr('x'), z: arr('z'), angle: arr('angle'), q6: arr('q6'), start: arr('startConfig') };
    const nT = pv.cart.time.length;
    for (const k of ['x', 'z', 'angle', 'q6'])
      if (pv.cart[k].length !== nT) throw new ml.MLError(`cartesianPath.${k} muss genauso viele Werte haben wie cartesianPath.time (${nT}).`);
    if (pv.cart.start.length < 4) throw new ml.MLError('cartesianPath.startConfig muss mindestens 4 Elemente haben.');
    for (let k = 1; k < nT; k++) if (!(pv.cart.time[k] > pv.cart.time[k - 1])) throw new ml.MLError('cartesianPath.time muss streng monoton steigen.');
    pv.cartMaxIter = num2(L.evalExpr(P.cartMaxIter, 'CartesianPath/maxIter'), 'maxIter');
    pv.cartWeights = L.evalExpr(P.cartWeights, 'CartesianPath/weights');
    // PickAndPlace
    const wp = ml.toMat(L.evalExpr(P.pnpWaypoints, 'PickAndPlace/Waypoints'));
    const tp = ml.toArr(L.evalExpr(P.pnpTimePoints, 'PickAndPlace/TimePoints'));
    if (wp.r !== 5) throw new ml.MLError(`PickAndPlace: Waypoints müssen 5 Zeilen haben (ist ${wp.r}×${wp.c}). Tipp: waypointData' (transponiert).`);
    if (wp.c !== tp.length) throw new ml.MLError(`PickAndPlace: Anzahl der Waypoints (${wp.c}) und TimePoints (${tp.length}) muss übereinstimmen.`);
    for (let k = 1; k < tp.length; k++) if (!(tp[k] > tp[k - 1])) throw new ml.MLError('PickAndPlace: TimePoints müssen streng monoton steigen.');
    pv.pnp = {
      wp: Array.from({ length: wp.c }, (_, j) => Array.from(wp.d.subarray(j * 5, j * 5 + 5))),
      tp,
      gT: ml.toArr(L.evalExpr(P.pnpGripT, 'PickAndPlace/Lookup Table (Zeiten)')),
      gV: ml.toArr(L.evalExpr(P.pnpGripV, 'PickAndPlace/Lookup Table (Greifer)')),
      loopT: num2(L.evalExpr(P.pnpLoopT, 'PickAndPlace/Compare To Constant'), 'Wiederholzeit'),
    };
    if (pv.pnp.gT.length !== pv.pnp.gV.length) throw new ml.MLError('PickAndPlace: Greifer-Lookup-Tabelle – Zeiten und Werte müssen gleich lang sein.');
    pv.rate = num2(L.evalExpr(P.rateLimit, 'simpleDynamics/Rate Limiter'), 'Ratenbegrenzung');
    if (!(pv.rate > 0)) throw new ml.MLError('Ratenbegrenzung muss > 0 sein.');
    return pv;
  }

  function resetState() {
    const q0 = HOME_Q.map((v) => v * D2R);
    L.st = {
      t: 0,
      acc: 0,
      chart: 'Pause',
      prevMode: 0,
      pb: [0, 90, 0, 90, 0, 20],
      qLimMem: [...q0, 20 * D2R],
      qLim: [...q0, 20 * D2R],
      dynMem: q0.slice(),
      dynOut: q0.slice(),
      pnp: { x: 0, y: 0, out: null },
      cart: { x: 0, y: 0, out: null, converged: 1 },
      replay: { x: 0, y: 0, out: null },
      teach: { W: null, num: 0, prevAdd: 0, prevReset: 0 },
      holders: { fk: new Map(), store: new Map(), ik: new Map() },
      fk: { orient: [0, 0, 0], tcp: [0, 0, 0], B: [0, 0, 0], C: [0, 0, 0] },
      alarm: false,
      log: { t: [], q: [], tcp: [], mode: [], ref: [] },
    };
  }

  L.start = function () {
    const sim = BS.sim;
    if (L.running) return true;
    if (sim.owner === 'program') {
      BS.toast('Arduino-Programm läuft – bitte zuerst stoppen', 'warn');
      return false;
    }
    L.lastError = null;
    if (L.ensureScene()) BS.toast('Laborszene geladen', 'ok');
    // Geometrie des Simulators an die Übung anpassen
    const g = C.geom;
    if (Object.keys(LAB_GEOM).some((k) => Math.abs(g[k] - LAB_GEOM[k]) > 1e-9)) {
      Object.assign(g, LAB_GEOM);
      BS.store.set('geom', Object.assign(BS.store.get('geom', {}), LAB_GEOM));
      BS.emit('geom');
      BS.toast('Robotergeometrie auf die Werte der Übung gesetzt', 'ok');
    }
    try {
      for (const f of L.files) if (f.kind === 'block') L.parsed(f.name);
      if (L.params.autoIni && !ws().has('braccio')) {
        print('Workspace leer → Braccio_kinematics.m wird ausgeführt …\n', 'info');
        if (!L.runFile('Braccio_kinematics.m')) throw new ml.MLError('Ini-Skript Braccio_kinematics.m enthält Fehler.');
      }
      L.pv = evalParams();
    } catch (e) {
      L.fail(e, 'Modell kann nicht gestartet werden');
      return false;
    }
    resetState();
    L.trails = [];
    L.trailsDirty = true;
    if (sim.play) sim.stopMotion();
    sim.owner = 'lab';
    sim.directMask = [1, 1, 1, 1, 1, 0];
    L.running = true;
    L.writeServos();
    sim.act.splice(0, 5, ...sim.cmd.slice(0, 5));
    sim.updateKinematics();
    print(`Simulation gestartet (Braccio_CtrlSimMdl, dt = ${DT * 1000} ms).\n`, 'info');
    BS.emit('owner');
    BS.emit('lab2:state');
    return true;
  };

  L.stop = function (reason) {
    if (!L.running) return;
    L.running = false;
    const sim = BS.sim;
    if (sim.owner === 'lab') sim.owner = 'manual';
    sim.directMask = null;
    for (let i = 0; i < 6; i++) sim.cmd[i] = sim.act[i];
    L.writeOut();
    print(`Simulation beendet${reason ? ' (' + reason + ')' : ''} bei t = ${L.st.t.toFixed(2)} s. Ergebnisse in Variable 'out'.\n`, 'info');
    BS.emit('owner');
    BS.emit('lab2:state');
    BS.emit('lab2:workspace');
  };

  L.fail = function (e, title) {
    const msg = e instanceof ml.MLError ? e.message : String((e && e.message) || e);
    L.lastError = { title: title || 'Fehler', msg, file: e.file, line: e.line, block: e.block };
    print(`${title || 'Fehler'}: ${errText(e)}\n`, 'err');
    if (L.running) L.stop('Fehler');
    BS.emit('lab2:error', L.lastError);
    BS.toast(title || 'Fehler im Labormodell', 'err');
  };

  /** Aufruf eines MATLAB-Function-Blocks */
  function callBlock(name, args, nargout) {
    try {
      return L.interp.callFile(L.parsed(name), null, args, nargout, L.st.holders[name === 'ForwardKinematics' ? 'fk' : name === 'StoreWaypoints' ? 'store' : 'ik']);
    } catch (e) {
      const me = e instanceof ml.MLError ? e : new ml.MLError(String(e.message || e));
      me.block = L.file(name).block;
      me.file = name;
      me.message = `Fehler im Block '${me.block}': ${me.message}`;
      throw me;
    }
  }
  function vec(v, n, what) {
    const a = ml.toArr(v);
    if (a.length !== n) throw new ml.MLError(`${what} muss ${n} Elemente haben (hat ${a.length}).`);
    return a;
  }
  function interp1(xs, ys, x) {
    if (x <= xs[0]) return ys[0];
    const n = xs.length;
    if (x >= xs[n - 1]) return ys[n - 1];
    let k = 0;
    while (k < n - 2 && x > xs[k + 1]) k++;
    return ys[k] + ((x - xs[k]) / (xs[k + 1] - xs[k])) * (ys[k + 1] - ys[k]);
  }
  /** Polynomial Trajectory (Robotics System Toolbox): stückweise Polynome mit v = a = 0 an den Wegpunkten */
  function polyTraj(wp, tp, t, method) {
    const n = tp.length;
    if (t <= tp[0]) return wp[0].slice();
    if (t >= tp[n - 1]) return wp[n - 1].slice();
    let k = 0;
    while (k < n - 2 && t > tp[k + 1]) k++;
    const tau = (t - tp[k]) / (tp[k + 1] - tp[k]);
    const s = method === 'cubic' ? tau * tau * (3 - 2 * tau) : tau * tau * tau * (10 - 15 * tau + 6 * tau * tau);
    return wp[k].map((a, i) => a + s * (wp[k + 1][i] - a));
  }
  /** Zeitintegrator mit Pegel-Reset (Discrete-Time Integrator + Memory + Compare To Constant) */
  function timeStep(s, loop, tEnd) {
    const reset = loop && s.y >= tEnd;
    const y = reset ? 0 : s.x;
    s.x = y + DT;
    s.y = y;
    return y;
  }

  function modelStep() {
    const S = L.st,
      P = L.pv,
      H = L.hmi,
      pul = L.pulses;
    const mode = H.mode;
    // ---------------- Chart (Stateflow) ----------------
    const runPauseTrig = !!pul.runPause;
    const pauseTrig = mode !== S.prevMode;
    S.prevMode = mode;
    const xNotaus = pul.notaus || H.notausHeld ? 1 : 0;
    const usrBtn = pul.usr ? 1 : 0;
    if (S.chart === 'Pause') {
      if (xNotaus) S.chart = 'Notaus';
      else if (runPauseTrig) S.chart = 'Run';
    } else if (S.chart === 'Run') {
      if (xNotaus) S.chart = 'Notaus';
      else if (runPauseTrig || pauseTrig) S.chart = 'Pause';
    } else if (S.chart === 'Notaus') {
      if (!xNotaus && usrBtn) S.chart = 'Pause';
    }
    const run = S.chart === 'Run';
    const lim = P.lim;
    // ---------------- 1 Manual (Slider) ----------------
    const phi = H.phi;
    const qMan = [phi[0], phi[1] + 90, phi[2], phi[3] + 90, phi[4]].map((v, i) => clamp(v, lim[i][0] * R2D, lim[i][1] * R2D) * D2R).concat([phi[5] * D2R]);
    // ---------------- 2 PushBtnCtrl ----------------
    if (run) {
      const a = H.axis - 1;
      const u = (H.up ? 1 : 0) - (H.down ? 1 : 0);
      if (u) S.pb[a] = clamp(S.pb[a] + PB_RATE[a] * u * DT, PB_LIM[a][0], PB_LIM[a][1]);
    }
    const qPB = S.pb.map((v) => v * D2R);
    // ---------------- TeachIn / StoreWaypoints ----------------
    const add = pul.store ? 1 : 0,
      rst = pul.reset ? 1 : 0;
    const addTrg = add > S.teach.prevAdd ? 1 : 0,
      rstTrg = rst > S.teach.prevReset ? 1 : 0;
    S.teach.prevAdd = add;
    S.teach.prevReset = rst;
    const tr = callBlock('StoreWaypoints', [addTrg, rstTrg, ml.colVec(S.qLimMem)], 4);
    const W = ml.toMat(tr[0]);
    if (W.r !== 6) throw new ml.MLError(`StoreWaypoints: StoredWaypoints muss 6 Zeilen haben (hat ${W.r}).`);
    S.teach.W = W;
    S.teach.num = num2(tr[3], 'numWaypoints');
    // ---------------- 3 Replay ----------------
    if (run && mode === 3) {
      const n = Math.min(S.teach.num + 1, W.c);
      if (S.teach.num >= 1 && n >= 2) {
        const wp = Array.from({ length: n }, (_, j) => Array.from(W.d.subarray(j * 6, j * 6 + 6)));
        const tp = wp.map((_, j) => 2 * j);
        const t = timeStep(S.replay, true, tp[n - 1]);
        S.replay.out = polyTraj(wp, tp, t, 'quintic');
      } else S.replay.out = S.qLimMem.slice();
    }
    // ---------------- 4 PickAndPlace ----------------
    if (run && mode === 4) {
      const p = P.pnp;
      const t = timeStep(S.pnp, L.params.pnpLoop, p.loopT);
      const q5 = polyTraj(p.wp, p.tp, t, L.params.pnpMethod);
      S.pnp.out = q5.concat([interp1(p.gT, p.gV, t) * D2R]);
    }
    // ---------------- 5 CartesianPath ----------------
    if (run && mode === 5) {
      const c = P.cart;
      const t = timeStep(S.cart, L.params.cartLoop, c.time[c.time.length - 1]);
      const r = callBlock(
        'inverseKinematics3dof',
        [interp1(c.time, c.x, t), interp1(c.time, c.z, t), interp1(c.time, c.angle, t), P.cartMaxIter, P.cartWeights, ml.colVec([c.start[1], c.start[2], c.start[3]])],
        4
      );
      S.cart.out = [0, num2(r[0], 'q1'), num2(r[1], 'q2'), num2(r[2], 'q3'), 0, interp1(c.time, c.q6, t)];
      S.cart.converged = num2(r[3], 'converged');
    }
    // ---------------- Umschalter & JointLimits ----------------
    const zero = [0, 0, 0, 0, 0, 0];
    const q = mode === 1 ? qMan : mode === 2 ? qPB : mode === 3 ? S.replay.out || zero : mode === 4 ? S.pnp.out || zero : mode === 5 ? S.cart.out || zero : zero;
    const qLim = q.map((v, i) => clamp(Number.isFinite(v) ? v : 0, lim[i][0], lim[i][1]));
    S.qLim = qLim;
    S.qLimMem = qLim;
    // ---------------- simpleDynamics ----------------
    const inp = run ? qLim.slice(0, 5) : S.dynMem;
    S.dynMem = inp;
    const dmax = P.rate * DT;
    S.dynOut = S.dynOut.map((y, i) => y + clamp(inp[i] - y, -dmax, dmax));
    // ---------------- ForwardKinematics ----------------
    const fk = callBlock('ForwardKinematics', [ml.colVec(S.dynOut), P.braccio], 4);
    S.fk = {
      orient: vec(fk[0], 3, 'ForwardKinematics: TCPorient'),
      tcp: vec(fk[1], 3, 'ForwardKinematics: TCPvec'),
      B: vec(fk[2], 3, 'ForwardKinematics: Bvec'),
      C: vec(fk[3], 3, 'ForwardKinematics: Cvec'),
    };
    // ---------------- Arbeitsraumüberwachung ----------------
    S.alarm = [S.fk.tcp[0], S.fk.B[0], S.fk.C[0]].some((x) => x <= P.efenceX);
    // ---------------- Aufzeichnung (To Workspace) ----------------
    const lg = S.log;
    lg.t.push(S.t);
    lg.q.push(S.dynOut.slice());
    lg.tcp.push(S.fk.tcp.slice());
    lg.mode.push(mode);
    const ref = L.refFK(S.dynOut).tcp;
    lg.ref.push(ref);
    if (!L.fkOk && Math.hypot(S.fk.tcp[0] - ref[0], S.fk.tcp[1] - ref[1], S.fk.tcp[2] - ref[2]) < 1e-6 && Math.abs(S.dynOut[1] - Math.PI / 2) > 0.05) L.fkOk = true;
    if (lg.t.length > 60000) for (const k of Object.keys(lg)) lg[k].splice(0, 10000);
    S.t += DT;
    L.pulses = {};
  }

  L.writeServos = function () {
    const S = L.st,
      sim = BS.sim;
    const M = qToM(S.dynOut, S.qLim[5]);
    for (let i = 0; i < 5; i++) sim.cmd[i] = M[i];
    sim.cmd[5] = clamp(M[5], C.limits[5][0], C.limits[5][1]);
  };

  function tick(dt) {
    if (!L.running) return;
    const S = L.st;
    S.acc += dt;
    let n = 0;
    while (S.acc >= DT - 1e-9 && n++ < 50) {
      S.acc -= DT;
      try {
        modelStep();
      } catch (e) {
        L.fail(e, 'Simulation abgebrochen');
        return;
      }
    }
    L.writeServos();
  }
  BS.sim.preStep.push(tick);

  /** Knopf drücken (Flanke für einen Abtastschritt) */
  L.press = function (name) {
    L.pulses[name] = 1;
    if (!L.running && name !== 'usr') BS.toast('Simulation ist nicht gestartet (▶ Simulation starten)', 'warn');
  };
  L.setMode = function (m) {
    L.hmi.mode = m;
    BS.emit('lab2:state');
  };

  L.statusText = function () {
    if (!L.running) return 'gestoppt';
    return `${L.st.chart} · CtrlMode ${L.hmi.mode}`;
  };

  /** Ergebnisse als Variable out (wie „Single simulation output“ in Simulink) */
  L.writeOut = function () {
    const S = L.st;
    if (!S || !S.log.t.length) return;
    const lg = S.log;
    const N = lg.t.length;
    const T = ml.colVec(lg.t);
    const mk = (rows, n) => {
      const M = new ml.Mat(N, n);
      for (let i = 0; i < N; i++) for (let j = 0; j < n; j++) M.d[j * N + i] = rows[i][j];
      return ml.norm(M);
    };
    const ts = (data) => new ml.Struct(new Map([['Time', T], ['Data', data]]));
    const out = new ml.Struct(
      new Map([
        ['tout', T],
        ['ConfigSim', ts(mk(lg.q, 5))],
        ['TCPpos', ts(mk(lg.tcp, 3))],
        ['CtrlMode', ts(ml.colVec(lg.mode))],
      ])
    );
    ws().set('out', out);
  };

  // ------------------------------------------------------------------
  // Prüfungen (Feedback zu den Aufgaben)
  // ------------------------------------------------------------------
  L.checkDH = function () {
    const b = ws().get('braccio');
    if (!(b instanceof ml.Struct)) return { ok: false, msgs: ["Variable 'braccio' fehlt – Braccio_kinematics.m ausführen."] };
    const msgs = [];
    let ok = true;
    for (const [k, ref] of Object.entries(DH_REF)) {
      if (!b.f.has(k)) {
        ok = false;
        msgs.push(`braccio.${k} fehlt.`);
        continue;
      }
      let v;
      try {
        v = ml.scalar(b.f.get(k));
      } catch (e) {
        ok = false;
        msgs.push(`braccio.${k} ist kein Skalar.`);
        continue;
      }
      if (Math.abs(v - ref) > 1e-6) {
        ok = false;
        let hint = '';
        if (Math.abs(v / 1000 - ref) < 1e-6) hint = ' – Einheit? Längen in Metern angeben.';
        else if (Math.abs(v * D2R - ref) < 1e-6) hint = ' – Einheit? Winkel in Radiant angeben (z. B. pi/2).';
        msgs.push(`braccio.${k} = ${ml.fmtNum(v)} stimmt nicht${hint}`);
      }
    }
    if (ok) msgs.push('Alle DH-Parameter stimmen mit der Tabelle überein.');
    return { ok, msgs };
  };
  L.checkFK = function () {
    const b = ws().get('braccio');
    if (!(b instanceof ml.Struct)) return { ok: false, msgs: ["Variable 'braccio' fehlt – Braccio_kinematics.m ausführen."] };
    let file;
    try {
      file = L.parsed('ForwardKinematics');
    } catch (e) {
      return { ok: false, msgs: ['Syntaxfehler: ' + errText(e)] };
    }
    const P = {};
    for (const k of Object.keys(DH_REF)) P[k] = b.f.has(k) ? ml.scalar(b.f.get(k)) : 0;
    const tests = [[0, 90, 0, 90, 0]];
    for (let k = 0; k < 12; k++) tests.push(LIM_DEF.slice(0, 5).map(([lo, hi]) => lo + Math.random() * (hi - lo)));
    let eT = 0,
      eB = 0,
      eC = 0,
      eR = 0,
      eRefT = 0,
      worst = null;
    for (const qd of tests) {
      const q = qd.map((v) => v * D2R);
      let r;
      try {
        r = L.interp.callFile(file, null, [ml.colVec(q), b], 4, new Map());
      } catch (e) {
        return { ok: false, msgs: ['Laufzeitfehler: ' + errText(e)] };
      }
      const ref = L.refFK(q, P);
      const tcp = ml.toArr(r[1]),
        Bv = ml.toArr(r[2]),
        Cv = ml.toArr(r[3]),
        eul = ml.toArr(r[0]);
      if (tcp.length !== 3 || Bv.length !== 3 || Cv.length !== 3 || eul.length !== 3) return { ok: false, msgs: ['Ausgänge müssen je 3 Elemente haben.'] };
      const d = (a, c) => Math.hypot(a[0] - c[0], a[1] - c[1], a[2] - c[2]);
      const dT = d(tcp, ref.tcp);
      if (dT > eT) (eT = dT), (worst = qd);
      eB = Math.max(eB, d(Bv, ref.B));
      eC = Math.max(eC, d(Cv, ref.C));
      eRefT = Math.max(eRefT, d(ref.tcp, L.refFK(q).tcp));
      // Orientierung: Rotationsmatrix aus XYZ-Euler-Winkeln mit T15 vergleichen
      const R = ml.toMat(ml.builtins.eul2rotm([ml.rowVec(eul), new ml.Str('XYZ')]));
      const T = ref.T15;
      let e = 0;
      for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) e = Math.max(e, Math.abs(R.d[j * 3 + i] - T[i * 4 + j]));
      eR = Math.max(eR, e);
    }
    const mm = (x) => (x * 1000).toFixed(2) + ' mm';
    const msgs = [];
    const ok = eT < 1e-6 && eB < 1e-6 && eC < 1e-6 && eR < 1e-6;
    msgs.push(`${eT < 1e-6 ? '✓' : '✗'} TCPvec: max. Abweichung ${mm(eT)}${eT >= 1e-6 && worst ? ' (z. B. bei q = [' + worst.map((v) => v.toFixed(0)).join(' ') + ']°)' : ''}`);
    msgs.push(`${eB < 1e-6 ? '✓' : '✗'} Bvec (Ellbogen): max. Abweichung ${mm(eB)}`);
    msgs.push(`${eC < 1e-6 ? '✓' : '✗'} Cvec (Handgelenk): max. Abweichung ${mm(eC)}`);
    msgs.push(`${eR < 1e-6 ? '✓' : '✗'} TCPorient: max. Abweichung der Rotationsmatrix ${eR.toFixed(4)}`);
    if (eRefT > 1e-6) msgs.push('Hinweis: Die DH-Parameter in braccio weichen von der Tabelle ab – verglichen wurde mit den Parametern aus dem Workspace.');
    return { ok, msgs };
  };

  /** Fortschritt der Teilaufgaben (automatisch erkannt) */
  L.progress = function () {
    const p = {};
    p.a = L.checkDH().ok;
    p.b = !!L.skeleton;
    p.c = Math.abs(efenceX() + 0.2) < 1e-9 && ws().has('efence1');
    p.d = ws().has(L.ikd.exportName) && L.ikd.configs.length > 0;
    p.e = !!L.everRun;
    p.f = !!L.fkOk;
    p.g = /waypointData/.test(L.params.pnpWaypoints) && !!L.ranMode[4];
    p.h = !!L.ranMode[3] && !!(L.st && L.st.teach.num > 0);
    p.i = !!L.ranMode[5];
    return p;
  };
  L.ranMode = {};
  BS.on('lab2:state', () => {
    if (L.running) L.everRun = true;
  });
  setInterval(() => {
    if (L.running && L.st && L.st.chart === 'Run') L.ranMode[L.hmi.mode] = true;
  }, 500);

  // ------------------------------------------------------------------
  // IK-Designer (Ersatz für inverseKinematicsDesigner)
  // ------------------------------------------------------------------
  const ikdSaved = BS.store.get('lab2ikd', {});
  const ikd = (L.ikd = {
    q: (ikdSaved.q || HOME_Q).map((v) => v * D2R),
    configs: (ikdSaved.configs || []).map((c) => ({ q: c.map((v) => v * D2R), status: 'NOT RUN', msg: '' })),
    idx: -1,
    scene: new Set(ikdSaved.scene || []),
    exportName: ikdSaved.exportName || 'waypointData',
    sel: 2,
  });
  ikd.save = () =>
    BS.store.set('lab2ikd', {
      q: ikd.q.map((v) => v * R2D),
      configs: ikd.configs.map((c) => c.q.map((v) => v * R2D)),
      scene: [...ikd.scene],
      exportName: ikd.exportName,
    });
  ikd.limits = function () {
    const out = [];
    for (let i = 1; i <= 5; i++) {
      let l = LIM_DEF[i - 1].map((v) => v * D2R);
      const j = ws().get('jnt' + i);
      try {
        if (j instanceof ml.Obj) l = ml.toArr(j.get('PositionLimits'));
      } catch (e) {
        /* Standardgrenzen */
      }
      out.push(l);
    }
    return out;
  };
  /** collisionBox-Objekte im Workspace */
  ikd.collisionObjects = function () {
    const out = [];
    for (const [name, v] of ws()) if (v instanceof ml.Obj && v.cls === 'collisionBox') out.push({ name, obj: v });
    return out;
  };
  ikd.set = function (i, v) {
    ikd.q[i] = v;
    ikd.save();
    BS.emit('lab2:ikd');
  };
  ikd.store = function () {
    ikd.configs.push({ q: ikd.q.slice(), status: 'NOT RUN', msg: '' });
    ikd.idx = ikd.configs.length - 1;
    ikd.save();
    BS.emit('lab2:ikd');
  };
  ikd.go = function (k) {
    if (k < 0 || k >= ikd.configs.length) return;
    ikd.idx = k;
    ikd.q = ikd.configs[k].q.slice();
    ikd.save();
    BS.emit('lab2:ikd');
  };
  ikd.update = function (k) {
    if (k < 0 || k >= ikd.configs.length) return;
    ikd.configs[k] = { q: ikd.q.slice(), status: 'NOT RUN', msg: '' };
    ikd.save();
    BS.emit('lab2:ikd');
  };
  ikd.remove = function (k) {
    ikd.configs.splice(k, 1);
    ikd.idx = Math.min(ikd.idx, ikd.configs.length - 1);
    ikd.save();
    BS.emit('lab2:ikd');
  };
  ikd.move = function (k, d) {
    const j = k + d;
    if (j < 0 || j >= ikd.configs.length) return;
    [ikd.configs[k], ikd.configs[j]] = [ikd.configs[j], ikd.configs[k]];
    ikd.idx = j;
    ikd.save();
    BS.emit('lab2:ikd');
  };
  ikd.snapToRobot = function () {
    ikd.q = L.running ? L.st.dynOut.slice() : mToQ(BS.sim.cmd);
    ikd.save();
    BS.emit('lab2:ikd');
  };
  /** Kollisionsprüfung einer Konfiguration gegen die gewählten Szenenobjekte */
  ikd.collide = function (q) {
    const lim = ikd.limits();
    for (let i = 0; i < 5; i++) if (q[i] < lim[i][0] - 1e-9 || q[i] > lim[i][1] + 1e-9) return { status: 'Gelenkgrenze', msg: `q${i + 1} = ${(q[i] * R2D).toFixed(1)}° außerhalb [${(lim[i][0] * R2D).toFixed(0)}, ${(lim[i][1] * R2D).toFixed(0)}]°` };
    const F = BS.kin.frames(qToM(q, 10 * D2R));
    const pts = BS.kin.armPoints(F, C.geom, 0).pts;
    const objs = ikd.collisionObjects().filter((o) => ikd.scene.has(o.name));
    for (const o of objs) {
      const X = ml.scalar(o.obj.get('X')),
        Y = ml.scalar(o.obj.get('Y')),
        Z = ml.scalar(o.obj.get('Z'));
      const P = ml.toMat(o.obj.get('Pose'));
      const R = (i, j) => P.d[j * 4 + i];
      const t = [P.d[12], P.d[13], P.d[14]];
      for (const pt of pts) {
        const p = toLab(pt.p).map((v, i) => v - t[i]);
        const l = [0, 1, 2].map((k) => R(0, k) * p[0] + R(1, k) * p[1] + R(2, k) * p[2]);
        if (Math.abs(l[0]) <= X / 2 && Math.abs(l[1]) <= Y / 2 && Math.abs(l[2]) <= Z / 2) return { status: 'KOLLISION', msg: `${pt.link} ↔ ${o.name}` };
      }
    }
    return { status: 'kollisionsfrei', msg: '' };
  };
  ikd.checkAll = function () {
    for (const c of ikd.configs) {
      const r = ikd.collide(c.q);
      c.status = r.status;
      c.msg = r.msg;
    }
    BS.emit('lab2:ikd');
  };
  ikd.export = function () {
    const name = ikd.exportName.trim();
    if (!/^[A-Za-z]\w*$/.test(name)) return BS.toast('Ungültiger Variablenname', 'err');
    if (!ikd.configs.length) return BS.toast('Keine Konfigurationen gespeichert', 'warn');
    const n = ikd.configs.length;
    const M = new ml.Mat(n, 5);
    ikd.configs.forEach((c, i) => c.q.forEach((v, j) => (M.d[j * n + i] = v)));
    ws().set(name, ml.norm(M));
    print(`IK-Designer: ${n} Konfigurationen als '${name}' (${n}×5, rad) in den Workspace exportiert.\n`, 'info');
    print(ml.display(name, M));
    ikd.save();
    BS.emit('lab2:workspace');
    BS.toast(`'${name}' exportiert`, 'ok');
  };
  /** Marker-Pose: TCP-Ziel (Labor-KS, m) und Werkzeugwinkel ψ [°] → Gelenkwinkel (geometrische IK des Simulators).
   *  Ist ψ nicht erreichbar, wird der nächstgelegene zulässige Werkzeugwinkel gesucht. Liefert {err} oder {psi}. */
  ikd.solve = function (x, y, z, psi) {
    const t = toSim([x, y, z]);
    const cur = qToM(ikd.q, 0);
    const lim = ikd.limits();
    const okQ = (m) => mToQ(m).every((v, i) => v >= lim[i][0] - 1e-9 && v <= lim[i][1] + 1e-9);
    const dist = (m) => m.slice(0, 4).reduce((s, v, i) => s + Math.abs(v - cur[i]), 0);
    let best = null,
      reach = false;
    for (let d = 0; d <= 180 && !best; d += 1) {
      for (const p of d ? [psi + d, psi - d] : [psi]) {
        if (p < -180 || p > 180) continue;
        const sols = BS.kin.ik({ x: t[0], y: t[1], z: t[2] }, p, C.geom, {});
        for (const s of sols) {
          if (s.reachable) reach = true;
          if (!s.valid || !okQ([...s.m.slice(0, 4), cur[4]])) continue;
          if (!best || dist(s.m) < dist(best.m)) best = { m: s.m, psi: p };
        }
      }
    }
    if (!best) return { err: reach ? 'Ziel nur mit Verletzung der Gelenkgrenzen erreichbar' : 'Ziel außerhalb des Arbeitsraums' };
    const m = best.m.slice();
    m[4] = cur[4];
    ikd.q = mToQ(m);
    ikd.save();
    BS.emit('lab2:ikd');
    return { psi: best.psi };
  };

  // ------------------------------------------------------------------
  // 3D: Laborszene, Marker, DH-Skelett
  // ------------------------------------------------------------------
  let G = null; // THREE-Gruppen
  function build3d() {
    const V = BS.view;
    if (!V.world || G) return;
    const g = new THREE.Group();
    g.visible = false;
    V.world.add(g);
    G = { root: g, boxes: {}, markers: {}, skel: null, trails: new THREE.Group() };
    // Boden unter der Plattform
    const floor = V.mesh(new THREE.PlaneGeometry(1600, 1200), V.std(0x7b838e, { roughness: 0.95 }), g, [0, 100, -20], null, false);
    floor.receiveShadow = true;
    const grid = new THREE.GridHelper(1600, 32, 0x6d747e, 0x8a919b);
    grid.rotation.x = Math.PI / 2;
    grid.position.set(0, 100, -19.8);
    grid.material.transparent = true;
    grid.material.opacity = 0.5;
    g.add(grid);
    for (const [name, b] of Object.entries(SCENE)) {
      const sb = simBox(b);
      const mat = V.std(b.color, { roughness: name === 'platform' ? 0.8 : 0.6 });
      const m = V.mesh(new THREE.BoxGeometry(sb.s[0], sb.s[1], sb.s[2]), mat, g, sb.c, null, true);
      m.castShadow = name !== 'platform';
      m.receiveShadow = true;
      G.boxes[name] = m;
    }
    // Raster auf der Plattform (Laborkoordinaten)
    const pg = new THREE.GridHelper(600, 12, 0x8a4a22, 0x9d5629);
    pg.scale.set(0.5, 1, 1);
    pg.rotation.x = Math.PI / 2;
    pg.position.set(0, 100, 0.3);
    g.add(pg);
    // virtuelle Wand
    const ef = new THREE.Mesh(new THREE.BoxGeometry(300, 2, 300), new THREE.MeshStandardMaterial({ color: 0x666666, transparent: true, opacity: 0.35, depthWrite: false }));
    const efEdge = new THREE.LineSegments(new THREE.EdgesGeometry(ef.geometry), new THREE.LineBasicMaterial({ color: 0x444444 }));
    ef.add(efEdge);
    g.add(ef);
    G.efence = ef;
    G.efEdge = efEdge;
    const efl = V.label('virtuelle Wand (efence1)', '#ffffff', 12);
    efl.position.set(0, 0, 165);
    ef.add(efl);
    // Labor-KS0
    const ax = new THREE.Group();
    const arr = (dir, col, txt) => {
      ax.add(new THREE.ArrowHelper(new THREE.Vector3(...dir), new THREE.Vector3(0, 0, 0.5), 90, col, 12, 6));
      const l = V.label(txt, '#' + col.toString(16).padStart(6, '0'), 13);
      l.position.set(dir[0] * 104, dir[1] * 104, dir[2] * 104 + 3);
      ax.add(l);
    };
    arr([0, 1, 0], 0xe03131, 'x₀');
    arr([-1, 0, 0], 0x2f9e44, 'y₀');
    arr([0, 0, 1], 0x1c7ed6, 'z₀');
    ax.position.set(-130, -170, 0.6);
    g.add(ax);
    // Marker aus ForwardKinematics (visu_TCP, visu_B, visu_C)
    const sph = (col, r) => {
      const m = new THREE.Mesh(new THREE.SphereGeometry(r, 18, 12), new THREE.MeshBasicMaterial({ color: col, depthTest: false, transparent: true, opacity: 0.9 }));
      m.renderOrder = 7;
      g.add(m);
      return m;
    };
    G.markers.tcp = sph(0xff00ff, 6);
    G.markers.B = sph(0x1c7ed6, 6);
    G.markers.C = sph(0x2fb344, 6);
    for (const [k, txt] of [
      ['tcp', 'TCP'],
      ['B', 'B'],
      ['C', 'C'],
    ]) {
      const l = V.label(txt, '#ffffff', 12);
      l.position.set(0, 0, 14);
      G.markers[k].add(l);
    }
    const tri = new THREE.Group();
    for (const [d, c] of [
      [[1, 0, 0], 0xff3030],
      [[0, 1, 0], 0x30d030],
      [[0, 0, 1], 0x3080ff],
    ])
      tri.add(new THREE.ArrowHelper(new THREE.Vector3(...d), new THREE.Vector3(), 40, c, 8, 4));
    G.markers.tcp.add(tri);
    G.tri = tri;
    G.trails = new THREE.Group();
    g.add(G.trails);
  }
  function setBox(m, b) {
    const sb = simBox(b);
    m.position.set(...sb.c);
  }

  /** show(braccioRobot, config): DH-Skelett im 3D-Fenster */
  L.showTree = function (data) {
    L.skeleton = data;
    L.skelDirty = true;
    if (data.cfg && data.cfg.length === 5 && !L.running) {
      // Pose zusätzlich als Geisterbild des Roboters zeigen
      L.ghostQ = data.cfg.slice();
    }
    BS.emit('lab2:skeleton');
  };
  L.hideSkeleton = function () {
    L.skeleton = null;
    L.ghostQ = null;
    L.skelDirty = true;
    BS.emit('lab2:skeleton');
  };
  function rebuildSkeleton() {
    if (G.skel) G.root.remove(G.skel);
    G.skel = null;
    if (!L.skeleton) return;
    const grp = new THREE.Group();
    const pts = [];
    const pos = (T) => toSim([T[0][3], T[1][3], T[2][3]]);
    const tubeMat = new THREE.MeshBasicMaterial({ color: 0xffd43b, depthTest: false, transparent: true, opacity: 0.85 });
    const fr = L.skeleton.frames;
    const all = new Map([['base', [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]]]);
    for (const f of fr) all.set(f.name, f.T);
    for (const f of fr) {
      const a = pos(all.get(f.parent)),
        b = pos(f.T);
      const va = new THREE.Vector3(...a),
        vb = new THREE.Vector3(...b);
      const len = va.distanceTo(vb);
      if (len > 0.5) {
        const tube = new THREE.Mesh(new THREE.CylinderGeometry(3, 3, len, 10), tubeMat);
        tube.position.copy(va).add(vb).multiplyScalar(0.5);
        tube.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), vb.clone().sub(va).normalize());
        tube.renderOrder = 8;
        grp.add(tube);
      }
      pts.push(b);
      // Koordinatensystem des Körpers (Achsen im Labor-KS → Simulator-KS)
      const T = f.T;
      for (const [k, col] of [
        [0, 0xff4040],
        [1, 0x40e040],
        [2, 0x4090ff],
      ]) {
        const dir = new THREE.Vector3(-T[1][k], T[0][k], T[2][k]);
        const ah = new THREE.ArrowHelper(dir.normalize(), new THREE.Vector3(...b), 34, col, 7, 4);
        ah.line.material.depthTest = false;
        ah.cone.material.depthTest = false;
        ah.renderOrder = 9;
        grp.add(ah);
      }
      const l = BS.view.label(f.name, '#ffe066', 11);
      l.position.set(b[0], b[1], b[2] + 10);
      grp.add(l);
    }
    for (const p of pts) {
      const s = new THREE.Mesh(new THREE.SphereGeometry(4, 12, 8), new THREE.MeshBasicMaterial({ color: 0xffe066, depthTest: false }));
      s.position.set(...p);
      s.renderOrder = 9;
      grp.add(s);
    }
    G.skel = grp;
    G.root.add(grp);
  }
  function rebuildTrails() {
    G.trails.clear();
    for (const h of L.trails) {
      if (!h._pts.length) continue;
      const geo = new THREE.BufferGeometry().setFromPoints(h._pts.map((p) => new THREE.Vector3(...toSim(p))));
      G.trails.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color: h._color })));
    }
  }

  BS.view.onSync.push(function () {
    const V = BS.view;
    if (!G) build3d();
    if (!G) return;
    const on = BS.sim.scenario === 'labor2';
    G.root.visible = on;
    if (V.table) V.table.visible = !on;
    if (V.grid) V.grid.visible = !on;
    if (!on) return;
    G.boxes.regal.visible = L.hmi.show.regal;
    // virtuelle Wand
    const ex = L.running ? L.pv.efenceX : efenceX();
    G.efence.position.set(0, ex * 1000, 150);
    G.efence.visible = L.hmi.show.efence;
    const alarm = L.running && L.st.alarm;
    G.efence.material.color.setHex(alarm ? 0xff0000 : 0x666666);
    G.efence.material.opacity = alarm ? 0.55 : 0.35;
    // Marker aus der (eigenen) Vorwärtskinematik
    const showM = L.running || (L.st && L.st.log.t.length);
    for (const k of ['tcp', 'B', 'C']) G.markers[k].visible = !!showM;
    if (showM) {
      const fk = L.st.fk;
      G.markers.tcp.position.set(...toSim(fk.tcp));
      G.markers.B.position.set(...toSim(fk.B));
      G.markers.C.position.set(...toSim(fk.C));
      const R = ml.toMat(ml.builtins.eul2rotm([ml.rowVec(fk.orient), new ml.Str('XYZ')]));
      // Achsen im Labor-KS → Simulator-KS (Drehung um z um +90°)
      const m4 = new THREE.Matrix4().set(-R.d[1], -R.d[4], -R.d[7], 0, R.d[0], R.d[3], R.d[6], 0, R.d[2], R.d[5], R.d[8], 0, 0, 0, 0, 1);
      G.tri.setRotationFromMatrix(m4);
    }
    if (L.skelDirty) {
      L.skelDirty = false;
      rebuildSkeleton();
    }
    if (L.trailsDirty) {
      L.trailsDirty = false;
      rebuildTrails();
    }
  });

  // Szenario gewechselt → Labormodell stoppen
  BS.on('scenario', () => {
    if (BS.sim.scenario !== 'labor2' && L.running) L.stop('anderes Szenario geladen');
  });
  BS.on('reset', () => {
    if (L.running) L.stop('Simulator zurückgesetzt');
  });
})();
