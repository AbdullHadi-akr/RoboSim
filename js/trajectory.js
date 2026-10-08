/* Braccio-Simulator – Trajektorienplanung (Skript Kap. 4)
 *  - Profile: Trapez (C0-stetige Geschwindigkeit), Polynom 3. Ordnung (C1), Polynom 5. Ordnung (C2), linear
 *  - PTP im Gelenkraum mit Halt an Wegpunkten, oder ohne Halt (Polynome 5. Ordnung mit Zwischengeschwindigkeiten)
 *  - LIN: geradlinige Bewegung des TCP (kartesisch) mit IK in jedem Abtastschritt
 */
(function () {
  'use strict';
  const BS = (typeof window !== 'undefined' ? window : globalThis).BS;
  const traj = (BS.traj = {});

  /** Normiertes Profil s(τ), τ = t/T ∈ [0,1] → {s, ds/dt, d²s/dt²} */
  traj.profile = function (type, tau, T, accFrac) {
    tau = BS.clamp(tau, 0, 1);
    switch (type) {
      case 'linear':
        return { s: tau, sd: 1 / T, sdd: 0 };
      case 'cubic':
        return { s: 3 * tau * tau - 2 * tau ** 3, sd: (6 * tau - 6 * tau * tau) / T, sdd: (6 - 12 * tau) / (T * T) };
      case 'quintic':
        return {
          s: 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5,
          sd: (30 * tau ** 2 - 60 * tau ** 3 + 30 * tau ** 4) / T,
          sdd: (60 * tau - 180 * tau ** 2 + 120 * tau ** 3) / (T * T),
        };
      case 'trapez':
      default: {
        const f = BS.clamp(accFrac || 0.25, 0.01, 0.5);
        const t = tau * T,
          ta = f * T;
        const v = 1 / (T - ta); // Maximalgeschwindigkeit (s läuft von 0 bis 1)
        const a = v / ta;
        if (t < ta) return { s: 0.5 * a * t * t, sd: a * t, sdd: a };
        if (t <= T - ta) return { s: 0.5 * a * ta * ta + v * (t - ta), sd: v, sdd: 0 };
        const tr = T - t;
        return { s: 1 - 0.5 * a * tr * tr, sd: a * tr, sdd: -a };
      }
    }
  };

  /** Polynom 5. Ordnung zwischen zwei Stützstellen mit (q, q̇, q̈) */
  function quinticCoeffs(q0, v0, a0, q1, v1, a1, T) {
    const T2 = T * T,
      T3 = T2 * T,
      T4 = T3 * T,
      T5 = T4 * T;
    const c0 = q0,
      c1 = v0,
      c2 = a0 / 2;
    const h = q1 - q0 - v0 * T - (a0 / 2) * T2;
    const hv = v1 - v0 - a0 * T;
    const ha = a1 - a0;
    const c3 = (10 * h - 4 * hv * T + 0.5 * ha * T2) / T3;
    const c4 = (-15 * h + 7 * hv * T - ha * T2) / T4;
    const c5 = (6 * h - 3 * hv * T + 0.5 * ha * T2) / T5;
    return [c0, c1, c2, c3, c4, c5];
  }
  function evalQuintic(c, t) {
    return {
      q: c[0] + c[1] * t + c[2] * t * t + c[3] * t ** 3 + c[4] * t ** 4 + c[5] * t ** 5,
      qd: c[1] + 2 * c[2] * t + 3 * c[3] * t * t + 4 * c[4] * t ** 3 + 5 * c[5] * t ** 4,
      qdd: 2 * c[2] + 6 * c[3] * t + 12 * c[4] * t * t + 20 * c[5] * t ** 3,
    };
  }

  /**
   * Plant eine Trajektorie.
   * wps: [{m:[6], dur: Sekunden vom vorherigen Wegpunkt, dwell: Verweilzeit}]
   * opts: {interp:'ptp'|'lin', profile, accFrac, stop:true|false, dt}
   */
  traj.plan = function (wps, opts) {
    opts = opts || {};
    const g = opts.g || BS.config.geom;
    const profile = opts.profile || 'quintic';
    const n = wps.length;
    const errors = [];
    if (n < 2) return { ok: false, errors: ['Mindestens 2 Wegpunkte erforderlich'], duration: 0 };
    // Zeitachse
    const segs = [];
    let t = 0;
    for (let i = 1; i < n; i++) {
      const T = Math.max(0.05, +wps[i].dur || 1);
      const dwell = Math.max(0, +wps[i - 1].dwell || 0);
      if (dwell > 0) {
        segs.push({ kind: 'hold', i: i - 1, t0: t, T: dwell });
        t += dwell;
      }
      segs.push({ kind: 'move', i0: i - 1, i1: i, t0: t, T });
      t += T;
    }
    if (+wps[n - 1].dwell > 0) {
      segs.push({ kind: 'hold', i: n - 1, t0: t, T: +wps[n - 1].dwell });
      t += +wps[n - 1].dwell;
    }
    const duration = t;

    // Zwischengeschwindigkeiten für "ohne Halt" (Heuristik: Mittelwert der Steigungen, 0 bei Vorzeichenwechsel)
    let viaVel = null;
    const continuous = opts.interp !== 'lin' && opts.stop === false;
    if (continuous) {
      const moves = segs.filter((s) => s.kind === 'move');
      viaVel = wps.map(() => [0, 0, 0, 0, 0, 0]);
      for (let i = 1; i < n - 1; i++) {
        const sa = moves[i - 1],
          sb = moves[i];
        // Halt, falls am Wegpunkt verweilt wird
        if (+wps[i].dwell > 0) continue;
        for (let j = 0; j < 6; j++) {
          const k1 = (wps[i].m[j] - wps[i - 1].m[j]) / sa.T;
          const k2 = (wps[i + 1].m[j] - wps[i].m[j]) / sb.T;
          viaVel[i][j] = Math.sign(k1) === Math.sign(k2) && k1 !== 0 ? (k1 + k2) / 2 : 0;
        }
      }
      for (const s of moves) {
        s.coef = [];
        for (let j = 0; j < 6; j++)
          s.coef.push(quinticCoeffs(wps[s.i0].m[j], viaVel[s.i0][j], 0, wps[s.i1].m[j], viaVel[s.i1][j], 0, s.T));
      }
    }

    // LIN: kartesische Stützposen
    let poses = null;
    if (opts.interp === 'lin') {
      poses = wps.map((w) => BS.kin.tcp(w.m, g));
    }

    let lastLin = null;
    function sample(tq) {
      tq = BS.clamp(tq, 0, duration);
      let s = segs[segs.length - 1];
      for (const sg of segs)
        if (tq <= sg.t0 + sg.T + 1e-12) {
          s = sg;
          break;
        }
      const out = { q: [0, 0, 0, 0, 0, 0], qd: [0, 0, 0, 0, 0, 0], qdd: [0, 0, 0, 0, 0, 0], ok: true, seg: s };
      if (s.kind === 'hold') {
        out.q = wps[s.i].m.slice();
        return out;
      }
      const lt = tq - s.t0;
      const A = wps[s.i0].m,
        B = wps[s.i1].m;
      if (continuous) {
        for (let j = 0; j < 6; j++) {
          const e = evalQuintic(s.coef[j], lt);
          out.q[j] = e.q;
          out.qd[j] = e.qd;
          out.qdd[j] = e.qdd;
        }
        return out;
      }
      const p = traj.profile(profile, lt / s.T, s.T, opts.accFrac);
      if (opts.interp === 'lin') {
        const P = poses[s.i0],
          Q = poses[s.i1];
        const tgt = { x: P.x + (Q.x - P.x) * p.s, y: P.y + (Q.y - P.y) * p.s, z: P.z + (Q.z - P.z) * p.s };
        const psi = P.psi + BS.wrap180(Q.psi - P.psi) * p.s;
        const ref = lastLin && lastLin.seg === s ? lastLin.q : A;
        // nur stetig anschließende Lösungen (kein Umklappen vorne/hinten an Gelenkgrenzen)
        const sol = BS.kin.ikStep(tgt, psi, g, ref);
        if (sol) {
          for (let j = 0; j < 4; j++) out.q[j] = sol.m[j];
        } else {
          out.ok = false;
          for (let j = 0; j < 4; j++) out.q[j] = A[j] + (B[j] - A[j]) * p.s;
        }
        for (let j = 4; j < 6; j++) {
          out.q[j] = A[j] + (B[j] - A[j]) * p.s;
          out.qd[j] = (B[j] - A[j]) * p.sd;
          out.qdd[j] = (B[j] - A[j]) * p.sdd;
        }
        out.cart = tgt;
        if (sol) lastLin = { seg: s, q: out.q.slice() };
        return out;
      }
      for (let j = 0; j < 6; j++) {
        out.q[j] = A[j] + (B[j] - A[j]) * p.s;
        out.qd[j] = (B[j] - A[j]) * p.sd;
        out.qdd[j] = (B[j] - A[j]) * p.sdd;
      }
      return out;
    }

    // Abtastung für Plots / Prüfung
    const dt = opts.dt || 0.01;
    const N = Math.min(20000, Math.ceil(duration / dt) + 1);
    const T = new Float64Array(N);
    const Q = [],
      QD = [],
      QDD = [];
    for (let j = 0; j < 6; j++) {
      Q.push(new Float64Array(N));
      QD.push(new Float64Array(N));
      QDD.push(new Float64Array(N));
    }
    const xyz = [];
    let linFail = 0;
    lastLin = null;
    for (let k = 0; k < N; k++) {
      const tk = Math.min(duration, k * dt);
      const sm = sample(tk);
      if (!sm.ok) linFail++;
      T[k] = tk;
      for (let j = 0; j < 6; j++) Q[j][k] = sm.q[j];
      const p = BS.kin.tcp(sm.q, g);
      xyz.push([p.x, p.y, p.z]);
    }
    // Für LIN (und als Kontrolle) numerische Ableitungen
    for (let j = 0; j < 6; j++) {
      for (let k = 0; k < N; k++) {
        const a = Math.max(0, k - 1),
          b = Math.min(N - 1, k + 1);
        QD[j][k] = b > a ? (Q[j][b] - Q[j][a]) / (T[b] - T[a]) : 0;
      }
      for (let k = 0; k < N; k++) {
        const a = Math.max(0, k - 1),
          b = Math.min(N - 1, k + 1);
        QDD[j][k] = b > a ? (QD[j][b] - QD[j][a]) / (T[b] - T[a]) : 0;
      }
    }
    if (linFail) errors.push(`LIN: ${linFail} Abtastpunkte nicht stetig erreichbar (Arbeitsraum/Gelenkgrenzen/Konfigurationswechsel) – dort wurde im Gelenkraum interpoliert.`);
    // Gelenkgrenzen & Geschwindigkeiten
    const vmax = BS.config.servo.vmax;
    const peaks = [];
    for (let j = 0; j < 6; j++) {
      let mv = 0,
        ma = 0,
        lo = Infinity,
        hi = -Infinity;
      for (let k = 0; k < N; k++) {
        mv = Math.max(mv, Math.abs(QD[j][k]));
        ma = Math.max(ma, Math.abs(QDD[j][k]));
        lo = Math.min(lo, Q[j][k]);
        hi = Math.max(hi, Q[j][k]);
      }
      peaks.push({ v: mv, a: ma, lo, hi });
      const L = BS.config.limits[j];
      if (lo < L[0] - 0.5 || hi > L[1] + 0.5) errors.push(`${BS.config.short[j]} verlässt den zulässigen Bereich [${L[0]}, ${L[1]}] (${lo.toFixed(1)} … ${hi.toFixed(1)}°).`);
      if (mv > vmax[j]) errors.push(`${BS.config.short[j]}: max. ${mv.toFixed(0)}°/s > Servo-Grenze ${vmax[j]}°/s – der Servo kann nicht folgen.`);
    }
    return { ok: true, duration, sample, segs, T, Q, QD, QDD, xyz, peaks, errors, wps: wps.map((w) => ({ m: w.m.slice(), dur: w.dur, dwell: w.dwell })), opts };
  };

  /** Einfache PTP-Bewegung von a nach b (Polynom 5. Ordnung), Dauer aus Abstand */
  traj.ptp = function (a, b, speed) {
    speed = speed || 60; // °/s mittlere Geschwindigkeit der führenden Achse
    let d = 0;
    for (let j = 0; j < 6; j++) d = Math.max(d, Math.abs(b[j] - a[j]) * (j === 5 ? 0.5 : 1));
    const T = Math.max(0.25, (d / speed) * 1.4);
    return traj.plan(
      [
        { m: a.slice(), dur: 0, dwell: 0 },
        { m: b.slice(), dur: T, dwell: 0 },
      ],
      { profile: 'quintic', interp: 'ptp', stop: true, dt: 0.02 }
    );
  };
})();
