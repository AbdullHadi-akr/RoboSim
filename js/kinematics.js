/* Braccio-Simulator – Kinematik
 *
 * Konvention (Servowinkel M1..M5 in Grad, Braccio-Bibliothek):
 *   Azimut           φ  = M1
 *   Oberarm-Elevation e2 = 180° − M2           (M2 = 90° → senkrecht)
 *   Unterarm-Elevation e3 = 270° − M2 − M3     (M3 = 90° → in Verlängerung)
 *   Werkzeug-Elevation e4 = 360° − M2 − M3 − M4 = ψ (Werkzeugwinkel ggü. Horizontale)
 *
 * DH-Parameter (Rot_z(θ) · Trans_z(d) · Trans_x(a) · Rot_x(α)):
 *   i | θ_i          | d_i  | a_i | α_i
 *   1 | M1           | d1   | 0   | 90°
 *   2 | 180° − M2    | 0    | a2  | 0
 *   3 | 90° − M3     | 0    | a3  | 0
 *   4 | 180° − M4    | 0    | 0   | 90°
 *   5 | M5           | lTcp | 0   | 0      (M5 = 90° → Greiferbacken öffnen quer zur Armebene)
 */
(function () {
  'use strict';
  const BS = (typeof window !== 'undefined' ? window : globalThis).BS;
  const { D2R, R2D, normServo, wrap180 } = BS;
  const C = BS.config;

  const kin = (BS.kin = {});

  kin.dhRows = function (m, g) {
    g = g || C.geom;
    return [
      { theta: m[0] * D2R, d: g.d1, a: 0, alpha: Math.PI / 2 },
      { theta: (180 - m[1]) * D2R, d: 0, a: g.a2, alpha: 0 },
      { theta: (90 - m[2]) * D2R, d: 0, a: g.a3, alpha: 0 },
      { theta: (180 - m[3]) * D2R, d: 0, a: 0, alpha: Math.PI / 2 },
      { theta: (m[4] == null ? 90 : m[4]) * D2R, d: g.lTcp, a: 0, alpha: 0 },
    ];
  };

  /** Homogene DH-Matrix (zeilenweise, 16 Elemente) */
  kin.dhMat = function (th, d, a, al) {
    const ct = Math.cos(th),
      st = Math.sin(th),
      ca = Math.cos(al),
      sa = Math.sin(al);
    return [ct, -st * ca, st * sa, a * ct, st, ct * ca, -ct * sa, a * st, 0, sa, ca, d, 0, 0, 0, 1];
  };

  kin.mul = function (A, B) {
    const R = new Array(16);
    for (let i = 0; i < 4; i++)
      for (let j = 0; j < 4; j++) {
        let s = 0;
        for (let k = 0; k < 4; k++) s += A[i * 4 + k] * B[k * 4 + j];
        R[i * 4 + j] = s;
      }
    return R;
  };

  kin.I = () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];

  /** Liefert [T0_0, T0_1, ..., T0_5] */
  kin.frames = function (m, g) {
    const rows = kin.dhRows(m, g);
    const out = [kin.I()];
    let T = out[0];
    for (const r of rows) {
      T = kin.mul(T, kin.dhMat(r.theta, r.d, r.a, r.alpha));
      out.push(T);
    }
    return out;
  };

  /** Ebene Größen der Armkette (r = radiale Koordinate in Richtung M1) */
  kin.planar = function (m, g) {
    g = g || C.geom;
    const e2 = (180 - m[1]) * D2R,
      e3 = (270 - m[1] - m[2]) * D2R,
      e4 = (360 - m[1] - m[2] - m[3]) * D2R;
    const er = { r: g.a2 * Math.cos(e2), z: g.d1 + g.a2 * Math.sin(e2) };
    const wr = { r: er.r + g.a3 * Math.cos(e3), z: er.z + g.a3 * Math.sin(e3) };
    const tcp = { r: wr.r + g.lTcp * Math.cos(e4), z: wr.z + g.lTcp * Math.sin(e4) };
    const tip = { r: wr.r + g.lTip * Math.cos(e4), z: wr.z + g.lTip * Math.sin(e4) };
    return { e2, e3, e4, shoulder: { r: 0, z: g.d1 }, elbow: er, wrist: wr, tcp, tip };
  };

  /** Vorwärtskinematik: TCP-Position und Werkzeugwinkel ψ */
  kin.tcp = function (m, g) {
    const p = kin.planar(m, g);
    const c = Math.cos(m[0] * D2R),
      s = Math.sin(m[0] * D2R);
    // e4: Werkzeugwinkel in der Armebene (bezogen auf die M1-Richtung)
    // ψ : Werkzeugwinkel bezogen auf die radial nach außen zeigende Horizontale
    const e4 = wrap180(p.e4 * R2D);
    const psi = p.tcp.r >= -1e-9 ? e4 : wrap180(180 - e4);
    return { x: p.tcp.r * c, y: p.tcp.r * s, z: p.tcp.z, r: p.tcp.r, psi, e4 };
  };

  /** Jacobi-Matrix d(x,y,z[,ψ])/d(M1..M4) mit Gelenkwinkeln in rad */
  kin.jacobian = function (m, g, withPsi) {
    g = g || C.geom;
    const p = kin.planar(m, g);
    const s2 = Math.sin(p.e2),
      s3 = Math.sin(p.e3),
      s4 = Math.sin(p.e4);
    const c2 = Math.cos(p.e2),
      c3 = Math.cos(p.e3),
      c4 = Math.cos(p.e4);
    const r = p.tcp.r;
    const dr = [0, g.a2 * s2 + g.a3 * s3 + g.lTcp * s4, g.a3 * s3 + g.lTcp * s4, g.lTcp * s4];
    const dz = [0, -(g.a2 * c2 + g.a3 * c3 + g.lTcp * c4), -(g.a3 * c3 + g.lTcp * c4), -g.lTcp * c4];
    const cp = Math.cos(m[0] * D2R),
      sp = Math.sin(m[0] * D2R);
    const J = [
      [-r * sp, cp * dr[1], cp * dr[2], cp * dr[3]],
      [r * cp, sp * dr[1], sp * dr[2], sp * dr[3]],
      [0, dz[1], dz[2], dz[3]],
    ];
    if (withPsi) J.push([0, -1, -1, -1]);
    return J;
  };

  /** Kennzahlen der Jacobi-Matrix (Positionsanteil) */
  kin.jacobianInfo = function (m, g) {
    const J = kin.jacobian(m, g, false);
    const JJt = BS.la.mul(J, BS.la.T(J));
    const ev = BS.la.eigSym(JJt).map((v) => Math.max(v, 0));
    const sv = ev.map(Math.sqrt);
    const w = Math.sqrt(Math.max(BS.la.det(JJt), 0));
    return { J, sv, w, cond: sv[2] > 1e-9 ? sv[0] / sv[2] : Infinity };
  };

  /** Punkte entlang der Armkette (für Kollisionsprüfung), nutzt Frames */
  kin.armPoints = function (F, g, jaw) {
    g = g || C.geom;
    const o = (T) => [T[3], T[7], T[11]];
    const sh = o(F[1]),
      el = o(F[2]),
      wr = o(F[3]),
      tcp = o(F[5]);
    const T5 = F[5];
    const zt = [T5[2], T5[6], T5[10]],
      xj = [T5[0], T5[4], T5[8]];
    const pts = [];
    const seg = (a, b, n, link) => {
      for (let i = 0; i <= n; i++) {
        const t = i / n;
        pts.push({ p: [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t], link });
      }
    };
    seg(sh, el, 8, 'Oberarm');
    seg(el, wr, 8, 'Unterarm');
    const ext = g.lTip - g.lTcp;
    const tip = [tcp[0] + zt[0] * ext, tcp[1] + zt[1] * ext, tcp[2] + zt[2] * ext];
    seg(wr, tip, 10, 'Greifer');
    const hw = (jaw || 0) / 2 + 4;
    for (const s of [-1, 1]) {
      pts.push({ p: [tip[0] + xj[0] * hw * s, tip[1] + xj[1] * hw * s, tip[2] + xj[2] * hw * s], link: 'Greiferfinger' });
      const mid = [tcp[0] - zt[0] * 30, tcp[1] - zt[1] * 30, tcp[2] - zt[2] * 30];
      pts.push({ p: [mid[0] + xj[0] * hw * s, mid[1] + xj[1] * hw * s, mid[2] + xj[2] * hw * s], link: 'Greiferfinger' });
    }
    return { pts, sh, el, wr, tcp, tip };
  };

  function inLimits(m, n) {
    const L = C.limits;
    const bad = [];
    for (let i = 0; i < n; i++) if (m[i] < L[i][0] - 1e-6 || m[i] > L[i][1] + 1e-6) bad.push(i);
    return bad;
  }
  kin.inLimits = inLimits;

  /** Prüft, ob eine (erreichbare) Pose unzulässig ist (Tisch / Basis) */
  kin.poseCollision = function (m, g) {
    g = g || C.geom;
    const p = kin.planar(m, g);
    const reasons = [];
    if (p.elbow.z < 0) reasons.push('Ellbogen unter Tisch');
    if (p.wrist.z < 0) reasons.push('Handgelenk unter Tisch');
    if (p.tip.z < -1) reasons.push('Greifer unter Tisch');
    const nearBase = (q) => Math.abs(q.r) < 62 && q.z < 85;
    if (nearBase(p.tip) || nearBase(p.tcp)) reasons.push('Kollision mit Basis');
    return reasons;
  };

  /**
   * Analytische (geometrische) inverse Kinematik.
   * t = {x,y,z} in mm, psi = Werkzeugwinkel ggü. Horizontale in Grad (−90 = senkrecht nach unten).
   * Liefert alle Lösungen (vorne/hinten × Ellbogen oben/unten) mit Gültigkeitsstatus.
   */
  kin.ik = function (t, psi, g, opts) {
    g = g || C.geom;
    opts = opts || {};
    const sols = [];
    const rh = Math.hypot(t.x, t.y);
    const phi = rh < 1e-6 ? (opts.m1 != null ? opts.m1 : 90) : Math.atan2(t.y, t.x) * R2D;
    for (const side of [1, -1]) {
      const m1 = normServo(side > 0 ? phi : phi + 180);
      const r = side * rh;
      const e4 = (side > 0 ? psi : 180 - psi) * D2R;
      const rw = r - g.lTcp * Math.cos(e4);
      const zw = t.z - g.d1 - g.lTcp * Math.sin(e4);
      let D = (rw * rw + zw * zw - g.a2 * g.a2 - g.a3 * g.a3) / (2 * g.a2 * g.a3);
      if (Math.abs(D) > 1 + 1e-9) {
        sols.push({ side, reachable: false, valid: false, m: null, reasons: ['außerhalb des Arbeitsraums'] });
        continue;
      }
      D = BS.clamp(D, -1, 1);
      for (const s of [-1, 1]) {
        if (Math.abs(D) === 1 && s > 0) continue; // nur eine Lösung bei gestrecktem/gefaltetem Arm
        const t3 = s * Math.acos(D);
        const e2 = Math.atan2(zw, rw) - Math.atan2(g.a3 * Math.sin(t3), g.a2 + g.a3 * Math.cos(t3));
        const e3 = e2 + t3;
        const m2 = normServo(180 - e2 * R2D);
        const m3 = normServo(270 - m2 - e3 * R2D);
        const m4 = normServo(360 - m2 - m3 - e4 * R2D);
        const m = [m1, m2, m3, m4];
        const bad = inLimits(m, 4);
        const reasons = bad.map((i) => `${C.short[i]} = ${m[i].toFixed(1)}° außerhalb [${C.limits[i][0]}, ${C.limits[i][1]}]`);
        const col = bad.length ? [] : kin.poseCollision(m, g);
        // Ellbogen oberhalb der Verbindungslinie Schulter–Handgelenk?
        const ez = g.a2 * Math.sin(e2),
          er = g.a2 * Math.cos(e2);
        const cross = rw * ez - zw * er;
        const elbowUp = side > 0 ? cross > 0 : cross < 0;
        sols.push({
          side,
          reachable: true,
          valid: bad.length === 0 && col.length === 0,
          limitsOk: bad.length === 0,
          collision: col,
          m,
          elbowUp,
          reasons: reasons.concat(col),
        });
      }
    }
    return sols;
  };

  function jointDist(a, b) {
    let s = 0;
    for (let i = 0; i < 4; i++) s += Math.abs(a[i] - b[i]);
    return s;
  }

  /** Beste gültige analytische Lösung (am nächsten zur aktuellen Pose) */
  kin.ikBest = function (t, psi, g, cur) {
    const sols = kin.ik(t, psi, g, { m1: cur ? cur[0] : undefined }).filter((s) => s.valid);
    if (!sols.length) return null;
    if (cur) sols.sort((a, b) => jointDist(a.m, cur) - jointDist(b.m, cur));
    else sols.sort((a, b) => b.side - a.side || (b.elbowUp ? 1 : 0) - (a.elbowUp ? 1 : 0));
    return sols[0];
  };

  /** IK mit automatischer Wahl von ψ (möglichst nah am Wunschwinkel) */
  kin.ikAuto = function (t, g, opts) {
    opts = opts || {};
    const pref = opts.psiPref != null ? opts.psiPref : -90;
    for (let d = 0; d <= 180; d += 1) {
      for (const sgn of d === 0 ? [1] : [1, -1]) {
        const psi = pref + sgn * d;
        if (psi < -180 || psi > 180) continue;
        const s = kin.ikBest(t, psi, g, opts.cur);
        if (s) return Object.assign({ psi }, s);
      }
    }
    return null;
  };

  /**
   * Numerische IK (Skript Kap. 3): Gradientenverfahren q+ = q + α Jᵀ e,
   * Pseudoinverse oder Damped Least Squares, jeweils mit Projektion auf die Gelenkgrenzen.
   */
  kin.ikNumeric = function (t, o) {
    const g = o.g || C.geom;
    const usePsi = !!o.usePsi;
    const wPsi = o.wPsi || 100; // mm pro rad
    let q = o.q0.slice(0, 4).map((v) => v * D2R);
    const lo = C.limits.slice(0, 4).map((l) => l[0] * D2R),
      hi = C.limits.slice(0, 4).map((l) => l[1] * D2R);
    const hist = [];
    const t0 = performance.now();
    let k = 0,
      converged = false,
      reason = 'k_max erreicht';
    const err = (qq) => {
      const m = qq.map((v) => v * R2D);
      const f = kin.tcp(m, g);
      const e = [t.x - f.x, t.y - f.y, t.z - f.z];
      if (usePsi) e.push(wrap180(o.psi - f.e4) * D2R * wPsi); // ψ in der Armebene (vorne)
      return e;
    };
    let e = err(q);
    const norm = (v) => Math.sqrt(v.reduce((s, x) => s + x * x, 0));
    const pos = (e) => Math.hypot(e[0], e[1], e[2]);
    hist.push({ m: q.map((v) => v * R2D), ep: pos(e), epsi: usePsi ? Math.abs(e[3] / wPsi) * R2D : 0 });
    for (k = 1; k <= o.kmax; k++) {
      const m = q.map((v) => v * R2D);
      const J = kin.jacobian(m, g, usePsi);
      if (usePsi) J[3] = J[3].map((v) => v * wPsi);
      const Jt = BS.la.T(J);
      let dq;
      if (o.method === 'gradient') {
        dq = BS.la.mulVec(Jt, e).map((v) => v * o.alpha);
      } else {
        const JJt = BS.la.mul(J, Jt);
        const lam2 = o.method === 'dls' ? o.lambda * o.lambda : 1e-9;
        for (let i = 0; i < JJt.length; i++) JJt[i][i] += lam2;
        const y = BS.la.solve(JJt, e) || e.map(() => 0);
        dq = BS.la.mulVec(Jt, y);
        // Schrittweitenbegrenzung (Stabilität nahe Singularitäten)
        const mx = Math.max(...dq.map(Math.abs));
        if (mx > 0.35) dq = dq.map((v) => (v * 0.35) / mx);
      }
      let qn = q.map((v, i) => BS.clamp(v + dq[i], lo[i], hi[i]));
      const step = norm(qn.map((v, i) => v - q[i]));
      q = qn;
      e = err(q);
      if (hist.length < 6000) hist.push({ m: q.map((v) => v * R2D), ep: pos(e), epsi: usePsi ? Math.abs(e[3] / wPsi) * R2D : 0 });
      if (pos(e) < (o.tol || 0.05) && (!usePsi || Math.abs(e[3] / wPsi) * R2D < 0.05)) {
        converged = true;
        reason = 'Toleranz erreicht';
        break;
      }
      if (step < o.eps) {
        reason = '‖q_{k+1} − q_k‖ < ε';
        converged = pos(e) < 1;
        break;
      }
    }
    return {
      m: q.map((v) => v * R2D),
      iters: Math.min(k, o.kmax),
      converged,
      reason,
      ep: pos(e),
      epsi: usePsi ? Math.abs(e[3] / wPsi) * R2D : null,
      hist,
      ms: performance.now() - t0,
    };
  };

  /**
   * Arbeitsraum in der vertikalen Ebene (r, z).
   * Kategorie je Zelle: 0 = unerreichbar, 1 = erreichbar, 2 = erreichbar aber unzulässig (Kollision)
   */
  kin.workspace = function (o) {
    const g = o.g || C.geom;
    const cell = o.cell || 5;
    const r0 = -500,
      r1 = 500,
      z0 = -150,
      z1 = 560;
    const nr = Math.round((r1 - r0) / cell),
      nz = Math.round((z1 - z0) / cell);
    const grid = new Uint8Array(nr * nz);
    const L = C.limits;
    const mark = (m, onlySign) => {
      const p = kin.planar(m, g);
      if (onlySign && Math.sign(p.tcp.r || 1) !== onlySign) return;
      const ix = Math.floor((p.tcp.r - r0) / cell),
        iz = Math.floor((p.tcp.z - z0) / cell);
      if (ix < 0 || ix >= nr || iz < 0 || iz >= nz) return;
      const idx = iz * nr + ix;
      if (grid[idx] === 1) return;
      const bad =
        p.elbow.z < 0 || p.wrist.z < 0 || p.tip.z < -1 || (Math.abs(p.tip.r) < 62 && p.tip.z < 85) || (Math.abs(p.tcp.r) < 62 && p.tcp.z < 85);
      grid[idx] = bad ? 2 : 1;
    };
    if (o.mode === 'fixed') {
      for (const side of [1, -1]) {
        const e4 = side > 0 ? o.psi : 180 - o.psi;
        for (let m2 = L[1][0]; m2 <= L[1][1]; m2 += 0.5)
          for (let m3 = L[2][0]; m3 <= L[2][1]; m3 += 0.5) {
            const m4 = normServo(360 - m2 - m3 - e4);
            if (m4 < L[3][0] || m4 > L[3][1]) continue;
            mark([0, m2, m3, m4], side);
          }
      }
    } else {
      for (let m2 = L[1][0]; m2 <= L[1][1]; m2 += 1.5)
        for (let m3 = L[2][0]; m3 <= L[2][1]; m3 += 2)
          for (let m4 = L[3][0]; m4 <= L[3][1]; m4 += 2) mark([0, m2, m3, m4]);
    }
    return { grid, nr, nz, r0, r1, z0, z1, cell };
  };

  /**
   * Kalibrierung: Schätzung der Gelenk-Offsets δ (Grad) mittels Levenberg-Marquardt, so dass
   * f_dir(m_k + δ) ≈ p_k für alle Messungen k.
   */
  kin.estimateOffsets = function (rows, joints, g) {
    g = g || C.geom;
    let d = [0, 0, 0, 0];
    const res = (dd) => {
      const r = [];
      for (const row of rows) {
        const m = row.m.slice(0, 4).map((v, i) => v + dd[i]);
        const f = kin.tcp(m, g);
        r.push(f.x - row.p[0], f.y - row.p[1], f.z - row.p[2]);
      }
      return r;
    };
    const rms = (r) => Math.sqrt(r.reduce((s, v) => s + v * v, 0) / (r.length / 3));
    let r = res(d);
    const rms0 = rms(r);
    let mu = 1e-3;
    for (let it = 0; it < 100; it++) {
      // Jacobi bzgl. der aktiven Offsets (in Grad)
      const Jfull = [];
      for (const row of rows) {
        const m = row.m.slice(0, 4).map((v, i) => v + d[i]);
        const J = kin.jacobian(m, g, false);
        for (const jr of J) Jfull.push(joints.map((j) => jr[j] * D2R));
      }
      const Jt = BS.la.T(Jfull);
      const A = BS.la.mul(Jt, Jfull);
      const b = BS.la.mulVec(Jt, r).map((v) => -v);
      for (let i = 0; i < A.length; i++) A[i][i] += mu * (A[i][i] + 1e-9);
      const step = BS.la.solve(A, b);
      if (!step) break;
      const dn = d.slice();
      joints.forEach((j, i) => (dn[j] += step[i]));
      const rn = res(dn);
      if (rms(rn) < rms(r)) {
        d = dn;
        r = rn;
        mu = Math.max(mu / 3, 1e-9);
        if (Math.max(...step.map(Math.abs)) < 1e-6) break;
      } else mu *= 4;
    }
    return { delta: d, rmsBefore: rms0, rmsAfter: rms(r), residuals: r };
  };
})();
