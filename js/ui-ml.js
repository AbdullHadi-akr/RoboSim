/* Tab „Machine Learning“: inverse Kinematik mit einem neuronalen Netz
 *  1. Trainingsdaten in der Simulation erzeugen (Vorwärtskinematik des Nennmodells oder des „realen“ Roboters)
 *  2. Netz konfigurieren und trainieren (Worker, Adam, Loss über Gelenkwinkel und/oder Vorwärtskinematik)
 *  3. Auswerten (Loss-Kurven, Positionsfehler, Histogramm, Fehlerkarte im 3D)
 *  4. Anwenden (Ziel anfahren, Vergleich mit analytischer IK, Verfeinerung per DLS, Kreisbahn)
 *  5. Modell speichern, laden, exportieren (JSON, Arduino-Header)
 */
(function () {
  'use strict';
  const BS = window.BS,
    U = BS.ui,
    h = BS.h,
    C = BS.config;
  const NN = BS.nn,
    core = NN.core;

  const st = Object.assign(
    {
      source: 'model',
      noise: 0.5,
      sampling: 'joint',
      n: 20000,
      elbow: 'up',
      psiMin: -90,
      psiMax: -20,
      dataSeed: 1,
      feat: 'cyl',
      hidden: '64,64',
      act: 'tanh',
      loss: 'joint',
      lr: 0.003,
      schedule: 'cosine',
      batch: 32,
      epochs: 80,
      l2: 0,
      seed: 1,
      view: 'data',
      tx: 0,
      ty: 230,
      tz: 120,
      tpsi: -45,
      refine: false,
      refineK: 3,
      live: false,
      cr: 60,
      cx: 0,
      cy: 230,
      cz: 120,
      cn: 60,
      cT: 8,
    },
    BS.store.get('ml', {})
  );
  const save = () => BS.store.set('ml', st);

  let data = null; // {train, val, rawVal, stats}
  let model = null; // {net, meta}
  let run = null; // laufendes Training
  let hist = []; // [{epoch, train, val, lr}]
  let evalRes = null; // Auswertung auf den Validierungsdaten
  let pending = false;
  // DOM
  let dataInfo, archInfo, trainInfo, progBar, lossPlot, evalBox, histCv, applyBox, modelInfo, btnTrain, btnCont, btnPause, btnStop, circleInfo;
  let inTx, inTy, inTz, inTpsi;

  // ------------------------------------------------------------------
  // Hilfen
  // ------------------------------------------------------------------
  const elbowUp = (m) => {
    // Ellbogen oberhalb der Verbindungslinie Schulter–Handgelenk (Konfiguration „vorne“)
    const p = BS.kin.planar(m, C.geom);
    const er = p.elbow.r,
      ez = p.elbow.z - C.geom.d1,
      wr = p.wrist.r,
      wz = p.wrist.z - C.geom.d1;
    return wr * ez - wz * er > 0;
  };
  const realOffsets = () => (st.source === 'real' && BS.sim.errOn ? BS.sim.errOff.slice(0, 4) : [0, 0, 0, 0]);
  /** Pose, die der Roboter bei Servobefehl m tatsächlich einnimmt */
  const reachedPose = (m, off) => BS.kin.tcp(m.map((v, i) => v + (off ? off[i] : 0)), C.geom);
  const lim = C.limits;
  const fmtN = (n) => n.toLocaleString('de-DE');
  const parseHidden = (s) =>
    String(s)
      .split(/[,; ]+/)
      .map((v) => parseInt(v, 10))
      .filter((v) => v > 0 && v <= 512);

  // ------------------------------------------------------------------
  // 1 · Trainingsdaten
  // ------------------------------------------------------------------
  function generate() {
    if (run) return BS.toast('Während des Trainings nicht möglich', 'warn');
    const g = C.geom,
      r = core.rng(st.dataSeed);
    const off = realOffsets();
    if (st.source === 'real' && !BS.sim.errOn) BS.toast('Hinweis: Im Tab „Kalibrierung“ sind keine Fertigungsfehler aktiv – Daten entsprechen dem Nennmodell', 'warn');
    const n = Math.max(200, Math.round(st.n));
    const feat = st.feat;
    const nIn = NN.featureNames[feat].length;
    const X = new Float32Array(n * nIn),
      Y = new Float32Array(n * 4),
      P = new Float32Array(n * 4),
      M = new Float32Array(n * 4);
    const uni = (a, b) => a + (b - a) * r();
    const psiLo = Math.min(st.psiMin, st.psiMax),
      psiHi = Math.max(st.psiMin, st.psiMax);
    let k = 0,
      tries = 0;
    const t0 = performance.now();
    while (k < n && tries < n * 400) {
      tries++;
      let m;
      const psi = uni(psiLo, psiHi);
      if (st.sampling === 'joint') {
        // gleichverteilt im Gelenkraum; M4 folgt aus dem gewünschten Werkzeugwinkel
        const m2 = uni(lim[1][0], lim[1][1]),
          m3 = uni(lim[2][0], lim[2][1]);
        m = [uni(lim[0][0], lim[0][1]), m2, m3, BS.normServo(360 - m2 - m3 - psi)];
      } else {
        // gleichverteilt im kartesischen Arbeitsraum (Zylinder), Beschriftung über die analytische IK
        const R0 = 40,
          R1 = 470;
        const rr = Math.sqrt(uni(R0 * R0, R1 * R1)),
          ph = uni(0, Math.PI);
        const t = { x: rr * Math.cos(ph), y: rr * Math.sin(ph), z: uni(-10, 480) };
        const sols = BS.kin.ik(t, psi, g).filter((s) => s.valid && s.side > 0 && (st.elbow === 'any' || s.elbowUp === (st.elbow === 'up')));
        if (!sols.length) continue;
        m = sols[Math.floor(r() * sols.length)].m;
      }
      if (BS.kin.inLimits(m, 4).length || BS.kin.poseCollision(m, g).length) continue;
      const nom = BS.kin.tcp(m, g);
      if (nom.r < 15) continue; // Konfiguration „vorne“, nicht über der Basisachse
      if (st.elbow !== 'any' && elbowUp(m) !== (st.elbow === 'up')) continue;
      // „Messung“ der erreichten Pose (realer Roboter: Fertigungsfehler + Messrauschen)
      const p = reachedPose(m, off);
      let x = p.x,
        y = p.y,
        z = p.z;
      if (st.source === 'real' && st.noise > 0) {
        x += r.gauss() * st.noise;
        y += r.gauss() * st.noise;
        z += r.gauss() * st.noise;
      }
      const f = NN.features(feat, x, y, z, p.psi);
      for (let i = 0; i < nIn; i++) X[k * nIn + i] = f[i];
      for (let i = 0; i < 4; i++) {
        Y[k * 4 + i] = (m[i] - 90) / 90;
        M[k * 4 + i] = m[i];
      }
      P.set([x, y, z, p.psi], k * 4);
      k++;
    }
    if (k < 50) {
      data = null;
      dataInfo.innerHTML = '<span class="err">Kaum gültige Posen gefunden – ψ-Bereich oder Konfiguration anpassen.</span>';
      return;
    }
    // Aufteilung 85 % Training / 15 % Validierung (Reihenfolge ist bereits zufällig)
    const nTr = Math.round(k * 0.85),
      nVa = k - nTr;
    const mu = new Array(nIn).fill(0),
      sd = new Array(nIn).fill(0);
    for (let s = 0; s < nTr; s++) for (let i = 0; i < nIn; i++) mu[i] += X[s * nIn + i] / nTr;
    for (let s = 0; s < nTr; s++) for (let i = 0; i < nIn; i++) sd[i] += (X[s * nIn + i] - mu[i]) ** 2 / nTr;
    for (let i = 0; i < nIn; i++) sd[i] = Math.sqrt(sd[i]) || 1;
    for (let s = 0; s < k; s++) for (let i = 0; i < nIn; i++) X[s * nIn + i] = (X[s * nIn + i] - mu[i]) / sd[i];
    const part = (a, b) => ({
      X: X.slice(a * nIn, b * nIn),
      Y: Y.slice(a * 4, b * 4),
      P: P.slice(a * 4, b * 4),
      M: M.slice(a * 4, b * 4),
      n: b - a,
      nIn,
      nOut: 4,
    });
    data = {
      train: part(0, nTr),
      val: part(nTr, k),
      meta: { feat, mu, sd, elbow: st.elbow, psiRange: [psiLo, psiHi], source: st.source, offsets: off, noise: st.source === 'real' ? st.noise : 0, sampling: st.sampling },
      tries,
      ms: performance.now() - t0,
    };
    // passt ein vorhandenes Modell nicht mehr (andere Merkmale), wird es verworfen
    if (model && model.meta.feat !== feat) {
      model = null;
      hist = [];
      evalRes = null;
    }
    renderData();
    if (model) evaluate();
    renderModel();
    showCloud();
    BS.toast(`${fmtN(k)} Trainingsbeispiele erzeugt`, 'ok');
  }

  function renderData() {
    if (!dataInfo) return;
    if (!data) {
      dataInfo.innerHTML = '<span class="muted">Noch keine Daten erzeugt.</span>';
      return;
    }
    const d = data,
      m = d.meta;
    const acc = (100 * (d.train.n + d.val.n)) / d.tries;
    dataInfo.innerHTML =
      `<div class="kv"><b>Beispiele</b><span class="mono">${fmtN(d.train.n)} Training · ${fmtN(d.val.n)} Validierung</span>` +
      `<b>Quelle</b><span>${m.source === 'real' ? `simulierter realer Roboter (Offsets ${m.offsets.map((v) => v.toFixed(2)).join(' / ')}°, Rauschen σ = ${m.noise} mm)` : 'Nennmodell (exakt)'}</span>` +
      `<b>Stichprobe</b><span>${m.sampling === 'joint' ? 'gleichverteilt im Gelenkraum' : 'gleichverteilt im Arbeitsraum'} · Annahmequote ${acc.toFixed(1)} % · ${d.ms.toFixed(0)} ms</span>` +
      `<b>Eingang</b><span class="mono">${NN.featureNames[m.feat].join(', ')}</span>` +
      `<b>Ausgang</b><span class="mono">M1 … M4, normiert (M − 90°) / 90°</span></div>`;
  }

  // ------------------------------------------------------------------
  // 2 · Netz & Training
  // ------------------------------------------------------------------
  function lossCfg() {
    const w = { joint: [1, 0], fk: [0, 1], both: [1, 0.5] }[st.loss] || [1, 0];
    return {
      wJ: w[0],
      wF: w[1],
      wL: w[1] > 0 ? 10 : 0, // Strafterm Gelenkgrenzen (nur mit FK-Loss nötig)
      sP: 50,
      wPsi: 1,
      geom: Object.assign({}, C.geom),
      lo: lim.slice(0, 4).map((l) => (l[0] - 90) / 90),
      hi: lim.slice(0, 4).map((l) => (l[1] - 90) / 90),
    };
  }

  function sizes() {
    return [NN.featureNames[st.feat].length, ...parseHidden(st.hidden), 4];
  }

  function startTraining(cont) {
    if (run) return;
    if (!data) generate();
    if (!data) return;
    const hid = parseHidden(st.hidden);
    if (!hid.length) return BS.toast('Verdeckte Schichten angeben, z. B. 64,64', 'err');
    const sz = sizes();
    const same = model && model.net.sizes.join() === sz.join() && model.net.act === st.act && model.meta.feat === data.meta.feat;
    if (!cont || !same) {
      model = { net: core.create(sz, st.act, st.seed), meta: Object.assign({}, data.meta) };
      hist = [];
      evalRes = null;
    } else model.meta = Object.assign({}, data.meta, { eval: model.meta.eval });
    const e0 = hist.length ? hist[hist.length - 1].epoch : 0;
    const opt = { lr: st.lr, batch: Math.max(1, Math.round(st.batch)), epochs: e0 + Math.max(1, Math.round(st.epochs)), e0, l2: st.l2, schedule: st.schedule, seed: st.seed + e0, snapEvery: 2 };
    const t0 = performance.now();
    run = {
      t0,
      e0,
      epochs: opt.epochs,
      paused: false,
      h: NN.startTraining({ net: model.net, opt, cfg: lossCfg(), train: data.train, val: data.val, startEpoch: e0 }, onMsg),
    };
    BS.toast(run.h.worker ? 'Training gestartet (Web Worker)' : 'Training gestartet (Haupt-Thread)', 'ok');
    renderButtons();
  }

  function onMsg(m) {
    if (!run) return;
    if (m.type === 'epoch') {
      hist.push({ epoch: m.epoch, train: m.train, val: m.val, lr: m.lr, ms: m.ms });
      run.last = m;
      drawLoss();
    } else if (m.type === 'snapshot') {
      if (model) {
        model.net.W = m.W;
        model.net.b = m.b;
        model.buf = null;
        if (!pending) {
          pending = true;
          setTimeout(() => {
            pending = false;
            evaluate();
            refreshApply();
          }, 0);
        }
      }
    } else if (m.type === 'done' || m.type === 'stopped') {
      const r = run;
      run = null;
      if (r && r.h.kill) setTimeout(() => r.h.kill(), 500);
      setTimeout(() => {
        evaluate();
        refreshApply();
        renderModel();
      }, 0);
      BS.toast(m.type === 'done' ? 'Training abgeschlossen' : 'Training gestoppt', 'ok');
      renderButtons();
    } else if (m.type === 'error') {
      BS.toast('Fehler im Training: ' + m.message, 'err');
      run = null;
      renderButtons();
    }
  }

  function renderButtons() {
    if (!btnTrain) return;
    btnTrain.disabled = !!run;
    btnCont.disabled = !!run || !model;
    btnPause.disabled = !run;
    btnStop.disabled = !run;
    btnPause.textContent = run && run.paused ? '▶ Fortsetzen' : '⏸ Pause';
  }

  function renderArch() {
    if (!archInfo) return;
    const sz = sizes();
    const net = { sizes: sz, W: sz.slice(1).map((n, l) => ({ length: n * sz[l] })), b: sz.slice(1).map((n) => ({ length: n })) };
    const p = core.paramCount(net);
    archInfo.innerHTML = `<span class="mono">${sz.join(' → ')}</span> · ${fmtN(p)} Parameter · ${((p * 4) / 1024).toFixed(1)} KiB (float32)`;
  }

  function drawLoss() {
    if (!lossPlot) return;
    const series = [
      { name: 'Training', color: '#19b6ae', data: hist.map((x) => [x.epoch, x.train]) },
      { name: 'Validierung', color: '#ffa94d', data: hist.map((x) => [x.epoch, x.val]) },
    ];
    lossPlot.draw({ series, logY: true, xLabel: 'Epoche', title: 'Verlust (logarithmisch)' });
  }

  // ------------------------------------------------------------------
  // 3 · Auswertung
  // ------------------------------------------------------------------
  function evaluate() {
    if (!model || !data) return;
    const v = data.val,
      off = data.meta.offsets;
    const pos = new Float32Array(v.n),
      psiE = new Float32Array(v.n);
    let jSq = 0,
      outLim = 0;
    for (let s = 0; s < v.n; s++) {
      const P = [v.P[s * 4], v.P[s * 4 + 1], v.P[s * 4 + 2], v.P[s * 4 + 3]];
      const m = NN.ik(model, P[0], P[1], P[2], P[3]);
      if (BS.kin.inLimits(m, 4).length) outLim++;
      const f = reachedPose(m, off);
      pos[s] = Math.hypot(f.x - P[0], f.y - P[1], f.z - P[2]);
      psiE[s] = Math.abs(BS.wrap180(f.psi - P[3]));
      for (let i = 0; i < 4; i++) jSq += (m[i] - v.M[s * 4 + i]) ** 2;
    }
    const sorted = Float32Array.from(pos).sort();
    const q = (f) => sorted[Math.min(sorted.length - 1, Math.floor(f * sorted.length))];
    const mean = pos.reduce((a, b) => a + b, 0) / v.n;
    // Rechenzeit: NN-Inferenz vs. analytische IK
    const P0 = [v.P[0], v.P[1], v.P[2], v.P[3]];
    const N = 2000;
    let t = performance.now();
    for (let i = 0; i < N; i++) NN.ik(model, P0[0], P0[1], P0[2], P0[3]);
    const tNN = ((performance.now() - t) / N) * 1000;
    t = performance.now();
    for (let i = 0; i < N / 4; i++) BS.kin.ik({ x: P0[0], y: P0[1], z: P0[2] }, P0[3], C.geom);
    const tAna = ((performance.now() - t) / (N / 4)) * 1000;
    evalRes = { pos, psiE, mean, median: q(0.5), p95: q(0.95), max: sorted[sorted.length - 1], psiMean: psiE.reduce((a, b) => a + b, 0) / v.n, jRms: Math.sqrt(jSq / (v.n * 4)), outLim: outLim / v.n, tNN, tAna, epoch: hist.length ? hist[hist.length - 1].epoch : model.meta.eval ? model.meta.eval.epoch : 0 };
    model.meta.eval = { mean, median: evalRes.median, p95: evalRes.p95, epoch: evalRes.epoch };
    renderEval();
    if (st.view === 'error') showCloud();
  }

  function renderEval() {
    if (!evalBox) return;
    if (!evalRes) {
      evalBox.innerHTML = '<span class="muted">Noch kein trainiertes Modell.</span>';
      drawHist();
      return;
    }
    const e = evalRes;
    const cls = (v, a, b) => (v < a ? 'ok' : v < b ? 'warn' : 'err');
    evalBox.innerHTML =
      `<table class="tbl compact"><tr><th>Validierung (${fmtN(data ? data.val.n : 0)} Posen, Epoche ${e.epoch})</th><th class="num">Wert</th></tr>` +
      `<tr><td>Positionsfehler Mittelwert</td><td class="num ${cls(e.mean, 2, 10)}">${e.mean.toFixed(2)} mm</td></tr>` +
      `<tr><td>Positionsfehler Median</td><td class="num">${e.median.toFixed(2)} mm</td></tr>` +
      `<tr><td>Positionsfehler 95-%-Quantil</td><td class="num">${e.p95.toFixed(2)} mm</td></tr>` +
      `<tr><td>Positionsfehler Maximum</td><td class="num">${e.max.toFixed(1)} mm</td></tr>` +
      `<tr><td>Werkzeugwinkel ψ, mittlerer Fehler</td><td class="num">${e.psiMean.toFixed(2)}°</td></tr>` +
      `<tr><td>Gelenkwinkel RMS-Abweichung zum Label</td><td class="num">${e.jRms.toFixed(2)}°</td></tr>` +
      `<tr><td>Lösungen außerhalb der Gelenkgrenzen</td><td class="num ${e.outLim > 0.01 ? 'warn' : ''}">${(e.outLim * 100).toFixed(1)} %</td></tr>` +
      `<tr><td>Rechenzeit NN · analytische IK (alle Lösungen)</td><td class="num">${e.tNN.toFixed(1)} µs · ${e.tAna.toFixed(1)} µs</td></tr></table>`;
    drawHist();
  }

  /** Histogramm der Positionsfehler */
  function drawHist() {
    if (!histCv) return;
    const c = histCv,
      ctx = c.getContext('2d');
    const dpr = window.devicePixelRatio || 1,
      W = c.clientWidth,
      H = c.clientHeight;
    if (!W || !H) return;
    c.width = Math.round(W * dpr);
    c.height = Math.round(H * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, W, H);
    const css = getComputedStyle(document.documentElement);
    const colText = css.getPropertyValue('--muted').trim(),
      colGrid = css.getPropertyValue('--grid').trim();
    ctx.font = '10px ui-monospace, SFMono-Regular, Menlo, monospace';
    ctx.fillStyle = colText;
    if (!evalRes) {
      ctx.fillText('Histogramm der Positionsfehler erscheint nach dem Training', 10, H / 2);
      return;
    }
    const L = 40,
      R = 8,
      T = 18,
      B = 28,
      pw = W - L - R,
      ph = H - T - B;
    const xmax = Math.max(0.5, evalRes.p95 * 1.6);
    const nb = 40,
      bins = new Array(nb).fill(0);
    for (const v of evalRes.pos) bins[Math.min(nb - 1, Math.floor((v / xmax) * nb))]++;
    const ymax = Math.max(...bins);
    ctx.strokeStyle = colGrid;
    ctx.strokeRect(L + 0.5, T + 0.5, pw, ph);
    for (let i = 0; i < nb; i++) {
      const x = L + (i / nb) * pw,
        bh = (bins[i] / ymax) * ph;
      const f = (i + 0.5) / nb;
      ctx.fillStyle = errColorCss(f * xmax);
      ctx.fillRect(x + 1, T + ph - bh, pw / nb - 2, bh);
    }
    ctx.fillStyle = colText;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    for (let k = 0; k <= 4; k++) ctx.fillText(((k / 4) * xmax).toFixed(1), L + (k / 4) * pw, T + ph + 3);
    ctx.textAlign = 'right';
    ctx.textBaseline = 'bottom';
    ctx.fillText('Positionsfehler [mm] (letzter Balken: alles darüber)', L + pw, H - 1);
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    ctx.fillText('Häufigkeit', L, 3);
    // Median-Linie
    const xm = L + (evalRes.median / xmax) * pw;
    ctx.strokeStyle = '#e9ecef';
    ctx.setLineDash([4, 3]);
    ctx.beginPath();
    ctx.moveTo(xm, T);
    ctx.lineTo(xm, T + ph);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  /** Farbskala grün (0 mm) → gelb (5 mm) → rot (≥ 15 mm) */
  function errRgb(e) {
    const t = Math.min(1, e / 15);
    if (t < 1 / 3) {
      const u = t * 3;
      return [0.25 + 0.73 * u, 0.75 + 0.05 * u, 0.34 - 0.3 * u];
    }
    const u = (t - 1 / 3) * 1.5;
    return [0.98, 0.8 - 0.55 * u, 0.04 + 0.3 * u];
  }
  function errColorCss(e) {
    const c = errRgb(e);
    return `rgb(${Math.round(c[0] * 255)},${Math.round(c[1] * 255)},${Math.round(c[2] * 255)})`;
  }

  /** Punktwolke im 3D: Trainingsdaten oder Fehlerkarte (nur solange der Tab aktiv ist) */
  function showCloud() {
    if (U.active !== 'ml' || !data || st.view === 'none') return BS.view.setCloud(null);
    const xyz = [],
      rgb = [];
    if (st.view === 'error' && evalRes) {
      const v = data.val;
      for (let s = 0; s < v.n; s++) {
        xyz.push(v.P[s * 4], v.P[s * 4 + 1], v.P[s * 4 + 2]);
        rgb.push(...errRgb(evalRes.pos[s]));
      }
      BS.view.setCloud(xyz, rgb, 7);
      return;
    }
    const add = (D, col, max) => {
      const step = Math.max(1, Math.floor(D.n / max));
      for (let s = 0; s < D.n; s += step) {
        xyz.push(D.P[s * 4], D.P[s * 4 + 1], D.P[s * 4 + 2]);
        rgb.push(...col);
      }
    };
    add(data.train, [0.1, 0.71, 0.68], 8000);
    add(data.val, [1, 0.66, 0.3], 2000);
    BS.view.setCloud(xyz, rgb, 4);
  }

  // ------------------------------------------------------------------
  // 4 · Anwenden
  // ------------------------------------------------------------------
  let applyRes = null;
  function full(m) {
    return [m[0], m[1], m[2], m[3], BS.sim.cmd[4], BS.sim.cmd[5]];
  }
  function solveTarget(x, y, z, psi) {
    const m = NN.ik(model, x, y, z, psi);
    const t0 = performance.now();
    for (let i = 0; i < 200; i++) NN.ik(model, x, y, z, psi);
    const tNN = ((performance.now() - t0) / 200) * 1000;
    const res = { nn: m, tNN };
    if (st.refine) {
      const r = BS.kin.ikNumeric({ x, y, z }, { q0: m, method: 'dls', lambda: 5, kmax: Math.max(1, Math.round(st.refineK)), eps: 1e-9, usePsi: true, psi, tol: 1e-4 });
      res.ref = r.m;
      res.refIters = r.iters;
    }
    const sols = BS.kin.ik({ x, y, z }, psi, C.geom).filter((s) => s.valid && s.side > 0);
    const pick = sols.filter((s) => model.meta.elbow === 'any' || s.elbowUp === (model.meta.elbow === 'up'));
    res.ana = (pick[0] || sols[0] || {}).m || null;
    return res;
  }
  function refreshApply() {
    if (!applyBox) return;
    if (!model) {
      applyBox.innerHTML = '<span class="muted">Zuerst ein Modell trainieren oder laden.</span>';
      applyRes = null;
      return;
    }
    const x = st.tx,
      y = st.ty,
      z = st.tz,
      psi = st.tpsi;
    applyRes = solveTarget(x, y, z, psi);
    BS.view.setIkTarget({ x, y, z }, psi, U.active === 'ml');
    const off = model.meta.source === 'real' ? realOffsets() : [0, 0, 0, 0];
    const row = (name, m, extra) => {
      if (!m) return `<tr><td>${name}</td><td colspan="6" class="muted">keine gültige Lösung</td></tr>`;
      const f = reachedPose(m, off);
      const e = Math.hypot(f.x - x, f.y - y, f.z - z);
      const bad = BS.kin.inLimits(m, 4);
      return `<tr><td>${name}</td>${m.map((v, i) => `<td class="num ${bad.includes(i) ? 'err' : ''}">${v.toFixed(1)}</td>`).join('')}<td class="num ${e < 2 ? 'ok' : e < 10 ? 'warn' : 'err'}">${e.toFixed(2)}</td><td class="muted">${extra || ''}</td></tr>`;
    };
    const r = applyRes;
    applyBox.innerHTML =
      `<table class="tbl compact"><tr><th>Lösung</th>${C.short
        .slice(0, 4)
        .map((n, i) => `<th class="num" style="color:${C.colors[i]}">${n}</th>`)
        .join('')}<th class="num">Fehler [mm]</th><th></th></tr>` +
      row('Neuronales Netz', r.nn, `${r.tNN.toFixed(1)}&nbsp;µs`) +
      (r.ref ? row(`NN + DLS (${r.refIters} It.)`, r.ref) : '') +
      row('Analytisch (Nennmodell)', r.ana) +
      `</table>` +
      (model.meta.source === 'real' && BS.sim.errOn
        ? '<p class="hint">Fehler jeweils am <b>realen</b> Roboter (mit Fertigungsfehlern). Die analytische IK kennt nur das Nennmodell – das auf Messdaten trainierte Netz kann die Abweichung lernen.</p>'
        : '');
    if (st.live) BS.sim.setManual(full(pickPose()));
  }
  function pickPose() {
    return applyRes.ref || applyRes.nn;
  }
  function setTarget(t) {
    st.tx = Math.round(t.x * 10) / 10;
    st.ty = Math.round(t.y * 10) / 10;
    st.tz = Math.round(t.z * 10) / 10;
    if (t.psi != null) st.tpsi = Math.round(t.psi);
    if (inTx) {
      inTx.value = st.tx;
      inTy.value = st.ty;
      inTz.value = st.tz;
      inTpsi.value = st.tpsi;
    }
    save();
    refreshApply();
  }
  BS.on('pickTable', (p) => {
    if (U.active === 'ml') setTarget({ x: p.x, y: p.y, z: st.tz });
  });

  function goTo() {
    if (!applyRes) return;
    const m = pickPose();
    if (BS.kin.inLimits(m, 4).length) return BS.toast('Gelenkgrenze verletzt – nicht anfahrbar', 'err');
    BS.sim.moveTo(full(m));
  }

  /** Kreisbahn mit dem Netz abfahren (Stützpunkte im Gelenkraum, Bahn als Polynomzug ohne Halt) */
  function runCircle() {
    if (!model) return;
    const N = Math.max(8, Math.round(st.cn)),
      T = Math.max(1, st.cT);
    const pts = [],
      wps = [{ m: BS.sim.cmd.slice(), dur: 0, dwell: 0 }];
    let maxE = 0,
      sumE = 0,
      bad = 0,
      unreach = 0;
    const off = model.meta.source === 'real' ? realOffsets() : [0, 0, 0, 0];
    for (let k = 0; k <= N; k++) {
      const a = (2 * Math.PI * k) / N;
      const x = st.cx + st.cr * Math.cos(a),
        y = st.cy + st.cr * Math.sin(a),
        z = st.cz;
      pts.push([x, y, z]);
      const s = solveTarget(x, y, z, st.tpsi);
      if (!s.ana) unreach++;
      if (BS.kin.inLimits(s.ref || s.nn, 4).length) bad++;
      // der Roboter fährt die auf die Gelenkgrenzen begrenzten Winkel
      const m = (s.ref || s.nn).map((v, i) => BS.clamp(v, lim[i][0], lim[i][1]));
      const f = reachedPose(m, off);
      const e = Math.hypot(f.x - x, f.y - y, f.z - z);
      maxE = Math.max(maxE, e);
      sumE += e;
      wps.push({ m: full(m), dur: k === 0 ? 1.5 : T / N, dwell: 0 });
    }
    const plan = BS.traj.plan(wps, { interp: 'ptp', profile: 'quintic', stop: false, dt: 0.02 });
    if (!plan.ok) return BS.toast('Bahn nicht planbar', 'err');
    BS.view.setPath(pts);
    BS.view.show.path = true;
    BS.view.show.trace = true;
    if (U.syncToggles) U.syncToggles();
    BS.view.clearTrace();
    BS.sim.playPlan(plan, { label: 'NN-Kreisbahn' });
    circleInfo.innerHTML = `Stützpunkte: Fehler Ø <b>${(sumE / (N + 1)).toFixed(2)} mm</b>, max. <b>${maxE.toFixed(2)} mm</b>${bad ? ` · <span class="warn">${bad} außerhalb der Gelenkgrenzen (begrenzt)${unreach ? `, davon ${unreach} auch analytisch nicht gültig erreichbar` : ''}</span>` : ''}. Gestrichelt: Sollkreis, orange: tatsächliche TCP-Spur.`;
  }

  // ------------------------------------------------------------------
  // 5 · Modell
  // ------------------------------------------------------------------
  function renderModel() {
    if (!modelInfo) return;
    if (!model) {
      modelInfo.innerHTML = '<span class="muted">Kein Modell geladen.</span>';
      return;
    }
    const m = model.meta;
    modelInfo.innerHTML =
      `<div class="kv"><b>Architektur</b><span class="mono">${model.net.sizes.join(' → ')} (${model.net.act})</span>` +
      `<b>Parameter</b><span class="mono">${fmtN(core.paramCount(model.net))} · Arduino: ${(NN.paramBytes(model.net) / 1024).toFixed(1)} KiB Flash</span>` +
      `<b>Daten</b><span>${m.source === 'real' ? 'realer Roboter (simuliert)' : 'Nennmodell'}, Ellbogen ${m.elbow === 'up' ? 'oben' : m.elbow === 'down' ? 'unten' : 'beliebig'}, ψ ∈ [${m.psiRange.join(', ')}]°</span>` +
      (m.eval ? `<b>Validierung</b><span class="mono">Ø ${m.eval.mean.toFixed(2)} mm · Median ${m.eval.median.toFixed(2)} mm · Epoche ${m.eval.epoch}</span>` : '') +
      `</div>`;
  }
  function saveBrowser() {
    if (!model) return;
    BS.store.set('mlModel', NN.toJSON(model));
    BS.toast('Modell im Browser gespeichert', 'ok');
  }
  function loadModel(j, quiet) {
    try {
      model = NN.fromJSON(j);
      hist = [];
      evalRes = null;
      if (data && data.meta.feat === model.meta.feat) evaluate();
      renderModel();
      refreshApply();
      renderButtons();
      if (!quiet) BS.toast('Modell geladen', 'ok');
    } catch (e) {
      BS.toast('Modell nicht lesbar: ' + e.message, 'err');
    }
  }

  // ------------------------------------------------------------------
  // Aufbau
  // ------------------------------------------------------------------
  const num = (key, o, after) =>
    U.num(st[key], (v) => {
      st[key] = o && o.min != null ? Math.max(o.min, v) : v;
      save();
      if (after) after();
    }, o);
  const sel = (key, opts, after) =>
    U.sel(opts, st[key], (v) => {
      st[key] = v;
      save();
      if (after) after();
    });

  U.register('ml', {
    title: 'Machine Learning',
    build(el) {
      el.appendChild(
        U.card(
          'Inverse Kinematik mit einem neuronalen Netz',
          U.hint(
            'Das Netz lernt die Abbildung <b>Zielpose (x, y, z, ψ) → Servowinkel (M1 … M4)</b>. Trainingsdaten entstehen ohne IK-Formel: ' +
              'zufällige Gelenkwinkel anfahren und die erreichte TCP-Pose „messen“ (Vorwärtskinematik). Weil eine Pose bis zu vier Lösungen hat ' +
              '(vorne/hinten × Ellbogen oben/unten), wird auf <b>eine Konfiguration</b> eingeschränkt – sonst mittelt ein Netz mit Gelenkwinkel-Loss zwischen den Lösungen.'
          )
        )
      );

      // 1 · Daten
      dataInfo = h('div');
      el.appendChild(
        U.card(
          '1 · Trainingsdaten aus der Simulation',
          U.row(
            U.field(
              'Quelle',
              sel('source', [
                ['model', 'Nennmodell (exakt)'],
                ['real', 'realer Roboter (Fertigungsfehler + Rauschen)'],
              ])
            ),
            U.field('σ [mm]', num('noise', { step: 0.1, min: 0, cls: 'w44' }))
          ),
          U.row(
            U.field(
              'Stichprobe',
              sel('sampling', [
                ['joint', 'gleichverteilt im Gelenkraum'],
                ['cart', 'gleichverteilt im Arbeitsraum'],
              ])
            ),
            U.field('Anzahl', num('n', { step: 1000, min: 200, cls: 'w84' }))
          ),
          U.row(
            U.field(
              'Ellbogen',
              sel('elbow', [
                ['up', 'oben'],
                ['down', 'unten'],
                ['any', 'beliebig (mehrdeutig!)'],
              ])
            ),
            U.field('ψ von', num('psiMin', { step: 5, cls: 'w52' })),
            U.field('bis', num('psiMax', { step: 5, cls: 'w52' })),
            U.field('Seed', num('dataSeed', { step: 1, min: 1, cls: 'w44' }))
          ),
          U.row(
            U.btn('Daten erzeugen', generate, 'primary'),
            U.field(
              '3D-Anzeige',
              sel(
                'view',
                [
                  ['data', 'Trainingsdaten'],
                  ['error', 'Fehlerkarte (Validierung)'],
                  ['none', 'aus'],
                ],
                showCloud
              )
            )
          ),
          dataInfo,
          U.hint(
            '„Realer Roboter“ nutzt die verborgenen Gelenk-Offsets aus dem Tab <b>Kalibrierung</b> (dort „Fehler aktiv“ einschalten) und addiert Messrauschen – wie Messungen mit einem Lasertracker. Das Label ist der Servobefehl, der Eingang die gemessene Pose.'
          )
        )
      );

      // 2 · Netz & Training
      archInfo = h('div', { class: 'hint' });
      trainInfo = h('div', { class: 'mono', style: { fontSize: '12px' } });
      progBar = h('i', { style: { width: '0%' } });
      const cv = h('canvas', { class: 'plot tall' });
      lossPlot = new BS.Plot(cv);
      btnTrain = U.btn('Trainieren', () => startTraining(false), 'primary', 'Netz neu initialisieren und trainieren');
      btnCont = U.btn('Weitertrainieren', () => startTraining(true), '', 'Mit den aktuellen Gewichten weitere Epochen trainieren');
      btnPause = U.btn('⏸ Pause', () => {
        if (!run) return;
        run.paused = !run.paused;
        run.paused ? run.h.pause() : run.h.resume();
        renderButtons();
      });
      btnStop = U.btn('■ Stopp', () => run && run.h.stop());
      el.appendChild(
        U.card(
          '2 · Netzarchitektur und Training',
          U.row(
            U.field(
              'Merkmale',
              sel(
                'feat',
                [
                  ['cyl', 'zylindrisch (r, φ, z, ψ)'],
                  ['cart', 'kartesisch (x, y, z, ψ)'],
                ],
                renderArch
              )
            ),
            U.field('Schichten', (() => {
              const i = h('input', { type: 'text', value: st.hidden, style: { width: '90px' }, title: 'Neuronen je verdeckter Schicht, z. B. 64,64' });
              i.addEventListener('change', () => {
                st.hidden = i.value;
                save();
                renderArch();
              });
              return i;
            })()),
            U.field(
              'Aktivierung',
              sel('act', [
                ['tanh', 'tanh'],
                ['relu', 'ReLU'],
                ['leaky', 'Leaky ReLU'],
              ])
            )
          ),
          archInfo,
          U.row(
            U.field(
              'Loss',
              sel('loss', [
                ['joint', 'Gelenkwinkel (überwacht)'],
                ['fk', 'Vorwärtskinematik (selbstüberwacht)'],
                ['both', 'kombiniert'],
              ])
            )
          ),
          U.row(
            U.field('Lernrate', num('lr', { step: 'any', cls: 'w84' })),
            U.field(
              '',
              sel('schedule', [
                ['cosine', 'Kosinus-Abfall'],
                ['const', 'konstant'],
              ])
            ),
            U.field('Batch', num('batch', { step: 8, min: 1, cls: 'w44' }))
          ),
          U.row(U.field('Epochen', num('epochs', { step: 10, min: 1, cls: 'w52' })), U.field('L2', num('l2', { step: 'any', min: 0, cls: 'w84' })), U.field('Seed', num('seed', { step: 1, min: 1, cls: 'w44' }))),
          U.row(btnTrain, btnCont, btnPause, btnStop),
          h('div', { class: 'bar', style: { margin: '6px 0' } }, progBar),
          trainInfo,
          cv,
          U.hint(
            '<b>Gelenkwinkel-Loss</b>: mittlerer quadratischer Fehler der normierten Servowinkel gegenüber den Labels. ' +
              '<b>Vorwärtskinematik-Loss</b>: das Netz wird direkt am Positions- und Winkelfehler f(q̂) − p gemessen (Gradient über die Jacobi-Matrix, Jᵀ·e); ' +
              'braucht keine Labels und ist auch bei „Ellbogen beliebig“ eindeutig, nutzt aber das Nennmodell. Adam-Optimierer, Mini-Batches, Training im Web Worker.'
          )
        )
      );

      // 3 · Auswertung
      evalBox = h('div');
      histCv = h('canvas', { class: 'plot' });
      el.appendChild(U.card('3 · Auswertung', evalBox, histCv, U.hint('Positionsfehler = Abstand zwischen Zielpose und der Pose, die der Roboter mit den vorhergesagten Servowinkeln tatsächlich erreicht. „Fehlerkarte“ in der 3D-Anzeige färbt die Validierungsposen von grün (genau) bis rot (≥ 15 mm).')));

      // 4 · Anwenden
      inTx = num('tx', { cls: 'w52' }, refreshApply);
      inTy = num('ty', { cls: 'w52' }, refreshApply);
      inTz = num('tz', { cls: 'w52' }, refreshApply);
      inTpsi = num('tpsi', { cls: 'w52' }, refreshApply);
      applyBox = h('div');
      circleInfo = h('p', { class: 'hint' });
      el.appendChild(
        U.card(
          '4 · Anwenden',
          U.row(U.field('x', inTx), U.field('y', inTy), U.field('z', inTz), U.field('ψ', inTpsi)),
          U.row(
            U.btn('Aktuelle TCP-Pose', () => {
              const p = BS.kin.tcp(BS.sim.cmd);
              setTarget({ x: p.x, y: p.y, z: p.z, psi: p.psi });
            }),
            U.chk('mit DLS verfeinern', st.refine, (v) => {
              st.refine = v;
              save();
              refreshApply();
            }),
            U.field('Schritte', num('refineK', { step: 1, min: 1, cls: 'w44' }, refreshApply))
          ),
          applyBox,
          U.row(
            U.btn('NN-Lösung anfahren', goTo, 'primary'),
            U.chk('Arm folgt live', st.live, (v) => {
              st.live = v;
              save();
              refreshApply();
            })
          ),
          U.hint('Klick auf den Tisch setzt das Ziel (x, y). Der Geist zeigt die Pose des Netzes.'),
          h('h4', { style: { margin: '10px 0 4px', fontWeight: 600 } }, 'Kreisbahn mit dem Netz abfahren'),
          U.row(U.field('Mitte x', num('cx', { cls: 'w52' })), U.field('y', num('cy', { cls: 'w52' })), U.field('z', num('cz', { cls: 'w52' }))),
          U.row(U.field('Radius', num('cr', { cls: 'w52', min: 5 })), U.field('Punkte', num('cn', { cls: 'w44', min: 8 })), U.field('Dauer [s]', num('cT', { cls: 'w44', step: 1, min: 1 })), U.btn('Abfahren', runCircle)),
          circleInfo
        )
      );

      // 5 · Modell
      modelInfo = h('div');
      const file = h('input', { type: 'file', accept: '.json,application/json', style: { display: 'none' } });
      file.addEventListener('change', () => {
        const f = file.files[0];
        if (!f) return;
        f.text().then((t) => {
          try {
            loadModel(JSON.parse(t));
          } catch (e) {
            BS.toast('Datei ist kein gültiges JSON', 'err');
          }
        });
        file.value = '';
      });
      el.appendChild(
        U.card(
          '5 · Modell',
          modelInfo,
          U.row(
            U.btn('Im Browser speichern', saveBrowser),
            U.btn('Aus Browser laden', () => {
              const j = BS.store.get('mlModel', null);
              j ? loadModel(j) : BS.toast('Kein gespeichertes Modell', 'warn');
            })
          ),
          U.row(
            U.btn('Export JSON', () => model && BS.download('braccio_nn_ik.json', JSON.stringify(NN.toJSON(model)), 'application/json')),
            U.btn('Import JSON', () => file.click()),
            U.btn('Export Arduino-Header', () => model && BS.download('braccio_nn_ik.h', NN.toArduino(model)), '', 'C-Header mit Gewichten (PROGMEM) und Funktion nnIK() für den echten Arduino'),
            file
          ),
          U.hint('Der Arduino-Header enthält die Gewichte im Flash und eine Funktion <span class="mono">nnIK(x, y, z, ψ, m)</span>. Ein Uno hat 32 KiB Flash – kleine Netze (z. B. 32,32) passen neben die Braccio-Bibliothek.')
        )
      );

      const saved = BS.store.get('mlModel', null);
      if (saved) loadModel(saved, true);
      renderData();
      renderArch();
      renderEval();
      renderModel();
      renderButtons();
      refreshApply();
    },
    enter() {
      drawLoss();
      drawHist();
      showCloud();
      refreshApply();
    },
    leave() {
      BS.view.setCloud(null);
      BS.view.setIkTarget(null, null, false);
    },
    update() {
      if (run && run.last) {
        const m = run.last;
        const f = (m.epoch - run.e0) / Math.max(1, run.epochs - run.e0);
        progBar.style.width = (100 * f).toFixed(1) + '%';
        const n = data ? data.train.n : 0;
        trainInfo.textContent = `Epoche ${m.epoch}/${run.epochs} · Loss ${m.train.toExponential(2)} / ${m.val.toExponential(2)} · lr ${m.lr.toExponential(1)} · ${m.ms} ms/Epoche · ${fmtN(Math.round((n / Math.max(1, m.ms)) * 1000))} Beispiele/s${run.paused ? ' · pausiert' : ''}`;
      } else if (!run && hist.length) {
        progBar.style.width = '100%';
        const l = hist[hist.length - 1];
        trainInfo.textContent = `Fertig nach ${l.epoch} Epochen · Loss ${l.train.toExponential(2)} / ${l.val.toExponential(2)}`;
      }
      BS.view.ghostPose = applyRes ? full(pickPose()) : null;
    },
  });
})();
