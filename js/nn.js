/* Braccio-Simulator – Neuronales Netz für die inverse Kinematik
 *
 *  - Mehrschichtiges Perzeptron (MLP), vollständig in JavaScript: Vorwärtsrechnung, Backpropagation, Adam
 *  - Verlustfunktionen:
 *      Gelenkwinkel (überwacht)        L_q  = mean((q̂ − q)²)                      q normiert: (M − 90°)/90°
 *      Vorwärtskinematik (selbstüberw.) L_fk = mean(e²), e = [(f(q̂) − p)/s_p, w_ψ·(ψ(q̂) − ψ)]  mit Gradient Jᵀ·e
 *      Gelenkgrenzen (Strafterm)        L_g  = Σ relu(q̂ − q_max)² + relu(q_min − q̂)²
 *  - Training in einem Web Worker (aus einer Blob-URL, funktioniert auch bei file://), Fallback im Haupt-Thread
 *
 * Die Funktion core() ist in sich abgeschlossen (keine Abhängigkeiten), damit sie unverändert im Worker läuft.
 */
(function () {
  'use strict';
  const BS = window.BS;

  function core() {
    const D2R = Math.PI / 180;

    // ---------------- Zufallszahlen (reproduzierbar) ----------------
    function rng(seed) {
      let a = seed >>> 0 || 1;
      const next = () => {
        a = (a + 0x6d2b79f5) >>> 0;
        let t = a;
        t = Math.imul(t ^ (t >>> 15), t | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
      };
      next.gauss = () => {
        let u = 0;
        while (u === 0) u = next();
        return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * next());
      };
      return next;
    }

    // ---------------- Aktivierungsfunktionen (Ableitung aus dem Ausgang y) ----------------
    const ACT = {
      tanh: { f: (x) => Math.tanh(x), d: (y) => 1 - y * y },
      relu: { f: (x) => (x > 0 ? x : 0), d: (y) => (y > 0 ? 1 : 0) },
      leaky: { f: (x) => (x > 0 ? x : 0.01 * x), d: (y) => (y > 0 ? 1 : 0.01) },
    };

    /** Netz anlegen: sizes = [n_in, h1, …, n_out]; Xavier- (tanh) bzw. He-Initialisierung (ReLU) */
    function create(sizes, act, seed) {
      const r = rng(seed || 1);
      const W = [],
        b = [];
      for (let l = 0; l < sizes.length - 1; l++) {
        const nIn = sizes[l],
          nOut = sizes[l + 1];
        const last = l === sizes.length - 2;
        const std = Math.sqrt((act === 'tanh' || last ? 1 : 2) / nIn);
        const w = new Float64Array(nIn * nOut);
        for (let i = 0; i < w.length; i++) w[i] = r.gauss() * std;
        W.push(w);
        b.push(new Float64Array(nOut));
      }
      return { sizes: sizes.slice(), act, W, b };
    }

    function paramCount(net) {
      let n = 0;
      for (let l = 0; l < net.W.length; l++) n += net.W[l].length + net.b[l].length;
      return n;
    }

    /** Puffer für Aktivierungen je Schicht */
    function buffers(net) {
      return net.sizes.map((n) => new Float64Array(n));
    }

    /** Vorwärtsrechnung; A[0] = Eingang, A[L] = Ausgang (lineare Ausgangsschicht) */
    function forward(net, x, A) {
      const f = ACT[net.act].f,
        L = net.W.length;
      const a0 = A[0];
      for (let i = 0; i < a0.length; i++) a0[i] = x[i];
      for (let l = 0; l < L; l++) {
        const w = net.W[l],
          bb = net.b[l],
          inp = A[l],
          out = A[l + 1];
        const nIn = inp.length,
          nOut = out.length,
          last = l === L - 1;
        for (let j = 0; j < nOut; j++) {
          let s = bb[j];
          const o = j * nIn;
          for (let i = 0; i < nIn; i++) s += w[o + i] * inp[i];
          out[j] = last ? s : f(s);
        }
      }
      return A[L];
    }

    function predict(net, x) {
      return Float64Array.from(forward(net, x, buffers(net)));
    }

    /** Backpropagation: dOut = ∂L/∂Ausgang; Gradienten werden in gW/gb aufsummiert */
    function backward(net, A, dOut, gW, gb, Dl) {
      const df = ACT[net.act].d,
        L = net.W.length;
      Dl[L].set(dOut);
      for (let l = L - 1; l >= 0; l--) {
        const w = net.W[l],
          inp = A[l],
          d = Dl[l + 1],
          gw = gW[l],
          g = gb[l];
        const nIn = inp.length,
          nOut = d.length;
        for (let j = 0; j < nOut; j++) {
          const dj = d[j];
          g[j] += dj;
          const o = j * nIn;
          for (let i = 0; i < nIn; i++) gw[o + i] += dj * inp[i];
        }
        if (l > 0) {
          const dp = Dl[l];
          for (let i = 0; i < nIn; i++) {
            let s = 0;
            for (let j = 0; j < nOut; j++) s += w[j * nIn + i] * d[j];
            dp[i] = s * df(inp[i]);
          }
        }
      }
    }

    // ---------------- Vorwärtskinematik des Braccio (Nennmodell, vorne: ψ = e4) ----------------
    /** m in Grad (M1..M4) → {x,y,z,psi (Grad), J (4×4, je rad)} */
    function fk(m, g, withJ) {
      const c1 = Math.cos(m[0] * D2R),
        s1 = Math.sin(m[0] * D2R);
      const e2 = (180 - m[1]) * D2R,
        e3 = (270 - m[1] - m[2]) * D2R,
        e4 = (360 - m[1] - m[2] - m[3]) * D2R;
      const c2 = Math.cos(e2),
        s2 = Math.sin(e2),
        c3 = Math.cos(e3),
        s3 = Math.sin(e3),
        c4 = Math.cos(e4),
        s4 = Math.sin(e4);
      const r = g.a2 * c2 + g.a3 * c3 + g.lTcp * c4;
      const z = g.d1 + g.a2 * s2 + g.a3 * s3 + g.lTcp * s4;
      let psi = (360 - m[1] - m[2] - m[3]) % 360;
      if (psi > 180) psi -= 360;
      if (psi <= -180) psi += 360;
      const out = { x: r * c1, y: r * s1, z, psi };
      if (withJ) {
        const dr = [0, g.a2 * s2 + g.a3 * s3 + g.lTcp * s4, g.a3 * s3 + g.lTcp * s4, g.lTcp * s4];
        const dz = [0, -(g.a2 * c2 + g.a3 * c3 + g.lTcp * c4), -(g.a3 * c3 + g.lTcp * c4), -g.lTcp * c4];
        out.J = [
          [-r * s1, c1 * dr[1], c1 * dr[2], c1 * dr[3]],
          [r * c1, s1 * dr[1], s1 * dr[2], s1 * dr[3]],
          [0, dz[1], dz[2], dz[3]],
          [0, -1, -1, -1],
        ];
      }
      return out;
    }
    const wrap180 = (a) => ((((a + 180) % 360) + 360) % 360) - 180;

    /**
     * Verlust und Gradient für ein Beispiel.
     * o: Netzausgang (normiert), y: Soll-Gelenkwinkel (normiert) oder null, p: Zielpose [x,y,z,ψ°]
     * cfg: {wJ, wF, wL, sP (mm), wPsi, geom, lo, hi (normiert)}
     */
    function lossGrad(o, y, p, cfg, d) {
      let loss = 0;
      const n = o.length;
      for (let i = 0; i < n; i++) d[i] = 0;
      if (cfg.wJ > 0 && y) {
        for (let i = 0; i < n; i++) {
          const e = o[i] - y[i];
          loss += (cfg.wJ * e * e) / n;
          d[i] += (cfg.wJ * 2 * e) / n;
        }
      }
      if (cfg.wF > 0) {
        const m = [90 + 90 * o[0], 90 + 90 * o[1], 90 + 90 * o[2], 90 + 90 * o[3]];
        const f = fk(m, cfg.geom, true);
        const e = [(f.x - p[0]) / cfg.sP, (f.y - p[1]) / cfg.sP, (f.z - p[2]) / cfg.sP, cfg.wPsi * wrap180(f.psi - p[3]) * D2R];
        const sc = [1 / cfg.sP, 1 / cfg.sP, 1 / cfg.sP, cfg.wPsi];
        for (let k = 0; k < 4; k++) loss += (cfg.wF * e[k] * e[k]) / 4;
        // ∂L/∂o_i = Σ_k 2 e_k ∂e_k/∂m_i · ∂m_i/∂o_i,  ∂m_i/∂o_i = 90° = π/2 rad
        for (let i = 0; i < 4; i++) {
          let s = 0;
          for (let k = 0; k < 4; k++) s += 2 * e[k] * f.J[k][i] * sc[k];
          d[i] += (cfg.wF * s * (Math.PI / 2)) / 4;
        }
      }
      if (cfg.wL > 0) {
        for (let i = 0; i < n; i++) {
          const a = o[i] - cfg.hi[i],
            b = cfg.lo[i] - o[i];
          if (a > 0) {
            loss += cfg.wL * a * a;
            d[i] += cfg.wL * 2 * a;
          } else if (b > 0) {
            loss += cfg.wL * b * b;
            d[i] -= cfg.wL * 2 * b;
          }
        }
      }
      return loss;
    }

    /** Trainer mit Adam (β1 = 0,9, β2 = 0,999) und Mini-Batches */
    function trainer(net, seed) {
      const zeros = (arr) => arr.map((a) => new Float64Array(a.length));
      const st = {
        mW: zeros(net.W),
        vW: zeros(net.W),
        mb: zeros(net.b),
        vb: zeros(net.b),
        gW: zeros(net.W),
        gb: zeros(net.b),
        A: buffers(net),
        Dl: buffers(net),
        dOut: new Float64Array(net.sizes[net.sizes.length - 1]),
        t: 0,
        rand: rng((seed || 1) * 7919 + 13),
      };
      function adam(lr, B, l2) {
        st.t++;
        const b1 = 0.9,
          b2 = 0.999,
          eps = 1e-8;
        const c1 = 1 - Math.pow(b1, st.t),
          c2 = 1 - Math.pow(b2, st.t);
        const upd = (P, G, M, V, decay) => {
          for (let i = 0; i < P.length; i++) {
            const g = G[i] / B + (decay ? l2 * P[i] : 0);
            M[i] = b1 * M[i] + (1 - b1) * g;
            V[i] = b2 * V[i] + (1 - b2) * g * g;
            P[i] -= (lr * (M[i] / c1)) / (Math.sqrt(V[i] / c2) + eps);
            G[i] = 0;
          }
        };
        for (let l = 0; l < net.W.length; l++) {
          upd(net.W[l], st.gW[l], st.mW[l], st.vW[l], true);
          upd(net.b[l], st.gb[l], st.mb[l], st.vb[l], false);
        }
      }
      /** eine Epoche; D = {X, Y, P, n, nIn, nOut} (Float32Arrays, zeilenweise) */
      function epoch(Dt, cfg, lr, batch, l2) {
        const n = Dt.n,
          nIn = Dt.nIn,
          nOut = Dt.nOut;
        const idx = new Uint32Array(n);
        for (let i = 0; i < n; i++) idx[i] = i;
        for (let i = n - 1; i > 0; i--) {
          const j = Math.floor(st.rand() * (i + 1));
          const t = idx[i];
          idx[i] = idx[j];
          idx[j] = t;
        }
        const x = new Float64Array(nIn),
          y = new Float64Array(nOut),
          p = new Float64Array(4);
        let sum = 0,
          inB = 0;
        for (let k = 0; k < n; k++) {
          const s = idx[k];
          for (let i = 0; i < nIn; i++) x[i] = Dt.X[s * nIn + i];
          for (let i = 0; i < nOut; i++) y[i] = Dt.Y[s * nOut + i];
          for (let i = 0; i < 4; i++) p[i] = Dt.P[s * 4 + i];
          const o = forward(net, x, st.A);
          sum += lossGrad(o, y, p, cfg, st.dOut);
          backward(net, st.A, st.dOut, st.gW, st.gb, st.Dl);
          if (++inB === batch || k === n - 1) {
            adam(lr, inB, l2);
            inB = 0;
          }
        }
        return sum / n;
      }
      return { epoch, st };
    }

    /** mittlerer Verlust ohne Training */
    function evaluate(net, Dt, cfg) {
      const A = buffers(net),
        d = new Float64Array(Dt.nOut);
      const x = new Float64Array(Dt.nIn),
        y = new Float64Array(Dt.nOut),
        p = new Float64Array(4);
      let sum = 0;
      for (let s = 0; s < Dt.n; s++) {
        for (let i = 0; i < Dt.nIn; i++) x[i] = Dt.X[s * Dt.nIn + i];
        for (let i = 0; i < Dt.nOut; i++) y[i] = Dt.Y[s * Dt.nOut + i];
        for (let i = 0; i < 4; i++) p[i] = Dt.P[s * 4 + i];
        sum += lossGrad(forward(net, x, A), y, p, cfg, d);
      }
      return Dt.n ? sum / Dt.n : NaN;
    }

    /** Lernrate: konstant oder Kosinus-Abfall auf 5 % (über die Epochen e0 … epochs dieser Sitzung) */
    function lrAt(o, ep) {
      if (o.schedule !== 'cosine') return o.lr;
      const e0 = o.e0 || 0;
      const f = Math.min(1, (ep - e0) / Math.max(1, o.epochs - e0));
      return o.lr * (0.05 + 0.95 * 0.5 * (1 + Math.cos(Math.PI * f)));
    }

    // ---------------- Trainingsschleife (Worker oder Haupt-Thread) ----------------
    /** Startet eine Trainingssitzung; post(msg) meldet Fortschritt; liefert {stop, pause, resume} */
    function session(msg, post, schedule) {
      const net = msg.net;
      const tr = trainer(net, msg.opt.seed);
      const o = msg.opt;
      let ep = msg.startEpoch || 0,
        stopped = false,
        paused = false;
      const t0 = Date.now();
      function step() {
        if (stopped) return;
        if (paused) return;
        const ts = Date.now();
        const lr = lrAt(o, ep);
        const lt = tr.epoch(msg.train, msg.cfg, lr, o.batch, o.l2);
        ep++;
        const lv = evaluate(net, msg.val, msg.cfg);
        const done = ep >= o.epochs;
        post({ type: 'epoch', epoch: ep, train: lt, val: lv, lr, ms: Date.now() - ts, total: Date.now() - t0 });
        if (done || ep % o.snapEvery === 0) post({ type: 'snapshot', epoch: ep, W: net.W, b: net.b });
        if (done) {
          post({ type: 'done', epoch: ep });
          return;
        }
        schedule(step);
      }
      schedule(step);
      return {
        stop() {
          stopped = true;
          post({ type: 'snapshot', epoch: ep, W: net.W, b: net.b });
          post({ type: 'stopped', epoch: ep });
        },
        pause() {
          paused = true;
        },
        resume() {
          if (!paused) return;
          paused = false;
          schedule(step);
        },
      };
    }

    return { rng, ACT, create, paramCount, buffers, forward, backward, predict, fk, wrap180, lossGrad, trainer, evaluate, lrAt, session };
  }

  // ------------------------------------------------------------------
  // Worker aus Blob-URL
  // ------------------------------------------------------------------
  function workerSource() {
    const C = core();
    let ses = null;
    self.onmessage = (e) => {
      const m = e.data;
      if (m.cmd === 'train') {
        if (ses) ses.stop();
        ses = C.session(m, (msg) => self.postMessage(msg), (fn) => setTimeout(fn, 0));
      } else if (ses && m.cmd === 'stop') {
        ses.stop();
        ses = null;
      } else if (ses && m.cmd === 'pause') ses.pause();
      else if (ses && m.cmd === 'resume') ses.resume();
    };
  }

  const NN = (BS.nn = { core: core() });

  /** Trainingsprozess starten; onMsg erhält epoch/snapshot/done/stopped-Meldungen */
  NN.startTraining = function (msg, onMsg) {
    let worker = null;
    try {
      const src = `const core = ${core.toString()};\n(${workerSource.toString()})();`;
      const url = URL.createObjectURL(new Blob([src], { type: 'text/javascript' }));
      worker = new Worker(url);
      URL.revokeObjectURL(url);
    } catch (e) {
      worker = null;
    }
    if (worker) {
      worker.onmessage = (e) => onMsg(e.data);
      worker.onerror = (e) => onMsg({ type: 'error', message: e.message || 'Worker-Fehler' });
      worker.postMessage(Object.assign({ cmd: 'train' }, msg));
      return {
        worker: true,
        stop() {
          worker.postMessage({ cmd: 'stop' });
          setTimeout(() => worker.terminate(), 2000);
        },
        pause: () => worker.postMessage({ cmd: 'pause' }),
        resume: () => worker.postMessage({ cmd: 'resume' }),
        kill: () => worker.terminate(),
      };
    }
    // Fallback: im Haupt-Thread (Epoche für Epoche, Oberfläche bleibt bedienbar)
    const ses = NN.core.session(msg, onMsg, (fn) => setTimeout(fn, 0));
    return { worker: false, stop: () => ses.stop(), pause: () => ses.pause(), resume: () => ses.resume(), kill: () => ses.stop() };
  };

  // ------------------------------------------------------------------
  // Merkmale und Inferenz
  // ------------------------------------------------------------------
  /** Rohmerkmale einer Zielpose (x, y, z in mm, ψ in Grad) */
  NN.features = function (feat, x, y, z, psi) {
    const a = psi * (Math.PI / 180);
    if (feat === 'cyl') {
      const r = Math.hypot(x, y),
        phi = Math.atan2(y, x);
      return [r, Math.cos(phi), Math.sin(phi), z, Math.cos(a), Math.sin(a)];
    }
    return [x, y, z, Math.cos(a), Math.sin(a)];
  };
  NN.featureNames = { cart: ['x', 'y', 'z', 'cos ψ', 'sin ψ'], cyl: ['r', 'cos φ', 'sin φ', 'z', 'cos ψ', 'sin ψ'] };

  /** Gelenkwinkel M1..M4 (Grad) für eine Zielpose aus einem trainierten Modell */
  NN.ik = function (model, x, y, z, psi) {
    const f = NN.features(model.meta.feat, x, y, z, psi);
    const xn = f.map((v, i) => (v - model.meta.mu[i]) / model.meta.sd[i]);
    const o = NN.core.forward(model.net, xn, model.buf || (model.buf = NN.core.buffers(model.net)));
    return [90 + 90 * o[0], 90 + 90 * o[1], 90 + 90 * o[2], 90 + 90 * o[3]];
  };

  // ------------------------------------------------------------------
  // Serialisierung
  // ------------------------------------------------------------------
  NN.toJSON = function (model) {
    return {
      format: 'braccio-nn-ik/1',
      sizes: model.net.sizes,
      act: model.net.act,
      W: model.net.W.map((w) => Array.from(w, (v) => +v.toPrecision(8))),
      b: model.net.b.map((w) => Array.from(w, (v) => +v.toPrecision(8))),
      meta: model.meta,
    };
  };
  NN.fromJSON = function (j) {
    if (!j || j.format !== 'braccio-nn-ik/1') throw new Error('Unbekanntes Modellformat');
    const net = { sizes: j.sizes, act: j.act, W: j.W.map((w) => Float64Array.from(w)), b: j.b.map((w) => Float64Array.from(w)) };
    return { net, meta: j.meta };
  };

  /** Arduino-Header mit Gewichten im Flash (PROGMEM) und Inferenzfunktion */
  NN.toArduino = function (model) {
    const net = model.net,
      meta = model.meta;
    const L = net.W.length;
    const arr = (name, a) => {
      const vals = Array.from(a, (v) => v.toPrecision(7) + 'f');
      const lines = [];
      for (let i = 0; i < vals.length; i += 8) lines.push('  ' + vals.slice(i, i + 8).join(', '));
      return `const float ${name}[${a.length}] PROGMEM = {\n${lines.join(',\n')}\n};\n`;
    };
    const act = net.act === 'tanh' ? 'tanh(s)' : net.act === 'relu' ? '(s > 0 ? s : 0)' : '(s > 0 ? s : 0.01f * s)';
    const maxW = Math.max(...net.sizes);
    let s = `/* Neuronales Netz für die inverse Kinematik des Braccio – erzeugt vom Braccio-Simulator
 * Architektur: ${net.sizes.join(' → ')} (${net.act}), Merkmale: ${meta.feat === 'cyl' ? 'r, cos φ, sin φ, z, cos ψ, sin ψ' : 'x, y, z, cos ψ, sin ψ'}
 * Konfiguration: vorne, Ellbogen ${meta.elbow === 'up' ? 'oben' : meta.elbow === 'down' ? 'unten' : 'beliebig'}, ψ ∈ [${meta.psiRange.join(', ')}]°
 * Validierung: mittlerer Positionsfehler ${meta.eval ? meta.eval.mean.toFixed(2) + ' mm' : '–'}
 * Speicherbedarf der Gewichte: ${(paramBytes(net) / 1024).toFixed(1)} KiB Flash.
 *
 * Verwendung:  float m[4];  if (nnIK(x, y, z, psi, m)) Braccio.ServoMovement(20, m[0], m[1], m[2], m[3], 90, 73);
 */
#pragma once
#include <avr/pgmspace.h>
#include <math.h>
#ifndef PI
#define PI 3.14159265358979f
#endif

`;
    s += `const int NN_IN = ${net.sizes[0]};\n`;
    s += arr('NN_MU', Float64Array.from(meta.mu)) + arr('NN_SD', Float64Array.from(meta.sd));
    for (let l = 0; l < L; l++) s += arr(`NN_W${l}`, net.W[l]) + arr(`NN_B${l}`, net.b[l]);
    s += `
static void nnLayer(const float *W, const float *B, const float *in, int nIn, float *out, int nOut, bool act) {
  for (int j = 0; j < nOut; j++) {
    float s = pgm_read_float(&B[j]);
    for (int i = 0; i < nIn; i++) s += pgm_read_float(&W[j * nIn + i]) * in[i];
    out[j] = act ? ${act} : s;
  }
}

/* x, y, z in mm, psi in Grad → m[0..3] = M1..M4 in Grad; false, wenn eine Gelenkgrenze verletzt wäre */
bool nnIK(float x, float y, float z, float psi, float m[4]) {
  float a[${maxW}], b[${maxW}];
  float ps = psi * (PI / 180.0f);
`;
    s +=
      meta.feat === 'cyl'
        ? `  float r = sqrt(x * x + y * y), phi = atan2(y, x);\n  float f[6] = { r, cos(phi), sin(phi), z, cos(ps), sin(ps) };\n`
        : `  float f[5] = { x, y, z, cos(ps), sin(ps) };\n`;
    s += `  for (int i = 0; i < NN_IN; i++) a[i] = (f[i] - pgm_read_float(&NN_MU[i])) / pgm_read_float(&NN_SD[i]);\n`;
    for (let l = 0; l < L; l++) {
      const src = l % 2 === 0 ? 'a' : 'b',
        dst = l % 2 === 0 ? 'b' : 'a';
      s += `  nnLayer(NN_W${l}, NN_B${l}, ${src}, ${net.sizes[l]}, ${dst}, ${net.sizes[l + 1]}, ${l < L - 1 ? 'true' : 'false'});\n`;
    }
    const outv = L % 2 === 1 ? 'b' : 'a';
    s += `  const float lo[4] = { 0, 15, 0, 0 }, hi[4] = { 180, 165, 180, 180 };
  bool ok = true;
  for (int i = 0; i < 4; i++) {
    m[i] = 90.0f + 90.0f * ${outv}[i];
    if (m[i] < lo[i] || m[i] > hi[i]) ok = false;
  }
  return ok;
}
`;
    return s;
  };
  function paramBytes(net) {
    return (NN.core.paramCount(net) + 2 * net.sizes[0]) * 4;
  }
  NN.paramBytes = paramBytes;
})();
