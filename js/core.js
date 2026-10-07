/* Braccio-Simulator – Grundfunktionen, Konfiguration, Hilfsfunktionen, Plot */
(function () {
  'use strict';
  const BS = (window.BS = window.BS || {});

  BS.D2R = Math.PI / 180;
  BS.R2D = 180 / Math.PI;
  BS.clamp = (v, a, b) => (v < a ? a : v > b ? b : v);
  BS.wrap180 = (a) => ((((a + 180) % 360) + 360) % 360) - 180;
  /** Winkel in das Fenster (-90, 270] legen – passend zu Servo-Bereichen 0..180 */
  BS.normServo = (a) => {
    a = ((a % 360) + 360) % 360;
    if (a > 270) a -= 360;
    return a;
  };

  // ------------------------------------------------------------------
  // Konfiguration (Braccio / TinkerKit, klassische Bibliothek Braccio.h)
  // ------------------------------------------------------------------
  BS.config = {
    // Geometrie in mm (editierbar im Tab "Modell")
    geom: {
      d1: 71.5, // Tisch -> Schulterachse
      a2: 125.0, // Schulter -> Ellbogen
      a3: 125.0, // Ellbogen -> Handgelenk
      lTcp: 175.0, // Handgelenk -> TCP (Greifmitte)
      lTip: 192.5, // Handgelenk -> Fingerspitze
    },
    // Grenzen wie in Braccio::ServoMovement()
    limits: [
      [0, 180],
      [15, 165],
      [0, 180],
      [0, 180],
      [0, 180],
      [10, 73],
    ],
    names: ['M1 Basis', 'M2 Schulter', 'M3 Ellbogen', 'M4 Handgelenk', 'M5 Handrotation', 'M6 Greifer'],
    short: ['M1', 'M2', 'M3', 'M4', 'M5', 'M6'],
    libNames: ['base', 'shoulder', 'elbow', 'wrist_ver', 'wrist_rot', 'gripper'],
    // Pins des Braccio-Shields
    pins: { 11: 0, 10: 1, 9: 2, 5: 3, 6: 4, 3: 5 },
    colors: ['#ff6b6b', '#ffa94d', '#ffd43b', '#69db7c', '#4dabf7', '#b197fc'],
    // Servo-Modell: Lageregelung mit PT1-Verhalten und Geschwindigkeitsbegrenzung
    servo: {
      vmax: [300, 250, 300, 400, 400, 400], // °/s
      tau: 0.04, // s
    },
    gripper: { wMax: 70, closed: 73, open: 10 }, // Öffnung in mm bei M6=10
    // Statik
    masses: { upper: 0.12, fore: 0.08, hand: 0.11 }, // kg
    stall: [1.2, 1.2, 1.2, 0.3, 0.3, 0.3], // N·m (SR431: M1–M3, SR311: M4–M6)
    servoType: ['SR431', 'SR431', 'SR431', 'SR311', 'SR311', 'SR311'],
    home: [90, 90, 90, 90, 90, 73],
    beginPose: [0, 40, 180, 170, 0, 73], // Pose nach Braccio.begin()
  };

  // ------------------------------------------------------------------
  // Events
  // ------------------------------------------------------------------
  const listeners = {};
  BS.on = (name, fn) => (listeners[name] = listeners[name] || []).push(fn);
  BS.emit = (name, data) => (listeners[name] || []).forEach((fn) => fn(data));

  // ------------------------------------------------------------------
  // Persistenz (localStorage, immer abgesichert)
  // ------------------------------------------------------------------
  const PREFIX = 'braccio-sim:';
  BS.store = {
    get(key, def) {
      try {
        const v = localStorage.getItem(PREFIX + key);
        return v == null ? def : JSON.parse(v);
      } catch (e) {
        return def;
      }
    },
    set(key, val) {
      try {
        localStorage.setItem(PREFIX + key, JSON.stringify(val));
      } catch (e) {
        /* ignorieren */
      }
    },
  };

  // ------------------------------------------------------------------
  // DOM-Helfer
  // ------------------------------------------------------------------
  BS.h = function (tag, attrs, ...children) {
    const el = document.createElement(tag);
    if (attrs) {
      for (const k in attrs) {
        const v = attrs[k];
        if (v == null || v === false) continue;
        if (k === 'class') el.className = v;
        else if (k === 'style' && typeof v === 'object') Object.assign(el.style, v);
        else if (k.startsWith('on') && typeof v === 'function') el.addEventListener(k.slice(2), v);
        else if (k === 'html') el.innerHTML = v;
        else if (k in el && k !== 'list' && typeof v !== 'string') el[k] = v;
        else el.setAttribute(k, v === true ? '' : v);
      }
    }
    for (const c of children.flat(Infinity)) {
      if (c == null || c === false) continue;
      el.appendChild(typeof c === 'string' || typeof c === 'number' ? document.createTextNode(String(c)) : c);
    }
    return el;
  };

  BS.download = function (filename, text, type) {
    const blob = new Blob([text], { type: type || 'text/plain;charset=utf-8' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    setTimeout(() => {
      URL.revokeObjectURL(a.href);
      a.remove();
    }, 500);
  };

  BS.fmt = (v, d = 1) => (Number.isFinite(v) ? v.toFixed(d) : '–');

  let toastTimer = null;
  BS.toast = function (msg, kind) {
    let el = document.getElementById('toast');
    if (!el) return;
    el.textContent = msg;
    el.className = 'toast show ' + (kind || '');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => (el.className = 'toast'), 2600);
  };

  // ------------------------------------------------------------------
  // Kleine lineare Algebra
  // ------------------------------------------------------------------
  BS.la = {
    /** Löst A x = b (A: n×n als Array von Zeilen) mit Gauß-Elimination */
    solve(A, b) {
      const n = A.length;
      const M = A.map((r, i) => r.slice().concat([b[i]]));
      for (let c = 0; c < n; c++) {
        let p = c;
        for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
        if (Math.abs(M[p][c]) < 1e-14) return null;
        [M[c], M[p]] = [M[p], M[c]];
        for (let r = 0; r < n; r++) {
          if (r === c) continue;
          const f = M[r][c] / M[c][c];
          for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
        }
      }
      return M.map((r, i) => r[n] / r[i]);
    },
    T(A) {
      return A[0].map((_, j) => A.map((r) => r[j]));
    },
    mul(A, B) {
      return A.map((r) => B[0].map((_, j) => r.reduce((s, v, k) => s + v * B[k][j], 0)));
    },
    mulVec(A, x) {
      return A.map((r) => r.reduce((s, v, k) => s + v * x[k], 0));
    },
    det(A) {
      const n = A.length;
      const M = A.map((r) => r.slice());
      let d = 1;
      for (let c = 0; c < n; c++) {
        let p = c;
        for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
        if (Math.abs(M[p][c]) < 1e-300) return 0;
        if (p !== c) {
          [M[c], M[p]] = [M[p], M[c]];
          d = -d;
        }
        d *= M[c][c];
        for (let r = c + 1; r < n; r++) {
          const f = M[r][c] / M[c][c];
          for (let k = c; k < n; k++) M[r][k] -= f * M[c][k];
        }
      }
      return d;
    },
    /** Eigenwerte einer symmetrischen Matrix (Jacobi-Rotationen) */
    eigSym(A) {
      const n = A.length;
      const M = A.map((r) => r.slice());
      for (let sweep = 0; sweep < 60; sweep++) {
        let off = 0;
        for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) off += M[i][j] * M[i][j];
        if (off < 1e-18) break;
        for (let p = 0; p < n; p++)
          for (let q = p + 1; q < n; q++) {
            if (Math.abs(M[p][q]) < 1e-300) continue;
            const th = (M[q][q] - M[p][p]) / (2 * M[p][q]);
            const t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1));
            const c = 1 / Math.sqrt(t * t + 1),
              s = t * c;
            for (let k = 0; k < n; k++) {
              const a = M[k][p],
                b = M[k][q];
              M[k][p] = c * a - s * b;
              M[k][q] = s * a + c * b;
            }
            for (let k = 0; k < n; k++) {
              const a = M[p][k],
                b = M[q][k];
              M[p][k] = c * a - s * b;
              M[q][k] = s * a + c * b;
            }
          }
      }
      return M.map((r, i) => r[i]).sort((a, b) => b - a);
    },
  };

  // ------------------------------------------------------------------
  // Einfacher Linien-Plot (Canvas)
  // ------------------------------------------------------------------
  function niceStep(range, target) {
    const raw = range / Math.max(1, target);
    const mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const n = raw / mag;
    return (n < 1.5 ? 1 : n < 3 ? 2 : n < 7 ? 5 : 10) * mag;
  }

  class Plot {
    constructor(canvas) {
      this.c = canvas;
      this.ctx = canvas.getContext('2d');
    }
    draw(o) {
      const c = this.c,
        ctx = this.ctx;
      const dpr = window.devicePixelRatio || 1;
      const W = c.clientWidth,
        H = c.clientHeight;
      if (!W || !H) return;
      if (c.width !== Math.round(W * dpr) || c.height !== Math.round(H * dpr)) {
        c.width = Math.round(W * dpr);
        c.height = Math.round(H * dpr);
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, W, H);
      const css = getComputedStyle(document.documentElement);
      const colText = css.getPropertyValue('--muted').trim() || '#9aa';
      const colGrid = css.getPropertyValue('--grid').trim() || '#333';
      const L = 48,
        R = 8,
        T = o.title ? 18 : 8,
        B = o.xLabel ? 30 : 18;
      const pw = W - L - R,
        ph = H - T - B;
      const series = (o.series || []).filter((s) => s.data && s.data.length);
      let [x0, x1] = o.xRange || [Infinity, -Infinity];
      let [y0, y1] = o.yRange || [Infinity, -Infinity];
      const tf = o.logY ? (v) => Math.log10(Math.max(v, 1e-12)) : (v) => v;
      if (!o.xRange || !o.yRange) {
        for (const s of series)
          for (const p of s.data) {
            if (!o.xRange) {
              if (p[0] < x0) x0 = p[0];
              if (p[0] > x1) x1 = p[0];
            }
            if (!o.yRange && Number.isFinite(p[1])) {
              const v = tf(p[1]);
              if (v < y0) y0 = v;
              if (v > y1) y1 = v;
            }
          }
      } else if (o.logY) {
        y0 = tf(y0);
        y1 = tf(y1);
      }
      if (!Number.isFinite(x0)) (x0 = 0), (x1 = 1);
      if (!Number.isFinite(y0)) (y0 = 0), (y1 = 1);
      if (x1 - x0 < 1e-9) x1 = x0 + 1;
      if (y1 - y0 < 1e-9) {
        y0 -= 1;
        y1 += 1;
      } else if (!o.yRange) {
        const pad = (y1 - y0) * 0.08;
        y0 -= pad;
        y1 += pad;
      }
      const X = (x) => L + ((x - x0) / (x1 - x0)) * pw;
      const Y = (y) => T + ph - ((tf(y) - y0) / (y1 - y0)) * ph;
      const Yr = (v) => T + ph - ((v - y0) / (y1 - y0)) * ph;
      ctx.font = '10px ui-monospace, SFMono-Regular, Menlo, monospace';
      ctx.lineWidth = 1;
      // Gitter
      ctx.strokeStyle = colGrid;
      ctx.fillStyle = colText;
      const xs = niceStep(x1 - x0, Math.max(2, pw / 70));
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      for (let x = Math.ceil(x0 / xs) * xs; x <= x1 + 1e-9; x += xs) {
        ctx.beginPath();
        ctx.moveTo(X(x) + 0.5, T);
        ctx.lineTo(X(x) + 0.5, T + ph);
        ctx.stroke();
        ctx.fillText(+x.toFixed(6) + '', X(x), T + ph + 3);
      }
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';
      if (o.logY) {
        for (let e = Math.ceil(y0); e <= y1; e++) {
          ctx.beginPath();
          ctx.moveTo(L, Yr(e) + 0.5);
          ctx.lineTo(L + pw, Yr(e) + 0.5);
          ctx.stroke();
          ctx.fillText('1e' + e, L - 4, Yr(e));
        }
      } else {
        const ys = niceStep(y1 - y0, Math.max(2, ph / 28));
        for (let y = Math.ceil(y0 / ys) * ys; y <= y1 + 1e-9; y += ys) {
          ctx.beginPath();
          ctx.moveTo(L, Yr(y) + 0.5);
          ctx.lineTo(L + pw, Yr(y) + 0.5);
          ctx.stroke();
          ctx.fillText(+y.toFixed(6) + '', L - 4, Yr(y));
        }
      }
      if (o.title) {
        ctx.textAlign = 'left';
        ctx.textBaseline = 'top';
        ctx.fillText(o.title, L, 3);
      }
      if (o.xLabel) {
        ctx.textAlign = 'right';
        ctx.textBaseline = 'bottom';
        ctx.fillText(o.xLabel, L + pw, H - 1);
      }
      // Linien
      ctx.save();
      ctx.beginPath();
      ctx.rect(L, T, pw, ph);
      ctx.clip();
      for (const s of series) {
        ctx.strokeStyle = s.color || '#fff';
        ctx.lineWidth = s.width || 1.5;
        ctx.setLineDash(s.dash || []);
        ctx.beginPath();
        let pen = false;
        for (const p of s.data) {
          if (!Number.isFinite(p[1])) {
            pen = false;
            continue;
          }
          const px = X(p[0]),
            py = Y(p[1]);
          if (!pen) ctx.moveTo(px, py);
          else ctx.lineTo(px, py);
          pen = true;
        }
        ctx.stroke();
      }
      ctx.setLineDash([]);
      if (o.hlines)
        for (const hl of o.hlines) {
          ctx.strokeStyle = hl.color;
          ctx.setLineDash([4, 4]);
          ctx.beginPath();
          ctx.moveTo(L, Y(hl.y));
          ctx.lineTo(L + pw, Y(hl.y));
          ctx.stroke();
          ctx.setLineDash([]);
        }
      if (o.cursor != null && o.cursor >= x0 && o.cursor <= x1) {
        ctx.strokeStyle = '#fff';
        ctx.globalAlpha = 0.6;
        ctx.beginPath();
        ctx.moveTo(X(o.cursor), T);
        ctx.lineTo(X(o.cursor), T + ph);
        ctx.stroke();
        ctx.globalAlpha = 1;
      }
      ctx.restore();
      ctx.strokeStyle = colGrid;
      ctx.strokeRect(L + 0.5, T + 0.5, pw, ph);
      // Legende
      if (o.legend !== false && series.some((s) => s.name)) {
        ctx.textAlign = 'left';
        ctx.textBaseline = 'middle';
        let lx = L + 6;
        for (const s of series) {
          if (!s.name || s.noLegend) continue;
          const w = ctx.measureText(s.name).width;
          if (lx + w + 20 > L + pw) break;
          ctx.fillStyle = s.color;
          ctx.fillRect(lx, T + 7, 10, 2);
          ctx.fillStyle = colText;
          ctx.fillText(s.name, lx + 13, T + 8);
          lx += w + 24;
        }
      }
      this.map = { X, Y, x0, x1, y0, y1, L, T, pw, ph };
    }
  }
  BS.Plot = Plot;
})();
