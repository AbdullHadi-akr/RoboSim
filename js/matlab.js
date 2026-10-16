/* Braccio-Simulator – MATLAB-Interpreter (Teilmenge)
 *
 * Führt die MATLAB-Dateien der Laborübung 2 im Browser aus: das Ini-Skript Braccio_kinematics.m,
 * die MATLAB-Function-Blöcke des Simulink-Modells und Eingaben im Befehlsfenster.
 *
 * Sprachumfang:
 *   Skalare, Matrizen (double/logical), Zeichenketten, Strukturen, einfache Zellen, Handle-Objekte
 *   Operatoren + - * / \ ^ .* ./ .\ .^ ' .' == ~= < <= > >= & | && || ~ :
 *   Indizierung mit (), {}, end, :, logischen Masken; Zuweisung mit automatischem Vergrößern
 *   if/elseif/else, for, while, switch, try/catch, break/continue/return
 *   Funktionen (auch lokale Funktionen in Skripten), persistent, Mehrfachrückgabe, ~ als Platzhalter
 *   Befehlssyntax (close all, grid on, axis equal …), Name=Wert-Argumente
 * Bibliothek: Rechenfunktionen, Lineare Algebra, Zeichenketten, Robotik (rigidBodyTree, trvec2tform,
 *   rotm2eul, collisionBox …), einfache Plots (figure, subplot, plot …).
 */
(function () {
  'use strict';
  const BS = (typeof window !== 'undefined' ? window : globalThis).BS;

  class MLError extends Error {
    constructor(msg, line) {
      super(msg);
      this.line = line;
    }
  }

  // ==================================================================
  // Werte
  // ==================================================================
  class Mat {
    constructor(r, c, d, logical) {
      this.r = r;
      this.c = c;
      this.d = d || new Float64Array(r * c);
      this.logical = !!logical;
    }
  }
  class Str {
    constructor(s, dq) {
      this.s = s;
      this.dq = !!dq;
    }
  }
  class Struct {
    constructor(f) {
      this.f = f || new Map();
    }
    clone() {
      return new Struct(new Map(this.f));
    }
  }
  class Cell {
    constructor(r, c, d) {
      this.r = r;
      this.c = c;
      this.d = d || new Array(r * c).fill(null).map(() => EMPTY);
    }
  }
  /** Handle-Objekt (Referenzsemantik wie MATLAB-Handle-Klassen) */
  class Obj {
    constructor(cls, props) {
      this.cls = cls;
      this.p = new Map(Object.entries(props || {}));
    }
    has(name) {
      return this.p.has(name);
    }
    get(name) {
      if (!this.p.has(name)) throw new MLError(`Unbekannte Eigenschaft '${name}' für Klasse '${this.cls}'.`);
      return this.p.get(name);
    }
    set(name, v) {
      if (!this.p.has(name)) throw new MLError(`Unbekannte Eigenschaft '${name}' für Klasse '${this.cls}'.`);
      this.p.set(name, v);
    }
  }
  /** Grafik-Handle: jede Eigenschaft ist les- und schreibbar */
  class GObj extends Obj {
    get(name) {
      if (!this.p.has(name)) this.p.set(name, new GObj('matlab.graphics.Graphics'));
      return this.p.get(name);
    }
    set(name, v) {
      this.p.set(name, v);
    }
  }

  const EMPTY = new Mat(0, 0);
  const COLON = { colon: true };
  const isNum = (v) => typeof v === 'number';

  function mat(r, c, arr, logical) {
    return new Mat(r, c, arr instanceof Float64Array ? arr : Float64Array.from(arr), logical);
  }
  const rowVec = (arr) => norm(mat(1, arr.length, arr));
  const colVec = (arr) => norm(mat(arr.length, 1, arr));
  function fromRows(rows) {
    const r = rows.length,
      c = r ? rows[0].length : 0;
    const m = new Mat(r, c);
    for (let i = 0; i < r; i++) for (let j = 0; j < c; j++) m.d[j * r + i] = rows[i][j];
    return m;
  }
  /** 1×1-Matrizen werden zu Zahlen */
  function norm(v) {
    if (v instanceof Mat && v.r === 1 && v.c === 1) return v.d[0];
    return v;
  }
  function toMat(v) {
    if (typeof v === 'number') return new Mat(1, 1, Float64Array.of(v));
    if (typeof v === 'boolean') return new Mat(1, 1, Float64Array.of(v ? 1 : 0), true);
    if (v instanceof Mat) return v;
    if (v instanceof Str) {
      const m = new Mat(v.s.length ? 1 : 0, v.s.length);
      for (let i = 0; i < v.s.length; i++) m.d[i] = v.s.charCodeAt(i);
      return m;
    }
    throw new MLError(`Operation für Werte der Klasse '${className(v)}' nicht definiert.`);
  }
  function scalar(v, what) {
    if (typeof v === 'number') return v;
    if (typeof v === 'boolean') return v ? 1 : 0;
    if (v instanceof Mat && v.r * v.c >= 1) return v.d[0];
    if (v instanceof Str && v.s.length === 1) return v.s.charCodeAt(0);
    throw new MLError((what || 'Wert') + ' muss ein Skalar sein.');
  }
  function toArr(v) {
    if (typeof v === 'number') return [v];
    if (v instanceof Mat) return Array.from(v.d);
    if (v instanceof Str) return Array.from(toMat(v).d);
    throw new MLError(`Zahlenwerte erwartet, nicht '${className(v)}'.`);
  }
  function str(v, what) {
    if (v instanceof Str) return v.s;
    throw new MLError((what || 'Argument') + ' muss eine Zeichenkette sein.');
  }
  function dims(v) {
    if (typeof v === 'number') return [1, 1];
    if (v instanceof Mat || v instanceof Cell) return [v.r, v.c];
    if (v instanceof Str) return [v.s.length ? 1 : 0, v.s.length];
    return [1, 1];
  }
  const numel = (v) => {
    const d = dims(v);
    return d[0] * d[1];
  };
  function className(v) {
    if (typeof v === 'number') return 'double';
    if (v instanceof Mat) return v.logical ? 'logical' : 'double';
    if (v instanceof Str) return v.dq ? 'string' : 'char';
    if (v instanceof Struct) return 'struct';
    if (v instanceof Cell) return 'cell';
    if (v instanceof Obj) return v.cls;
    return 'unbekannt';
  }
  function truth(v) {
    if (typeof v === 'number') return v !== 0 && !Number.isNaN(v);
    if (v instanceof Mat) {
      if (!v.d.length) return false;
      for (let i = 0; i < v.d.length; i++) if (v.d[i] === 0 || Number.isNaN(v.d[i])) return false;
      return true;
    }
    if (v instanceof Str) return v.s.length > 0 && !v.s.includes('\0');
    throw new MLError(`Bedingung vom Typ '${className(v)}' kann nicht ausgewertet werden.`);
  }
  function eye(n, m) {
    m = m == null ? n : m;
    const R = new Mat(n, m);
    for (let i = 0; i < Math.min(n, m); i++) R.d[i * n + i] = 1;
    return R;
  }
  const getRC = (m, i, j) => m.d[j * m.r + i];

  // ==================================================================
  // Rechenoperationen
  // ==================================================================
  function ew(a, b, f, logical) {
    if (typeof a === 'number' && typeof b === 'number') return logical ? (f(a, b) ? 1 : 0) : f(a, b);
    a = toMat(a);
    b = toMat(b);
    const r = a.r === b.r ? a.r : a.r === 1 ? b.r : b.r === 1 ? a.r : -1;
    const c = a.c === b.c ? a.c : a.c === 1 ? b.c : b.c === 1 ? a.c : -1;
    if (r < 0 || c < 0) throw new MLError(`Matrixdimensionen müssen übereinstimmen (${a.r}×${a.c} und ${b.r}×${b.c}).`);
    const out = new Float64Array(r * c);
    const ar1 = a.r === 1,
      ac1 = a.c === 1,
      br1 = b.r === 1,
      bc1 = b.c === 1;
    for (let j = 0; j < c; j++)
      for (let i = 0; i < r; i++) {
        const x = a.d[(ac1 ? 0 : j) * a.r + (ar1 ? 0 : i)],
          y = b.d[(bc1 ? 0 : j) * b.r + (br1 ? 0 : i)];
        out[j * r + i] = logical ? (f(x, y) ? 1 : 0) : f(x, y);
      }
    return norm(new Mat(r, c, out, logical));
  }
  function map1(v, f, logical) {
    if (typeof v === 'number') return logical ? (f(v) ? 1 : 0) : f(v);
    const m = toMat(v);
    const out = new Float64Array(m.d.length);
    for (let i = 0; i < out.length; i++) out[i] = logical ? (f(m.d[i]) ? 1 : 0) : f(m.d[i]);
    return norm(new Mat(m.r, m.c, out, logical));
  }
  function transpose(v) {
    if (typeof v === 'number') return v;
    if (v instanceof Cell) {
      const R = new Cell(v.c, v.r);
      for (let i = 0; i < v.r; i++) for (let j = 0; j < v.c; j++) R.d[i * v.c + j] = v.d[j * v.r + i];
      return R;
    }
    const m = toMat(v);
    const R = new Mat(m.c, m.r, null, m.logical);
    for (let i = 0; i < m.r; i++) for (let j = 0; j < m.c; j++) R.d[i * m.c + j] = m.d[j * m.r + i];
    return norm(R);
  }
  function matmul(a, b) {
    if (typeof a === 'number' && typeof b === 'number') return a * b;
    const A = toMat(a),
      B = toMat(b);
    if (A.r * A.c === 1 || B.r * B.c === 1) return ew(A, B, (x, y) => x * y);
    if (A.c !== B.r) throw new MLError(`Falsche Dimensionen für Matrixmultiplikation (${A.r}×${A.c} * ${B.r}×${B.c}). Für elementweise Multiplikation '.*' verwenden.`);
    const R = new Mat(A.r, B.c);
    for (let j = 0; j < B.c; j++)
      for (let k = 0; k < A.c; k++) {
        const bkj = B.d[j * B.r + k];
        if (bkj === 0) continue;
        for (let i = 0; i < A.r; i++) R.d[j * A.r + i] += A.d[k * A.r + i] * bkj;
      }
    return norm(R);
  }
  /** Löst A·X = B (A quadratisch: LU mit Pivotsuche, sonst kleinste Quadrate) */
  function solve(a, b) {
    const A = toMat(a),
      B = toMat(b);
    if (A.r * A.c === 1) return ew(B, A.d[0], (x, y) => x / y);
    if (A.r !== B.r) throw new MLError(`Matrixdimensionen müssen übereinstimmen (${A.r}×${A.c} \\ ${B.r}×${B.c}).`);
    if (A.r !== A.c) {
      const At = transpose(A);
      return solve(matmul(At, A), matmul(At, B));
    }
    const n = A.r,
      m = B.c;
    const M = Array.from({ length: n }, (_, i) => {
      const row = new Float64Array(n + m);
      for (let j = 0; j < n; j++) row[j] = A.d[j * n + i];
      for (let j = 0; j < m; j++) row[n + j] = B.d[j * n + i];
      return row;
    });
    let singular = false;
    for (let c = 0; c < n; c++) {
      let p = c;
      for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
      if (Math.abs(M[p][c]) < 1e-14) {
        singular = true;
        continue;
      }
      [M[c], M[p]] = [M[p], M[c]];
      for (let r = 0; r < n; r++) {
        if (r === c) continue;
        const f = M[r][c] / M[c][c];
        if (f === 0) continue;
        for (let k = c; k < n + m; k++) M[r][k] -= f * M[c][k];
      }
    }
    if (singular && curInterp) curInterp.warn('Matrix ist singulär oder schlecht konditioniert.');
    const X = new Mat(n, m);
    for (let i = 0; i < n; i++) for (let j = 0; j < m; j++) X.d[j * n + i] = M[i][n + j] / M[i][i];
    return norm(X);
  }
  function mrdivide(a, b) {
    if (typeof b === 'number' || numel(b) === 1) return ew(a, b, (x, y) => x / y);
    return transpose(solve(transpose(b), transpose(a)));
  }
  function mldivide(a, b) {
    if (typeof a === 'number' || numel(a) === 1) return ew(b, a, (x, y) => x / y);
    return solve(a, b);
  }
  function mpower(a, b) {
    if (typeof a === 'number' && typeof b === 'number') return Math.pow(a, b);
    const A = toMat(a);
    if (A.r * A.c === 1 && numel(b) === 1) return Math.pow(A.d[0], scalar(b));
    const p = scalar(b);
    if (A.r !== A.c || !Number.isInteger(p)) throw new MLError("Für Matrixpotenzen muss die Matrix quadratisch und der Exponent ganzzahlig sein. Für elementweise Potenz '.^' verwenden.");
    let R = eye(A.r),
      B = p < 0 ? toMat(inv(A)) : A;
    for (let k = 0; k < Math.abs(p); k++) R = toMat(matmul(R, B));
    return norm(R);
  }
  function inv(a) {
    const A = toMat(a);
    if (A.r !== A.c) throw new MLError('Matrix muss quadratisch sein.');
    return solve(A, eye(A.r));
  }
  function det(a) {
    const A = toMat(a);
    if (A.r !== A.c) throw new MLError('Matrix muss quadratisch sein.');
    const n = A.r;
    const M = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => A.d[j * n + i]));
    let d = 1;
    for (let c = 0; c < n; c++) {
      let p = c;
      for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
      if (M[p][c] === 0) return 0;
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
  }

  const BINOPS = {
    '+': (a, b) => (typeof a === 'number' && typeof b === 'number' ? a + b : ew(a, b, (x, y) => x + y)),
    '-': (a, b) => (typeof a === 'number' && typeof b === 'number' ? a - b : ew(a, b, (x, y) => x - y)),
    '*': matmul,
    '/': mrdivide,
    '\\': mldivide,
    '^': mpower,
    '.*': (a, b) => (typeof a === 'number' && typeof b === 'number' ? a * b : ew(a, b, (x, y) => x * y)),
    './': (a, b) => (typeof a === 'number' && typeof b === 'number' ? a / b : ew(a, b, (x, y) => x / y)),
    '.\\': (a, b) => ew(a, b, (x, y) => y / x),
    '.^': (a, b) => (typeof a === 'number' && typeof b === 'number' ? Math.pow(a, b) : ew(a, b, Math.pow)),
    '==': (a, b) => (a instanceof Str && b instanceof Str && a.dq && b.dq ? +(a.s === b.s) : ew(a, b, (x, y) => x === y, true)),
    '~=': (a, b) => (a instanceof Str && b instanceof Str && a.dq && b.dq ? +(a.s !== b.s) : ew(a, b, (x, y) => x !== y, true)),
    '<': (a, b) => ew(a, b, (x, y) => x < y, true),
    '<=': (a, b) => ew(a, b, (x, y) => x <= y, true),
    '>': (a, b) => ew(a, b, (x, y) => x > y, true),
    '>=': (a, b) => ew(a, b, (x, y) => x >= y, true),
    '&': (a, b) => ew(a, b, (x, y) => x !== 0 && y !== 0, true),
    '|': (a, b) => ew(a, b, (x, y) => x !== 0 || y !== 0, true),
  };

  function range(a, s, b) {
    a = scalar(a, 'Bereichsgrenze');
    s = scalar(s, 'Schrittweite');
    b = scalar(b, 'Bereichsgrenze');
    if (s === 0 || (s > 0 && a > b) || (s < 0 && a < b) || !Number.isFinite(a) || !Number.isFinite(b)) return new Mat(1, 0);
    const n = Math.floor((b - a) / s + 1e-10) + 1;
    if (n > 1e7) throw new MLError('Bereich ist zu groß.');
    const d = new Float64Array(n);
    for (let i = 0; i < n; i++) d[i] = a + i * s;
    return norm(new Mat(1, n, d));
  }

  // ==================================================================
  // Indizierung
  // ==================================================================
  function idxList(spec, n, forAssign) {
    if (spec === COLON) {
      const L = new Array(n);
      for (let i = 0; i < n; i++) L[i] = i;
      return L;
    }
    if (typeof spec === 'number') {
      if (!Number.isInteger(spec) || spec < 1) throw new MLError(`Indizes müssen positive ganze Zahlen oder logische Werte sein (Index ${fmtNum(spec)}).`);
      if (!forAssign && spec > n) throw new MLError(`Index überschreitet die Matrixdimensionen (Index ${spec}, Größe ${n}).`);
      return [spec - 1];
    }
    if (spec instanceof Str && spec.s === ':') return idxList(COLON, n);
    const m = toMat(spec);
    const L = [];
    if (m.logical) {
      if (m.d.length > n && !forAssign) {
        for (let i = n; i < m.d.length; i++) if (m.d[i]) throw new MLError('Logischer Index überschreitet die Matrixdimensionen.');
      }
      for (let i = 0; i < m.d.length; i++) if (m.d[i]) L.push(i);
      return L;
    }
    for (let i = 0; i < m.d.length; i++) {
      const v = m.d[i];
      if (!Number.isInteger(v) || v < 1) throw new MLError(`Indizes müssen positive ganze Zahlen oder logische Werte sein (Index ${fmtNum(v)}).`);
      if (!forAssign && v > n) throw new MLError(`Index überschreitet die Matrixdimensionen (Index ${v}, Größe ${n}).`);
      L.push(v - 1);
    }
    return L;
  }
  function checkExtraIdx(idx) {
    for (let k = 2; k < idx.length; k++) {
      const s = idx[k];
      if (s === COLON || (typeof s === 'number' && s === 1)) continue;
      throw new MLError('Nur zweidimensionale Matrizen werden unterstützt.');
    }
  }

  function indexRead(v, idx) {
    if (v instanceof Struct || v instanceof Obj) {
      for (const s of idx) if (!(s === COLON || s === 1)) throw new MLError('Index überschreitet die Dimensionen (Struktur/Objekt ist 1×1).');
      return v;
    }
    if (v instanceof Cell) {
      const R = indexRead(mat(v.r, v.c, v.d.map((_, i) => i + 1)), idx);
      const m = toMat(R);
      return new Cell(m.r, m.c, Array.from(m.d, (k) => v.d[k - 1]));
    }
    if (v instanceof Str) {
      const R = indexRead(toMat(v), idx);
      return new Str(String.fromCharCode(...toArr(R)), v.dq);
    }
    if (typeof v === 'number') {
      if (idx.length === 1 && idx[0] === 1) return v;
      if (idx.length === 2 && idx[0] === 1 && idx[1] === 1) return v;
    }
    const m = toMat(v);
    if (idx.length === 0) return v;
    if (idx.length === 1) {
      const s = idx[0];
      const n = m.r * m.c;
      if (typeof s === 'number') {
        if (Number.isInteger(s) && s >= 1 && s <= n) return m.d[s - 1];
        idxList(s, n);
      }
      const L = idxList(s, n);
      let r, c;
      if (s === COLON) (r = L.length), (c = 1);
      else if (s instanceof Mat && !s.logical && s.r !== 1 && s.c !== 1) (r = s.r), (c = s.c);
      else if (m.r === 1 && n !== 1) (r = 1), (c = L.length);
      else if (m.c === 1 && n !== 1) (r = L.length), (c = 1);
      else if (s instanceof Mat && s.r === 1) (r = 1), (c = L.length);
      else (r = L.length), (c = 1);
      const out = new Float64Array(L.length);
      for (let i = 0; i < L.length; i++) out[i] = m.d[L[i]];
      return norm(new Mat(r, c, out, m.logical));
    }
    checkExtraIdx(idx);
    const a = idx[0],
      b = idx[1];
    if (typeof a === 'number' && typeof b === 'number' && Number.isInteger(a) && Number.isInteger(b) && a >= 1 && b >= 1 && a <= m.r && b <= m.c) return m.d[(b - 1) * m.r + a - 1];
    const R = idxList(a, m.r),
      C = idxList(b, m.c);
    const out = new Float64Array(R.length * C.length);
    for (let j = 0; j < C.length; j++) for (let i = 0; i < R.length; i++) out[j * R.length + i] = m.d[C[j] * m.r + R[i]];
    return norm(new Mat(R.length, C.length, out, m.logical));
  }

  function indexAssign(cur, idx, rhs) {
    // Grafik-Handles (z. B. ai(1) = subplot(…)) als Handle-Array speichern
    if (rhs instanceof GObj && (cur === undefined || cur instanceof Cell || (cur instanceof Mat && cur.r * cur.c === 0))) return cellAssign(cur instanceof Cell ? cur : new Cell(0, 0), idx, rhs, true);
    if (cur instanceof Cell) {
      if (!(rhs instanceof Cell)) throw new MLError("Zuweisung an Zellen mit {} statt () vornehmen.");
      return cellAssign(cur, idx, rhs, false);
    }
    if (rhs instanceof Struct || rhs instanceof Obj || rhs instanceof Cell) {
      const empty = cur === undefined || (cur instanceof Mat && cur.r * cur.c === 0);
      const one = idx.every((s) => s === 1 || s === COLON);
      if (empty && one) return rhs;
      if ((cur instanceof Struct || cur instanceof Obj) && one) return rhs;
      throw new MLError('Struktur- und Objekt-Arrays werden nicht unterstützt.');
    }
    const isStr = cur instanceof Str && rhs instanceof Str;
    const base = cur === undefined ? EMPTY : toMat(cur);
    const rm = toMat(rhs);
    const rn = rm.r * rm.c;
    // Löschen: A(idx) = []
    if (rhs instanceof Mat && rn === 0 && rm.r === 0 && rm.c === 0) return deleteIdx(base, idx);
    let m;
    if (idx.length === 1) {
      const s = idx[0];
      let n = base.r * base.c;
      const L = s === COLON ? idxList(COLON, n) : idxList(s, n, true);
      if (rn !== 1 && rn !== L.length) throw new MLError(`Zuweisung: ${L.length} Elemente links, ${rn} rechts.`);
      let max = 0;
      for (const k of L) if (k + 1 > max) max = k + 1;
      if (max > n) {
        if (n === 0) m = new Mat(rm.c === 1 && rm.r > 1 && s === COLON ? max : 1, rm.c === 1 && rm.r > 1 && s === COLON ? 1 : max);
        else if (base.r === 1) m = grow(base, 1, max);
        else if (base.c === 1) m = grow(base, max, 1);
        else throw new MLError('Bei linearer Indizierung kann eine Matrix nicht vergrößert werden.');
      } else m = new Mat(base.r, base.c, Float64Array.from(base.d));
      for (let i = 0; i < L.length; i++) m.d[L[i]] = rn === 1 ? rm.d[0] : rm.d[i];
    } else {
      checkExtraIdx(idx);
      const a = idx[0],
        b = idx[1];
      let R, C;
      if (a === COLON) R = idxList(COLON, base.r || (b === COLON ? rm.r : rn === 1 ? 1 : rm.r * rm.c / Math.max(1, numelSpec(b))));
      else R = idxList(a, base.r, true);
      if (b === COLON) C = idxList(COLON, base.c || (rn === 1 ? 1 : rm.r * rm.c / Math.max(1, R.length)));
      else C = idxList(b, base.c, true);
      if (rn !== 1 && rn !== R.length * C.length) throw new MLError(`Zuweisung: Dimensionen ${R.length}×${C.length} links und ${rm.r}×${rm.c} rechts passen nicht.`);
      let nr = base.r,
        nc = base.c;
      for (const i of R) if (i + 1 > nr) nr = i + 1;
      for (const j of C) if (j + 1 > nc) nc = j + 1;
      m = nr !== base.r || nc !== base.c ? grow(base, nr, nc) : new Mat(base.r, base.c, Float64Array.from(base.d));
      let k = 0;
      for (let j = 0; j < C.length; j++) for (let i = 0; i < R.length; i++, k++) m.d[C[j] * m.r + R[i]] = rn === 1 ? rm.d[0] : rm.d[k];
    }
    if (isStr) return new Str(String.fromCharCode(...m.d), cur.dq);
    return norm(m);
  }
  function numelSpec(s) {
    if (s === COLON) return 1;
    if (typeof s === 'number') return 1;
    const m = toMat(s);
    return m.logical ? m.d.filter((x) => x).length : m.d.length;
  }
  function grow(m, nr, nc) {
    const R = new Mat(nr, nc);
    for (let j = 0; j < m.c; j++) for (let i = 0; i < m.r; i++) R.d[j * nr + i] = m.d[j * m.r + i];
    return R;
  }
  function deleteIdx(m, idx) {
    if (idx.length === 1) {
      const del = new Set(idxList(idx[0], m.r * m.c));
      const keep = [];
      for (let i = 0; i < m.d.length; i++) if (!del.has(i)) keep.push(m.d[i]);
      return m.c === 1 && m.r !== 1 ? colVec(keep) : keep.length ? rowVec(keep) : new Mat(m.r === 1 ? 1 : 0, 0);
    }
    const [a, b] = idx;
    if (a === COLON || (a instanceof Mat && idxList(a, m.r).length === m.r && m.r > 1)) {
      const del = new Set(idxList(b, m.c));
      const cols = [];
      for (let j = 0; j < m.c; j++) if (!del.has(j)) cols.push(j);
      const R = new Mat(m.r, cols.length);
      cols.forEach((j, k) => R.d.set(m.d.subarray(j * m.r, j * m.r + m.r), k * m.r));
      return R;
    }
    if (b === COLON) return transpose(deleteIdx(toMat(transpose(m)), [COLON, a]));
    throw new MLError('Löschen ist nur für ganze Zeilen oder Spalten möglich.');
  }
  function cellAssign(cur, idx, rhs, brace) {
    const base = cur instanceof Cell ? cur : new Cell(0, 0);
    const pos = indexAssign(mat(base.r, base.c, base.d.map((_, i) => i + 1)), idx, brace ? -1 : rhs instanceof Cell ? mat(rhs.r, rhs.c, rhs.d.map((_, i) => -(i + 2))) : -1);
    const pm = toMat(pos);
    const R = new Cell(pm.r, pm.c);
    for (let i = 0; i < pm.d.length; i++) {
      const k = pm.d[i];
      R.d[i] = k > 0 ? base.d[k - 1] : k === -1 ? rhs : k === 0 ? EMPTY : rhs.d[-k - 2];
    }
    return R;
  }

  // ==================================================================
  // Anzeige
  // ==================================================================
  function fmtNum(x) {
    if (Number.isNaN(x)) return 'NaN';
    if (!Number.isFinite(x)) return x > 0 ? 'Inf' : '-Inf';
    if (Number.isInteger(x) && Math.abs(x) < 1e10) return String(x);
    const ax = Math.abs(x);
    if (ax >= 1e5 || ax < 1e-4) return x.toExponential(4).replace(/e([+-])(\d)$/, 'e$10$2');
    return x.toFixed(4);
  }
  function fmtMatrix(m) {
    if (m.r * m.c === 0) return `     [](${m.r}×${m.c})`;
    const vals = Array.from(m.d);
    const fin = vals.filter((v) => Number.isFinite(v));
    const allInt = fin.every((v) => Number.isInteger(v));
    const maxAbs = fin.reduce((s, v) => Math.max(s, Math.abs(v)), 0);
    const expo = !allInt && (maxAbs >= 1e5 || (maxAbs < 1e-3 && maxAbs > 0));
    const f = (v) => {
      if (Number.isNaN(v)) return 'NaN';
      if (!Number.isFinite(v)) return v > 0 ? 'Inf' : '-Inf';
      if (allInt) return String(v);
      if (v === 0) return '0';
      if (expo) return v.toExponential(4).replace(/e([+-])(\d)$/, 'e$10$2');
      return v.toFixed(4);
    };
    const cells = [];
    let w = 0;
    for (let i = 0; i < m.r; i++) {
      const row = [];
      for (let j = 0; j < m.c; j++) {
        const s = f(m.d[j * m.r + i]);
        if (s.length > w) w = s.length;
        row.push(s);
      }
      cells.push(row);
    }
    return cells.map((row) => row.map((s) => ' '.repeat(3 + w - s.length) + s).join('')).join('\n');
  }
  function brief(v) {
    if (typeof v === 'number') return fmtNum(v);
    if (v instanceof Str) return v.dq ? `"${v.s}"` : `'${v.s}'`;
    if (v instanceof Mat) {
      if (v.r * v.c === 0) return '[]';
      if (v.r === 1 && v.c <= 10) return '[' + Array.from(v.d, fmtNum).join(' ') + ']';
      return `[${v.r}×${v.c} ${className(v)}]`;
    }
    if (v instanceof Struct) return '[1×1 struct]';
    if (v instanceof Cell) return `{${v.r}×${v.c} cell}`;
    if (v instanceof Obj) return `[1×1 ${v.cls}]`;
    return String(v);
  }
  function display(name, v) {
    if (typeof v === 'number') return `${name} = ${fmtNum(v)}\n`;
    if (v instanceof Str) return `${name} = ${brief(v)}\n`;
    if (v instanceof Mat) {
      if (v.r * v.c === 0) return `${name} =\n\n     []\n`;
      return `${name} =\n\n${fmtMatrix(v)}\n`;
    }
    if (v instanceof Struct) {
      const keys = [...v.f.keys()];
      const w = keys.reduce((s, k) => Math.max(s, k.length), 0);
      return `${name} = \n  struct with fields:\n\n` + keys.map((k) => ' '.repeat(4 + w - k.length) + k + ': ' + brief(v.f.get(k))).join('\n') + '\n';
    }
    if (v instanceof Cell) {
      return `${name} =\n  ${v.r}×${v.c} cell array\n` + (v.r * v.c <= 12 ? '    {' + v.d.map(brief).join('}    {') + '}\n' : '');
    }
    if (v instanceof Obj) {
      const lines = [];
      for (const [k, pv] of v.p) if (!k.startsWith('_')) lines.push(`    ${k}: ${brief(pv)}`);
      return `${name} = \n  ${v.cls} with properties:\n\n${lines.join('\n')}\n`;
    }
    return `${name} = ${String(v)}\n`;
  }

  // ==================================================================
  // Lexer
  // ==================================================================
  const KW = new Set(['if', 'elseif', 'else', 'end', 'for', 'parfor', 'while', 'break', 'continue', 'return', 'function', 'switch', 'case', 'otherwise', 'try', 'catch', 'global', 'persistent']);
  const OPS3 = ['...'];
  const OPS2 = ['.*', './', '.\\', '.^', ".'", '==', '~=', '!=', '<=', '>=', '&&', '||'];
  const OPS1 = '+-*/\\^<>&|~!=()[]{},;:.@';

  function lex(src) {
    const toks = [];
    const n = src.length;
    let i = 0,
      line = 1,
      space = false;
    const stack = [];
    const inMat = () => stack.length > 0 && (stack[stack.length - 1] === '[' || stack[stack.length - 1] === '{');
    const last = () => toks[toks.length - 1];
    const push = (type, v) => {
      toks.push({ type, v, line, sp: space });
      space = false;
    };
    const endsValue = () => {
      const t = last();
      if (!t) return false;
      if (t.type === 'num' || t.type === 'str' || t.type === 'id') return true;
      if (t.type === 'kw') return t.v === 'end';
      return t.type === 'op' && (t.v === ')' || t.v === ']' || t.v === '}' || t.v === "'" || t.v === ".'");
    };
    const startsValue = (k) => {
      const c = src[k],
        c1 = src[k + 1];
      if (/[A-Za-z0-9_'"([{@]/.test(c)) return true;
      if (c === '.' && /[0-9]/.test(c1)) return true;
      if ((c === '~' || c === '!') && c1 !== '=') return true;
      if ((c === '+' || c === '-') && c1 !== ' ' && c1 !== '\t' && c1 !== '=' && c1 !== '\n') return true;
      return false;
    };
    const atLineStart = (k) => {
      let j = k - 1;
      while (j >= 0 && (src[j] === ' ' || src[j] === '\t')) j--;
      return j < 0 || src[j] === '\n';
    };
    const restOfLineBlank = (k) => {
      let j = k;
      while (j < n && (src[j] === ' ' || src[j] === '\t' || src[j] === '\r')) j++;
      return j >= n || src[j] === '\n';
    };
    while (i < n) {
      const ch = src[i];
      // Blockkommentar %{ … %}
      if (ch === '%' && src[i + 1] === '{' && atLineStart(i) && restOfLineBlank(i + 2)) {
        let depth = 0;
        while (i < n) {
          const eol = src.indexOf('\n', i);
          const lineStr = src.slice(i, eol < 0 ? n : eol).trim();
          if (lineStr === '%{') depth++;
          else if (lineStr === '%}') depth--;
          i = eol < 0 ? n : eol + 1;
          line++;
          if (depth === 0) break;
        }
        if (!inMat()) push('nl');
        continue;
      }
      if (ch === '%' || (ch === '#' && false)) {
        while (i < n && src[i] !== '\n') i++;
        continue;
      }
      if (ch === '.' && src[i + 1] === '.' && src[i + 2] === '.') {
        while (i < n && src[i] !== '\n') i++;
        i++;
        line++;
        space = true;
        continue;
      }
      if (ch === ' ' || ch === '\t' || ch === '\r') {
        i++;
        space = true;
        continue;
      }
      if (ch === '\n') {
        if (inMat()) {
          const t = last();
          if (t && !(t.type === 'op' && (t.v === ';' || t.v === '[' || t.v === '{' || t.v === ','))) push('op', ';');
        } else push('nl');
        line++;
        i++;
        space = false;
        continue;
      }
      // Leerzeichen trennt Elemente in [ ] und { }
      if (inMat() && space && endsValue() && startsValue(i)) {
        const t = last();
        // "a '" in Matrix: Zeichenkette, "a'" : Transponieren
        push('op', ',');
        toks[toks.length - 1].sp = t.sp;
        space = true;
      }
      // Zahl
      if (/[0-9]/.test(ch) || (ch === '.' && /[0-9]/.test(src[i + 1]))) {
        const m = /^(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?/.exec(src.slice(i, i + 40));
        push('num', parseFloat(m[0]));
        i += m[0].length;
        continue;
      }
      // Bezeichner / Schlüsselwort
      if (/[A-Za-z_]/.test(ch)) {
        let j = i + 1;
        while (j < n && /[A-Za-z0-9_]/.test(src[j])) j++;
        const w = src.slice(i, j);
        const prev = last();
        const afterDot = prev && prev.type === 'op' && prev.v === '.' && !prev.spAfter;
        push(KW.has(w) && !afterDot ? 'kw' : 'id', w);
        i = j;
        continue;
      }
      // Zeichenkette oder Transponieren
      if (ch === "'") {
        if (endsValue() && !(space && inMat())) {
          push('op', "'");
          i++;
          continue;
        }
        let j = i + 1,
          s = '';
        for (;;) {
          if (j >= n || src[j] === '\n') throw new MLError('Zeichenkette ist nicht abgeschlossen.', line);
          if (src[j] === "'") {
            if (src[j + 1] === "'") {
              s += "'";
              j += 2;
              continue;
            }
            break;
          }
          s += src[j++];
        }
        push('str', s);
        i = j + 1;
        continue;
      }
      if (ch === '"') {
        let j = i + 1,
          s = '';
        for (;;) {
          if (j >= n || src[j] === '\n') throw new MLError('Zeichenkette ist nicht abgeschlossen.', line);
          if (src[j] === '"') {
            if (src[j + 1] === '"') {
              s += '"';
              j += 2;
              continue;
            }
            break;
          }
          s += src[j++];
        }
        push('str', s);
        toks[toks.length - 1].dq = true;
        i = j + 1;
        continue;
      }
      // Operatoren
      const op2 = src.substr(i, 2);
      if (OPS2.includes(op2)) {
        push('op', op2 === '!=' ? '~=' : op2);
        i += 2;
        continue;
      }
      if (OPS1.includes(ch)) {
        if (ch === '(' || ch === '[' || ch === '{') stack.push(ch);
        else if (ch === ')' || ch === ']' || ch === '}') {
          // Trennzeichen vor schließender Klammer entfernen
          const t = last();
          if ((ch === ']' || ch === '}') && t && t.type === 'op' && (t.v === ';' || t.v === ',') && inMat()) toks.pop();
          stack.pop();
        }
        push('op', ch === '!' ? '~' : ch);
        i++;
        continue;
      }
      throw new MLError(`Unzulässiges Zeichen '${ch}'.`, line);
    }
    push('nl');
    push('eof');
    return toks;
  }

  // ==================================================================
  // Parser
  // ==================================================================
  const CMD_WORDS = new Set(['close', 'clear', 'clc', 'grid', 'hold', 'axis', 'format', 'clearvars', 'warning', 'disp', 'more', 'figure', 'box', 'rotate3d', 'drawnow', 'clf']);

  class Parser {
    constructor(src, fileName) {
      this.t = lex(src);
      this.k = 0;
      this.file = fileName;
      this.inIndex = 0;
      this.inMatrix = 0;
      this.fns = new Map();
    }
    peek(o = 0) {
      return this.t[this.k + o];
    }
    next() {
      return this.t[this.k++];
    }
    is(type, v, o = 0) {
      const t = this.t[this.k + o];
      return t.type === type && (v === undefined || t.v === v);
    }
    isOp(v, o) {
      return this.is('op', v, o);
    }
    err(msg, t) {
      t = t || this.peek();
      return new MLError(msg, t.line);
    }
    expectOp(v) {
      const t = this.next();
      if (t.type !== 'op' || t.v !== v) throw this.err(`'${v}' erwartet, gefunden: ${tokStr(t)}.`, t);
      return t;
    }
    expectKw(v) {
      const t = this.next();
      if (t.type !== 'kw' || t.v !== v) throw this.err(`'${v}' erwartet, gefunden: ${tokStr(t)}.`, t);
      return t;
    }
    skipSep() {
      while (this.is('nl') || this.isOp(';') || this.isOp(',')) this.k++;
    }

    parseFile() {
      const body = [];
      for (;;) {
        this.skipSep();
        if (this.is('eof')) break;
        if (this.is('kw', 'function')) {
          const f = this.parseFunction();
          if (this.fns.has(f.name)) throw new MLError(`Funktion '${f.name}' ist mehrfach definiert.`, f.line);
          this.fns.set(f.name, f);
          continue;
        }
        body.push(this.parseStatement());
      }
      return { name: this.file, body, fns: this.fns };
    }
    parseFunction() {
      const ft = this.expectKw('function');
      let outs = [];
      if (this.isOp('[')) {
        this.next();
        while (!this.isOp(']')) {
          if (this.isOp(',')) {
            this.next();
            continue;
          }
          const t = this.next();
          if (t.type !== 'id') throw this.err('Name eines Ausgabearguments erwartet.', t);
          outs.push(t.v);
        }
        this.next();
        this.expectOp('=');
      } else if (this.is('id') && this.isOp('=', 1)) {
        outs = [this.next().v];
        this.next();
      }
      const nt = this.next();
      if (nt.type !== 'id') throw this.err('Funktionsname erwartet.', nt);
      const params = [];
      if (this.isOp('(')) {
        this.next();
        while (!this.isOp(')')) {
          if (this.isOp(',')) {
            this.next();
            continue;
          }
          const t = this.next();
          if (t.type === 'id') params.push(t.v);
          else if (t.type === 'op' && t.v === '~') params.push('~');
          else throw this.err('Parametername erwartet.', t);
        }
        this.next();
      }
      const body = this.parseBlock(['end', 'function'], true);
      if (this.is('kw', 'end')) this.next();
      return { name: nt.v, params, outs, body, line: ft.line, file: this.file };
    }
    /** Anweisungen bis zu einem der Schlüsselwörter (nicht verbraucht) */
    parseBlock(terms, inFunction) {
      const body = [];
      for (;;) {
        this.skipSep();
        const t = this.peek();
        if (t.type === 'eof') {
          if (inFunction) return body;
          throw this.err(`'end' fehlt (Block nicht abgeschlossen).`, t);
        }
        if (t.type === 'kw' && terms.includes(t.v)) return body;
        body.push(this.parseStatement());
      }
    }
    endStmt(st) {
      const t = this.peek();
      if (t.type === 'op' && t.v === ';') {
        this.next();
        st.print = false;
      } else if (t.type === 'op' && t.v === ',') {
        this.next();
        st.print = true;
      } else if (t.type === 'nl' || t.type === 'eof') st.print = true;
      else if (t.type === 'kw' && ['end', 'else', 'elseif', 'case', 'otherwise', 'catch', 'function'].includes(t.v)) st.print = true;
      else throw this.err(`Unerwartetes ${tokStr(t)} – fehlt ein Operator, ein Komma oder ein Semikolon?`, t);
      return st;
    }
    parseStatement() {
      const t = this.peek();
      const line = t.line;
      if (t.type === 'kw') {
        switch (t.v) {
          case 'if':
            return this.parseIf();
          case 'for':
          case 'parfor':
            return this.parseFor();
          case 'while': {
            this.next();
            const cond = this.parseExpr();
            const body = this.parseBlock(['end']);
            this.next();
            return { t: 'while', cond, body, line };
          }
          case 'switch':
            return this.parseSwitch();
          case 'try': {
            this.next();
            const body = this.parseBlock(['catch', 'end']);
            let id = null,
              cbody = [];
            if (this.is('kw', 'catch')) {
              this.next();
              const ct = this.t[this.k - 1];
              if (this.is('id') && this.peek().line === ct.line && !this.isOp('(', 1) && !this.isOp('=', 1)) id = this.next().v;
              cbody = this.parseBlock(['end']);
            }
            this.next();
            return { t: 'try', body, id, cbody, line };
          }
          case 'break':
          case 'continue':
          case 'return':
            this.next();
            return this.endStmt({ t: t.v, line });
          case 'global':
          case 'persistent': {
            this.next();
            const names = [];
            while (this.is('id')) names.push(this.next().v);
            return this.endStmt({ t: t.v, names, line });
          }
          case 'function':
            throw this.err('Funktionsdefinitionen sind hier nicht erlaubt.');
          default:
            throw this.err(`Unerwartetes Schlüsselwort '${t.v}'.`);
        }
      }
      // Befehlssyntax: close all, grid on, axis equal …
      if (t.type === 'id' && CMD_WORDS.has(t.v)) {
        const n1 = this.peek(1);
        if ((n1.type === 'id' || n1.type === 'num' || n1.type === 'kw') && n1.sp && !(this.isOp('=', 2) || this.isOp('(', 2) && !this.peek(2).sp)) {
          this.next();
          const args = [];
          while (this.is('id') || this.is('num') || this.is('kw')) args.push({ t: 'str', v: String(this.next().v) });
          return this.endStmt({ t: 'expr', e: { t: 'call', f: { t: 'id', name: t.v }, args }, line });
        }
      }
      // Mehrfachzuweisung [a, b] = …
      if (this.isOp('[')) {
        let j = this.k + 1,
          depth = 1;
        while (depth > 0 && this.t[j].type !== 'eof' && this.t[j].type !== 'nl') {
          const tj = this.t[j];
          if (tj.type === 'op' && (tj.v === '[' || tj.v === '(' || tj.v === '{')) depth++;
          if (tj.type === 'op' && (tj.v === ']' || tj.v === ')' || tj.v === '}')) depth--;
          j++;
        }
        if (depth === 0 && this.t[j].type === 'op' && this.t[j].v === '=') {
          this.next();
          const lhs = [];
          while (!this.isOp(']')) {
            if (this.isOp(',')) {
              this.next();
              continue;
            }
            if (this.isOp('~')) {
              this.next();
              lhs.push(null);
              continue;
            }
            lhs.push(this.toLValue(this.parsePostfix()));
          }
          this.next();
          this.expectOp('=');
          const rhs = this.parseExpr();
          return this.endStmt({ t: 'assign', lhs, rhs, multi: true, line });
        }
      }
      const e = this.parseExpr();
      if (this.isOp('=')) {
        this.next();
        const lv = this.toLValue(e);
        const rhs = this.parseExpr();
        return this.endStmt({ t: 'assign', lhs: [lv], rhs, multi: false, line });
      }
      return this.endStmt({ t: 'expr', e, line });
    }
    toLValue(e) {
      if (e.t === 'id') return e;
      if (e.t === 'call' || e.t === 'field' || e.t === 'dfield') {
        this.toLValue(e.t === 'call' ? e.f : e.a);
        return e;
      }
      throw new MLError('Ungültiges Ziel einer Zuweisung.', this.peek().line);
    }
    parseIf() {
      const line = this.next().line;
      const clauses = [];
      const cond = this.parseExpr();
      clauses.push({ cond, body: this.parseBlock(['elseif', 'else', 'end']) });
      let els = null;
      for (;;) {
        const t = this.next();
        if (t.v === 'elseif') {
          const c = this.parseExpr();
          clauses.push({ cond: c, body: this.parseBlock(['elseif', 'else', 'end']) });
        } else if (t.v === 'else') {
          els = this.parseBlock(['end']);
        } else break;
      }
      return { t: 'if', clauses, els, line };
    }
    parseFor() {
      const line = this.next().line;
      let paren = false;
      if (this.isOp('(')) {
        paren = true;
        this.next();
      }
      const vt = this.next();
      if (vt.type !== 'id') throw this.err('Schleifenvariable erwartet.', vt);
      this.expectOp('=');
      const e = this.parseExpr();
      if (paren) this.expectOp(')');
      const body = this.parseBlock(['end']);
      this.next();
      return { t: 'for', v: vt.v, e, body, line };
    }
    parseSwitch() {
      const line = this.next().line;
      const e = this.parseExpr();
      this.skipSep();
      const cases = [];
      let other = null;
      for (;;) {
        this.skipSep();
        const t = this.next();
        if (t.type === 'kw' && t.v === 'case') {
          const v = this.parseExpr();
          cases.push({ v, body: this.parseBlock(['case', 'otherwise', 'end']) });
        } else if (t.type === 'kw' && t.v === 'otherwise') {
          other = this.parseBlock(['case', 'otherwise', 'end']);
        } else if (t.type === 'kw' && t.v === 'end') break;
        else throw this.err("'case', 'otherwise' oder 'end' erwartet.", t);
      }
      return { t: 'switch', e, cases, other, line };
    }

    // ---------------- Ausdrücke ----------------
    parseExpr() {
      return this.parseOrOr();
    }
    bin(sub, ops) {
      let a = sub();
      while (this.is('op') && ops.includes(this.peek().v)) {
        const t = this.next();
        const b = sub();
        a = { t: 'bin', op: t.v, a, b, line: t.line };
      }
      return a;
    }
    parseOrOr() {
      return this.bin(() => this.parseAndAnd(), ['||']);
    }
    parseAndAnd() {
      return this.bin(() => this.parseOr(), ['&&']);
    }
    parseOr() {
      return this.bin(() => this.parseAnd(), ['|']);
    }
    parseAnd() {
      return this.bin(() => this.parseCmp(), ['&']);
    }
    parseCmp() {
      return this.bin(() => this.parseRange(), ['==', '~=', '<', '<=', '>', '>=']);
    }
    parseRange() {
      if (this.inIndex && this.isOp(':') && (this.isOp(',', 1) || this.isOp(')', 1) || this.isOp('}', 1))) {
        this.next();
        return { t: 'colon' };
      }
      const a = this.parseAdd();
      if (this.isOp(':') && !(this.inMatrix && false)) {
        const line = this.next().line;
        const b = this.parseAdd();
        if (this.isOp(':')) {
          this.next();
          const c = this.parseAdd();
          return { t: 'range', a, s: b, b: c, line };
        }
        return { t: 'range', a, s: null, b, line };
      }
      return a;
    }
    parseAdd() {
      return this.bin(() => this.parseMul(), ['+', '-']);
    }
    parseMul() {
      return this.bin(() => this.parseUnary(), ['*', '/', '\\', '.*', './', '.\\']);
    }
    parseUnary() {
      if (this.isOp('-') || this.isOp('+') || this.isOp('~')) {
        const t = this.next();
        const a = this.parseUnary();
        if (t.v === '+') return a;
        if (t.v === '-' && a.t === 'num') return { t: 'num', v: -a.v };
        return { t: 'un', op: t.v, a, line: t.line };
      }
      return this.parsePower();
    }
    parsePower() {
      let a = this.parsePostfix();
      while (this.isOp('^') || this.isOp('.^')) {
        const t = this.next();
        const b = this.parsePowerOperand();
        a = { t: 'bin', op: t.v, a, b, line: t.line };
      }
      return a;
    }
    parsePowerOperand() {
      if (this.isOp('-') || this.isOp('+') || this.isOp('~')) {
        const t = this.next();
        const a = this.parsePowerOperand();
        return t.v === '+' ? a : { t: 'un', op: t.v, a, line: t.line };
      }
      return this.parsePostfix();
    }
    parsePostfix() {
      let e = this.parsePrimary();
      for (;;) {
        const t = this.peek();
        if (t.type !== 'op') break;
        if (t.v === '(' && !(this.inMatrix && t.sp && !this.inIndex)) {
          this.next();
          const args = this.parseArgs(')');
          e = { t: 'call', f: e, args, line: t.line };
        } else if (t.v === '{' && !(this.inMatrix && t.sp)) {
          this.next();
          const args = this.parseArgs('}');
          e = { t: 'call', f: e, args, brace: true, line: t.line };
        } else if (t.v === '.' && !t.sp) {
          this.next();
          if (this.isOp('(')) {
            this.next();
            const fe = this.parseExpr();
            this.expectOp(')');
            e = { t: 'dfield', a: e, e: fe, line: t.line };
          } else {
            const nt = this.next();
            if (nt.type !== 'id' && nt.type !== 'kw') throw this.err('Feldname erwartet.', nt);
            e = { t: 'field', a: e, name: nt.v, line: t.line };
          }
        } else if ((t.v === "'" || t.v === ".'") && !t.sp) {
          this.next();
          e = { t: 'post', op: t.v, a: e, line: t.line };
        } else if ((t.v === "'" || t.v === ".'") && !this.inMatrix) {
          this.next();
          e = { t: 'post', op: t.v, a: e, line: t.line };
        } else break;
      }
      return e;
    }
    parseArgs(close) {
      const args = [];
      const saveM = this.inMatrix;
      this.inMatrix = 0;
      this.inIndex++;
      while (!this.isOp(close)) {
        if (this.isOp(',')) {
          this.next();
          continue;
        }
        if (this.is('nl')) {
          this.next();
          continue;
        }
        if (this.is('id') && this.isOp('=', 1)) {
          const nt = this.next();
          this.next();
          args.push({ t: 'nv', name: nt.v, v: this.parseExpr() });
          continue;
        }
        args.push(this.parseExpr());
        if (!this.isOp(',') && !this.isOp(close)) throw this.err(`',' oder '${close}' erwartet, gefunden: ${tokStr(this.peek())}.`);
      }
      this.next();
      this.inIndex--;
      this.inMatrix = saveM;
      return args;
    }
    parsePrimary() {
      const t = this.next();
      switch (t.type) {
        case 'num':
          return { t: 'num', v: t.v };
        case 'str':
          return { t: 'str', v: t.v, dq: !!t.dq };
        case 'id':
          return { t: 'id', name: t.v, line: t.line };
        case 'kw':
          if (t.v === 'end' && this.inIndex) return { t: 'end' };
          throw this.err(`Unerwartetes Schlüsselwort '${t.v}'.`, t);
        case 'op':
          if (t.v === '(') {
            const saveM = this.inMatrix,
              saveI = this.inIndex;
            this.inMatrix = 0;
            const e = this.parseExpr();
            this.inMatrix = saveM;
            this.inIndex = saveI;
            this.expectOp(')');
            return { t: 'paren', e };
          }
          if (t.v === '[' || t.v === '{') return this.parseMatrix(t.v === '[' ? ']' : '}', t);
          if (t.v === '@') {
            if (this.isOp('(')) {
              this.next();
              const params = [];
              while (!this.isOp(')')) {
                const p = this.next();
                if (p.type === 'id') params.push(p.v);
              }
              this.next();
              const body = this.parseExpr();
              return { t: 'anon', params, body };
            }
            const nt = this.next();
            return { t: 'fhandle', name: nt.v };
          }
          if (t.v === ':' && this.inIndex) return { t: 'colon' };
          throw this.err(`Unerwartetes ${tokStr(t)}.`, t);
        case 'nl':
          throw this.err('Ausdruck unvollständig.', t);
        default:
          throw this.err(`Unerwartetes ${tokStr(t)}.`, t);
      }
    }
    parseMatrix(close, open) {
      const rows = [];
      let row = [];
      this.inMatrix++;
      const saveI = this.inIndex;
      this.inIndex = saveI; // end ist in [ ] innerhalb einer Indizierung erlaubt
      for (;;) {
        const t = this.peek();
        if (t.type === 'eof') throw this.err(`'${close}' fehlt.`, open);
        if (t.type === 'op' && t.v === close) {
          this.next();
          break;
        }
        if (t.type === 'op' && t.v === ',') {
          this.next();
          continue;
        }
        if ((t.type === 'op' && t.v === ';') || t.type === 'nl') {
          this.next();
          if (row.length) rows.push(row);
          row = [];
          continue;
        }
        row.push(this.parseExpr());
      }
      if (row.length) rows.push(row);
      this.inMatrix--;
      return { t: close === ']' ? 'mat' : 'cell', rows, line: open.line };
    }
  }
  function tokStr(t) {
    if (t.type === 'eof') return 'Dateiende';
    if (t.type === 'nl') return 'Zeilenende';
    if (t.type === 'str') return `Zeichenkette '${t.v}'`;
    return `'${t.v}'`;
  }

  function parse(src, fileName) {
    return new Parser(src.replace(/\r\n?/g, '\n'), fileName || 'Befehl').parseFile();
  }

  // ==================================================================
  // Interpreter
  // ==================================================================
  const BRK = 1,
    CONT = 2,
    RET = 3;
  let curInterp = null;

  class Scope {
    constructor(file, fn, pstore) {
      this.vars = new Map();
      this.file = file;
      this.fn = fn;
      this.pstore = pstore;
      this.persist = null;
      this.globals = null;
    }
  }

  class Interp {
    constructor(opts) {
      opts = opts || {};
      this.print = opts.print || ((s) => console.log(s));
      this.onEvent = opts.onEvent || (() => {});
      this.base = new Scope(null, null, null);
      this.globals = new Map();
      this.pstores = new Map(); // Datei → Map(Funktion → Map(Variable → Wert))
      this.figs = [];
      this.curFig = null;
      this.figSeq = 1;
      this.steps = 0;
      this.maxSteps = 2e7;
      this.depth = 0;
      this.warned = new Set();
      this.extra = opts.builtins || {};
    }
    warn(msg) {
      this.print('Warnung: ' + msg + '\n', 'warn');
    }

    // ---------------- Ausführung ----------------
    /** Führt eine Datei (Skript) bzw. Befehlszeilen im Basis-Workspace aus */
    runScript(src, fileName, opts) {
      opts = opts || {};
      const file = typeof src === 'string' ? parse(src, fileName) : src;
      const scope = this.base;
      const prevFile = scope.file;
      scope.file = file;
      const prev = curInterp;
      curInterp = this;
      this.steps = 0;
      this.maxSteps = opts.maxSteps || 2e7;
      try {
        const r = this.exec(file.body, scope);
        if (r === BRK || r === CONT) throw new MLError("'break'/'continue' außerhalb einer Schleife.");
      } catch (e) {
        throw wrapErr(e, file.name);
      } finally {
        scope.file = prevFile || file;
        curInterp = prev;
        this.onEvent('figures');
        this.onEvent('workspace');
      }
    }
    /** Ruft die erste (oder benannte) Funktion einer Datei auf – für MATLAB-Function-Blöcke */
    callFile(file, fnName, args, nargout, holder, opts) {
      const fn = fnName ? file.fns.get(fnName) : file.fns.values().next().value;
      if (!fn) throw new MLError(`Datei '${file.name}' enthält keine Funktion${fnName ? " '" + fnName + "'" : ''}.`);
      const prev = curInterp;
      curInterp = this;
      this.steps = 0;
      this.maxSteps = (opts && opts.maxSteps) || 2e6;
      try {
        return this.callUser(fn, file, args, nargout, holder);
      } catch (e) {
        throw wrapErr(e, file.name);
      } finally {
        curInterp = prev;
      }
    }

    exec(stmts, scope) {
      for (let i = 0; i < stmts.length; i++) {
        const st = stmts[i];
        if (++this.steps > this.maxSteps) throw new MLError('Ausführung abgebrochen: zu viele Rechenschritte (Endlosschleife?).', st.line);
        let r;
        try {
          r = this.execOne(st, scope);
        } catch (e) {
          if (e instanceof MLError && e.line == null) e.line = st.line;
          if (!(e instanceof MLError)) {
            const me = new MLError(e && e.message ? e.message : String(e), st.line);
            throw me;
          }
          throw e;
        }
        if (r) return r;
      }
      return 0;
    }
    execOne(st, scope) {
      switch (st.t) {
        case 'expr': {
          const e = st.e;
          if (e.t === 'id' && scope.vars.has(e.name)) {
            if (st.print) this.print(display(e.name, scope.vars.get(e.name)));
            return 0;
          }
          const vals = this.evMulti(e, scope, 0);
          if (vals.length && vals[0] !== undefined) {
            scope.vars.set('ans', vals[0]);
            if (st.print) this.print(display('ans', vals[0]));
          }
          return 0;
        }
        case 'assign': {
          if (!st.multi) {
            const v = this.ev(st.rhs, scope);
            if (v === undefined) throw new MLError('Der Ausdruck rechts liefert keinen Wert.');
            const name = this.assign(st.lhs[0], v, scope);
            if (st.print) this.print(display(name, scope.vars.get(name)));
            return 0;
          }
          const vals = this.evMulti(st.rhs, scope, st.lhs.length);
          st.lhs.forEach((lv, k) => {
            if (!lv) return;
            if (vals[k] === undefined) throw new MLError(`Zu viele Ausgabeargumente (${st.lhs.length} angefordert, ${vals.length} geliefert).`);
            const name = this.assign(lv, vals[k], scope);
            if (st.print) this.print(display(name, scope.vars.get(name)));
          });
          return 0;
        }
        case 'if':
          for (const c of st.clauses) if (truth(this.ev(c.cond, scope))) return this.exec(c.body, scope);
          if (st.els) return this.exec(st.els, scope);
          return 0;
        case 'for': {
          const e = st.e;
          if (e.t === 'range') {
            const a = scalar(this.ev(e.a, scope)),
              s = e.s ? scalar(this.ev(e.s, scope)) : 1,
              b = scalar(this.ev(e.b, scope));
            if (s === 0 || (s > 0 && a > b) || (s < 0 && a < b)) return 0;
            const n = Math.floor((b - a) / s + 1e-10) + 1;
            for (let i = 0; i < n; i++) {
              scope.vars.set(st.v, a + i * s);
              const r = this.exec(st.body, scope);
              if (r === BRK) break;
              if (r === RET) return RET;
            }
            return 0;
          }
          const v = this.ev(e, scope);
          const [r, c] = dims(v);
          for (let j = 0; j < c; j++) {
            let col;
            if (typeof v === 'number') col = v;
            else if (v instanceof Cell) col = new Cell(r, 1, v.d.slice(j * r, j * r + r));
            else if (v instanceof Str) col = new Str(v.s[j]);
            else if (v instanceof Mat) col = r === 1 ? v.d[j] : norm(new Mat(r, 1, v.d.slice(j * r, j * r + r)));
            else col = v;
            scope.vars.set(st.v, col);
            const res = this.exec(st.body, scope);
            if (res === BRK) break;
            if (res === RET) return RET;
          }
          return 0;
        }
        case 'while':
          while (truth(this.ev(st.cond, scope))) {
            const r = this.exec(st.body, scope);
            if (r === BRK) break;
            if (r === RET) return RET;
            if (++this.steps > this.maxSteps) throw new MLError('Ausführung abgebrochen: zu viele Rechenschritte (Endlosschleife?).', st.line);
          }
          return 0;
        case 'switch': {
          const v = this.ev(st.e, scope);
          const match = (cv) => {
            if (cv instanceof Cell) return cv.d.some(match);
            if (v instanceof Str || cv instanceof Str) return v instanceof Str && cv instanceof Str && v.s === cv.s;
            return numel(v) >= 1 && numel(cv) >= 1 && scalar(v) === scalar(cv);
          };
          for (const c of st.cases) if (match(this.ev(c.v, scope))) return this.exec(c.body, scope);
          if (st.other) return this.exec(st.other, scope);
          return 0;
        }
        case 'try':
          try {
            return this.exec(st.body, scope);
          } catch (e) {
            if (!(e instanceof MLError)) throw e;
            if (st.id) {
              const ex = new Obj('MException', { message: new Str(e.message), identifier: new Str(''), stack: new Struct() });
              scope.vars.set(st.id, ex);
            }
            return this.exec(st.cbody, scope);
          }
        case 'break':
          return BRK;
        case 'continue':
          return CONT;
        case 'return':
          return RET;
        case 'persistent': {
          if (!scope.pstore) throw new MLError("'persistent' ist nur in Funktionen erlaubt.");
          scope.persist = scope.persist || new Set();
          for (const n of st.names) {
            if (!scope.pstore.has(n)) scope.pstore.set(n, EMPTY);
            scope.vars.set(n, scope.pstore.get(n));
            scope.persist.add(n);
          }
          return 0;
        }
        case 'global': {
          scope.globals = scope.globals || new Set();
          for (const n of st.names) {
            if (!this.globals.has(n)) this.globals.set(n, EMPTY);
            scope.vars.set(n, this.globals.get(n));
            scope.globals.add(n);
          }
          return 0;
        }
      }
      throw new MLError('Unbekannte Anweisung.');
    }

    /** Zuweisung an ein L-Wert-AST, liefert den Variablennamen */
    assign(lv, v, scope) {
      if (lv.t === 'id') {
        scope.vars.set(lv.name, v);
        if (scope.globals && scope.globals.has(lv.name)) this.globals.set(lv.name, v);
        return lv.name;
      }
      // Kette zerlegen
      const chain = [];
      let e = lv;
      while (e.t !== 'id') {
        chain.unshift(e);
        e = e.t === 'call' ? e.f : e.a;
      }
      const name = e.name;
      const cur = scope.vars.get(name);
      const nv = this.assignInto(cur, chain, 0, v, scope);
      scope.vars.set(name, nv);
      if (scope.globals && scope.globals.has(name)) this.globals.set(name, nv);
      return name;
    }
    assignInto(cur, chain, i, v, scope) {
      if (i === chain.length) return v;
      const a = chain[i];
      if (a.t === 'field' || a.t === 'dfield') {
        const fname = a.t === 'field' ? a.name : str(this.ev(a.e, scope), 'Dynamischer Feldname');
        if (cur instanceof Obj) {
          const old = cur instanceof GObj || cur.has(fname) ? (i + 1 < chain.length ? cur.get(fname) : undefined) : undefined;
          cur.set(fname, this.assignInto(old, chain, i + 1, v, scope));
          return cur;
        }
        let s;
        if (cur === undefined || (cur instanceof Mat && cur.r * cur.c === 0)) s = new Struct();
        else if (cur instanceof Struct) s = cur.clone();
        else throw new MLError(`Feldzuweisung an einen Wert der Klasse '${className(cur)}' ist nicht möglich.`);
        s.f.set(fname, this.assignInto(s.f.get(fname), chain, i + 1, v, scope));
        return s;
      }
      // Indizierung
      const idx = this.evIdx(a.args, cur === undefined ? EMPTY : cur, scope);
      if (a.brace) {
        if (cur !== undefined && !(cur instanceof Cell) && !(cur instanceof Mat && cur.r * cur.c === 0)) throw new MLError("'{}'-Indizierung nur für Zellen.");
        const c = cur instanceof Cell ? cur : new Cell(0, 0);
        let inner;
        if (i + 1 < chain.length) {
          const old = this.tryIndex(c, idx, true);
          inner = this.assignInto(old, chain, i + 1, v, scope);
        } else inner = v;
        return cellAssign(c, idx, inner, true);
      }
      if (i + 1 < chain.length) {
        const old = cur === undefined ? undefined : this.tryIndex(cur, idx, false);
        const inner = this.assignInto(old, chain, i + 1, v, scope);
        if (cur === undefined || cur instanceof Struct || cur instanceof Obj) {
          if (idx.every((s) => s === 1)) return inner;
        }
        return indexAssign(cur, idx, inner);
      }
      return indexAssign(cur, idx, v);
    }
    tryIndex(v, idx, brace) {
      try {
        const r = indexRead(v, idx);
        if (brace && r instanceof Cell) return r.d.length === 1 ? r.d[0] : undefined;
        return r;
      } catch (e) {
        return undefined;
      }
    }

    evIdx(args, v, scope) {
      const out = new Array(args.length);
      const n = args.length;
      for (let k = 0; k < n; k++) {
        const a = args[k];
        if (a.t === 'colon') out[k] = COLON;
        else if (a.t === 'num') out[k] = a.v;
        else {
          this.endStack = this.endStack || [];
          this.endStack.push({ v, k, n });
          try {
            out[k] = this.ev(a, scope);
          } finally {
            this.endStack.pop();
          }
        }
      }
      return out;
    }
    evArgs(args, scope) {
      const out = [];
      for (const a of args) {
        if (a.t === 'nv') {
          out.push(new Str(a.name), this.ev(a.v, scope));
        } else if (a.t === 'colon') out.push(new Str(':'));
        else if (a.t === 'call' && a.brace) {
          for (const x of this.evMulti(a, scope, -1)) out.push(x);
        } else {
          const v = this.ev(a, scope);
          if (v === undefined) throw new MLError('Argument liefert keinen Wert.');
          out.push(v);
        }
      }
      return out;
    }

    ev(e, scope) {
      switch (e.t) {
        case 'num':
          return e.v;
        case 'val':
          return e.v;
        case 'str':
          return new Str(e.v, e.dq);
        case 'paren':
          return this.ev(e.e, scope);
        case 'id': {
          const v = scope.vars.get(e.name);
          if (v !== undefined) return v;
          const r = this.callFn(e.name, [], 1, scope, e);
          if (!r.length) throw new MLError(`'${e.name}' liefert keinen Wert.`);
          return r[0];
        }
        case 'bin': {
          if (e.op === '&&') return truth(this.ev(e.a, scope)) && truth(this.ev(e.b, scope)) ? 1 : 0;
          if (e.op === '||') return truth(this.ev(e.a, scope)) || truth(this.ev(e.b, scope)) ? 1 : 0;
          const a = this.ev(e.a, scope),
            b = this.ev(e.b, scope);
          return BINOPS[e.op](a, b);
        }
        case 'un': {
          const a = this.ev(e.a, scope);
          if (e.op === '-') return typeof a === 'number' ? -a : map1(a, (x) => -x);
          return map1(a, (x) => x === 0, true);
        }
        case 'post':
          return transpose(this.ev(e.a, scope));
        case 'range':
          return range(this.ev(e.a, scope), e.s ? this.ev(e.s, scope) : 1, this.ev(e.b, scope));
        case 'end': {
          const ctx = this.endStack && this.endStack[this.endStack.length - 1];
          if (!ctx) throw new MLError("'end' ist nur innerhalb einer Indizierung erlaubt.");
          const d = dims(ctx.v);
          if (ctx.n === 1) return d[0] * d[1];
          return ctx.k === 0 ? d[0] : ctx.k === 1 ? d[1] : 1;
        }
        case 'colon':
          return new Str(':');
        case 'call':
        case 'field':
        case 'dfield': {
          const r = this.evMulti(e, scope, 1);
          if (!r.length || r[0] === undefined) throw new MLError('Ausdruck liefert keinen Wert.');
          return r[0];
        }
        case 'mat':
          return this.concat(e, scope);
        case 'cell': {
          const rows = e.rows.map((row) => row.map((x) => this.ev(x, scope)));
          const r = rows.length,
            c = r ? rows[0].length : 0;
          for (const row of rows) if (row.length !== c) throw new MLError('Zeilen der Zelle haben unterschiedliche Länge.');
          const C = new Cell(r, c);
          for (let i = 0; i < r; i++) for (let j = 0; j < c; j++) C.d[j * r + i] = rows[i][j];
          return C;
        }
        case 'anon': {
          const self = this,
            captured = new Map(scope.vars);
          const f = new Obj('function_handle', {});
          f.call = (args) => {
            const sc = new Scope(scope.file, null, null);
            sc.vars = new Map(captured);
            e.params.forEach((p, k) => sc.vars.set(p, args[k]));
            return [self.ev(e.body, sc)];
          };
          return f;
        }
        case 'fhandle': {
          const self = this;
          const f = new Obj('function_handle', {});
          f.call = (args, nargout) => self.callFn(e.name, args, nargout || 1, scope, e);
          return f;
        }
      }
      throw new MLError('Unbekannter Ausdruck.');
    }

    /** Wertet aus und liefert ggf. mehrere Rückgabewerte (Funktionsaufrufe) */
    evMulti(e, scope, nargout) {
      if (e.t === 'call') {
        const f = e.f;
        if (!e.brace && f.t === 'id') {
          const v = scope.vars.get(f.name);
          if (v !== undefined) {
            if (v instanceof Obj && v.call) return v.call(this.evArgs(e.args, scope), nargout);
            return [indexRead(v, this.evIdx(e.args, v, scope))];
          }
          const args = this.evArgs(e.args, scope);
          return this.callFn(f.name, args, nargout, scope, e);
        }
        const base = this.ev(f, scope);
        if (e.brace) {
          if (!(base instanceof Cell)) throw new MLError("'{}'-Indizierung ist nur für Zellen möglich.");
          const sub = indexRead(base, this.evIdx(e.args, base, scope));
          return sub.d;
        }
        if (base instanceof Obj && base.call) return base.call(this.evArgs(e.args, scope), nargout);
        return [indexRead(base, this.evIdx(e.args, base, scope))];
      }
      if (e.t === 'field' || e.t === 'dfield') {
        const base = this.ev(e.a, scope);
        const name = e.t === 'field' ? e.name : str(this.ev(e.e, scope), 'Dynamischer Feldname');
        if (base instanceof Struct) {
          if (!base.f.has(name)) throw new MLError(`Unbekanntes Feld '${name}'.`);
          return [base.f.get(name)];
        }
        if (base instanceof Obj) return [base.get(name)];
        throw new MLError(`Punkt-Indizierung ist für Werte der Klasse '${className(base)}' nicht möglich (Feld '${name}').`);
      }
      if (e.t === 'id') {
        const v = scope.vars.get(e.name);
        if (v !== undefined) return [v];
        return this.callFn(e.name, [], nargout, scope, e);
      }
      return [this.ev(e, scope)];
    }

    concat(e, scope) {
      const rows = [];
      let anyStr = false,
        anyCell = false;
      for (const row of e.rows) {
        const vals = [];
        for (const x of row) {
          if (x.t === 'call' && x.brace) vals.push(...this.evMulti(x, scope, -1));
          else vals.push(this.ev(x, scope));
        }
        for (const v of vals) {
          if (v instanceof Str) anyStr = true;
          else if (v instanceof Cell) anyCell = true;
          else if (v instanceof Struct || v instanceof Obj) {
            if (e.rows.length === 1 && row.length === 1) return v;
            throw new MLError('Struktur- und Objekt-Arrays werden nicht unterstützt.');
          }
        }
        rows.push(vals);
      }
      if (anyCell) {
        // Zellen verketten
        const blocks = rows.map((vals) => vals.map((v) => (v instanceof Cell ? v : new Cell(1, 1, [v]))));
        const hz = blocks.map((bs) => {
          const nonEmpty = bs.filter((b) => b.r * b.c);
          if (!nonEmpty.length) return new Cell(0, 0);
          const r = nonEmpty[0].r;
          const d = [];
          let c = 0;
          for (const b of nonEmpty) {
            if (b.r !== r) throw new MLError('Dimensionen der verketteten Zellen passen nicht.');
            d.push(...b.d);
            c += b.c;
          }
          return new Cell(r, c, d);
        });
        const nz = hz.filter((b) => b.r * b.c);
        if (!nz.length) return new Cell(0, 0);
        const c = nz[0].c;
        const R = new Cell(nz.reduce((s, b) => s + b.r, 0), c);
        let r0 = 0;
        for (const b of nz) {
          if (b.c !== c) throw new MLError('Dimensionen der verketteten Zellen passen nicht.');
          for (let i = 0; i < b.r; i++) for (let j = 0; j < c; j++) R.d[j * R.r + r0 + i] = b.d[j * b.r + i];
          r0 += b.r;
        }
        return R;
      }
      if (anyStr) {
        const lines = rows.map((vals) => vals.map((v) => (v instanceof Str ? v.s : String.fromCharCode(...toArr(v)))).join(''));
        const dq = rows.some((vals) => vals.some((v) => v instanceof Str && v.dq));
        if (lines.length === 1) return new Str(lines[0], dq);
        const nonEmpty = lines.filter((l) => l.length);
        if (nonEmpty.length === 1) return new Str(nonEmpty[0], dq);
        if (nonEmpty.some((l) => l.length !== nonEmpty[0].length)) throw new MLError('Zeilen einer Zeichenmatrix müssen gleich lang sein.');
        return new Str(nonEmpty.join('\n'), dq);
      }
      // Zahlen: zuerst horizontal, dann vertikal
      const hrows = [];
      for (const vals of rows) {
        const ms = vals.map(toMat).filter((m) => m.r * m.c > 0 || (m.r > 0 && vals.length === 1));
        if (!ms.length) continue;
        const r = ms[0].r;
        let c = 0;
        let logical = true;
        for (const m of ms) {
          if (m.r !== r) throw new MLError(`Dimensionen der verketteten Arrays passen nicht (horizontal: ${r} und ${m.r} Zeilen).`);
          c += m.c;
          logical = logical && m.logical;
        }
        const R = new Mat(r, c, null, logical);
        let off = 0;
        for (const m of ms) {
          R.d.set(m.d, off);
          off += m.d.length;
        }
        hrows.push(R);
      }
      if (!hrows.length) return EMPTY;
      if (hrows.length === 1) return norm(hrows[0]);
      const c = hrows[0].c;
      let r = 0;
      for (const m of hrows) {
        if (m.c !== c) throw new MLError(`Dimensionen der verketteten Arrays passen nicht (vertikal: ${c} und ${m.c} Spalten).`);
        r += m.r;
      }
      const R = new Mat(r, c, null, hrows.every((m) => m.logical));
      let r0 = 0;
      for (const m of hrows) {
        for (let j = 0; j < c; j++) for (let i = 0; i < m.r; i++) R.d[j * r + r0 + i] = m.d[j * m.r + i];
        r0 += m.r;
      }
      return norm(R);
    }

    // ---------------- Funktionen ----------------
    callFn(name, args, nargout, scope, node) {
      const file = scope.file;
      if (file && file.fns && file.fns.has(name)) return this.callUser(file.fns.get(name), file, args, nargout, scope.pstoreHolder);
      const ext = this.extra[name];
      if (ext) return wrapOut(ext.call(this, args, nargout, scope));
      const b = BUILTINS[name];
      if (b) return wrapOut(b.call(this, args, nargout, scope));
      throw new MLError(`Nicht erkannte Funktion oder Variable '${name}'.`, node && node.line);
    }
    /** Speicher für persistente Variablen: holder (z. B. je Simulink-Block) → Funktionsname → Variablen */
    callUser(fn, file, args, nargout, holder) {
      if (args.length > fn.params.length) throw new MLError(`Zu viele Eingabeargumente für '${fn.name}' (${args.length} statt höchstens ${fn.params.length}).`);
      if (this.depth >= 120) throw new MLError('Maximale Rekursionstiefe erreicht.');
      this.depth++;
      if (!holder) {
        if (!this.pstores.has(file.name)) this.pstores.set(file.name, new Map());
        holder = this.pstores.get(file.name);
      }
      if (!holder.has(fn.name)) holder.set(fn.name, new Map());
      const sc = new Scope(file, fn, holder.get(fn.name));
      sc.pstoreHolder = holder;
      for (let k = 0; k < args.length; k++) if (fn.params[k] !== '~') sc.vars.set(fn.params[k], args[k]);
      sc.nargin = args.length;
      sc.nargout = nargout;
      try {
        this.exec(fn.body, sc);
      } finally {
        this.depth--;
        if (sc.persist) for (const n of sc.persist) sc.pstore.set(n, sc.vars.get(n));
        if (sc.globals) for (const n of sc.globals) this.globals.set(n, sc.vars.get(n));
      }
      const out = [];
      const want = Math.max(1, nargout < 0 ? fn.outs.length : nargout);
      for (let k = 0; k < Math.min(want, fn.outs.length); k++) {
        const v = sc.vars.get(fn.outs[k]);
        if (v === undefined) {
          if (nargout === 0 && k === 0) break;
          throw new MLError(`Ausgabeargument '${fn.outs[k]}' wurde in '${fn.name}' nicht gesetzt.`, fn.line);
        }
        out.push(v);
      }
      if (nargout > fn.outs.length) throw new MLError(`Zu viele Ausgabeargumente für '${fn.name}'.`);
      return out;
    }

    // ---------------- Abbildungen ----------------
    fig() {
      if (!this.curFig || !this.figs.includes(this.curFig)) this.newFig();
      return this.curFig;
    }
    newFig(name) {
      const f = { id: this.figSeq++, name: name || '', axes: [], cur: null };
      this.figs.push(f);
      this.curFig = f;
      return f;
    }
    ax() {
      const f = this.fig();
      if (!f.cur) {
        f.cur = newAxes(1, 1, [1]);
        f.axes.push(f.cur);
      }
      return f.cur;
    }
  }
  function newAxes(m, n, p) {
    return { m, n, p, lines: [], xlabel: '', ylabel: '', title: '', grid: false, hold: false, xlim: null, ylim: null, equal: false, legend: null };
  }
  function wrapOut(r) {
    if (r === undefined) return [];
    if (Array.isArray(r)) return r;
    return [r];
  }
  function wrapErr(e, fileName) {
    if (e instanceof MLError) {
      if (!e.file) e.file = fileName;
      return e;
    }
    const me = new MLError(e && e.message ? e.message : String(e));
    me.file = fileName;
    return me;
  }

  // ==================================================================
  // Bibliothek
  // ==================================================================
  const B = {};
  const arg = (a, k, def) => (a.length > k ? a[k] : def);
  const num = (v, what) => scalar(v, what);
  const elem = (f) =>
    function (a) {
      if (!a.length) throw new MLError('Nicht genügend Eingabeargumente.');
      return map1(a[0], f);
    };
  const elem2 = (f) =>
    function (a) {
      if (a.length < 2) throw new MLError('Nicht genügend Eingabeargumente.');
      return ew(a[0], a[1], f);
    };
  const D2R = Math.PI / 180;

  // Konstanten
  B.pi = () => Math.PI;
  B.eps = (a) => (a.length ? Math.pow(2, Math.floor(Math.log2(Math.abs(num(a[0])))) - 52) : 2.220446049250313e-16);
  B.Inf = B.inf = () => Infinity;
  B.NaN = B.nan = () => NaN;
  B.true = (a) => (a.length ? map1(fill(a, 1), () => 1, true) : 1);
  B.false = (a) => (a.length ? map1(fill(a, 0), () => 0, true) : 0);

  // Erzeugung
  function sizeArgs(a) {
    const nums = a.filter((x) => !(x instanceof Str));
    if (!nums.length) return [1, 1];
    if (nums.length === 1) {
      const v = toArr(nums[0]);
      return v.length === 1 ? [v[0], v[0]] : [v[0], v[1]];
    }
    for (let k = 2; k < nums.length; k++) if (num(nums[k]) !== 1) throw new MLError('Nur zweidimensionale Matrizen werden unterstützt.');
    return [num(nums[0]), num(nums[1])];
  }
  function fill(a, val) {
    let [r, c] = sizeArgs(a);
    r = Math.max(0, Math.floor(r));
    c = Math.max(0, Math.floor(c));
    if (r * c > 5e6) throw new MLError('Matrix ist zu groß.');
    const m = new Mat(r, c);
    if (val !== 0) m.d.fill(val);
    return norm(m);
  }
  B.zeros = (a) => fill(a, 0);
  B.ones = (a) => fill(a, 1);
  B.NaNmat = (a) => fill(a, NaN);
  B.eye = (a) => {
    const [r, c] = sizeArgs(a);
    return norm(eye(r, c));
  };
  B.rand = (a) => map1(fill(a, 0), () => Math.random());
  B.randn = (a) =>
    map1(fill(a, 0), () => {
      let u = 0,
        v = 0;
      while (u === 0) u = Math.random();
      while (v === 0) v = Math.random();
      return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
    });
  B.randi = (a) => {
    const lim = toArr(a[0]);
    const lo = lim.length > 1 ? lim[0] : 1,
      hi = lim.length > 1 ? lim[1] : lim[0];
    return map1(fill(a.slice(1), 0), () => lo + Math.floor(Math.random() * (hi - lo + 1)));
  };
  B.linspace = (a) => {
    const x0 = num(a[0]),
      x1 = num(a[1]),
      n = a.length > 2 ? Math.floor(num(a[2])) : 100;
    if (n < 2) return x1;
    return rowVec(Array.from({ length: n }, (_, i) => x0 + ((x1 - x0) * i) / (n - 1)));
  };
  B.colon = (a) => range(a[0], a.length > 2 ? a[1] : 1, a.length > 2 ? a[2] : a[1]);
  B.diag = (a) => {
    const m = toMat(a[0]);
    const k = a.length > 1 ? num(a[1]) : 0;
    if (m.r === 1 || m.c === 1) {
      const n = m.d.length + Math.abs(k);
      const R = new Mat(n, n);
      for (let i = 0; i < m.d.length; i++) {
        const r = k >= 0 ? i : i - k,
          c = k >= 0 ? i + k : i;
        R.d[c * n + r] = m.d[i];
      }
      return norm(R);
    }
    const out = [];
    for (let i = 0; i < Math.min(m.r, m.c); i++) {
      const r = k >= 0 ? i : i - k,
        c = k >= 0 ? i + k : i;
      if (r < m.r && c < m.c) out.push(m.d[c * m.r + r]);
    }
    return colVec(out);
  };
  B.repmat = (a) => {
    const m = toMat(a[0]);
    const [rr, cc] = sizeArgs(a.slice(1));
    const R = new Mat(m.r * rr, m.c * cc);
    for (let bj = 0; bj < cc; bj++) for (let bi = 0; bi < rr; bi++) for (let j = 0; j < m.c; j++) for (let i = 0; i < m.r; i++) R.d[(bj * m.c + j) * R.r + bi * m.r + i] = m.d[j * m.r + i];
    return norm(R);
  };
  B.reshape = (a) => {
    const m = toMat(a[0]);
    let [r, c] = a.length === 2 ? toArr(a[1]) : [a[1], a[2]].map((x) => (x instanceof Mat && x.d.length === 0 ? -1 : num(x)));
    if (r < 0) r = m.d.length / c;
    if (c < 0) c = m.d.length / r;
    if (r * c !== m.d.length) throw new MLError('reshape: Anzahl der Elemente darf sich nicht ändern.');
    return norm(new Mat(r, c, Float64Array.from(m.d), m.logical));
  };
  B.cat = function (a) {
    const dim = num(a[0]);
    return this.concat({ rows: dim === 1 ? a.slice(1).map((v) => [{ t: 'val', v }]) : [a.slice(1).map((v) => ({ t: 'val', v }))] }, VALSCOPE);
  };
  B.horzcat = function (a) {
    return B.cat.call(this, [2, ...a]);
  };
  B.vertcat = function (a) {
    return B.cat.call(this, [1, ...a]);
  };

  // Größe & Typ
  B.size = (a, nargout) => {
    const d = dims(a[0]);
    if (a.length > 1) {
      const k = num(a[1]);
      return k <= 2 ? d[k - 1] : 1;
    }
    if (nargout <= 1) return rowVec(d);
    const out = [d[0], d[1]];
    for (let k = 2; k < nargout; k++) out.push(1);
    return out;
  };
  B.numel = (a) => numel(a[0]);
  B.length = (a) => {
    const d = dims(a[0]);
    return d[0] * d[1] === 0 ? 0 : Math.max(d[0], d[1]);
  };
  B.ndims = () => 2;
  B.isempty = (a) => (numel(a[0]) === 0 ? 1 : 0);
  B.isscalar = (a) => (numel(a[0]) === 1 ? 1 : 0);
  B.isvector = (a) => {
    const d = dims(a[0]);
    return (d[0] === 1 || d[1] === 1) && d[0] * d[1] >= 1 ? 1 : 0;
  };
  B.isrow = (a) => (dims(a[0])[0] === 1 ? 1 : 0);
  B.iscolumn = (a) => (dims(a[0])[1] === 1 ? 1 : 0);
  B.ismatrix = () => 1;
  B.class = (a) => new Str(className(a[0]));
  B.isa = (a) => {
    const c = className(a[0]),
      want = str(a[1]);
    if (want === 'numeric' || want === 'float') return c === 'double' ? 1 : 0;
    return c === want ? 1 : 0;
  };
  B.isnumeric = (a) => (className(a[0]) === 'double' ? 1 : 0);
  B.isfloat = B.isnumeric;
  B.isreal = () => 1;
  B.islogical = (a) => (a[0] instanceof Mat && a[0].logical ? 1 : 0);
  B.ischar = (a) => (a[0] instanceof Str && !a[0].dq ? 1 : 0);
  B.isstring = (a) => (a[0] instanceof Str && a[0].dq ? 1 : 0);
  B.iscell = (a) => (a[0] instanceof Cell ? 1 : 0);
  B.isstruct = (a) => (a[0] instanceof Struct ? 1 : 0);
  B.isobject = (a) => (a[0] instanceof Obj ? 1 : 0);
  B.isprop = (a) => (a[0] instanceof Obj && a[0].has(str(a[1])) ? 1 : 0);
  B.isfield = (a) => {
    if (!(a[0] instanceof Struct)) return 0;
    if (a[1] instanceof Cell) return norm(mat(a[1].r, a[1].c, a[1].d.map((x) => (a[0].f.has(x.s) ? 1 : 0)), true));
    return a[0].f.has(str(a[1])) ? 1 : 0;
  };
  B.double = (a) => (a[0] instanceof Str ? norm(toMat(a[0])) : map1(a[0], (x) => x));
  B.single = B.double;
  B.logical = (a) => map1(a[0], (x) => x !== 0, true);
  for (const [nm, lo, hi] of [
    ['int8', -128, 127],
    ['int16', -32768, 32767],
    ['int32', -2147483648, 2147483647],
    ['int64', -9007199254740991, 9007199254740991],
    ['uint8', 0, 255],
    ['uint16', 0, 65535],
    ['uint32', 0, 4294967295],
    ['uint64', 0, 9007199254740991],
  ])
    B[nm] = (a) => map1(a[0], (x) => Math.min(hi, Math.max(lo, Math.round(x))));
  B.squeeze = (a) => a[0];
  B.full = (a) => a[0];

  // Elementweise Mathematik
  B.abs = elem(Math.abs);
  B.sqrt = elem(Math.sqrt);
  B.exp = elem(Math.exp);
  B.log = elem(Math.log);
  B.log2 = elem(Math.log2);
  B.log10 = elem(Math.log10);
  B.floor = elem(Math.floor);
  B.ceil = elem(Math.ceil);
  B.fix = elem(Math.trunc);
  B.round = (a) => {
    if (a.length > 1) {
      const f = Math.pow(10, num(a[1]));
      return map1(a[0], (x) => Math.round(x * f) / f);
    }
    return map1(a[0], (x) => (x < 0 ? -Math.round(-x) : Math.round(x)));
  };
  B.sign = elem(Math.sign);
  B.sin = elem(Math.sin);
  B.cos = elem(Math.cos);
  B.tan = elem(Math.tan);
  B.asin = elem(Math.asin);
  B.acos = elem(Math.acos);
  B.atan = elem(Math.atan);
  B.sinh = elem(Math.sinh);
  B.cosh = elem(Math.cosh);
  B.tanh = elem(Math.tanh);
  B.sind = elem((x) => Math.sin(x * D2R));
  B.cosd = elem((x) => Math.cos(x * D2R));
  B.tand = elem((x) => Math.tan(x * D2R));
  B.asind = elem((x) => Math.asin(x) / D2R);
  B.acosd = elem((x) => Math.acos(x) / D2R);
  B.atand = elem((x) => Math.atan(x) / D2R);
  B.atan2 = elem2(Math.atan2);
  B.atan2d = elem2((y, x) => Math.atan2(y, x) / D2R);
  B.hypot = elem2(Math.hypot);
  B.mod = elem2((x, y) => (y === 0 ? x : x - Math.floor(x / y) * y));
  B.rem = elem2((x, y) => (y === 0 ? x : x - Math.trunc(x / y) * y));
  B.power = elem2(Math.pow);
  B.plus = (a) => BINOPS['+'](a[0], a[1]);
  B.minus = (a) => BINOPS['-'](a[0], a[1]);
  B.times = (a) => BINOPS['.*'](a[0], a[1]);
  B.rdivide = (a) => BINOPS['./'](a[0], a[1]);
  B.mtimes = (a) => matmul(a[0], a[1]);
  B.mldivide = (a) => mldivide(a[0], a[1]);
  B.mrdivide = (a) => mrdivide(a[0], a[1]);
  B.uminus = (a) => map1(a[0], (x) => -x);
  B.deg2rad = elem((x) => x * D2R);
  B.rad2deg = elem((x) => x / D2R);
  B.wrapToPi = elem((x) => {
    const y = ((((x + Math.PI) % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)) - Math.PI;
    return y === -Math.PI && x > 0 ? Math.PI : y;
  });
  B.wrapTo2Pi = elem((x) => {
    const y = ((x % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
    return y === 0 && x > 0 ? 2 * Math.PI : y;
  });
  B.wrapTo180 = elem((x) => {
    const y = ((((x + 180) % 360) + 360) % 360) - 180;
    return y === -180 && x > 0 ? 180 : y;
  });
  B.wrapTo360 = elem((x) => {
    const y = ((x % 360) + 360) % 360;
    return y === 0 && x > 0 ? 360 : y;
  });
  B.real = (a) => a[0];
  B.imag = (a) => map1(a[0], () => 0);
  B.conj = (a) => a[0];
  B.isnan = (a) => map1(a[0], Number.isNaN, true);
  B.isinf = (a) => map1(a[0], (x) => x === Infinity || x === -Infinity, true);
  B.isfinite = (a) => map1(a[0], Number.isFinite, true);
  B.not = (a) => map1(a[0], (x) => x === 0, true);
  B.and = (a) => BINOPS['&'](a[0], a[1]);
  B.or = (a) => BINOPS['|'](a[0], a[1]);
  B.xor = (a) => ew(a[0], a[1], (x, y) => (x !== 0) !== (y !== 0), true);
  B.transpose = B.ctranspose = (a) => transpose(a[0]);

  // Reduktionen
  /** Wendet f spaltenweise an (oder über die gegebene Dimension) */
  function reduce(v, dim, f) {
    const m = toMat(v);
    if (dim == null) dim = m.r === 1 ? 2 : 1;
    if (m.r * m.c === 0 && dim === 1 && m.r === 0 && m.c === 0) return f([]);
    if (dim === 1) {
      const out = new Float64Array(m.c);
      for (let j = 0; j < m.c; j++) out[j] = f(Array.from(m.d.subarray(j * m.r, j * m.r + m.r)));
      return norm(new Mat(1, m.c, out));
    }
    const out = new Float64Array(m.r);
    for (let i = 0; i < m.r; i++) {
      const row = [];
      for (let j = 0; j < m.c; j++) row.push(m.d[j * m.r + i]);
      out[i] = f(row);
    }
    return norm(new Mat(m.r, 1, out));
  }
  const dimArg = (a, k) => (a.length > k && !(a[k] instanceof Str) ? num(a[k]) : null);
  B.sum = (a) => reduce(a[0], dimArg(a, 1), (x) => x.reduce((s, v) => s + v, 0));
  B.prod = (a) => reduce(a[0], dimArg(a, 1), (x) => x.reduce((s, v) => s * v, 1));
  B.mean = (a) => reduce(a[0], dimArg(a, 1), (x) => (x.length ? x.reduce((s, v) => s + v, 0) / x.length : NaN));
  B.median = (a) =>
    reduce(a[0], dimArg(a, 1), (x) => {
      if (!x.length) return NaN;
      const s = x.slice().sort((p, q) => p - q);
      const h = s.length >> 1;
      return s.length % 2 ? s[h] : (s[h - 1] + s[h]) / 2;
    });
  B.std = (a) =>
    reduce(a[0], dimArg(a, 2), (x) => {
      const n = x.length;
      if (n < 2) return 0;
      const mu = x.reduce((s, v) => s + v, 0) / n;
      return Math.sqrt(x.reduce((s, v) => s + (v - mu) * (v - mu), 0) / (n - 1));
    });
  B.any = (a) => {
    const r = reduce(a[0], dimArg(a, 1), (x) => (x.some((v) => v !== 0 && !Number.isNaN(v)) ? 1 : 0));
    return r instanceof Mat ? new Mat(r.r, r.c, r.d, true) : r;
  };
  B.all = (a) => {
    const r = reduce(a[0], dimArg(a, 1), (x) => (x.every((v) => v !== 0) ? 1 : 0));
    return r instanceof Mat ? new Mat(r.r, r.c, r.d, true) : r;
  };
  B.nnz = (a) => toArr(a[0]).filter((v) => v !== 0).length;
  function minmax(a, nargout, better) {
    if (a.length >= 2 && numel(a[1]) > 0) return ew(a[0], a[1], (x, y) => (Number.isNaN(x) ? y : Number.isNaN(y) ? x : better(y, x) ? y : x));
    const m = toMat(a[0]);
    const dim = a.length > 2 ? num(a[2]) : m.r === 1 ? 2 : 1;
    if (m.r * m.c === 0) return nargout > 1 ? [EMPTY, EMPTY] : EMPTY;
    let iv = null;
    const vals = reduce(m, dim, (x) => {
      let bi = -1;
      for (let i = 0; i < x.length; i++) if (!Number.isNaN(x[i]) && (bi < 0 || better(x[i], x[bi]))) bi = i;
      return bi < 0 ? NaN : x[bi];
    });
    if (nargout > 1)
      iv = reduce(m, dim, (x) => {
        let bi = -1;
        for (let i = 0; i < x.length; i++) if (!Number.isNaN(x[i]) && (bi < 0 || better(x[i], x[bi]))) bi = i;
        return bi + 1;
      });
    return nargout > 1 ? [vals, iv] : vals;
  }
  B.max = (a, nargout) => minmax(a, nargout, (x, y) => x > y);
  B.min = (a, nargout) => minmax(a, nargout, (x, y) => x < y);
  B.cumsum = (a) => {
    const m = toMat(a[0]);
    const R = new Mat(m.r, m.c);
    if (m.r === 1) {
      let s = 0;
      for (let j = 0; j < m.c; j++) R.d[j] = s += m.d[j];
    } else
      for (let j = 0; j < m.c; j++) {
        let s = 0;
        for (let i = 0; i < m.r; i++) R.d[j * m.r + i] = s += m.d[j * m.r + i];
      }
    return norm(R);
  };
  B.cumprod = (a) => {
    const m = toMat(a[0]);
    const R = new Mat(m.r, m.c);
    let s = 1;
    for (let i = 0; i < m.d.length; i++) R.d[i] = s *= m.d[i];
    return norm(R);
  };
  B.diff = (a) => {
    const m = toMat(a[0]);
    if (m.r === 1 || m.c === 1) {
      const x = Array.from(m.d);
      const d = x.slice(1).map((v, i) => v - x[i]);
      return m.r === 1 ? rowVec(d) : colVec(d);
    }
    const R = new Mat(m.r - 1, m.c);
    for (let j = 0; j < m.c; j++) for (let i = 0; i < m.r - 1; i++) R.d[j * (m.r - 1) + i] = m.d[j * m.r + i + 1] - m.d[j * m.r + i];
    return norm(R);
  };
  B.find = (a, nargout) => {
    const m = toMat(a[0]);
    const limit = a.length > 1 ? num(a[1]) : Infinity;
    const idx = [];
    for (let i = 0; i < m.d.length && idx.length < limit; i++) if (m.d[i] !== 0) idx.push(i);
    if (nargout > 1) return [colVec(idx.map((k) => (k % m.r) + 1)), colVec(idx.map((k) => Math.floor(k / m.r) + 1))];
    const v = idx.map((k) => k + 1);
    return m.r === 1 ? (v.length ? rowVec(v) : new Mat(1, 0)) : v.length ? colVec(v) : new Mat(0, 1);
  };
  B.sort = (a, nargout) => {
    const m = toMat(a[0]);
    const desc = a.length > 1 && a[a.length - 1] instanceof Str && /^desc/i.test(a[a.length - 1].s);
    const x = Array.from(m.d).map((v, i) => [v, i]);
    x.sort((p, q) => (desc ? q[0] - p[0] : p[0] - q[0]) || p[1] - q[1]);
    const v = new Mat(m.r, m.c, Float64Array.from(x.map((p) => p[0])));
    const i = new Mat(m.r, m.c, Float64Array.from(x.map((p) => p[1] + 1)));
    return nargout > 1 ? [norm(v), norm(i)] : norm(v);
  };
  B.fliplr = (a) => {
    const m = toMat(a[0]);
    const R = new Mat(m.r, m.c);
    for (let j = 0; j < m.c; j++) R.d.set(m.d.subarray(j * m.r, j * m.r + m.r), (m.c - 1 - j) * m.r);
    return norm(R);
  };
  B.flipud = (a) => transpose(B.fliplr([transpose(a[0])]));
  B.flip = (a) => (dims(a[0])[0] === 1 ? B.fliplr(a) : B.flipud(a));
  B.unique = (a) => {
    const v = [...new Set(toArr(a[0]))].sort((p, q) => p - q);
    return dims(a[0])[0] === 1 ? rowVec(v) : colVec(v);
  };

  // Lineare Algebra
  B.det = (a) => det(a[0]);
  B.inv = (a) => inv(a[0]);
  B.pinv = (a) => {
    const A = toMat(a[0]);
    const At = transpose(A);
    return A.r >= A.c ? matmul(inv(matmul(At, A)), At) : matmul(At, inv(matmul(A, At)));
  };
  B.trace = (a) => {
    const m = toMat(a[0]);
    let s = 0;
    for (let i = 0; i < Math.min(m.r, m.c); i++) s += m.d[i * m.r + i];
    return s;
  };
  B.rank = (a) => {
    const m = toMat(a[0]);
    const M = Array.from({ length: m.r }, (_, i) => Array.from({ length: m.c }, (_, j) => m.d[j * m.r + i]));
    let rank = 0;
    for (let c = 0; c < m.c && rank < m.r; c++) {
      let p = rank;
      for (let r = rank + 1; r < m.r; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
      if (Math.abs(M[p][c]) < 1e-10) continue;
      [M[rank], M[p]] = [M[p], M[rank]];
      for (let r = rank + 1; r < m.r; r++) {
        const f = M[r][c] / M[rank][c];
        for (let k = c; k < m.c; k++) M[r][k] -= f * M[rank][k];
      }
      rank++;
    }
    return rank;
  };
  B.norm = (a) => {
    const m = toMat(a[0]);
    const p = a.length > 1 ? (a[1] instanceof Str ? a[1].s : num(a[1])) : 2;
    if (m.r === 1 || m.c === 1 || m.d.length === 0) {
      const x = Array.from(m.d);
      if (p === Infinity || p === 'inf' || p === 'Inf') return x.reduce((s, v) => Math.max(s, Math.abs(v)), 0);
      if (p === 1) return x.reduce((s, v) => s + Math.abs(v), 0);
      if (p === 'fro' || p === 2) return Math.sqrt(x.reduce((s, v) => s + v * v, 0));
      return Math.pow(x.reduce((s, v) => s + Math.pow(Math.abs(v), p), 0), 1 / p);
    }
    if (p === 'fro') return Math.sqrt(Array.from(m.d).reduce((s, v) => s + v * v, 0));
    if (p === 1) return Math.max(...toArr(reduce(map1(m, Math.abs), 1, (x) => x.reduce((s, v) => s + v, 0))));
    if (p === Infinity || p === 'inf') return Math.max(...toArr(reduce(map1(m, Math.abs), 2, (x) => x.reduce((s, v) => s + v, 0))));
    const AtA = toMat(matmul(transpose(m), m));
    const rows = Array.from({ length: AtA.r }, (_, i) => Array.from({ length: AtA.c }, (_, j) => AtA.d[j * AtA.r + i]));
    return Math.sqrt(Math.max(0, BS.la.eigSym(rows)[0]));
  };
  B.cross = (a) => {
    const x = toArr(a[0]),
      y = toArr(a[1]);
    if (x.length !== 3 || y.length !== 3) throw new MLError('cross: Vektoren der Länge 3 erwartet.');
    const r = [x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0]];
    return dims(a[0])[0] === 1 ? rowVec(r) : colVec(r);
  };
  B.dot = (a) => {
    const x = toArr(a[0]),
      y = toArr(a[1]);
    if (x.length !== y.length) throw new MLError('dot: Vektoren müssen gleich lang sein.');
    return x.reduce((s, v, i) => s + v * y[i], 0);
  };
  B.kron = (a) => {
    const A = toMat(a[0]),
      Bm = toMat(a[1]);
    const R = new Mat(A.r * Bm.r, A.c * Bm.c);
    for (let i = 0; i < A.r; i++) for (let j = 0; j < A.c; j++) for (let k = 0; k < Bm.r; k++) for (let l = 0; l < Bm.c; l++) R.d[(j * Bm.c + l) * R.r + i * Bm.r + k] = A.d[j * A.r + i] * Bm.d[l * Bm.r + k];
    return norm(R);
  };
  B.isequal = (a) => {
    for (let k = 1; k < a.length; k++) if (!eq(a[0], a[k])) return 0;
    return 1;
  };
  function eq(x, y) {
    if (x instanceof Str || y instanceof Str) return x instanceof Str && y instanceof Str && x.s === y.s;
    if (x instanceof Struct && y instanceof Struct) {
      if (x.f.size !== y.f.size) return false;
      for (const [k, v] of x.f) if (!y.f.has(k) || !eq(v, y.f.get(k))) return false;
      return true;
    }
    if (x instanceof Obj || y instanceof Obj) return x === y;
    if (x instanceof Cell || y instanceof Cell) return x instanceof Cell && y instanceof Cell && x.r === y.r && x.c === y.c && x.d.every((v, i) => eq(v, y.d[i]));
    const a = toMat(x),
      b = toMat(y);
    if (a.r !== b.r || a.c !== b.c) return false;
    for (let i = 0; i < a.d.length; i++) if (a.d[i] !== b.d[i]) return false;
    return true;
  }
  B.interp1 = (a) => {
    const x = toArr(a[0]);
    const vm = toMat(a[1]);
    const xq = a[2];
    const method = a.length > 3 && a[3] instanceof Str ? a[3].s.toLowerCase() : 'linear';
    const extrap = a.some((v) => v instanceof Str && v.s === 'extrap');
    const vcols = vm.r === 1 || vm.c === 1 ? [Array.from(vm.d)] : Array.from({ length: vm.c }, (_, j) => Array.from(vm.d.subarray(j * vm.r, j * vm.r + vm.r)));
    const f = (col, q) => {
      const n = x.length;
      if (q < x[0] || q > x[n - 1]) {
        if (!extrap) return NaN;
      }
      let k = 0;
      while (k < n - 2 && q > x[k + 1]) k++;
      if (method === 'nearest') {
        let bi = 0;
        for (let i = 1; i < n; i++) if (Math.abs(x[i] - q) < Math.abs(x[bi] - q)) bi = i;
        return col[bi];
      }
      if (method === 'previous') {
        let bi = -1;
        for (let i = 0; i < n; i++) if (x[i] <= q) bi = i;
        return bi < 0 ? NaN : col[bi];
      }
      if (method === 'next') {
        for (let i = 0; i < n; i++) if (x[i] >= q) return col[i];
        return NaN;
      }
      const t = (q - x[k]) / (x[k + 1] - x[k]);
      return col[k] + t * (col[k + 1] - col[k]);
    };
    if (vcols.length === 1) return map1(xq, (q) => f(vcols[0], q));
    const qs = toArr(xq);
    const R = new Mat(qs.length, vcols.length);
    vcols.forEach((col, j) => qs.forEach((q, i) => (R.d[j * qs.length + i] = f(col, q))));
    return norm(R);
  };

  // Zeichenketten & Ausgabe
  function sprintf(fmt, args) {
    const items = [];
    for (const v of args) {
      if (v instanceof Str) items.push(v.s);
      else if (v instanceof Cell) v.d.forEach((x) => items.push(x instanceof Str ? x.s : num(x)));
      else for (const x of toArr(v)) items.push(x);
    }
    fmt = fmt.replace(/\\(n|t|\\|r|a)/g, (_, c) => ({ n: '\n', t: '\t', '\\': '\\', r: '\r', a: '' })[c]);
    const parts = [];
    const re = /%([-+ 0#]*)(\d+|\*)?(?:\.(\d+|\*))?([diufeEgGsScxXo%])/g;
    let last = 0,
      m;
    while ((m = re.exec(fmt))) {
      if (m.index > last) parts.push({ lit: fmt.slice(last, m.index) });
      parts.push(m[4] === '%' ? { lit: '%' } : { flags: m[1], width: m[2], prec: m[3], conv: m[4] });
      last = re.lastIndex;
    }
    if (last < fmt.length) parts.push({ lit: fmt.slice(last) });
    const nConv = parts.filter((p) => p.conv).length;
    let out = '',
      k = 0;
    do {
      for (const p of parts) {
        if (p.lit != null) {
          out += p.lit;
          continue;
        }
        if (k >= items.length) {
          if (items.length) return out;
          continue;
        }
        let width = p.width,
          prec = p.prec;
        if (width === '*') width = items[k++];
        if (prec === '*') prec = items[k++];
        out += fmtOne(items[k++], p.flags, width, prec, p.conv);
      }
    } while (nConv && k < items.length);
    return out;
  }
  function fmtOne(v, flags, width, prec, conv) {
    let s;
    const p = prec != null ? +prec : undefined;
    if (conv === 's' || conv === 'S') s = typeof v === 'string' ? v : fmtG(v, p);
    else if (conv === 'c') s = typeof v === 'string' ? v : String.fromCharCode(v);
    else {
      if (typeof v === 'string') {
        if (conv === 'd' || conv === 'i') return v;
        v = Number(v);
      }
      if ((conv === 'd' || conv === 'i' || conv === 'u') && !Number.isInteger(v)) conv = 'e';
      switch (conv) {
        case 'd':
        case 'i':
        case 'u':
          s = String(Math.abs(v));
          break;
        case 'f':
          s = Number.isFinite(v) ? Math.abs(v).toFixed(p == null ? 6 : p) : Number.isNaN(v) ? 'NaN' : 'Inf';
          break;
        case 'e':
        case 'E':
          s = Number.isFinite(v) ? Math.abs(v).toExponential(p == null ? 6 : p).replace(/e([+-])(\d)$/, 'e$10$2') : Number.isNaN(v) ? 'NaN' : 'Inf';
          if (conv === 'E') s = s.toUpperCase();
          break;
        case 'g':
        case 'G':
          s = fmtG(Math.abs(v), p);
          break;
        case 'x':
          s = Math.abs(Math.round(v)).toString(16);
          break;
        case 'X':
          s = Math.abs(Math.round(v)).toString(16).toUpperCase();
          break;
        case 'o':
          s = Math.abs(Math.round(v)).toString(8);
          break;
      }
      const neg = v < 0 || Object.is(v, -0);
      if (neg && !(Number.isNaN(v))) s = '-' + s;
      else if (flags.includes('+')) s = '+' + s;
      else if (flags.includes(' ')) s = ' ' + s;
      if (width && flags.includes('0') && !flags.includes('-') && s.length < +width) {
        const sign = /^[-+ ]/.test(s) ? s[0] : '';
        s = sign + '0'.repeat(+width - s.length) + s.slice(sign.length);
      }
    }
    if (width && s.length < +width) s = flags.includes('-') ? s + ' '.repeat(+width - s.length) : ' '.repeat(+width - s.length) + s;
    return s;
  }
  function fmtG(v, p) {
    if (!Number.isFinite(v)) return Number.isNaN(v) ? 'NaN' : v > 0 ? 'Inf' : '-Inf';
    p = p == null ? 6 : p || 1;
    if (v === 0) return '0';
    const e = Math.floor(Math.log10(Math.abs(v)));
    if (e < -4 || e >= p) return v.toExponential(p - 1).replace(/\.?0+e/, 'e').replace(/e([+-])(\d)$/, 'e$10$2');
    return String(+v.toFixed(Math.max(0, p - 1 - e)));
  }
  B.sprintf = (a) => new Str(sprintf(str(a[0], 'Format'), a.slice(1)));
  B.fprintf = function (a) {
    let k = 0;
    if (typeof a[0] === 'number') k = 1;
    if (!a.length) return;
    const s = a[k] instanceof Str ? sprintf(a[k].s, a.slice(k + 1)) : sprintf('%g', a.slice(k));
    this.print(s);
  };
  B.disp = function (a) {
    const v = a[0];
    if (v instanceof Str) this.print(v.s + '\n');
    else if (typeof v === 'number') this.print(fmtNum(v).padStart(String(fmtNum(v)).length + 4) + '\n');
    else if (v instanceof Mat) this.print(v.r * v.c ? fmtMatrix(v) + '\n' : '');
    else this.print(display('', v).replace(/^ = ?\n?/, ''));
  };
  B.display = function (a) {
    this.print(display('ans', a[0]));
  };
  B.num2str = (a) => {
    const v = a[0];
    if (v instanceof Str) return v;
    if (a.length > 1) {
      if (a[1] instanceof Str) return new Str(sprintf(a[1].s, [v]));
      const p = num(a[1]);
      return new Str(toArr(v).map((x) => fmtG(x, p)).join('  '));
    }
    const m = toMat(v);
    const f = (x) => (Number.isInteger(x) ? String(x) : fmtG(x, Math.max(5, Math.floor(Math.log10(Math.max(Math.abs(x), 1))) + 5)));
    if (m.r <= 1) return new Str(Array.from(m.d, f).join('  '));
    const rows = [];
    for (let i = 0; i < m.r; i++) {
      const row = [];
      for (let j = 0; j < m.c; j++) row.push(f(m.d[j * m.r + i]));
      rows.push(row.join('  '));
    }
    return new Str(rows.join('\n'));
  };
  B.mat2str = (a) => {
    const v = a[0];
    if (v instanceof Str) return new Str("'" + v.s + "'");
    const p = a.length > 1 ? num(a[1]) : 15;
    const m = toMat(v);
    const f = (x) => (Number.isInteger(x) ? String(x) : String(+x.toPrecision(p)));
    if (m.r * m.c === 1) return new Str(f(m.d[0]));
    const rows = [];
    for (let i = 0; i < m.r; i++) {
      const row = [];
      for (let j = 0; j < m.c; j++) row.push(f(m.d[j * m.r + i]));
      rows.push(row.join(' '));
    }
    return new Str('[' + rows.join(';') + ']');
  };
  B.int2str = (a) => new Str(toArr(B.round([a[0]])).join('  '));
  B.str2num = B.str2double = (a) => {
    const v = parseFloat(str(a[0]));
    return Number.isNaN(v) ? NaN : v;
  };
  B.char = (a) => (a[0] instanceof Str ? new Str(a[0].s) : new Str(String.fromCharCode(...toArr(a[0]))));
  B.string = (a) => (a[0] instanceof Str ? new Str(a[0].s, true) : new Str(fmtG(num(a[0]), 5), true));
  B.strcat = (a) => new Str(a.map((v) => (v instanceof Str ? v.s.replace(/\s+$/, '') : String.fromCharCode(...toArr(v)))).join(''));
  B.strcmp = (a) => (a[0] instanceof Str && a[1] instanceof Str && a[0].s === a[1].s ? 1 : 0);
  B.strcmpi = (a) => (a[0] instanceof Str && a[1] instanceof Str && a[0].s.toLowerCase() === a[1].s.toLowerCase() ? 1 : 0);
  B.strncmp = (a) => (a[0] instanceof Str && a[1] instanceof Str && a[0].s.slice(0, num(a[2])) === a[1].s.slice(0, num(a[2])) ? 1 : 0);
  B.upper = (a) => new Str(str(a[0]).toUpperCase(), a[0].dq);
  B.lower = (a) => new Str(str(a[0]).toLowerCase(), a[0].dq);
  B.strtrim = (a) => new Str(str(a[0]).trim(), a[0].dq);
  B.strrep = (a) => new Str(str(a[0]).split(str(a[1])).join(str(a[2])), a[0].dq);
  B.contains = (a) => (str(a[0]).includes(str(a[1])) ? 1 : 0);
  B.startsWith = (a) => (str(a[0]).startsWith(str(a[1])) ? 1 : 0);
  B.endsWith = (a) => (str(a[0]).endsWith(str(a[1])) ? 1 : 0);
  B.strsplit = (a) => {
    const parts = str(a[0]).split(a.length > 1 ? str(a[1]) : /\s+/);
    return new Cell(1, parts.length, parts.map((p) => new Str(p)));
  };
  B.strjoin = (a) => new Str(a[0].d.map((x) => x.s).join(a.length > 1 ? str(a[1]) : ' '));
  B.newline = () => new Str('\n');
  B.blanks = (a) => new Str(' '.repeat(num(a[0])));
  B.error = function (a) {
    if (!a.length) return;
    let k = 0;
    if (a.length > 1 && a[0] instanceof Str && /^[\w-]+(:[\w-]+)+$/.test(a[0].s)) k = 1;
    throw new MLError(a[k] instanceof Str ? sprintf(a[k].s, a.slice(k + 1)) : 'Fehler');
  };
  B.warning = function (a) {
    if (!a.length) return;
    if (a[0] instanceof Str && (a[0].s === 'off' || a[0].s === 'on' || a[0].s === 'query')) return;
    let k = 0;
    if (a.length > 1 && a[0] instanceof Str && /^[\w-]+(:[\w-]+)+$/.test(a[0].s)) k = 1;
    this.warn(sprintf(str(a[k]), a.slice(k + 1)));
  };
  B.assert = (a) => {
    if (!truth(a[0])) throw new MLError(a.length > 1 ? sprintf(str(a[1]), a.slice(2)) : 'Assertion fehlgeschlagen.');
  };

  // Strukturen & Zellen
  B.struct = (a) => {
    const s = new Struct();
    for (let k = 0; k + 1 < a.length; k += 2) s.f.set(str(a[k], 'Feldname'), a[k + 1] instanceof Cell && a[k + 1].d.length === 1 ? a[k + 1].d[0] : a[k + 1]);
    return s;
  };
  B.fieldnames = (a) => {
    const v = a[0];
    const keys = v instanceof Struct ? [...v.f.keys()] : v instanceof Obj ? [...v.p.keys()].filter((k) => !k.startsWith('_')) : [];
    return new Cell(keys.length, 1, keys.map((k) => new Str(k)));
  };
  B.rmfield = (a) => {
    const s = a[0].clone();
    s.f.delete(str(a[1]));
    return s;
  };
  B.setfield = (a) => {
    const s = a[0].clone();
    s.f.set(str(a[1]), a[2]);
    return s;
  };
  B.getfield = (a) => {
    if (!a[0].f.has(str(a[1]))) throw new MLError(`Unbekanntes Feld '${str(a[1])}'.`);
    return a[0].f.get(str(a[1]));
  };
  B.cell = (a) => {
    const [r, c] = sizeArgs(a);
    return new Cell(r, c);
  };
  B.num2cell = (a) => {
    const m = toMat(a[0]);
    return new Cell(m.r, m.c, Array.from(m.d));
  };
  B.cell2mat = function (a) {
    const c = a[0];
    return this.concat({ rows: Array.from({ length: c.r }, (_, i) => Array.from({ length: c.c }, (_, j) => ({ t: 'val', v: c.d[j * c.r + i] }))) }, VALSCOPE);
  };
  B.feval = function (a, nargout, scope) {
    const f = a[0];
    if (f instanceof Obj && f.call) return f.call(a.slice(1), nargout);
    return this.callFn(str(f), a.slice(1), nargout, scope);
  };
  B.cellfun = function (a, nargout) {
    const f = a[0];
    const c = a[1];
    const out = c.d.map((x) => (f.call ? f.call([x], 1)[0] : this.callFn(str(f), [x], 1, this.base)[0]));
    if (a.some((v) => v instanceof Str && v.s === 'UniformOutput')) return new Cell(c.r, c.c, out);
    return norm(mat(c.r, c.c, out.map((v) => num(v))));
  };
  B.arrayfun = function (a) {
    const f = a[0];
    const m = toMat(a[1]);
    const out = Array.from(m.d, (x) => num(f.call([x], 1)[0]));
    return norm(mat(m.r, m.c, out));
  };

  // Umgebung
  B.clc = function () {
    this.onEvent('clc');
  };
  B.clear = function (a, nargout, scope) {
    const names = a.map((v) => str(v));
    if (!names.length || names.includes('all') || names.includes('-all')) {
      scope.vars.clear();
      if (names.includes('all')) this.pstores.clear();
    } else for (const n of names) scope.vars.delete(n);
    this.onEvent('workspace');
  };
  B.clearvars = B.clear;
  B.exist = function (a, nargout, scope) {
    const n = str(a[0]);
    if (scope.vars.has(n)) return 1;
    if (BUILTINS[n] || this.extra[n]) return 5;
    if (scope.file && scope.file.fns.has(n)) return 2;
    return 0;
  };
  B.nargin = function (a, nargout, scope) {
    if (scope.nargin == null) throw new MLError("'nargin' ist nur in Funktionen gültig.");
    return scope.nargin;
  };
  B.nargout = function (a, nargout, scope) {
    if (scope.nargout == null) throw new MLError("'nargout' ist nur in Funktionen gültig.");
    return scope.nargout;
  };
  let ticT = 0;
  B.tic = () => {
    ticT = performance.now();
  };
  B.toc = function (a, nargout) {
    const t = (performance.now() - ticT) / 1000;
    if (nargout > 0) return t;
    this.print(`Elapsed time is ${t.toFixed(6)} seconds.\n`);
  };
  B.format = () => {};
  B.more = () => {};
  B.pause = () => {};
  B.beep = () => {};
  B.datestr = () => new Str(new Date().toLocaleString('de-DE'));
  B.isdeployed = () => 0;
  B.coder = () => {};

  // ------------------------------------------------------------------
  // Robotik
  // ------------------------------------------------------------------
  function T4(rows) {
    return fromRows(rows);
  }
  const Rx = (a) => [
    [1, 0, 0],
    [0, Math.cos(a), -Math.sin(a)],
    [0, Math.sin(a), Math.cos(a)],
  ];
  const Ry = (a) => [
    [Math.cos(a), 0, Math.sin(a)],
    [0, 1, 0],
    [-Math.sin(a), 0, Math.cos(a)],
  ];
  const Rz = (a) => [
    [Math.cos(a), -Math.sin(a), 0],
    [Math.sin(a), Math.cos(a), 0],
    [0, 0, 1],
  ];
  const mul3 = (A, Bm) => A.map((r) => [0, 1, 2].map((j) => r[0] * Bm[0][j] + r[1] * Bm[1][j] + r[2] * Bm[2][j]));
  function rotOf(v) {
    const m = toMat(v);
    if (m.r < 3 || m.c < 3) throw new MLError('Rotationsmatrix (3×3) oder homogene Transformation (4×4) erwartet.');
    return [0, 1, 2].map((i) => [0, 1, 2].map((j) => m.d[j * m.r + i]));
  }
  B.trvec2tform = (a) => {
    const v = toArr(a[0]);
    if (v.length < 3) throw new MLError('trvec2tform: Translationsvektor [x y z] erwartet.');
    return T4([
      [1, 0, 0, v[0]],
      [0, 1, 0, v[1]],
      [0, 0, 1, v[2]],
      [0, 0, 0, 1],
    ]);
  };
  B.tform2trvec = (a) => {
    const m = toMat(a[0]);
    return rowVec([m.d[12], m.d[13], m.d[14]]);
  };
  B.tform2rotm = (a) => fromRows(rotOf(a[0]));
  B.rotm2tform = (a) => {
    const R = rotOf(a[0]);
    return T4([[...R[0], 0], [...R[1], 0], [...R[2], 0], [0, 0, 0, 1]]);
  };
  function eul2R(e, seq) {
    const f = { X: Rx, Y: Ry, Z: Rz };
    let R = [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1],
    ];
    for (let k = 0; k < 3; k++) R = mul3(R, f[seq[k]](e[k]));
    return R;
  }
  B.eul2rotm = (a) => fromRows(eul2R(toArr(a[0]), a.length > 1 ? str(a[1]).toUpperCase() : 'ZYX'));
  B.eul2tform = (a) => {
    const R = eul2R(toArr(a[0]), a.length > 1 ? str(a[1]).toUpperCase() : 'ZYX');
    return T4([[...R[0], 0], [...R[1], 0], [...R[2], 0], [0, 0, 0, 1]]);
  };
  B.rotm2eul = (a) => {
    const R = rotOf(a[0]);
    const seq = a.length > 1 ? str(a[1]).toUpperCase() : 'ZYX';
    const cl = (x) => Math.max(-1, Math.min(1, x));
    if (seq === 'XYZ') {
      const b = Math.asin(cl(R[0][2]));
      if (Math.abs(R[0][2]) > 1 - 1e-12) return rowVec([Math.atan2(R[1][0], R[1][1]), b, 0]);
      return rowVec([Math.atan2(-R[1][2], R[2][2]), b, Math.atan2(-R[0][1], R[0][0])]);
    }
    if (seq === 'ZYX') {
      const b = -Math.asin(cl(R[2][0]));
      if (Math.abs(R[2][0]) > 1 - 1e-12) return rowVec([Math.atan2(-R[0][1], R[1][1]), b, 0]);
      return rowVec([Math.atan2(R[1][0], R[0][0]), b, Math.atan2(R[2][1], R[2][2])]);
    }
    if (seq === 'ZYZ') {
      const b = Math.acos(cl(R[2][2]));
      if (Math.abs(Math.sin(b)) < 1e-12) return rowVec([Math.atan2(R[1][0], R[0][0]), b, 0]);
      return rowVec([Math.atan2(R[1][2], R[0][2]), b, Math.atan2(R[2][1], -R[2][0])]);
    }
    throw new MLError(`rotm2eul: Reihenfolge '${seq}' wird nicht unterstützt (XYZ, ZYX, ZYZ).`);
  };
  B.rotx = (a) => fromRows(Rx(num(a[0]) * D2R));
  B.roty = (a) => fromRows(Ry(num(a[0]) * D2R));
  B.rotz = (a) => fromRows(Rz(num(a[0]) * D2R));
  B.axang2rotm = (a) => {
    const v = toArr(a[0]);
    const n = Math.hypot(v[0], v[1], v[2]) || 1;
    const [x, y, z] = [v[0] / n, v[1] / n, v[2] / n],
      t = v[3],
      c = Math.cos(t),
      s = Math.sin(t),
      C = 1 - c;
    return fromRows([
      [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
      [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
      [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
    ]);
  };

  /** DH-Matrix Rot_z(θ)·Trans_z(d)·Trans_x(a)·Rot_x(α) */
  function dhRows(a, al, d, th) {
    const ct = Math.cos(th),
      st = Math.sin(th),
      ca = Math.cos(al),
      sa = Math.sin(al);
    return [
      [ct, -st * ca, st * sa, a * ct],
      [st, ct * ca, -ct * sa, a * st],
      [0, sa, ca, d],
      [0, 0, 0, 1],
    ];
  }
  // rigidBodyTree & Co.
  function makeJoint(name, type) {
    type = type || 'fixed';
    if (!['revolute', 'prismatic', 'fixed'].includes(type)) throw new MLError(`rigidBodyJoint: unbekannter Gelenktyp '${type}'.`);
    const lim = type === 'revolute' ? rowVec([-Math.PI, Math.PI]) : type === 'prismatic' ? rowVec([-0.5, 0.5]) : rowVec([0, 0]);
    return new Obj('rigidBodyJoint', {
      Name: new Str(name),
      Type: new Str(type),
      PositionLimits: lim,
      HomePosition: 0,
      JointAxis: type === 'fixed' ? rowVec([0, 0, 0]) : rowVec([0, 0, 1]),
      JointToParentTransform: eye(4),
      ChildToJointTransform: eye(4),
    });
  }
  function makeBody(name) {
    return new Obj('rigidBody', {
      Name: new Str(name),
      Joint: makeJoint(name + '_jnt', 'fixed'),
      Mass: 1,
      CenterOfMass: rowVec([0, 0, 0]),
      Inertia: rowVec([1, 1, 1, 0, 0, 0]),
      Parent: EMPTY,
      Children: new Cell(1, 0),
      Visuals: new Cell(1, 0),
      Collisions: new Cell(1, 0),
    });
  }
  B.rigidBodyJoint = (a) => makeJoint(str(a[0], 'Gelenkname'), a.length > 1 ? str(a[1]) : 'fixed');
  B.rigidBody = (a) => makeBody(str(a[0], 'Körpername'));
  B.rigidBodyTree = (a) => {
    const t = new Obj('rigidBodyTree', {
      NumBodies: 0,
      Bodies: new Cell(1, 0),
      Base: makeBody('base'),
      BodyNames: new Cell(1, 0),
      BaseName: new Str('base'),
      Gravity: rowVec([0, 0, 0]),
      DataFormat: new Str('struct'),
    });
    t._bodies = [];
    for (let k = 0; k + 1 < a.length; k += 2) if (str(a[k]) === 'DataFormat') t.p.set('DataFormat', a[k + 1]);
    return t;
  };
  B.setFixedTransform = (a) => {
    const j = a[0];
    if (!(j instanceof Obj) || j.cls !== 'rigidBodyJoint') throw new MLError('setFixedTransform: erstes Argument muss ein rigidBodyJoint sein.');
    const conv = a.length > 2 ? str(a[2]).toLowerCase() : null;
    const m = toMat(a[1]);
    if (conv === 'dh' || conv === 'mdh') {
      if (m.d.length !== 4) throw new MLError('setFixedTransform: DH-Parameter [a alpha d theta] erwartet.');
      const [aa, al, d, th] = m.d;
      const type = j.get('Type').s;
      // θ (Drehgelenk) bzw. d (Schubgelenk) ist die Gelenkvariable und wird ignoriert
      const thF = type === 'revolute' ? 0 : th,
        dF = type === 'prismatic' ? 0 : d;
      if (conv === 'dh') {
        j.set('JointToParentTransform', eye(4));
        j.set('ChildToJointTransform', fromRows(dhRows(aa, al, dF, thF)));
      } else {
        const ca = Math.cos(al),
          sa = Math.sin(al),
          ct = Math.cos(thF),
          st = Math.sin(thF);
        j.set(
          'JointToParentTransform',
          fromRows([
            [ct, -st, 0, aa],
            [st * ca, ct * ca, -sa, -sa * dF],
            [st * sa, ct * sa, ca, ca * dF],
            [0, 0, 0, 1],
          ])
        );
        j.set('ChildToJointTransform', eye(4));
      }
      j._dh = { a: aa, alpha: al, d, theta: th, conv };
      return;
    }
    if (m.r !== 4 || m.c !== 4) throw new MLError('setFixedTransform: homogene 4×4-Transformation oder DH-Parameter erwartet.');
    j.set('JointToParentTransform', m);
    j.set('ChildToJointTransform', eye(4));
  };
  B.addBody = (a) => {
    const t = a[0],
      b = a[1];
    if (!(t instanceof Obj) || t.cls !== 'rigidBodyTree') throw new MLError('addBody: erstes Argument muss ein rigidBodyTree sein.');
    if (!(b instanceof Obj) || b.cls !== 'rigidBody') throw new MLError('addBody: zweites Argument muss ein rigidBody sein.');
    const parent = str(a[2], 'Name des Elternkörpers');
    const names = t._bodies.map((x) => x.body.get('Name').s);
    const bn = b.get('Name').s;
    if (names.includes(bn) || bn === 'base') throw new MLError(`addBody: Körper '${bn}' existiert bereits.`);
    if (parent !== 'base' && !names.includes(parent)) throw new MLError(`addBody: Elternkörper '${parent}' existiert nicht.`);
    const jn = b.get('Joint').get('Name').s;
    for (const x of t._bodies) if (x.joint.get('Name').s === jn) throw new MLError(`addBody: Gelenkname '${jn}' ist bereits vergeben.`);
    // MATLAB kopiert den Körper beim Hinzufügen (Gelenkdaten zum Zeitpunkt von addBody)
    const j = b.get('Joint');
    const jc = new Obj('rigidBodyJoint', Object.fromEntries(j.p));
    jc._dh = j._dh;
    t._bodies.push({ body: b, joint: jc, parent });
    t.p.set('NumBodies', t._bodies.length);
    t.p.set('BodyNames', new Cell(1, t._bodies.length, t._bodies.map((x) => x.body.get('Name'))));
    t.p.set('Bodies', new Cell(1, t._bodies.length, t._bodies.map((x) => x.body)));
  };
  B.addVisual = () => {};
  B.addCollision = () => {};
  B.clearVisual = () => {};
  /** Posen aller Körper (Liste von 4×4-Zeilenmatrizen) für eine Gelenkkonfiguration */
  function treeFrames(t, cfg) {
    const nonFixed = t._bodies.filter((x) => x.joint.get('Type').s !== 'fixed');
    let q = null;
    if (cfg) {
      if (cfg instanceof Struct || cfg instanceof Cell) throw new MLError('Konfiguration als Vektor erwarten (DataFormat row/column).');
      q = toArr(cfg);
      if (q.length !== nonFixed.length) throw new MLError(`Konfiguration muss ${nonFixed.length} Elemente haben (hat ${q.length}).`);
    }
    const poses = new Map();
    poses.set('base', [
      [1, 0, 0, 0],
      [0, 1, 0, 0],
      [0, 0, 1, 0],
      [0, 0, 0, 1],
    ]);
    const mm = (A, Bm) => A.map((r) => [0, 1, 2, 3].map((j) => r[0] * Bm[0][j] + r[1] * Bm[1][j] + r[2] * Bm[2][j] + r[3] * Bm[3][j]));
    const rows = (m) => [0, 1, 2, 3].map((i) => [0, 1, 2, 3].map((j) => m.d[j * 4 + i]));
    let k = 0;
    const out = [];
    for (const x of t._bodies) {
      const j = x.joint;
      const type = j.get('Type').s;
      let qi = 0;
      if (type !== 'fixed') {
        qi = q ? q[k] : num(j.get('HomePosition'));
        k++;
      }
      const ax = toArr(j.get('JointAxis'));
      let Jq = [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
      ];
      if (type === 'revolute') {
        const R = toMat(B.axang2rotm([rowVec([...ax, qi])]));
        const Rr = [0, 1, 2].map((i) => [0, 1, 2].map((jj) => R.d[jj * 3 + i]));
        Jq = [[...Rr[0], 0], [...Rr[1], 0], [...Rr[2], 0], [0, 0, 0, 1]];
      } else if (type === 'prismatic') {
        Jq[0][3] = ax[0] * qi;
        Jq[1][3] = ax[1] * qi;
        Jq[2][3] = ax[2] * qi;
      }
      const P = poses.get(x.parent);
      const T = mm(mm(mm(P, rows(j.get('JointToParentTransform'))), Jq), rows(j.get('ChildToJointTransform')));
      poses.set(x.body.get('Name').s, T);
      out.push({ name: x.body.get('Name').s, parent: x.parent, T, type });
    }
    return { out, poses };
  }
  B.homeConfiguration = (a) => {
    const t = a[0];
    const q = t._bodies.filter((x) => x.joint.get('Type').s !== 'fixed').map((x) => num(x.joint.get('HomePosition')));
    return t.get('DataFormat').s === 'row' ? rowVec(q) : colVec(q);
  };
  B.randomConfiguration = (a) => {
    const t = a[0];
    const q = t._bodies
      .filter((x) => x.joint.get('Type').s !== 'fixed')
      .map((x) => {
        const l = toArr(x.joint.get('PositionLimits'));
        return l[0] + Math.random() * (l[1] - l[0]);
      });
    return t.get('DataFormat').s === 'row' ? rowVec(q) : colVec(q);
  };
  B.getTransform = (a) => {
    const t = a[0];
    const { poses } = treeFrames(t, a[1]);
    const name = str(a[2], 'Körpername');
    if (!poses.has(name)) throw new MLError(`getTransform: Körper '${name}' existiert nicht.`);
    let T = poses.get(name);
    if (a.length > 3) {
      const S = poses.get(str(a[3]));
      const R = [0, 1, 2].map((i) => [0, 1, 2].map((j) => S[j][i]));
      const p = [0, 1, 2].map((i) => -(R[i][0] * S[0][3] + R[i][1] * S[1][3] + R[i][2] * S[2][3]));
      const Si = [[...R[0], p[0]], [...R[1], p[1]], [...R[2], p[2]], [0, 0, 0, 1]];
      T = Si.map((r) => [0, 1, 2, 3].map((j) => r[0] * T[0][j] + r[1] * T[1][j] + r[2] * T[2][j] + r[3] * T[3][j]));
    }
    return fromRows(T);
  };
  B.showdetails = function (a) {
    const t = a[0];
    let s = '--------------------\nRobot: (' + t._bodies.length + ' bodies)\n\n Idx    Body Name     Joint Name     Joint Type    Parent Name\n ---    ---------     ----------     ----------    -----------\n';
    t._bodies.forEach((x, i) => {
      s += ` ${String(i + 1).padStart(3)}   ${x.body.get('Name').s.padEnd(13)} ${x.joint.get('Name').s.padEnd(14)} ${x.joint.get('Type').s.padEnd(13)} ${x.parent}\n`;
    });
    this.print(s + '--------------------\n');
  };
  B.show = function (a, nargout) {
    const t = a[0];
    if (!(t instanceof Obj) || t.cls !== 'rigidBodyTree') throw new MLError('show: rigidBodyTree erwartet.');
    let cfg = null;
    let k = 1;
    if (a.length > 1 && !(a[1] instanceof Str)) {
      cfg = a[1];
      k = 2;
    }
    const opts = {};
    for (; k + 1 < a.length; k += 2) opts[str(a[k])] = a[k + 1];
    const fr = treeFrames(t, cfg);
    if (!(opts.Parent instanceof GObj)) this.ax().robot = true;
    this.onEvent('showTree', { frames: fr.out, cfg: cfg ? toArr(cfg) : null, opts });
    this.onEvent('figures');
    const ax = new GObj('matlab.graphics.axis.Axes');
    return nargout > 0 ? ax : undefined;
  };
  B.interactiveRigidBodyTree = function () {
    this.print('interactiveRigidBodyTree: im Simulator stattdessen den Bereich „IK-Designer“ verwenden (Gelenke per Schieberegler, Posen speichern).\n', 'info');
    this.onEvent('openIkDesigner', { focus: false });
    return new GObj('interactiveRigidBodyTree');
  };
  B.inverseKinematicsDesigner = function () {
    this.print('inverseKinematicsDesigner: wird im Simulator durch den Bereich „IK-Designer“ nachgebildet.\n', 'info');
    this.onEvent('openIkDesigner', { focus: false });
  };
  B.collisionBox = (a) => {
    const o = new Obj('collisionBox', { X: num(a[0]), Y: num(a[1]), Z: num(a[2]), Pose: eye(4) });
    return o;
  };
  B.collisionSphere = (a) => new Obj('collisionSphere', { Radius: num(a[0]), Pose: eye(4) });
  B.collisionCylinder = (a) => new Obj('collisionCylinder', { Radius: num(a[0]), Length: num(a[1]), Pose: eye(4) });
  B.checkCollision = function (a) {
    this.warn('checkCollision wird im Simulator nicht ausgewertet – Kollisionen im Bereich „IK-Designer“ prüfen.');
    return 0;
  };

  // STL-Dateien gibt es im Browser nicht: Funktionen ohne Wirkung
  B.stlread = () => new Obj('triangulation', { ConnectivityList: EMPTY, Points: EMPTY });
  B.triangulation = (a) => new Obj('triangulation', { ConnectivityList: a[0] || EMPTY, Points: a[1] || EMPTY });
  B.reducepatch = (a) => a[0];
  B.stlwrite = () => {};
  B.fullfile = (a) => new Str(a.map((v) => str(v)).join('/').replace(/\/+/g, '/'));
  B.tempdir = () => new Str('/tmp/');
  B.pwd = () => new Str('/Uebung2_Braccio');
  B.load = function (a) {
    this.warn(`load('${a.length ? str(a[0]) : ''}'): Dateien werden im Simulator nicht geladen.`);
    return new Struct();
  };
  B.save = function () {
    this.warn('save: Dateien werden im Simulator nicht gespeichert.');
  };

  // ------------------------------------------------------------------
  // Abbildungen
  // ------------------------------------------------------------------
  const COLORS = { r: '#ff6b6b', g: '#51cf66', b: '#4dabf7', c: '#3bc9db', m: '#f783ac', y: '#ffd43b', k: '#ced4da', w: '#f8f9fa' };
  const ORDER = ['#4dabf7', '#ff922b', '#ffd43b', '#b197fc', '#69db7c', '#3bc9db', '#f06595'];
  function parseSpec(s) {
    const o = { color: null, line: null, marker: null };
    let rest = s;
    for (const ls of ['--', '-.', ':', '-']) {
      if (rest.includes(ls)) {
        o.line = ls;
        rest = rest.replace(ls, '');
        break;
      }
    }
    for (const ch of rest) {
      if (COLORS[ch]) o.color = COLORS[ch];
      else if ('.o*+xsd^v<>ph'.includes(ch)) o.marker = ch;
    }
    if (!o.line && !o.marker) o.line = '-';
    if (!o.line && o.marker) o.line = 'none';
    return o;
  }
  B.figure = function (a, nargout) {
    if (a.length === 1 && !(a[0] instanceof Str)) {
      const id = num(a[0]);
      let f = this.figs.find((x) => x.id === id);
      if (!f) {
        f = { id, name: '', axes: [], cur: null };
        this.figs.push(f);
        this.figSeq = Math.max(this.figSeq, id + 1);
      }
      this.curFig = f;
      return nargout > 0 ? id : undefined;
    }
    let name = '';
    for (let k = 0; k + 1 < a.length; k += 2) if (str(a[k]).toLowerCase() === 'name') name = str(a[k + 1]);
    const f = this.newFig(name);
    this.onEvent('figures');
    return nargout > 0 ? f.id : undefined;
  };
  B.gcf = function () {
    return this.fig().id;
  };
  B.gca = function () {
    this.ax();
    return new GObj('matlab.graphics.axis.Axes');
  };
  B.subplot = function (a, nargout) {
    const m = num(a[0]),
      n = num(a[1]),
      p = toArr(a[2]);
    const f = this.fig();
    const key = m + ',' + n + ',' + p.join(' ');
    let ax = f.axes.find((x) => x.m + ',' + x.n + ',' + x.p.join(' ') === key);
    if (!ax) {
      // überlappende Achsen ersetzen
      f.axes = f.axes.filter((x) => !(x.m === m && x.n === n && x.p.some((v) => p.includes(v))));
      ax = newAxes(m, n, p);
      f.axes.push(ax);
    }
    f.cur = ax;
    return nargout > 0 ? new GObj('matlab.graphics.axis.Axes') : undefined;
  };
  B.plot = function (a) {
    const ax = this.ax();
    const args = a.slice();
    if (args[0] instanceof GObj) args.shift();
    if (!ax.hold) ax.lines = [];
    const groups = [];
    let k = 0;
    while (k < args.length) {
      if (args[k] instanceof Str) {
        // Name-Wert-Paare am Ende
        if (k + 1 < args.length && /^(linewidth|color|marker|markersize|displayname|linestyle)$/i.test(args[k].s)) {
          const key = args[k].s.toLowerCase();
          for (const g of groups) g.opts[key] = args[k + 1];
          k += 2;
          continue;
        }
        k++;
        continue;
      }
      let x = null,
        y = args[k];
      k++;
      if (k < args.length && !(args[k] instanceof Str)) {
        x = y;
        y = args[k];
        k++;
      }
      let spec = null;
      if (k < args.length && args[k] instanceof Str && !/^(linewidth|color|marker|markersize|displayname|linestyle)$/i.test(args[k].s)) {
        spec = args[k].s;
        k++;
      }
      groups.push({ x, y, spec, opts: {} });
    }
    for (const g of groups) {
      const ym = toMat(g.y);
      const cols = ym.r === 1 || ym.c === 1 ? [Array.from(ym.d)] : Array.from({ length: ym.c }, (_, j) => Array.from(ym.d.subarray(j * ym.r, j * ym.r + ym.r)));
      const xm = g.x == null ? null : toMat(g.x);
      cols.forEach((yc, ci) => {
        let xc;
        if (!xm) xc = yc.map((_, i) => i + 1);
        else if (xm.r === 1 || xm.c === 1) xc = Array.from(xm.d);
        else xc = Array.from(xm.d.subarray(ci * xm.r, ci * xm.r + xm.r));
        if (xc.length !== yc.length) throw new MLError('plot: Vektoren müssen gleich lang sein.');
        const sp = parseSpec(g.spec || '');
        let color = sp.color || ORDER[ax.lines.length % ORDER.length];
        if (g.opts.color) {
          const c = g.opts.color;
          if (c instanceof Str) color = COLORS[c.s[0]] || c.s;
          else {
            const v = toArr(c).map((x) => Math.round(x * 255));
            color = `rgb(${v[0]},${v[1]},${v[2]})`;
          }
        }
        ax.lines.push({ x: xc, y: yc, color, line: sp.line, marker: sp.marker, width: g.opts.linewidth ? num(g.opts.linewidth) : 1.5, name: g.opts.displayname ? g.opts.displayname.s : null });
      });
    }
    this.onEvent('figures');
  };
  B.stairs = B.plot;
  B.scatter = function (a) {
    B.plot.call(this, [a[0], a[1], new Str('o')]);
  };
  const axText = (key) =>
    function (a) {
      const args = a[0] instanceof GObj ? a.slice(1) : a;
      if (!args.length) return;
      this.ax()[key] = args[0] instanceof Str ? args[0].s : args[0] instanceof Cell ? args[0].d.map((x) => x.s).join(' ') : String(num(args[0]));
      this.onEvent('figures');
    };
  B.xlabel = axText('xlabel');
  B.ylabel = axText('ylabel');
  B.zlabel = () => {};
  B.title = axText('title');
  B.sgtitle = function (a) {
    this.fig().name = str(a[0]);
  };
  B.legend = function (a) {
    const ax = this.ax();
    const names = [];
    for (const v of a) {
      if (v instanceof Cell) v.d.forEach((x) => names.push(x.s));
      else if (v instanceof Str) {
        if (/^(location|orientation|box)$/i.test(v.s)) break;
        names.push(v.s);
      }
    }
    ax.legend = names;
    this.onEvent('figures');
  };
  B.grid = function (a) {
    const args = a[0] instanceof GObj ? a.slice(1) : a;
    const ax = this.ax();
    ax.grid = args.length ? str(args[0]) !== 'off' : !ax.grid;
    this.onEvent('figures');
  };
  B.hold = function (a) {
    const args = a[0] instanceof GObj ? a.slice(1) : a;
    const ax = this.ax();
    ax.hold = args.length ? str(args[0]) !== 'off' : !ax.hold;
  };
  B.axis = function (a) {
    const args = a[0] instanceof GObj ? a.slice(1) : a;
    if (!args.length) return;
    const ax = this.ax();
    for (const v of args) {
      if (v instanceof Str) {
        if (v.s === 'equal') ax.equal = true;
        else if (v.s === 'normal' || v.s === 'auto' || v.s === 'tight') (ax.equal = v.s === 'equal'), v.s !== 'equal' && ((ax.xlim = null), (ax.ylim = null));
      } else {
        const l = toArr(v);
        if (l.length >= 4) {
          ax.xlim = [l[0], l[1]];
          ax.ylim = [l[2], l[3]];
        }
      }
    }
    this.onEvent('figures');
  };
  B.xlim = function (a) {
    const args = a[0] instanceof GObj ? a.slice(1) : a;
    if (args.length && !(args[0] instanceof Str)) this.ax().xlim = toArr(args[0]);
    this.onEvent('figures');
  };
  B.ylim = function (a) {
    const args = a[0] instanceof GObj ? a.slice(1) : a;
    if (args.length && !(args[0] instanceof Str)) this.ax().ylim = toArr(args[0]);
    this.onEvent('figures');
  };
  B.close = function (a) {
    if (a.length && a[0] instanceof Obj) return; // z. B. close(vidObj)
    if (a.length && a[0] instanceof Str && a[0].s === 'all') this.figs = [];
    else if (a.length && !(a[0] instanceof Str)) {
      const id = num(a[0]);
      this.figs = this.figs.filter((f) => f.id !== id);
    } else if (this.curFig) this.figs = this.figs.filter((f) => f !== this.curFig);
    this.curFig = this.figs[this.figs.length - 1] || null;
    this.onEvent('figures');
  };
  B.clf = function () {
    const f = this.fig();
    f.axes = [];
    f.cur = null;
    this.onEvent('figures');
  };
  B.drawnow = function () {};
  B.linkaxes = () => {};
  B.view = () => {};
  B.camproj = () => {};
  B.rotate3d = () => {};
  B.box = () => {};
  B.colorbar = () => {};
  B.set = () => {};
  B.light = () => {};
  B.lighting = () => {};
  B.animatedline = function (a) {
    const h = new GObj('matlab.graphics.animation.AnimatedLine');
    h._pts = [];
    let color = null;
    for (let k = 0; k + 1 < a.length; k += 2)
      if (a[k] instanceof Str && a[k].s.toLowerCase() === 'color') color = a[k + 1] instanceof Str ? COLORS[a[k + 1].s[0]] || a[k + 1].s : '#fff';
    h._color = color || '#ff922b';
    this.onEvent('animatedline', h);
    return h;
  };
  B.addpoints = function (a) {
    const h = a[0];
    const x = toArr(a[1]),
      y = toArr(a[2]),
      z = a.length > 3 ? toArr(a[3]) : x.map(() => 0);
    for (let i = 0; i < x.length; i++) h._pts.push([x[i], y[i], z[i]]);
    this.onEvent('addpoints', h);
  };
  B.VideoWriter = function () {
    this.warn('VideoWriter: Videos werden im Simulator nicht erzeugt.');
    return new GObj('VideoWriter');
  };
  B.open = () => {};
  B.writeVideo = () => {};
  B.getframe = () => new GObj('frame');

  const BUILTINS = B;
  // Platzhalter-Scope für interne Verkettung (cat, cell2mat)
  const VALSCOPE = new Scope(null, null, null);

  BS.ml = {
    Interp,
    MLError,
    Mat,
    Str,
    Struct,
    Cell,
    Obj,
    GObj,
    EMPTY,
    parse,
    lex,
    mat,
    rowVec,
    colVec,
    fromRows,
    toArr,
    toMat,
    scalar,
    norm,
    display,
    fmtNum,
    className,
    dims,
    numel,
    brief,
    builtins: BUILTINS,
    treeFrames,
    sprintf,
  };
})();
