/* Braccio-Simulator – Arduino-Interpreter
 *
 * Übersetzt eine (Teil-)Menge von Arduino-C++ nach JavaScript und führt sie in Simulationszeit aus.
 * Unterstützt: Braccio.begin(), Braccio.ServoMovement(), Servo-Objekte (attach/write/read),
 * delay(), millis(), Serial, digitalRead/Write (Förderband/Lichtschranken), math. Funktionen,
 * eigene Funktionen, globale/lokale Variablen, Arrays (auch 2D), #define, for/while/do/switch.
 */
(function () {
  'use strict';
  const BS = (typeof window !== 'undefined' ? window : globalThis).BS;
  const C = BS.config;

  const TYPES =
    'void|bool|boolean|char|byte|int|word|long|short|float|double|String|size_t|int8_t|uint8_t|int16_t|uint16_t|int32_t|uint32_t|int64_t|uint64_t';
  const INT_TYPES = new Set(['int', 'long', 'short', 'byte', 'word', 'size_t', 'int8_t', 'uint8_t', 'int16_t', 'uint16_t', 'int32_t', 'uint32_t', 'int64_t', 'uint64_t']);
  const KEYWORDS_BEFORE_OPERAND = /(?:^|[^\w])(return|case|else|do|typeof|new|throw|await)$/;

  function matchClose(s, k, open, close) {
    // s[k] === open; liefert Index NACH der passenden schließenden Klammer
    let d = 0;
    for (let i = k; i < s.length; i++) {
      if (s[i] === open) d++;
      else if (s[i] === close) {
        d--;
        if (d === 0) return i + 1;
      }
    }
    return s.length;
  }

  /** Ende eines Ausdrucks auf Ebene 0 (vor , ; ) } ) */
  function scanExpr(s, k) {
    let d = 0;
    for (let i = k; i < s.length; i++) {
      const c = s[i];
      if (c === '(' || c === '[' || c === '{') d++;
      else if (c === ')' || c === ']' || c === '}') {
        if (d === 0) return i;
        d--;
      } else if ((c === ',' || c === ';') && d === 0) return i;
    }
    return s.length;
  }

  function scanOperand(s, j) {
    let k = j;
    while (s[k] === '-' || s[k] === '+' || s[k] === '!' || s[k] === '~' || s[k] === ' ') k++;
    if (s[k] === '(') k = matchClose(s, k, '(', ')');
    else {
      const m = /^(?:[A-Za-z_]\w*|\d+(?:\.\d*)?(?:[eE][-+]?\d+)?|\.\d+)/.exec(s.slice(k));
      if (!m) return j;
      k += m[0].length;
    }
    for (;;) {
      if (s[k] === '[') k = matchClose(s, k, '[', ']');
      else if (s[k] === '(') k = matchClose(s, k, '(', ')');
      else if (s[k] === '.' && /[A-Za-z_]/.test(s[k + 1] || '')) {
        k++;
        k += /^[A-Za-z_]\w*/.exec(s.slice(k))[0].length;
      } else break;
    }
    return k;
  }

  function transpile(src) {
    const warnings = [];
    const strings = [];
    // 1) Kommentare entfernen, Strings auslagern (Zeilenumbrüche bleiben erhalten)
    let s = '';
    {
      let i = 0;
      const n = src.length;
      while (i < n) {
        const c = src[i],
          d = src[i + 1];
        if (c === '/' && d === '/') {
          while (i < n && src[i] !== '\n') i++;
          continue;
        }
        if (c === '/' && d === '*') {
          i += 2;
          while (i < n && !(src[i] === '*' && src[i + 1] === '/')) {
            if (src[i] === '\n') s += '\n';
            i++;
          }
          i += 2;
          continue;
        }
        if (c === '"' || c === "'") {
          let j = i + 1;
          while (j < n && src[j] !== c && src[j] !== '\n') {
            if (src[j] === '\\') j++;
            j++;
          }
          strings.push(src.slice(i, j + 1));
          s += '__S' + (strings.length - 1) + '__';
          i = j + 1;
          continue;
        }
        s += c;
        i++;
      }
    }
    // 2) Präprozessor
    s = s
      .split('\n')
      .map((line, li) => {
        const t = line.trim();
        if (!t.startsWith('#')) return line;
        let m;
        if ((m = t.match(/^#define\s+([A-Za-z_]\w*)\(([^)]*)\)\s+(.+)$/))) return `const ${m[1]} = (${m[2]}) => (${m[3]});`;
        if ((m = t.match(/^#define\s+([A-Za-z_]\w*)\s+(.+)$/))) return `const ${m[1]} = ${m[2]};`;
        if (/^#(include|define|pragma)/.test(t)) return '';
        warnings.push(`Zeile ${li + 1}: Präprozessor-Anweisung „${t.split(/\s/)[0]}“ wird ignoriert`);
        return '';
      })
      .join('\n');
    // 3) Typen normalisieren
    if (/\bstatic\b/.test(s)) warnings.push('„static“ wird ignoriert (lokale static-Variablen behalten ihren Wert nicht).');
    s = s
      .replace(/\bPROGMEM\b/g, '')
      .replace(/\b(static|volatile|inline|register|extern)\b[ \t]*/g, '')
      .replace(/\bunsigned\s+long\s+long\b/g, 'long')
      .replace(/\blong\s+long\b/g, 'long')
      .replace(/\b(?:unsigned|signed)\s+(char|int|long|short)\b/g, '$1')
      .replace(/\blong\s+(int|double)\b/g, 'long')
      .replace(/\bshort\s+int\b/g, 'short')
      .replace(/\bunsigned\b/g, 'int')
      .replace(/(?<![\w.])((?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)[fF](?![\w])/g, '$1')
      .replace(/(?<![\w.])(\d+)(?:UL|ul|LU|lu|L|l|U|u)(?![\w])/g, '$1')
      .replace(/\bpgm_read_(byte|word|dword|float)(?:_near)?\s*\(\s*&/g, 'pgm_read_$1(')
      .replace(/\btrue\b/g, 'true');
    // 4) Servo-Objekte
    s = s.replace(/\bServo\s+([A-Za-z_]\w*(?:\s*,\s*[A-Za-z_]\w*)*)\s*;/g, (all, names) =>
      names
        .split(',')
        .map((n) => n.trim())
        .map((n) => `let ${n} = new __Servo("${n}");`)
        .join(' ')
    );
    // 5) Funktionsdefinitionen / Prototypen
    const fnNames = [];
    const reFn = new RegExp(`(^|[;{}\\n])([ \\t]*)(?:const\\s+)?(?:${TYPES})(?:\\s*[*&]+\\s*|\\s+)([A-Za-z_]\\w*)\\s*\\(([^()]*)\\)(\\s*)(\\{|;)`, 'g');
    s = s.replace(reFn, (all, pre, ws, name, params, ws2, end) => {
      const nl = (params.match(/\n/g) || []).length;
      if (end === ';') return pre + ws + '\n'.repeat(nl + (ws2.match(/\n/g) || []).length);
      if (!fnNames.includes(name)) fnNames.push(name);
      const ps = params
        .split(',')
        .map((p) => p.trim())
        .filter((p) => p && p !== 'void')
        .map((p) => {
          p = p.replace(/\[[^\]]*\]/g, '').replace(/[&*]/g, ' ');
          const parts = p.split('=');
          const nm = parts[0].trim().split(/\s+/).pop();
          return parts.length > 1 ? `${nm} = ${parts.slice(1).join('=').trim()}` : nm;
        })
        .join(', ');
      return `${pre}${ws}async function ${name}(${ps})${ws2}{${'\n'.repeat(nl)}`;
    });
    // 6) Casts
    {
      const re = /\(\s*(int|long|short|byte|word|char|size_t|u?int(?:8|16|32|64)_t|float|double|bool|boolean)\s*\)/g;
      let out = '',
        last = 0,
        m;
      while ((m = re.exec(s))) {
        const before = s.slice(0, m.index).replace(/\s+$/, '');
        const pc = before[before.length - 1];
        if (pc && /[\w)\]]/.test(pc) && !KEYWORDS_BEFORE_OPERAND.test(before)) continue;
        let j = re.lastIndex;
        const end = scanOperand(s, j);
        if (end <= j) continue;
        const operand = s.slice(j, end);
        const t = m[1];
        const conv = /float|double/.test(t) ? `(${operand})` : /bool/.test(t) ? `Boolean(${operand})` : `Math.trunc(${operand})`;
        out += s.slice(last, m.index) + conv;
        last = end;
        re.lastIndex = end;
      }
      s = out + s.slice(last);
    }
    // 7) Deklarationen (Arrays und Skalare)
    {
      const re = new RegExp(`(^|[^\\w.])(const\\s+)?(${TYPES})(?:\\s*[*&]+\\s*|\\s+)(?=[A-Za-z_])`, 'g');
      let out = '',
        last = 0,
        m;
      while ((m = re.exec(s))) {
        const type = m[3];
        const kw = m[2] ? 'const' : 'let';
        const isInt = INT_TYPES.has(type);
        const def = type === 'String' ? '""' : type === 'bool' || type === 'boolean' ? 'false' : '0';
        let k = re.lastIndex;
        const parts = [];
        let ok = true;
        for (;;) {
          const nm = /^(\s*)([A-Za-z_]\w*)(\s*)/.exec(s.slice(k));
          if (!nm) {
            ok = false;
            break;
          }
          k += nm[0].length;
          const part = { lead: nm[1], name: nm[2], mid: nm[3], dims: [], init: null };
          while (s[k] === '[') {
            const e = matchClose(s, k, '[', ']');
            part.dims.push(s.slice(k + 1, e - 1).trim());
            k = e;
            while (s[k] === ' ' || s[k] === '\t') k++;
          }
          if (s[k] === '=' && s[k + 1] !== '=') {
            k++;
            let st = k;
            while (s[st] === ' ' || s[st] === '\t') st++;
            if (part.dims.length && s[st] === '{') {
              const e = matchClose(s, st, '{', '}');
              part.init = s.slice(k, st) + s.slice(st, e).replace(/\{/g, '[').replace(/\}/g, ']');
              k = e;
            } else {
              const e = scanExpr(s, k);
              part.init = s.slice(k, e);
              k = e;
            }
          }
          parts.push(part);
          if (s[k] === ',') {
            k++;
            continue;
          }
          break;
        }
        if (!ok || !(s[k] === ';' || s[k] === ')' || (s[k] === ':' && false))) {
          continue; // keine Deklaration (z. B. Funktionsaufruf) – unverändert lassen
        }
        const decl = parts
          .map((p) => {
            let rhs;
            if (p.init != null) {
              rhs = p.dims.length ? p.init : isInt ? ` __int(${p.init})` : p.init;
            } else if (p.dims.length) {
              rhs = ` __arr(${def}, ${p.dims.map((d) => d || '0').join(', ')})`;
            } else rhs = ' ' + def;
            return `${p.lead}${p.name}${p.mid} =${rhs}`;
          })
          .join(',');
        out += s.slice(last, m.index) + m[1] + kw + ' ' + decl.replace(/^\s+/, (w) => (w.includes('\n') ? w : ''));
        last = k;
        re.lastIndex = k;
      }
      s = out + s.slice(last);
    }
    // 8) Schleifenwächter (verhindert Einfrieren bei Endlosschleifen ohne delay)
    {
      const ins = [];
      const re = /\b(for|while)\s*\(/g;
      let m;
      while ((m = re.exec(s))) {
        const k = s.indexOf('(', m.index);
        const e = matchClose(s, k, '(', ')');
        let j = e;
        while (/\s/.test(s[j] || '')) j++;
        if (s[j] === '{') ins.push({ at: j + 1, text: ' await __lg();', del: 0 });
        else if (s[j] === ';') {
          const before = s.slice(0, m.index).replace(/\s+$/, '');
          if (m[1] === 'while' && before.endsWith('}')) {
            // do { ... } while (...);  → passende öffnende Klammer suchen und auf "do" prüfen
            let d = 0,
              q = before.length - 1;
            for (; q >= 0; q--) {
              if (before[q] === '}') d++;
              else if (before[q] === '{' && --d === 0) break;
            }
            if (/\bdo\s*$/.test(before.slice(0, Math.max(0, q)))) continue;
          }
          ins.push({ at: j, text: '{ await __lg(); }', del: 1 });
        }
      }
      const reDo = /\bdo\s*\{/g;
      while ((m = reDo.exec(s))) ins.push({ at: m.index + m[0].length, text: ' await __lg();', del: 0 });
      ins.sort((a, b) => b.at - a.at);
      for (const i of ins) s = s.slice(0, i.at) + i.text + s.slice(i.at + i.del);
    }
    // 9) await für asynchrone Aufrufe
    const asyncNames = fnNames.concat(['delay', 'delayMicroseconds']);
    if (asyncNames.length) {
      const re = new RegExp(`(?<![\\w.$])(?<!function )(${asyncNames.join('|')})\\s*\\(`, 'g');
      s = s.replace(re, 'await $1(');
    }
    s = s.replace(/\bBraccio\s*\.\s*(ServoMovement|begin)\s*\(/g, 'await Braccio.$1(');
    // 10) Strings zurück
    s = s.replace(/__S(\d+)__/g, (a, i) => strings[+i]);
    return { js: s, fnNames, warnings };
  }

  // ------------------------------------------------------------------
  // Laufzeitumgebung
  // ------------------------------------------------------------------
  const STOP = { stop: true };
  const ard = (BS.arduino = {
    transpile,
    running: false,
    lastJs: '',
    libStep: null,
    serialListeners: [],
    statusListeners: [],
    stopReason: '',
  });

  function serialOut(text) {
    ard.serialListeners.forEach((f) => f(text));
  }
  function status(st) {
    ard.statusListeners.forEach((f) => f(st));
  }

  function fmtPrint(v, d) {
    if (typeof v === 'number') {
      if (d != null && Number.isInteger(v) && (d === 2 || d === 8 || d === 16)) return v.toString(d).toUpperCase();
      if (d != null) return v.toFixed(d);
      return Number.isInteger(v) ? String(v) : v.toFixed(2);
    }
    if (typeof v === 'boolean') return v ? '1' : '0';
    return String(v);
  }

  function makeApi() {
    const sim = BS.sim;
    const check = () => {
      if (!ard.running) throw STOP;
    };
    let lgCount = 0;
    const write = (i, v) => {
      sim.cmd[i] = BS.clamp(Math.round(+v || 0), 0, 180);
    };
    class Servo {
      constructor(name) {
        this.name = name;
        this.idx = C.libNames.indexOf(name);
        this.last = 90;
      }
      attach(pin) {
        if (C.pins[pin] != null) this.idx = C.pins[pin];
        return 1;
      }
      detach() {}
      attached() {
        return this.idx >= 0;
      }
      write(a) {
        check();
        a = +a;
        if (a > 200) a = ((a - 544) * 180) / (2400 - 544); // Mikrosekunden
        this.last = BS.clamp(Math.round(a), 0, 180);
        if (this.idx >= 0) write(this.idx, this.last);
      }
      writeMicroseconds(us) {
        this.write(((us - 544) * 180) / (2400 - 544));
      }
      read() {
        return this.last;
      }
    }
    const Braccio = {
      async begin(level) {
        check();
        const p = C.beginPose;
        for (let i = 0; i < 6; i++) write(i, p[i]);
        ard.libStep = p.slice();
        serialOut('');
        if (level !== -999) await sim.sleep(1000); // Soft-Start
      },
      async ServoMovement(stepDelay, ...v) {
        check();
        if (v.length < 6) throw new Error('Braccio.ServoMovement() erwartet 7 Argumente (stepDelay, M1..M6)');
        stepDelay = BS.clamp(Math.round(+stepDelay), 10, 30);
        const tgt = v.map((x, i) => BS.clamp(Math.round(+x), C.limits[i][0], C.limits[i][1]));
        v.forEach((x, i) => {
          if (Math.round(+x) !== tgt[i]) warnOnce(`ServoMovement: ${C.short[i]} = ${x} wurde auf ${tgt[i]} begrenzt (zulässig ${C.limits[i][0]}..${C.limits[i][1]})`);
        });
        if (!ard.libStep) {
          warnOnce('Braccio.begin() wurde nicht aufgerufen – Startwerte = aktuelle Servo-Stellung');
          ard.libStep = sim.cmd.map((x) => Math.round(x));
        }
        const st = ard.libStep;
        for (;;) {
          for (let i = 0; i < 6; i++) {
            if (st[i] !== tgt[i]) {
              st[i] += tgt[i] > st[i] ? 1 : -1;
              write(i, st[i]);
            }
          }
          await sim.sleep(stepDelay);
          if (st.every((x, i) => x === tgt[i])) break;
        }
        return 0;
      },
    };
    const warned = new Set();
    function warnOnce(msg) {
      if (warned.has(msg)) return;
      warned.add(msg);
      serialOut(`⚠ ${msg}\n`);
    }
    const pinRead = (p) => {
      const cv = sim.conveyor;
      if (cv) {
        if (p === cv.pinLow) return cv.lsLowState ? 1 : 0;
        if (p === cv.pinHigh) return cv.lsHighState ? 1 : 0;
      }
      return sim.pins.out[p] ? 1 : 0;
    };
    let seed = 12345;
    const rnd = () => {
      seed = (seed * 1103515245 + 12345) & 0x7fffffff;
      return seed / 0x7fffffff;
    };
    const api = {
      Braccio,
      __Servo: Servo,
      delay: async (ms) => {
        check();
        await sim.sleep(+ms || 0);
      },
      delayMicroseconds: async (us) => {
        check();
        await sim.sleep((+us || 0) / 1000);
      },
      millis: () => Math.floor(sim.t * 1000 - ard.t0 * 1000),
      micros: () => Math.floor(sim.t * 1e6 - ard.t0 * 1e6),
      Serial: {
        begin() {},
        end() {},
        flush() {},
        setTimeout() {},
        available: () => 0,
        read: () => -1,
        peek: () => -1,
        parseInt: () => 0,
        parseFloat: () => 0,
        readString: () => '',
        readStringUntil: () => '',
        print: (v, d) => serialOut(fmtPrint(v, d)),
        println: (v, d) => serialOut((v === undefined ? '' : fmtPrint(v, d)) + '\n'),
        write: (v) => serialOut(typeof v === 'number' ? String.fromCharCode(v) : String(v)),
      },
      pinMode: (p, m) => {
        sim.pins.mode[p] = m;
      },
      digitalWrite: (p, v) => {
        check();
        sim.pins.out[p] = v ? 1 : 0;
        BS.emit('pins');
      },
      digitalRead: (p) => pinRead(p),
      analogRead: () => 512,
      analogWrite: (p, v) => {
        sim.pins.out[p] = v > 0 ? 1 : 0;
      },
      map: (x, a, b, c, d) => Math.trunc(((x - a) * (d - c)) / (b - a) + c),
      constrain: (x, a, b) => (x < a ? a : x > b ? b : x),
      min: (a, b) => (a < b ? a : b),
      max: (a, b) => (a > b ? a : b),
      abs: (x) => (x < 0 ? -x : x),
      fabs: Math.abs,
      sq: (x) => x * x,
      sqrt: Math.sqrt,
      pow: Math.pow,
      exp: Math.exp,
      log: Math.log,
      log10: Math.log10,
      sin: Math.sin,
      cos: Math.cos,
      tan: Math.tan,
      asin: Math.asin,
      acos: Math.acos,
      atan: Math.atan,
      atan2: Math.atan2,
      floor: Math.floor,
      ceil: Math.ceil,
      round: Math.round,
      trunc: Math.trunc,
      fmod: (a, b) => a % b,
      hypot: Math.hypot,
      isnan: Number.isNaN,
      isinf: (x) => !Number.isFinite(x) && !Number.isNaN(x),
      radians: (d) => (d * Math.PI) / 180,
      degrees: (r) => (r * 180) / Math.PI,
      random: (a, b) => {
        if (b === undefined) {
          b = a;
          a = 0;
        }
        return Math.floor(a + rnd() * (b - a));
      },
      randomSeed: (s) => {
        seed = Math.floor(s) & 0x7fffffff;
      },
      PI: Math.PI,
      HALF_PI: Math.PI / 2,
      TWO_PI: Math.PI * 2,
      DEG_TO_RAD: Math.PI / 180,
      RAD_TO_DEG: 180 / Math.PI,
      EULER: Math.E,
      HIGH: 1,
      LOW: 0,
      INPUT: 0,
      OUTPUT: 1,
      INPUT_PULLUP: 2,
      LED_BUILTIN: 13,
      NULL: null,
      BIN: 2,
      OCT: 8,
      DEC: 10,
      HEX: 16,
      SOFT_START_DISABLED: -999,
      SOFT_START_DEFAULT: 0,
      int: (x) => Math.trunc(+x),
      long: (x) => Math.trunc(+x),
      byte: (x) => Math.trunc(+x) & 255,
      word: (x) => Math.trunc(+x) & 65535,
      float: (x) => +x,
      double: (x) => +x,
      F: (x) => x,
      sizeof: (x) => (Array.isArray(x) ? x.length * api.sizeof(x[0]) : 1),
      pgm_read_byte: (x) => x,
      pgm_read_word: (x) => x,
      pgm_read_dword: (x) => x,
      pgm_read_float: (x) => x,
      __int: (x) => (typeof x === 'number' ? Math.trunc(x) : x),
      __arr: function mk(def, ...dims) {
        const n = Math.max(0, Math.trunc(+dims[0] || 0));
        const a = new Array(n);
        for (let i = 0; i < n; i++) a[i] = dims.length > 1 ? mk(def, ...dims.slice(1)) : def;
        return a;
      },
      __lg: () => {
        if (!ard.running) throw STOP;
        if (++lgCount % 2000 === 0) return sim.sleep(1);
      },
    };
    return api;
  }

  function loadScript(code) {
    return new Promise((resolve, reject) => {
      const url = URL.createObjectURL(new Blob([code], { type: 'text/javascript' }));
      let err = null;
      const onErr = (ev) => {
        if (ev.filename === url) {
          err = ev;
          ev.preventDefault();
        }
      };
      window.addEventListener('error', onErr);
      const el = document.createElement('script');
      el.src = url;
      const done = () => {
        window.removeEventListener('error', onErr);
        el.remove();
        if (err) reject({ message: err.message, line: err.lineno - 1 });
        else resolve(url);
      };
      el.onload = done;
      el.onerror = () => {
        // Fallback (z. B. wenn Blob-Skripte blockiert sind): direkt auswerten
        window.removeEventListener('error', onErr);
        el.remove();
        try {
          (0, eval)(code + '\n//# sourceURL=braccio-sketch.js');
          resolve('braccio-sketch.js');
        } catch (e) {
          reject({ message: e.message, line: null });
        }
      };
      document.head.appendChild(el);
    });
  }

  ard.start = async function (src) {
    const sim = BS.sim;
    if (ard.running) {
      ard.stop('Neustart');
      await new Promise((r) => setTimeout(r, 30));
    }
    const token = (ard.runId = (ard.runId || 0) + 1);
    const st = (x) => ard.runId === token && status(x);
    if (sim.estop) {
      BS.toast('Not-Halt aktiv – bitte zuerst quittieren', 'err');
      return;
    }
    let tr;
    try {
      tr = transpile(src);
    } catch (e) {
      st({ state: 'error', msg: 'Übersetzungsfehler: ' + e.message });
      return;
    }
    const api = makeApi();
    const userNames = new Set(tr.fnNames);
    const apiNames = Object.keys(api).filter((k) => !userNames.has(k));
    const header = `window.__braccioSketch = async function (__api) { const { ${apiNames.join(', ')} } = __api;`;
    const code = header + '\n' + tr.js + `\n;return { setup: typeof setup === 'function' ? setup : null, loop: typeof loop === 'function' ? loop : null };\n};`;
    ard.lastJs = tr.js;
    let url;
    try {
      url = await loadScript(code);
    } catch (e) {
      st({ state: 'error', line: e.line, msg: `Syntaxfehler${e.line ? ' in Zeile ' + e.line : ''}: ${String(e.message).replace(/^Uncaught SyntaxError: /, '')} – fehlt evtl. ein „;“ oder eine Klammer (auch in der Zeile davor)?` });
      return;
    }
    const factory = window.__braccioSketch;
    if (sim.play) sim.stopMotion();
    ard.running = true;
    ard.libStep = null;
    ard.t0 = sim.t;
    ard.stopReason = '';
    sim.owner = 'program';
    BS.emit('owner');
    for (const w of tr.warnings) serialOut('ℹ ' + w + '\n');
    st({ state: 'running' });
    try {
      const prog = await factory(api);
      if (prog.setup) await prog.setup();
      if (!prog.loop) throw new Error('Funktion loop() fehlt');
      while (ard.running) {
        await prog.loop();
        await sim.sleep(1);
      }
      st({ state: 'stopped', msg: ard.stopReason || 'Gestoppt' });
    } catch (e) {
      if (e === STOP) st({ state: 'stopped', msg: ard.stopReason || 'Gestoppt' });
      else {
        let line = null;
        const m = String(e && e.stack).match(new RegExp(url.replace(/[.*+?^${}()|[\]\\]/g, '\\$&') + ':(\\d+):(\\d+)'));
        if (m) line = +m[1] - 1;
        const msg = (e && e.message) || String(e);
        st({ state: 'error', line, msg: `Laufzeitfehler${line ? ' in Zeile ' + line : ''}: ${msg}` });
      }
    } finally {
      URL.revokeObjectURL(url);
      if (ard.runId === token) {
        ard.running = false;
        if (sim.owner === 'program') sim.owner = 'manual';
        BS.emit('owner');
      }
      sim.notifyIdle();
    }
  };

  ard.stop = function (reason) {
    if (!ard.running) return;
    ard.running = false;
    ard.stopReason = reason || 'Gestoppt';
    BS.sim.cancelPending(STOP);
    if (BS.sim.owner === 'program') BS.sim.owner = 'manual';
    BS.emit('owner');
  };

  // ------------------------------------------------------------------
  // Beispielprogramme
  // ------------------------------------------------------------------
  const HEAD = `#include <Braccio.h>
#include <Servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;
`;
  const IK = `
// ---------------- Geometrie des Braccio [mm] ----------------
const float D1 = 71.5;    // Tisch -> Schulterachse
const float A2 = 125.0;   // Oberarm (Schulter -> Ellbogen)
const float A3 = 125.0;   // Unterarm (Ellbogen -> Handgelenk)
const float L4 = 175.0;   // Handgelenk -> TCP (Greifmitte)

int q[4];  // Ergebnis der IK: M1..M4 in Grad

// Geometrische inverse Kinematik (Ellbogen oben, nur vorne: y >= 0)
// psi = Werkzeugwinkel gegenüber der Horizontalen in Grad (-90 = senkrecht nach unten)
bool ik(float x, float y, float z, float psi) {
  float m1 = degrees(atan2(y, x));
  if (m1 < 0 || m1 > 180) return false;
  float r  = sqrt(x * x + y * y);
  float p  = radians(psi);
  float rw = r - L4 * cos(p);              // Handgelenkpunkt in der Armebene
  float zw = z - D1 - L4 * sin(p);
  float D  = (rw * rw + zw * zw - A2 * A2 - A3 * A3) / (2 * A2 * A3);
  if (D < -1.0 || D > 1.0) return false;   // außerhalb des Arbeitsraums
  float t3 = -acos(D);                     // Ellbogen oben
  float e2 = atan2(zw, rw) - atan2(A3 * sin(t3), A2 + A3 * cos(t3));
  float e3 = e2 + t3;
  // Elevationswinkel -> Braccio-Servowinkel
  float m2 = 180 - degrees(e2);
  float m3 = 270 - m2 - degrees(e3);
  float m4 = 360 - m2 - m3 - psi;
  if (m2 < 15 || m2 > 165 || m3 < 0 || m3 > 180 || m4 < 0 || m4 > 180) return false;
  q[0] = round(m1);
  q[1] = round(m2);
  q[2] = round(m3);
  q[3] = round(m4);
  return true;
}

// Sucht einen zulässigen Werkzeugwinkel – zuerst möglichst senkrecht
bool ikAuto(float x, float y, float z) {
  for (int psi = -90; psi <= 10; psi += 2) {
    if (ik(x, y, z, psi)) return true;
  }
  return false;
}
`;
  ard.examples = {
    grund: {
      name: 'Grundbewegung (ServoMovement)',
      scenario: null,
      code: `/*
  Braccio – Grundbewegung
  Braccio.ServoMovement(stepDelay, M1, M2, M3, M4, M5, M6)
    stepDelay : 10..30 ms je 1°-Schritt
    M1 Basis 0..180, M2 Schulter 15..165, M3 Ellbogen 0..180,
    M4 Handgelenk 0..180, M5 Handrotation 0..180,
    M6 Greifer 10 (offen) .. 73 (geschlossen)
*/
${HEAD}
void setup() {
  Serial.begin(9600);
  Braccio.begin();            // fährt in die Sicherheitsposition
  Serial.println("Braccio bereit");
}

void loop() {
  //                   step  M1   M2   M3   M4   M5   M6
  Braccio.ServoMovement(20,   90,  90,  90,  90,  90,  73);   // aufrecht
  delay(500);
  Braccio.ServoMovement(20,   45, 110, 150, 160,  90,  10);   // rechts vorne, Greifer auf
  Braccio.ServoMovement(20,   45, 110, 150, 160,  90,  73);   // Greifer zu
  delay(300);
  Braccio.ServoMovement(20,  135, 100, 130, 160,   0,  73);   // nach links
  Serial.print("Zeit [ms]: ");
  Serial.println(millis());
  delay(1000);
}
`,
    },
    pnp: {
      name: "Pick'n'Place mit eigener IK (Farbsortierung)",
      scenario: 'sortieren',
      code: `/*
  Pick'n'Place mit geometrischer inverser Kinematik
  Szenario: "Farbsortierung" – die Würfel werden in die gleichfarbigen Ziele gelegt.
  Koordinaten in mm, Ursprung = Mitte der Roboterbasis auf dem Tisch,
  x nach rechts, y nach vorne (M1 = 90°), z nach oben.
*/
${HEAD}${IK}
const int OFFEN = 10;
const int ZU    = 73;
const int TEMPO = 15;    // stepDelay

// TCP anfahren (Greifer bleibt im Zustand 'greifer')
bool moveTo(float x, float y, float z, int greifer) {
  if (!ikAuto(x, y, z)) {
    Serial.print("Nicht erreichbar: ");
    Serial.print(x); Serial.print(", ");
    Serial.print(y); Serial.print(", ");
    Serial.println(z);
    return false;
  }
  Braccio.ServoMovement(TEMPO, q[0], q[1], q[2], q[3], 90, greifer);
  return true;
}

void pickPlace(float px, float py, float tx, float ty) {
  const float H = 22;       // Greifhöhe: oberhalb der Würfelmitte (Kante 30 mm), damit die Finger den Tisch nicht berühren
  const float UEBER = 70;   // Sicherheitshöhe über dem Objekt
  moveTo(px, py, H + UEBER, OFFEN);
  moveTo(px, py, H, OFFEN);
  moveTo(px, py, H, ZU);              // greifen
  moveTo(px, py, H + UEBER, ZU);      // anheben
  moveTo(tx, ty, H + UEBER, ZU);
  moveTo(tx, ty, H + 3, ZU);
  moveTo(tx, ty, H + 3, OFFEN);       // ablegen
  moveTo(tx, ty, H + UEBER, OFFEN);
}

// Würfel-Positionen und Ziele (x, y)
float wuerfel[3][2] = { {-120, 230}, {0, 260}, {120, 230} };
float ziele[3][2]   = { { 230,  60}, {260, 170}, {200, 270} };

void setup() {
  Serial.begin(9600);
  Braccio.begin();
  for (int i = 0; i < 3; i++) {
    Serial.print("Würfel ");
    Serial.println(i + 1);
    pickPlace(wuerfel[i][0], wuerfel[i][1], ziele[i][0], ziele[i][1]);
  }
  Braccio.ServoMovement(20, 90, 90, 90, 90, 90, 73);
  Serial.println("Fertig!");
}

void loop() {
}
`,
    },
    band: {
      name: 'Förderband mit Lichtschranken sortieren',
      scenario: 'band',
      code: `/*
  Förderband-Sortierung (vgl. Vorlesung, Kap. 1: "Herausforderungen der Robotik")
  Pins:
    D2 = Lichtschranke unten (HIGH = Objekt erkannt, jede Box)
    D4 = Lichtschranke oben  (HIGH = Objekt erkannt, nur große Boxen)
    D7 = Förderband-Motor    (HIGH = läuft)
  Kleine Boxen -> "Ablage klein", große Boxen -> "Ablage groß".
*/
${HEAD}${IK}
const int LS_UNTEN = 2;
const int LS_OBEN  = 4;
const int BAND     = 7;
const int OFFEN = 10;
const int ZU    = 73;

// Abholposition am Bandende (Anschlag bei y = 140 mm, Bandhöhe 30 mm)
const float BAND_X = -230;
const float BAND_Y0 = 140;
const float BAND_Z = 30;

int anzahlKlein = 0;
int anzahlGross = 0;

bool moveTo(float x, float y, float z, int greifer) {
  if (!ikAuto(x, y, z)) {
    Serial.println("Ziel nicht erreichbar!");
    return false;
  }
  // M5 = 90: Backen öffnen waagrecht, quer zur Armebene
  Braccio.ServoMovement(12, q[0], q[1], q[2], q[3], 90, greifer);
  return true;
}

void setup() {
  Serial.begin(9600);
  pinMode(LS_UNTEN, INPUT);
  pinMode(LS_OBEN, INPUT);
  pinMode(BAND, OUTPUT);
  Braccio.begin();
  Braccio.ServoMovement(20, 90, 90, 90, 90, 90, OFFEN);
  digitalWrite(BAND, HIGH);
}

void loop() {
  if (digitalRead(LS_UNTEN) == HIGH) {
    delay(800);                           // Box läuft bis zum Anschlag
    digitalWrite(BAND, LOW);              // Band stoppen
    bool gross = digitalRead(LS_OBEN) == HIGH;
    float s  = gross ? 46 : 28;           // Kantenlänge
    float py = BAND_Y0 + s / 2;
    float pz = BAND_Z + s / 2;
    Serial.println(gross ? "große Box" : "kleine Box");

    moveTo(BAND_X, py, pz + 60, OFFEN);
    moveTo(BAND_X, py, pz, OFFEN);
    moveTo(BAND_X, py, pz, ZU);
    moveTo(BAND_X, py, pz + 60, ZU);
    digitalWrite(BAND, HIGH);             // Band darf weiterlaufen

    // Ablageplatz: 2×2-Raster in der jeweiligen Ablage
    int k = gross ? anzahlGross++ : anzahlKlein++;
    int spalte = k % 2;
    int reihe = (k / 2) % 2;
    int lage = k / 4;                     // ab der 5. Box wird gestapelt
    float d = gross ? 30 : 25;
    float tx = (gross ? 170 : 230) + (spalte ? d : -d);
    float ty = (gross ? 300 : 110) + (reihe ? d : -d);
    float tz = s / 2 + 14 + lage * s;     // knapp über dem Boden loslassen
    moveTo(tx, ty, tz + 50, ZU);
    moveTo(tx, ty, tz, ZU);
    moveTo(tx, ty, tz, OFFEN);
    moveTo(tx, ty, tz + 50, OFFEN);
    Braccio.ServoMovement(15, 120, 90, 90, 90, 90, OFFEN);
  }
  delay(20);
}
`,
    },
    kalib: {
      name: 'Kalibrierung: Referenzpunkte K1–K6 anfahren',
      scenario: null,
      code: `/*
  Kalibrierung – Referenzpunkte anfahren
  1. Im Tab "Kalibrierung" die Fertigungsfehler aktivieren.
  2. Programm starten: der TCP sollte genau auf die gelben Kugeln K1..K6 zeigen.
     Die Abweichung zeigt der Tab "Kalibrierung" live an.
  3. Offsets identifizieren und unten in OFFSET[] eintragen (Soll = Modell − Offset).
*/
${HEAD}${IK}
// Identifizierte Gelenk-Offsets in Grad (M1..M4)
float OFFSET[4] = {0, 0, 0, 0};

float K[6][3] = {
  { 150, 150,  60},
  {-150, 150,  60},
  {   0, 260,  40},
  {   0, 170, 130},
  { 220,  60,  80},
  {-200, 250, 100}
};

void setup() {
  Serial.begin(9600);
  Braccio.begin();
}

void loop() {
  for (int i = 0; i < 6; i++) {
    if (ikAuto(K[i][0], K[i][1], K[i][2])) {
      Braccio.ServoMovement(20,
        q[0] - OFFSET[0], q[1] - OFFSET[1], q[2] - OFFSET[2], q[3] - OFFSET[3], 90, 73);
      Serial.print("K");
      Serial.print(i + 1);
      Serial.print(" angefahren: M1..M4 = ");
      Serial.print(q[0]); Serial.print(" ");
      Serial.print(q[1]); Serial.print(" ");
      Serial.print(q[2]); Serial.print(" ");
      Serial.println(q[3]);
      delay(2000);
    }
  }
}
`,
    },
    traj: {
      name: 'Trajektorie (Polynom 5. Ordnung) mit Servo.write',
      scenario: null,
      code: `/*
  Ruckarme Punkt-zu-Punkt-Bewegung mit Polynom 5. Ordnung (Kap. 4)
  s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5,  tau = t / T
  Die Servos werden direkt über Servo.write() angesteuert (Abtastzeit 20 ms).
*/
${HEAD}
float A[6] = { 30, 110, 140, 160, 90, 10};
float B[6] = {150, 100, 130, 150, 90, 73};

void schreibe(float m[]) {
  base.write(m[0]);
  shoulder.write(m[1]);
  elbow.write(m[2]);
  wrist_ver.write(m[3]);
  wrist_rot.write(m[4]);
  gripper.write(m[5]);
}

void ptp(float von[], float nach[], float T) {
  float m[6];
  unsigned long t0 = millis();
  float t = 0;
  while (t < T) {
    t = (millis() - t0) / 1000.0;
    float tau = constrain(t / T, 0.0, 1.0);
    float s = 10 * pow(tau, 3) - 15 * pow(tau, 4) + 6 * pow(tau, 5);
    for (int i = 0; i < 6; i++) {
      m[i] = von[i] + (nach[i] - von[i]) * s;
    }
    schreibe(m);
    delay(20);
  }
}

void setup() {
  Serial.begin(9600);
  Braccio.begin();
  Braccio.ServoMovement(20, 30, 110, 140, 160, 90, 10);
}

void loop() {
  ptp(A, B, 2.0);
  delay(500);
  ptp(B, A, 2.0);
  delay(500);
}
`,
    },
    zone: {
      name: 'Arbeitsraumüberwachung testen',
      scenario: null,
      code: `/*
  Arbeitsraumüberwachung testen
  Im Tab "Arbeitsraum" die Beispielzonen laden und als Reaktion z. B. "Not-Halt" wählen.
  Der Arm schwenkt flach über den Tisch und fährt dabei in die Sperrzone.
*/
${HEAD}
void setup() {
  Serial.begin(9600);
  Braccio.begin();
  Braccio.ServoMovement(20, 0, 110, 140, 150, 90, 73);
}

void loop() {
  for (int a = 0; a <= 180; a += 30) {
    Braccio.ServoMovement(10, a, 110, 140, 150, 90, 73);
    Serial.print("M1 = ");
    Serial.println(a);
  }
  Braccio.ServoMovement(10, 0, 110, 140, 150, 90, 73);
}
`,
    },
    leer: {
      name: 'Leeres Programm',
      scenario: null,
      code: `${HEAD}
void setup() {
  Serial.begin(9600);
  Braccio.begin();
}

void loop() {
  // Braccio.ServoMovement(20, M1, M2, M3, M4, M5, M6);
}
`,
    },
  };
})();
