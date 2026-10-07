/* Tab „Code“: Arduino-Editor, Programmausführung, serieller Monitor */
(function () {
  'use strict';
  const BS = window.BS,
    U = BS.ui,
    h = BS.h;

  let cm = null,
    ta = null,
    statusEl,
    serialEl,
    pinsEl,
    jsBox,
    startBtn,
    stopBtn,
    errMark = null,
    tsOn = false,
    lineStart = true;

  const getCode = () => (cm ? cm.getValue() : ta.value);
  const setCode = (s) => {
    if (cm) cm.setValue(s);
    else ta.value = s;
    BS.store.set('code', s);
  };
  BS.on('loadCode', (s) => {
    setCode(s);
    U.show('code');
  });

  function setStatus(st) {
    statusEl.className = 'status ' + (st.state || '');
    statusEl.textContent =
      st.state === 'running' ? '▶ Programm läuft …' : st.state === 'error' ? st.msg : st.state === 'stopped' ? '■ ' + (st.msg || 'Gestoppt') : 'Bereit';
    if (cm && errMark != null) {
      cm.removeLineClass(errMark, 'background', 'cm-errline');
      errMark = null;
    }
    if (st.state === 'error' && st.line && cm) {
      errMark = st.line - 1;
      cm.addLineClass(errMark, 'background', 'cm-errline');
      cm.scrollIntoView({ line: errMark, ch: 0 }, 80);
    }
  }

  function appendSerial(text) {
    if (!serialEl) return;
    let out = '';
    for (const ch of text) {
      if (lineStart && tsOn) out += `[${BS.sim.t.toFixed(3)}] `;
      out += ch;
      lineStart = ch === '\n';
    }
    serialEl.textContent += out;
    if (serialEl.textContent.length > 40000) serialEl.textContent = serialEl.textContent.slice(-30000);
    serialEl.scrollTop = serialEl.scrollHeight;
  }

  U.register('code', {
    title: 'Arduino-Code',
    build(el) {
      const ex = BS.arduino.examples;
      const exSel = U.sel([['', 'Beispiel laden …']].concat(Object.entries(ex).map(([k, e]) => [k, e.name])), '', (k) => {
        if (!k) return;
        const e = ex[k];
        setCode(e.code);
        if (e.scenario) {
          BS.sim.loadScenario(e.scenario);
          BS.toast('Szenario „' + BS.sim.scenarios[e.scenario].name + '“ geladen', 'ok');
        }
        exSel.value = '';
      });
      startBtn = U.btn('▶ Hochladen & Starten', () => {
        serialEl.textContent = '';
        lineStart = true;
        BS.arduino.start(getCode());
      }, 'primary');
      stopBtn = U.btn('■ Stopp', () => BS.arduino.stop('Vom Benutzer gestoppt'), 'danger');
      const fileIn = h('input', { type: 'file', accept: '.ino,.cpp,.c,.txt', style: { display: 'none' } });
      fileIn.addEventListener('change', () => {
        const f = fileIn.files[0];
        if (!f) return;
        f.text().then((t) => {
          setCode(t);
          BS.toast(f.name + ' geladen', 'ok');
        });
        fileIn.value = '';
      });
      el.appendChild(
        U.card(
          null,
          U.row(startBtn, stopBtn, h('span', { class: 'grow' }), exSel),
          U.row(
            U.btn('Öffnen …', () => fileIn.click(), 'small'),
            U.btn('Speichern (.ino)', () => BS.download('braccio_sketch.ino', getCode()), 'small'),
            U.btn('JS anzeigen', () => {
              jsBox.style.display = jsBox.style.display === 'none' ? 'block' : 'none';
              jsBox.textContent = BS.arduino.lastJs || BS.arduino.transpile(getCode()).js;
            }, 'small'),
            fileIn
          )
        )
      );
      const wrap = h('div', { class: 'editor-wrap' });
      ta = h('textarea', { spellcheck: false });
      wrap.appendChild(ta);
      el.appendChild(wrap);
      const initial = BS.store.get('code', null) || ex.pnp.code;
      ta.value = initial;
      if (window.CodeMirror) {
        cm = CodeMirror.fromTextArea(ta, {
          mode: 'text/x-c++src',
          theme: 'material-darker',
          lineNumbers: true,
          matchBrackets: true,
          indentUnit: 2,
          tabSize: 2,
          extraKeys: { Tab: (c) => c.replaceSelection('  ') },
        });
        let t = null;
        cm.on('change', () => {
          clearTimeout(t);
          t = setTimeout(() => BS.store.set('code', cm.getValue()), 400);
        });
        setTimeout(() => cm.refresh(), 50);
      } else {
        ta.addEventListener('input', () => BS.store.set('code', ta.value));
      }
      statusEl = h('div', { class: 'status' }, 'Bereit');
      el.appendChild(statusEl);
      jsBox = h('pre', { class: 'code', style: { display: 'none', maxHeight: '240px', overflow: 'auto', marginBottom: '10px' } });
      el.appendChild(jsBox);

      pinsEl = h('div');
      serialEl = h('pre', { class: 'serial' });
      el.appendChild(
        U.card(
          'Serieller Monitor',
          serialEl,
          U.row(
            U.btn('Leeren', () => {
              serialEl.textContent = '';
              lineStart = true;
            }, 'small'),
            U.chk('Zeitstempel', false, (v) => (tsOn = v)),
            h('span', { class: 'grow' }),
            pinsEl
          )
        )
      );
      el.appendChild(
        U.card(
          null,
          h(
            'details',
            null,
            h('summary', null, 'Unterstützte Arduino-Funktionen & Grenzen'),
            h('ul', {
              html: `
<li><span class="mono">Braccio.begin()</span>, <span class="mono">Braccio.ServoMovement(stepDelay, M1..M6)</span> – wie die Bibliothek: 1°-Schritte, Begrenzung auf zulässige Winkel</li>
<li><span class="mono">Servo</span>: <span class="mono">attach(pin)</span>, <span class="mono">write()</span>, <span class="mono">writeMicroseconds()</span>, <span class="mono">read()</span> (Pins: 11 Basis, 10 Schulter, 9 Ellbogen, 5 Handgelenk, 6 Handrotation, 3 Greifer)</li>
<li><span class="mono">delay</span>, <span class="mono">millis</span>, <span class="mono">micros</span> laufen in Simulationszeit (Geschwindigkeit oben einstellbar)</li>
<li><span class="mono">Serial.print/println</span> (auch mit Nachkommastellen), <span class="mono">digitalRead/Write</span>, <span class="mono">pinMode</span></li>
<li>Förderband-Szenario: D2/D4 = Lichtschranken (HIGH = belegt), D7 = Bandmotor</li>
<li>Mathematik: <span class="mono">sin, cos, atan2, acos, sqrt, pow, radians, degrees, map, constrain, min, max, abs, round, random …</span></li>
<li>Eigene Funktionen, globale/lokale Variablen, Arrays (auch 2D), <span class="mono">#define</span> (auch mit Parametern), Casts, <span class="mono">for/while/do/switch</span></li>
<li><b>Grenzen:</b> keine Zeiger/Structs/Klassen/Referenzen; <span class="mono">static</span> wird ignoriert; Ganzzahl-Division (z. B. <span class="mono">7/2</span>) nur bei der Initialisierung von <span class="mono">int</span>-Variablen exakt; <span class="mono">Serial</span>-Eingaben liefern nichts.</li>`,
            })
          )
        )
      );
      BS.arduino.serialListeners.push(appendSerial);
      BS.arduino.statusListeners.push(setStatus);
    },
    enter() {
      if (cm) setTimeout(() => cm.refresh(), 10);
    },
    update() {
      const sim = BS.sim;
      startBtn.disabled = false;
      stopBtn.disabled = !BS.arduino.running;
      const cv = sim.conveyor;
      const led = (on, txt, green) => `<span class="led ${green ? 'green' : ''} ${on ? 'on' : ''}">${txt}</span>`;
      pinsEl.innerHTML =
        led(!!sim.pins.out[13], 'L13', true) +
        (cv ? led(cv.lsLowState, 'D2') + led(cv.lsHighState, 'D4') + led(cv.running, 'D7 Band', true) : '');
    },
  });
})();
