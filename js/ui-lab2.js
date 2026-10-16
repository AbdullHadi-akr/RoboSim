/* Tab „Labor Ü2“: Laborübung 2 (Simulink-Modell Braccio_CtrlSimMdl) mit MATLAB-Editor,
 * Befehlsfenster, Bedienpanel (HMI), IK-Designer und Blockparametern */
(function () {
  'use strict';
  const BS = window.BS,
    U = BS.ui,
    h = BS.h,
    L = BS.lab2,
    ml = BS.ml;
  const D2R = Math.PI / 180,
    R2D = 180 / Math.PI;

  const SUBS = [
    ['aufgabe', 'Aufgabe'],
    ['matlab', 'MATLAB'],
    ['panel', 'Bedienpanel'],
    ['ikd', 'IK-Designer'],
    ['simulink', 'Simulink'],
  ];
  let sub = BS.store.get('lab2sub', 'aufgabe');
  if (!SUBS.some((s) => s[0] === sub)) sub = 'aufgabe';
  const panes = {},
    subBtns = {};
  let startBtn, stopBtn, statePill, timeEl, errBox, skelBtn;

  function showSub(id) {
    sub = id;
    BS.store.set('lab2sub', id);
    for (const [k] of SUBS) {
      panes[k].style.display = k === id ? '' : 'none';
      subBtns[k].classList.toggle('on', k === id);
    }
    if (id === 'matlab' && cm) setTimeout(() => cm.refresh(), 0);
    if (id === 'matlab') renderWorkspace();
    if (id === 'ikd') renderIkd();
    if (id === 'aufgabe') renderTasks();
  }

  // ------------------------------------------------------------------
  // Kopfzeile: Simulation starten/stoppen, Fehler
  // ------------------------------------------------------------------
  function buildHead(el) {
    startBtn = U.btn('▶ Simulation starten', () => L.start(), 'primary', 'Modell Braccio_CtrlSimMdl starten (wie „Run“ in Simulink)');
    stopBtn = U.btn('■ Stopp', () => L.stop('vom Benutzer gestoppt'), 'danger');
    statePill = h('span', { class: 'pill' }, 'gestoppt');
    timeEl = h('span', { class: 'simtime' }, '');
    errBox = h('div', { class: 'status error', style: { display: 'none' } });
    skelBtn = U.btn('DH-Skelett ausblenden', () => L.hideSkeleton(), 'small', 'Darstellung von show(braccioRobot) im 3D-Fenster entfernen');
    const nav = h(
      'div',
      { class: 'subnav' },
      SUBS.map(([k, t]) => (subBtns[k] = h('button', { class: 'btn small', type: 'button', onclick: () => showSub(k) }, t)))
    );
    el.append(h('div', { class: 'labhead' }, U.row(startBtn, stopBtn, statePill, timeEl, h('span', { class: 'grow' }), skelBtn), errBox, nav));
  }
  function showError(e) {
    if (!e) {
      errBox.style.display = 'none';
      return;
    }
    errBox.style.display = '';
    errBox.innerHTML = '';
    errBox.append(h('b', null, e.title + ': '), document.createTextNode(e.msg));
    if (e.file && L.file(e.file)) {
      errBox.append(
        ' ',
        U.btn(`→ ${e.file}${e.line ? ', Zeile ' + e.line : ''}`, () => openFile(e.file, e.line), 'small'),
        ' ',
        U.btn('✕', () => {
          L.lastError = null;
          showError(null);
        }, 'small icon')
      );
    } else errBox.append(' ', U.btn('✕', () => ((L.lastError = null), showError(null)), 'small icon'));
  }
  BS.on('lab2:error', showError);

  // ------------------------------------------------------------------
  // Aufgabe
  // ------------------------------------------------------------------
  const TASKS = [
    {
      k: 'a',
      t: 'DH-Parameter parametrieren',
      d: 'Im Ini-Skript <code>Braccio_kinematics.m</code> die DH-Parameter (Abschnitt „TODO“) nach der Tabelle der Übung eintragen – Längen in m, Winkel in rad.',
      how: 'Reiter <b>MATLAB</b> → Datei <code>Braccio_kinematics.m</code>, Werte eintragen, <b>▶ Ausführen</b>. Mit <b>✓ DH prüfen</b> vergleichen.',
      act: [['Datei öffnen', () => openFile('Braccio_kinematics.m', 24)], ['✓ DH prüfen', () => runCheck('dh')]],
    },
    {
      k: 'b',
      t: 'rigidBodyTree mit show prüfen',
      d: 'Das aus den DH-Parametern erzeugte Objekt <code>braccioRobot</code> mit <code>show</code> anzeigen (abschnittsweise Ausführung, Strg+Enter).',
      how: 'Cursor in den Abschnitt „%% Visualisierung“ setzen und <b>Strg+Enter</b>. <code>show</code> zeichnet die Körper-Koordinatensysteme als gelbes <b>DH-Skelett</b> ins 3D-Fenster – es muss auf dem Roboter liegen. <code>show(braccioRobot, config)</code> zeigt zusätzlich eine Pose als Geisterbild.',
      act: [['Datei öffnen', () => openFile('Braccio_kinematics.m', 186)], ['Skelett ausblenden', () => L.hideSkeleton()]],
    },
    {
      k: 'c',
      t: 'Virtuelle Wand bei x = −200 mm',
      d: 'Mit <code>collisionBox</code> und <code>trvec2tform</code> eine virtuelle Wand an der Position x = −0,2 m parametrieren (Abschnitt „E-Fences“).',
      how: 'Die Wand erscheint grau im 3D-Fenster (Labor-x zeigt nach vorne zum Regal). Die Überwachung im Modell nutzt <code>efence1_coord.x</code>.',
      act: [['Datei öffnen', () => openFile('Braccio_kinematics.m', 238)]],
    },
    {
      k: 'd',
      t: 'Posen mit dem Inverse Kinematics Designer',
      d: 'Wand als Kollisionsobjekt hinzufügen, Posen über die Gelenk-Schieberegler einstellen, mit „Store Configuration“ speichern, Abfolge prüfen, Kollisionen prüfen und exportieren.',
      how: 'Reiter <b>IK-Designer</b>: „Add Collision Object“ = Häkchen bei <code>efence1</code>; Posen werden als Geisterbild gezeigt; <b>Export</b> legt <code>waypointData</code> (N×5, rad) im Workspace an.',
      act: [['IK-Designer öffnen', () => showSub('ikd')]],
    },
    {
      k: 'e',
      t: 'Simulationsmodell kennenlernen (CtrlMode 1)',
      d: 'Modell im manuellen Modus starten, nach jedem Modus-Wechsel RUN/PAUSE drücken und über die Slider Servo-Winkel φ vorgeben (φ = 0 ⇒ aufrechte Default-Pose).',
      how: '<b>▶ Simulation starten</b>, Reiter <b>Bedienpanel</b>: Mode „Manual (Default Pose)“, <b>▶/❚❚</b>, dann Slider φ1…φ5. Es gilt q2 = φ2 + 90°, q4 = φ4 + 90°, sonst q = φ.',
      act: [['Bedienpanel öffnen', () => showSub('panel')]],
    },
    {
      k: 'f',
      t: 'Vorwärtskinematik im Block ForwardKinematics',
      d: 'Die homogenen DH-Matrizen T1…T5 sind vorbereitet. Gesamttransformation, TCP-Position und -Orientierung sowie die Punkte B (Ellbogen) und C (Handgelenk) berechnen.',
      how: 'Reiter <b>MATLAB</b> → <code>ForwardKinematics</code> (Abschnitt TODO). Die Ausgänge steuern die Marker <b>TCP</b> (magenta), <b>B</b> (blau) und <b>C</b> (grün) im 3D-Fenster. Bei x ≤ x_Wand leuchtet die Wand rot und „Lim X“ geht an. <b>✓ FK prüfen</b> vergleicht mit der Referenz.',
      act: [['Block öffnen', () => openFile('ForwardKinematics', 39)], ['✓ FK prüfen', () => runCheck('fk')]],
    },
    {
      k: 'g',
      t: 'Pick-and-Place (CtrlMode 4)',
      d: 'Im Subsystem PickAndPlace die exportierten Waypoints als Eingang des Blocks „Polynomial Trajectory“ übernehmen und passende TimePoints wählen.',
      how: 'Reiter <b>Simulink</b> → Blockparameter PickAndPlace: Waypoints z. B. <code>waypointData\'</code>, TimePoints mit gleich vielen Einträgen. Dann Mode „Pick\'n\'Place“ und RUN.',
      act: [['Blockparameter', () => showSub('simulink')]],
    },
    {
      k: 'h',
      t: 'Teach-In (CtrlMode 2 → 3)',
      d: 'Mit den +/−-Tasten Achsen verfahren, Posen mit „TEACH-IN POSITION“ speichern und im Replay-Modus abfahren.',
      how: 'Bedienpanel: Mode „Manual (Push Buttons)“, RUN, Achse wählen, <b>−</b>/<b>+</b> gedrückt halten, „TEACH-IN POSITION“. Danach Mode „Replay“ und RUN. Gespeichert wird im Block <code>StoreWaypoints</code>.',
      act: [['Bedienpanel öffnen', () => showSub('panel')]],
    },
    {
      k: 'i',
      t: 'Regal bestücken mit CartesianPath (CtrlMode 5)',
      d: 'Acht Posen bestimmen, die x- und z-Sollwerte sowie die Startkonfiguration im Ini-Skript eintragen und den TCP in kartesischen x-z-Koordinaten verfahren.',
      how: 'Im IK-Designer werden zu jeder Pose die TCP-Koordinaten angezeigt. Werte in <code>cartesianPath.x/z/startConfig</code> eintragen, Ini-Skript ausführen, Simulation neu starten, Mode „Cartesian Control“ und RUN. Die IK läuft im Block <code>inverseKinematics3dof</code>.',
      act: [['Datei öffnen', () => openFile('Braccio_kinematics.m', 250)], ['IK-Designer', () => showSub('ikd')]],
    },
  ];
  let taskEls = {};
  function buildTasks(el) {
    el.append(
      U.card(
        'Laborübung 2 – Braccio im Simulink-Modell',
        U.hint(
          'Dieser Reiter bildet das Simulink-Modell <code>Braccio_CtrlSimMdl.slx</code> mit seiner Bibliothek nach. Die MATLAB-Dateien der Übung lassen sich im Reiter <b>MATLAB</b> bearbeiten und ausführen – der Code der MATLAB-Function-Blöcke läuft während der Simulation in jedem 10-ms-Schritt.'
        ),
        h(
          'ul',
          { class: 'tight' },
          h('li', { html: '<b>Koordinaten:</b> Labor-KS0 in m, x zeigt nach vorne (Würfel, Regal), y nach links, z nach oben. Die virtuelle Wand steht hinter dem Roboter.' }),
          h('li', { html: '<b>Winkel:</b> Das Modell rechnet mit DH-Gelenkwinkeln q (rad). Die Slider geben Servo-Winkel φ vor: q2 = φ2 + 90°, q4 = φ4 + 90°.' }),
          h('li', { html: '<b>Ablauf in Simulink nachgebildet:</b> Ini-Skript ausführen → <b>▶ Simulation starten</b> → Mode wählen → <b>RUN/PAUSE</b>. Nach jedem Modus-Wechsel geht das Modell in Pause.' }),
          h('li', { html: 'Nach dem Stoppen liegen die Ergebnisse wie in Simulink in <code>out</code> (ConfigSim, TCPpos, CtrlMode) – z. B. für <code>Braccio_plotSimResults.m</code>.' })
        )
      )
    );
    taskEls = {};
    for (const t of TASKS) {
      const badge = h('span', { class: 'badge' }, 'offen');
      taskEls[t.k] = badge;
      el.append(
        h(
          'div',
          { class: 'card task' },
          h('h3', null, h('span', { class: 'tk' }, t.k + ')'), ' ', t.t, h('span', { class: 'grow' }), badge),
          h('p', { class: 'hint', html: t.d }),
          h('p', { class: 'hint how', html: '<b>Im Simulator:</b> ' + t.how }),
          U.row(...t.act.map(([lbl, fn]) => U.btn(lbl, fn, 'small')))
        )
      );
    }
  }
  function renderTasks() {
    const p = L.progress();
    for (const [k, b] of Object.entries(taskEls)) {
      b.className = 'badge ' + (p[k] ? 'ok' : '');
      b.textContent = p[k] ? '✓ erledigt' : 'offen';
    }
  }

  // ------------------------------------------------------------------
  // MATLAB: Editor, Befehlsfenster, Workspace, Abbildungen
  // ------------------------------------------------------------------
  let cm = null,
    ta = null,
    curFile = BS.store.get('lab2file', 'Braccio_kinematics.m'),
    fileBtns = {},
    fileInfo,
    toolRow,
    checkBox,
    consoleEl,
    cmdIn,
    wsEl,
    figsEl,
    errLine = null;
  const docs = {};
  const hist = BS.store.get('lab2hist', []);
  let histPos = -1;

  function openFile(name, line) {
    if (!L.file(name)) return;
    showSub('matlab');
    selectFile(name);
    if (line && cm) {
      cm.focus();
      cm.setCursor({ line: line - 1, ch: 0 });
      cm.scrollIntoView({ line: line - 1, ch: 0 }, 120);
    }
  }
  L.openFile = openFile;

  function selectFile(name) {
    if (!L.file(name)) name = L.files[0].name;
    curFile = name;
    BS.store.set('lab2file', name);
    for (const [k, b] of Object.entries(fileBtns)) b.classList.toggle('on', k === name);
    const f = L.file(name);
    if (cm) {
      if (!docs[name]) {
        docs[name] = CodeMirror.Doc(f.src, 'octave');
        docs[name].on('change', (d) => {
          L.setSource(name, d.getValue());
          markModified();
        });
      }
      cm.swapDoc(docs[name]);
      clearErrLine();
      markError();
    } else ta.value = f.src;
    fileInfo.innerHTML =
      f.kind === 'block'
        ? `MATLAB-Function-Block <code>${f.block}</code> – wird während der Simulation in jedem Schritt (10 ms) aufgerufen. Änderungen wirken beim nächsten Start.`
        : `m-Skript – läuft im Basis-Workspace. <b>Strg+Enter</b> führt den Abschnitt (%%) am Cursor aus.`;
    buildTools(f);
    checkBox.style.display = 'none';
    markModified();
  }
  function markModified() {
    for (const f of L.files) if (fileBtns[f.name]) fileBtns[f.name].textContent = f.name + (f.src !== f.orig ? ' •' : '');
  }
  function clearErrLine() {
    if (errLine && cm) {
      errLine.doc.removeLineClass(errLine.line, 'background', 'cm-errline');
      errLine = null;
    }
  }
  function markError() {
    const e = L.lastError && L.lastError.file === curFile ? L.lastError : L.lastScriptError && L.lastScriptError.file === curFile ? L.lastScriptError : null;
    if (!e || !e.line || !cm) return;
    const doc = cm.getDoc();
    if (e.line - 1 >= doc.lineCount()) return;
    doc.addLineClass(e.line - 1, 'background', 'cm-errline');
    errLine = { doc, line: e.line - 1 };
  }
  function getSrc() {
    return cm ? cm.getValue() : ta.value;
  }
  function currentSection() {
    const lines = getSrc().split('\n');
    const cur = cm ? cm.getCursor().line : 0;
    let a = cur;
    while (a > 0 && !/^\s*%%/.test(lines[a])) a--;
    if (!/^\s*%%/.test(lines[a])) a = 0;
    let b = cur + 1;
    while (b < lines.length && !/^\s*%%/.test(lines[b])) b++;
    return { src: lines.slice(a, b).join('\n'), line0: a, a, b };
  }
  function runCur(section) {
    clearErrLine();
    const f = L.file(curFile);
    if (f.kind !== 'script') return;
    let sec = null;
    if (section) {
      sec = currentSection();
      if (cm) {
        // Abschnitt kurz hervorheben
        const doc = cm.getDoc();
        const marks = [];
        for (let i = sec.a; i < sec.b; i++) marks.push(doc.addLineClass(i, 'background', 'cm-runline'));
        setTimeout(() => marks.forEach((m) => doc.removeLineClass(m, 'background', 'cm-runline')), 400);
      }
    }
    const ok = L.runFile(curFile, sec);
    if (!ok) markError();
    renderWorkspace();
  }
  function runCheck(kind) {
    if (kind === 'dh' && !BS.lab2.interp.base.vars.has('braccio')) L.runFile('Braccio_kinematics.m');
    const r = kind === 'dh' ? L.checkDH() : L.checkFK();
    if (kind === 'fk' && r.ok) L.fkOk = true;
    if (sub !== 'matlab') openFile(kind === 'dh' ? 'Braccio_kinematics.m' : 'ForwardKinematics');
    else if (kind === 'fk' && curFile !== 'ForwardKinematics') selectFile('ForwardKinematics');
    else if (kind === 'dh' && curFile !== 'Braccio_kinematics.m') selectFile('Braccio_kinematics.m');
    checkBox.style.display = '';
    checkBox.className = 'status ' + (r.ok ? 'okbox' : 'error');
    checkBox.innerHTML = '';
    checkBox.append(h('b', null, (r.ok ? '✓ ' : '✗ ') + (kind === 'dh' ? 'DH-Parameter' : 'Vorwärtskinematik') + (r.ok ? ' korrekt' : ' noch nicht korrekt')), h('ul', { class: 'tight' }, r.msgs.map((m) => h('li', null, m))));
    renderTasks();
  }
  function buildTools(f) {
    toolRow.innerHTML = '';
    if (f.kind === 'script') {
      toolRow.append(U.btn('▶ Ausführen', () => runCur(false), 'primary', 'Ganzes Skript ausführen (F5)'), U.btn('▶ Abschnitt', () => runCur(true), '', 'Abschnitt am Cursor ausführen (Strg+Enter)'));
      if (f.name === 'Braccio_kinematics.m') toolRow.append(U.btn('✓ DH prüfen', () => runCheck('dh')));
    } else {
      toolRow.append(
        U.btn('✓ Syntax prüfen', () => {
          clearErrLine();
          try {
            const p = ml.parse(getSrc(), f.name);
            checkBox.style.display = '';
            checkBox.className = 'status okbox';
            checkBox.textContent = `✓ Keine Syntaxfehler (Funktion${p.fns.size > 1 ? 'en' : ''}: ${[...p.fns.keys()].join(', ')})`;
          } catch (e) {
            checkBox.style.display = '';
            checkBox.className = 'status error';
            checkBox.textContent = '✗ ' + L.errText(e);
            L.lastScriptError = { file: f.name, line: e.line, msg: e.message };
            markError();
          }
        })
      );
      if (f.name === 'ForwardKinematics') toolRow.append(U.btn('✓ FK prüfen', () => runCheck('fk'), 'primary', 'Mit der Referenz-Vorwärtskinematik vergleichen'));
    }
    toolRow.append(
      h('span', { class: 'grow' }),
      U.btn('↺ Original', () => {
        if (!confirm(`„${f.name}“ auf den Originalinhalt zurücksetzen? Eigene Änderungen gehen verloren.`)) return;
        if (cm) docs[f.name].setValue(f.orig);
        else ta.value = f.orig;
        L.setSource(f.name, f.orig);
        markModified();
      }, 'small'),
      U.btn('⬇', () => BS.download(f.name.endsWith('.m') ? f.name : f.name + '.m', getSrc()), 'small icon', 'Als .m-Datei speichern')
    );
  }

  function appendConsole(text, kind) {
    if (!consoleEl) return;
    const span = h('span', { class: kind ? 'c-' + kind : null }, text);
    consoleEl.appendChild(span);
    while (consoleEl.childNodes.length > 1500) consoleEl.removeChild(consoleEl.firstChild);
    consoleEl.scrollTop = consoleEl.scrollHeight;
  }
  BS.on('lab2:print', (d) => appendConsole(d.text, d.kind));
  BS.on('lab2:clc', () => consoleEl && (consoleEl.textContent = ''));
  let wsTimer = null;
  BS.on('lab2:workspace', () => {
    clearTimeout(wsTimer);
    wsTimer = setTimeout(() => {
      if (sub === 'matlab') renderWorkspace();
      if (sub === 'ikd') renderIkd();
    }, 60);
  });
  let figTimer = null;
  BS.on('lab2:figures', () => {
    clearTimeout(figTimer);
    figTimer = setTimeout(renderFigures, 30);
  });

  function buildMatlab(el) {
    const tabs = h(
      'div',
      { class: 'filetabs' },
      L.files.map((f) => (fileBtns[f.name] = h('button', { class: 'btn small', type: 'button', title: f.title, onclick: () => selectFile(f.name) }, f.name)))
    );
    fileInfo = h('p', { class: 'hint' });
    toolRow = h('div', { class: 'row' });
    checkBox = h('div', { class: 'status', style: { display: 'none' } });
    const wrap = h('div', { class: 'editor-wrap lab' });
    el.append(U.card(null, tabs, fileInfo, toolRow, wrap, checkBox));
    if (window.CodeMirror) {
      cm = CodeMirror(wrap, {
        value: '',
        mode: 'octave',
        theme: 'material-darker',
        lineNumbers: true,
        matchBrackets: true,
        indentUnit: 4,
        tabSize: 4,
        extraKeys: {
          'Ctrl-Enter': () => runCur(true),
          'Cmd-Enter': () => runCur(true),
          F5: () => runCur(false),
          'Ctrl-R': (c) => toggleComment(c, true),
          'Ctrl-T': (c) => toggleComment(c, false),
          'Cmd-/': (c) => toggleComment(c),
          'Ctrl-/': (c) => toggleComment(c),
        },
      });
    } else {
      ta = h('textarea', { spellcheck: false });
      ta.addEventListener('input', () => L.setSource(curFile, ta.value));
      wrap.appendChild(ta);
    }
    // Befehlsfenster
    consoleEl = h('pre', { class: 'serial mlconsole' });
    for (const c of L.consoleBuf) appendConsole(c.text, c.kind);
    cmdIn = h('input', { type: 'text', class: 'mono cmdin', placeholder: 'MATLAB-Befehl, z. B. braccio  oder  T = getTransform(braccioRobot, homeConfiguration(braccioRobot), \'tcp\')', spellcheck: false });
    cmdIn.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') {
        const v = cmdIn.value;
        if (!v.trim()) return;
        hist.unshift(v);
        if (hist.length > 100) hist.pop();
        BS.store.set('lab2hist', hist);
        histPos = -1;
        cmdIn.value = '';
        L.runCommand(v);
        renderWorkspace();
      } else if (e.key === 'ArrowUp') {
        if (histPos < hist.length - 1) cmdIn.value = hist[++histPos];
        e.preventDefault();
      } else if (e.key === 'ArrowDown') {
        histPos = Math.max(-1, histPos - 1);
        cmdIn.value = histPos >= 0 ? hist[histPos] : '';
        e.preventDefault();
      }
    });
    el.append(
      U.card(
        ['Befehlsfenster', h('span', { class: 'grow' }), U.btn('clc', () => L.runCommand('clc'), 'small')],
        consoleEl,
        h('div', { class: 'cmdrow' }, h('span', { class: 'mono prompt' }, '>>'), cmdIn)
      )
    );
    wsEl = h('div', { class: 'tblwrap ws' });
    el.append(U.card(['Workspace', h('span', { class: 'grow' }), U.btn('clear', () => (L.runCommand('clear'), renderWorkspace()), 'small')], wsEl));
    figsEl = h('div', { class: 'figs' });
    el.append(figsEl);
    selectFile(curFile);
    renderFigures();
  }
  function toggleComment(c, force) {
    const doc = c.getDoc();
    const from = doc.getCursor('from').line,
      to = doc.getCursor('to').line;
    const lines = [];
    for (let i = from; i <= to; i++) lines.push(doc.getLine(i));
    const commented = lines.every((l) => /^\s*%/.test(l) || !l.trim());
    const add = force != null ? force : !commented;
    doc.operation(() => {
      for (let i = from; i <= to; i++) {
        const l = doc.getLine(i);
        if (add) doc.replaceRange('%', { line: i, ch: 0 });
        else {
          const m = /^(\s*)%/.exec(l);
          if (m) doc.replaceRange('', { line: i, ch: m[1].length }, { line: i, ch: m[1].length + 1 });
        }
      }
    });
  }

  function renderWorkspace() {
    if (!wsEl) return;
    const vars = [...L.interp.base.vars.entries()].filter(([k]) => !k.startsWith('__')).sort((a, b) => a[0].localeCompare(b[0]));
    if (!vars.length) {
      wsEl.innerHTML = '<p class="hint" style="padding:6px 8px">Workspace ist leer – Braccio_kinematics.m ausführen.</p>';
      return;
    }
    const t = h('table', { class: 'tbl compact' }, h('tr', null, h('th', null, 'Name'), h('th', null, 'Wert'), h('th', null, 'Größe'), h('th', null, 'Klasse')));
    for (const [k, v] of vars) {
      const d = ml.dims(v);
      t.appendChild(
        h(
          'tr',
          { class: 'hov', title: 'Klick: im Befehlsfenster anzeigen', onclick: () => L.runCommand(k) },
          h('td', { class: 'mono' }, k),
          h('td', { class: 'mono wsval' }, ml.brief(v)),
          h('td', null, d[0] + '×' + d[1]),
          h('td', null, ml.className(v))
        )
      );
    }
    wsEl.innerHTML = '';
    wsEl.appendChild(t);
  }

  // ---------------- Abbildungen (figure / subplot / plot) ----------------
  function niceStep(range, target) {
    const raw = range / Math.max(1, target);
    const mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const n = raw / mag;
    return (n < 1.5 ? 1 : n < 3 ? 2 : n < 7 ? 5 : 10) * mag;
  }
  function drawFigure(cv, fig) {
    const dpr = window.devicePixelRatio || 1;
    const W = cv.clientWidth,
      H = cv.clientHeight;
    if (!W || !H) return;
    cv.width = Math.round(W * dpr);
    cv.height = Math.round(H * dpr);
    const ctx = cv.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = '#0f1216';
    ctx.fillRect(0, 0, W, H);
    ctx.font = '10px ui-monospace, Menlo, monospace';
    for (const ax of fig.axes) {
      const cells = ax.p.map((p) => [Math.floor((p - 1) / ax.n), (p - 1) % ax.n]);
      const r0 = Math.min(...cells.map((c) => c[0])),
        r1 = Math.max(...cells.map((c) => c[0])),
        c0 = Math.min(...cells.map((c) => c[1])),
        c1 = Math.max(...cells.map((c) => c[1]));
      const cw = W / ax.n,
        ch = H / ax.m;
      const X0 = c0 * cw + 50,
        Y0 = r0 * ch + (ax.title ? 18 : 8),
        PW = (c1 - c0 + 1) * cw - 60,
        PH = (r1 - r0 + 1) * ch - (ax.title ? 18 : 8) - (ax.xlabel ? 30 : 20);
      if (PW < 20 || PH < 20) continue;
      if (ax.robot && !ax.lines.length) {
        ctx.strokeStyle = '#3a414c';
        ctx.setLineDash([5, 4]);
        ctx.strokeRect(X0 + 0.5, Y0 + 0.5, PW, PH);
        ctx.setLineDash([]);
        ctx.fillStyle = '#ffe066';
        ctx.font = '12px system-ui, sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText('show(braccioRobot) → Darstellung im 3D-Fenster (gelbes DH-Skelett)', X0 + PW / 2, Y0 + PH / 2);
        ctx.font = '10px ui-monospace, Menlo, monospace';
        continue;
      }
      let x0 = Infinity,
        x1 = -Infinity,
        y0 = Infinity,
        y1 = -Infinity;
      for (const ln of ax.lines)
        for (let i = 0; i < ln.x.length; i++) {
          if (!Number.isFinite(ln.x[i]) || !Number.isFinite(ln.y[i])) continue;
          x0 = Math.min(x0, ln.x[i]);
          x1 = Math.max(x1, ln.x[i]);
          y0 = Math.min(y0, ln.y[i]);
          y1 = Math.max(y1, ln.y[i]);
        }
      if (!Number.isFinite(x0)) (x0 = 0), (x1 = 1), (y0 = 0), (y1 = 1);
      if (x1 - x0 < 1e-12) (x0 -= 0.5), (x1 += 0.5);
      if (y1 - y0 < 1e-12) (y0 -= Math.max(0.5, Math.abs(y0) * 0.1)), (y1 += Math.max(0.5, Math.abs(y1) * 0.1));
      else {
        const pad = (y1 - y0) * 0.06;
        y0 -= pad;
        y1 += pad;
      }
      if (ax.xlim) [x0, x1] = ax.xlim;
      if (ax.ylim) [y0, y1] = ax.ylim;
      if (ax.equal) {
        const sx = PW / (x1 - x0),
          sy = PH / (y1 - y0);
        if (sx > sy) {
          const m = (x0 + x1) / 2,
            hw = PW / sy / 2;
          (x0 = m - hw), (x1 = m + hw);
        } else {
          const m = (y0 + y1) / 2,
            hh = PH / sx / 2;
          (y0 = m - hh), (y1 = m + hh);
        }
      }
      const X = (x) => X0 + ((x - x0) / (x1 - x0)) * PW,
        Y = (y) => Y0 + PH - ((y - y0) / (y1 - y0)) * PH;
      ctx.fillStyle = '#161a20';
      ctx.fillRect(X0, Y0, PW, PH);
      // Achsen und Gitter
      ctx.strokeStyle = ax.grid ? '#2f3642' : '#232831';
      ctx.fillStyle = '#9aa4b2';
      ctx.lineWidth = 1;
      const xs = niceStep(x1 - x0, Math.max(2, PW / 70)),
        ys = niceStep(y1 - y0, Math.max(3, PH / 30));
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      for (let x = Math.ceil(x0 / xs) * xs; x <= x1 + 1e-9; x += xs) {
        if (ax.grid) {
          ctx.beginPath();
          ctx.moveTo(X(x) + 0.5, Y0);
          ctx.lineTo(X(x) + 0.5, Y0 + PH);
          ctx.stroke();
        }
        ctx.fillText(+x.toFixed(6) + '', X(x), Y0 + PH + 3);
      }
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';
      for (let y = Math.ceil(y0 / ys) * ys; y <= y1 + 1e-9; y += ys) {
        if (ax.grid) {
          ctx.beginPath();
          ctx.moveTo(X0, Y(y) + 0.5);
          ctx.lineTo(X0 + PW, Y(y) + 0.5);
          ctx.stroke();
        }
        ctx.fillText(+y.toFixed(6) + '', X0 - 4, Y(y));
      }
      ctx.strokeStyle = '#59616d';
      ctx.strokeRect(X0 + 0.5, Y0 + 0.5, PW, PH);
      // Linien
      ctx.save();
      ctx.beginPath();
      ctx.rect(X0, Y0, PW, PH);
      ctx.clip();
      for (const ln of ax.lines) {
        ctx.strokeStyle = ctx.fillStyle = ln.color;
        ctx.lineWidth = ln.width || 1.5;
        if (ln.line && ln.line !== 'none') {
          ctx.setLineDash(ln.line === '--' ? [6, 4] : ln.line === ':' ? [2, 3] : ln.line === '-.' ? [6, 3, 2, 3] : []);
          ctx.beginPath();
          let pen = false;
          for (let i = 0; i < ln.x.length; i++) {
            if (!Number.isFinite(ln.x[i]) || !Number.isFinite(ln.y[i])) {
              pen = false;
              continue;
            }
            if (!pen) ctx.moveTo(X(ln.x[i]), Y(ln.y[i]));
            else ctx.lineTo(X(ln.x[i]), Y(ln.y[i]));
            pen = true;
          }
          ctx.stroke();
          ctx.setLineDash([]);
        }
        if (ln.marker) {
          ctx.lineWidth = 1.2;
          for (let i = 0; i < ln.x.length; i++) {
            const px = X(ln.x[i]),
              py = Y(ln.y[i]);
            if (!Number.isFinite(px) || !Number.isFinite(py)) continue;
            ctx.beginPath();
            switch (ln.marker) {
              case '.':
                ctx.arc(px, py, 2.4, 0, 2 * Math.PI);
                ctx.fill();
                continue;
              case 'o':
                ctx.arc(px, py, 3.5, 0, 2 * Math.PI);
                break;
              case 's':
                ctx.rect(px - 3.5, py - 3.5, 7, 7);
                break;
              case 'd':
                ctx.moveTo(px, py - 4.5);
                ctx.lineTo(px + 4, py);
                ctx.lineTo(px, py + 4.5);
                ctx.lineTo(px - 4, py);
                ctx.closePath();
                break;
              case '^':
                ctx.moveTo(px, py - 4);
                ctx.lineTo(px + 4, py + 3);
                ctx.lineTo(px - 4, py + 3);
                ctx.closePath();
                break;
              case 'x':
                ctx.moveTo(px - 3.5, py - 3.5);
                ctx.lineTo(px + 3.5, py + 3.5);
                ctx.moveTo(px + 3.5, py - 3.5);
                ctx.lineTo(px - 3.5, py + 3.5);
                break;
              case '*':
              case '+':
                ctx.moveTo(px - 4, py);
                ctx.lineTo(px + 4, py);
                ctx.moveTo(px, py - 4);
                ctx.lineTo(px, py + 4);
                if (ln.marker === '*') {
                  ctx.moveTo(px - 3, py - 3);
                  ctx.lineTo(px + 3, py + 3);
                  ctx.moveTo(px + 3, py - 3);
                  ctx.lineTo(px - 3, py + 3);
                }
                break;
              default:
                ctx.arc(px, py, 3, 0, 2 * Math.PI);
            }
            ctx.stroke();
          }
        }
      }
      ctx.restore();
      ctx.fillStyle = '#dfe4ea';
      ctx.font = '11px system-ui, sans-serif';
      if (ax.title) {
        ctx.textAlign = 'center';
        ctx.textBaseline = 'top';
        ctx.fillText(ax.title, X0 + PW / 2, Y0 - 16);
      }
      if (ax.xlabel) {
        ctx.textAlign = 'center';
        ctx.textBaseline = 'bottom';
        ctx.fillText(ax.xlabel, X0 + PW / 2, Y0 + PH + 28);
      }
      if (ax.ylabel) {
        ctx.save();
        ctx.translate(X0 - 40, Y0 + PH / 2);
        ctx.rotate(-Math.PI / 2);
        ctx.textAlign = 'center';
        ctx.textBaseline = 'top';
        ctx.fillText(ax.ylabel, 0, -6);
        ctx.restore();
      }
      if (ax.legend && ax.legend.length) {
        ctx.textAlign = 'left';
        ctx.textBaseline = 'middle';
        let ly = Y0 + 10;
        const lx = X0 + PW - 8 - Math.max(...ax.legend.map((s) => ctx.measureText(s).width)) - 22;
        ax.legend.forEach((name, i) => {
          const ln = ax.lines[i];
          if (!ln) return;
          ctx.fillStyle = ln.color;
          ctx.fillRect(lx, ly - 1, 14, 2);
          ctx.fillStyle = '#dfe4ea';
          ctx.fillText(name, lx + 18, ly);
          ly += 14;
        });
      }
      ctx.font = '10px ui-monospace, Menlo, monospace';
    }
  }
  function renderFigures() {
    if (!figsEl) return;
    figsEl.innerHTML = '';
    for (const f of L.interp.figs) {
      const rows = Math.max(1, ...f.axes.map((a) => a.m));
      const onlyRobot = f.axes.length && f.axes.every((a) => a.robot && !a.lines.length);
      const cv = h('canvas', { class: 'mlfig', style: { height: (onlyRobot ? 70 : Math.min(560, 120 + rows * 120)) + 'px' } });
      figsEl.appendChild(
        U.card(
          [
            `Figure ${f.id}${f.name ? ': ' + f.name : ''}`,
            h('span', { class: 'grow' }),
            U.btn('✕', () => {
              L.interp.figs = L.interp.figs.filter((x) => x !== f);
              renderFigures();
            }, 'small icon', 'Abbildung schließen'),
          ],
          f.axes.length ? cv : U.hint('Leere Abbildung')
        )
      );
      if (f.axes.length) requestAnimationFrame(() => drawFigure(cv, f));
    }
  }

  // ------------------------------------------------------------------
  // Bedienpanel (HMI wie Abb. 5)
  // ------------------------------------------------------------------
  let lamps = {},
    modeRadios = [],
    axisRadios = [],
    phiRows = [],
    sigEl,
    numWpEl,
    gripSw;
  function lamp(label, color) {
    const el = h('span', { class: 'lamp ' + (color || 'red') }, h('i'), h('span', null, label));
    return el;
  }
  function holdBtn(label, key, cls) {
    const b = h('button', { class: 'btn hmi ' + (cls || ''), type: 'button' }, label);
    const on = (e) => {
      e.preventDefault();
      L.hmi[key] = true;
      b.classList.add('pressed');
      if (!L.running) BS.toast('Simulation ist nicht gestartet (▶ Simulation starten)', 'warn');
    };
    const off = () => {
      L.hmi[key] = false;
      b.classList.remove('pressed');
    };
    b.addEventListener('pointerdown', on);
    ['pointerup', 'pointerleave', 'pointercancel'].forEach((ev) => b.addEventListener(ev, off));
    return b;
  }
  function pulseBtn(label, name, cls, title) {
    return h('button', { class: 'btn hmi ' + (cls || ''), type: 'button', title, onclick: () => L.press(name) }, label);
  }
  function buildPanel(el) {
    lamps = {
      notaus: lamp('NOTAUS AKTIV', 'red'),
      limx: lamp('Lim X', 'red'),
      limy: lamp('Lim Y', 'red'),
      limz: lamp('Lim Z', 'red'),
      run: lamp('RUN', 'green'),
      auto: lamp('AUTO', 'white'),
    };
    const board = h(
      'div',
      { class: 'hmiboard' },
      h('div', { class: 'nucleo' }, h('span', { class: 'nlabel' }, 'NUCLEO'), pulseBtn('INIT', 'usr', 'usr', 'USR BTN: Notaus quittieren (Notaus → Pause)'), h('span', { class: 'grow' }), lamps.notaus),
      h('div', { class: 'hmirow' }, lamps.limx, lamps.limy, lamps.limz, h('span', { class: 'grow' }), h('button', { class: 'btn hmi notaus', type: 'button', title: 'NOTAUS', onclick: () => L.press('notaus') }, 'NOTAUS'))
    );
    modeRadios = L.MODES.map((m, i) => {
      const r = h('input', { type: 'radio', name: 'lab2mode', value: i + 1 });
      r.addEventListener('change', () => L.setMode(i + 1));
      return h('label', { class: 'radio', title: i === 5 ? 'Im Modell nicht belegt: der Eingang des Umschalters ist offen (q = 0, nach JointLimits begrenzt).' : '' }, r, h('span', null, m));
    });
    const axes = ['1 - Base', '2 - Shoulder', '3 - Elbow', '4 - Wrist', '5 - Rot Wrist', '6 - Gripper'];
    axisRadios = axes.map((a, i) => {
      const r = h('input', { type: 'radio', name: 'lab2axis', value: i + 1 });
      r.addEventListener('change', () => (L.hmi.axis = i + 1));
      return h('label', { class: 'radio' }, r, h('span', null, a));
    });
    numWpEl = h('span', { class: 'mono' }, '–');
    el.append(
      U.card(
        'Bedienpanel',
        board,
        h(
          'div',
          { class: 'hmigrid' },
          h('div', { class: 'hmicol' }, h('button', { class: 'btn hmi runpause', type: 'button', title: 'RUN/PAUSE', onclick: () => L.press('runPause') }, '▶ / ❚❚'), h('div', { class: 'hmirow' }, lamps.run, lamps.auto)),
          h('fieldset', { class: 'radios' }, h('legend', null, 'Mode'), modeRadios),
          h('fieldset', { class: 'radios' }, h('legend', null, 'Axis'), axisRadios)
        ),
        h(
          'div',
          { class: 'hmirow btns' },
          pulseBtn('TEACH-IN POSITION', 'store'),
          pulseBtn('TEACH-IN RESET', 'reset'),
          h('span', { class: 'grow' }),
          holdBtn('−', 'down', 'pm'),
          holdBtn('+', 'up', 'pm')
        ),
        h('p', { class: 'hint' }, 'Gespeicherte Teach-In-Posen (numWaypoints): ', numWpEl, ' · Nach jedem Modus-Wechsel ist ein Klick auf ▶/❚❚ nötig.')
      )
    );
    // Slider
    phiRows = [];
    const sliders = h('div');
    for (let i = 0; i < 5; i++) {
      const range = h('input', { type: 'range', min: -90, max: 90, step: 1, value: L.hmi.phi[i] });
      const numEl = h('input', { type: 'number', min: -90, max: 90, step: 1, value: L.hmi.phi[i], class: 'n' });
      const set = (v) => {
        L.hmi.phi[i] = BS.clamp(+v, -90, 90);
        range.value = numEl.value = L.hmi.phi[i];
      };
      range.addEventListener('input', () => set(range.value));
      numEl.addEventListener('change', () => set(numEl.value));
      phiRows.push({ range, numEl });
      sliders.appendChild(h('div', { class: 'jrow' }, h('span', { class: 'jname', style: { '--c': BS.config.colors[i] } }, `φ${i + 1}`), range, numEl, h('span', { class: 'jist mono' }, '°')));
    }
    gripSw = U.sel(
      [
        ['10', 'Open (10°)'],
        ['60', 'Closed (60°)'],
      ],
      String(L.hmi.phi[5]),
      (v) => (L.hmi.phi[5] = +v)
    );
    el.append(
      U.card(
        'Slider – Servo-Winkel φ (CtrlMode 1)',
        sliders,
        U.row(U.field('Greifer', gripSw), h('span', { class: 'grow' }), U.btn('Alle φ = 0', () => {
          for (let i = 0; i < 5; i++) {
            L.hmi.phi[i] = 0;
            phiRows[i].range.value = phiRows[i].numEl.value = 0;
          }
        }, 'small')),
        U.hint('q1 = φ1, q2 = φ2 + 90°, q3 = φ3, q4 = φ4 + 90°, q5 = φ5 – begrenzt auf die Gelenkgrenzen jnt1…jnt5.PositionLimits.')
      )
    );
    // Szene
    const sw = (key, label) => U.chk(label, L.hmi.show[key], (v) => L.setShow(key, v));
    el.append(
      U.card(
        'Szene',
        U.row(sw('cube1', 'Würfel 1'), sw('cube2', 'Würfel 2'), sw('regal', 'Regal'), sw('efence', 'virtuelle Wand')),
        U.row(
          U.btn('Szene neu laden', () => {
            BS.sim.loadScenario('labor2');
            BS.toast('Laborszene neu geladen', 'ok');
          }, 'small'),
          U.btn('Spur löschen', () => BS.view.clearTrace(), 'small')
        )
      )
    );
    sigEl = h('div', { class: 'sigs' });
    el.append(U.card('Signale', sigEl));
  }
  function fmtV(v, d) {
    return '[' + v.map((x) => (Number.isFinite(x) ? x.toFixed(d == null ? 4 : d) : 'NaN')).join('  ') + ']';
  }
  function updatePanel() {
    const S = L.st,
      run = L.running;
    const chart = run ? S.chart : '';
    lamps.notaus.classList.toggle('on', chart === 'Notaus');
    lamps.limx.classList.toggle('on', !!(run && S.alarm));
    lamps.run.classList.toggle('on', chart === 'Run');
    lamps.auto.classList.toggle('on', L.hmi.mode >= 3);
    modeRadios.forEach((r, i) => (r.firstChild.checked = L.hmi.mode === i + 1));
    axisRadios.forEach((r, i) => (r.firstChild.checked = L.hmi.axis === i + 1));
    numWpEl.textContent = S && S.teach ? String(S.teach.num) : '–';
    sigEl.innerHTML = sigHtml();
  }
  function sigHtml() {
    const S = L.st;
    if (!S) return '<p class="hint">Simulation noch nicht gestartet.</p>';
    const q = S.dynOut.map((v) => v * R2D);
    const ref = L.refFK(S.dynOut);
    const ok = Math.hypot(...S.fk.tcp.map((v, i) => v - ref.tcp[i])) < 1e-6;
    return `<table class="tbl compact">
      <tr><td>Chart</td><td class="mono">${L.running ? S.chart : 'gestoppt'} · CtrlMode ${L.hmi.mode} (${L.MODES[L.hmi.mode - 1]})</td></tr>
      <tr><td>q [°] (simpleDynamics)</td><td class="mono">${fmtV(q, 1)}</td></tr>
      <tr><td>q6 Greifer [°]</td><td class="mono">${(S.qLim[5] * R2D).toFixed(1)}</td></tr>
      <tr><td>TCPvec [m] (Ihre FK)</td><td class="mono">${fmtV(S.fk.tcp)}</td></tr>
      <tr><td>TCP Referenz [m]</td><td class="mono ${ok ? 'ok' : 'warn'}">${fmtV(ref.tcp)}</td></tr>
      <tr><td>Bvec / Cvec [m]</td><td class="mono">${fmtV(S.fk.B, 3)} / ${fmtV(S.fk.C, 3)}</td></tr>
      <tr><td>TCPorient [rad] (XYZ)</td><td class="mono">${fmtV(S.fk.orient, 3)}</td></tr>
      <tr><td>Wand efence1_coord.x</td><td class="mono">${L.pv ? L.pv.efenceX.toFixed(3) : '–'} m · Alarm: <b class="${S.alarm ? 'err' : 'ok'}">${S.alarm ? 'JA' : 'nein'}</b></td></tr>
      ${L.hmi.mode === 5 ? `<tr><td>IK konvergiert</td><td class="mono">${S.cart.converged ? 'ja' : '<span class="warn">nein</span>'}</td></tr>` : ''}
    </table>`;
  }

  // ------------------------------------------------------------------
  // IK-Designer
  // ------------------------------------------------------------------
  let ikdEls = null;
  function buildIkd(el) {
    const objList = h('div', { class: 'objlist' });
    const jointRows = [];
    const jl = h('div');
    for (let i = 0; i < 5; i++) {
      const range = h('input', { type: 'range', step: 0.5 });
      const numEl = h('input', { type: 'number', step: 0.5, class: 'n' });
      const set = (v) => L.ikd.set(i, +v * D2R);
      range.addEventListener('input', () => set(range.value));
      numEl.addEventListener('change', () => set(numEl.value));
      jointRows.push({ range, numEl });
      jl.appendChild(h('div', { class: 'jrow' }, h('span', { class: 'jname', style: { '--c': BS.config.colors[i] } }, `jnt${i + 1}`), range, numEl, h('span', { class: 'jist mono' }, '°')));
    }
    const tcpEl = h('div', { class: 'mono hint' });
    const mx = h('input', { type: 'number', step: 0.005, value: 0.25, class: 'n' }),
      my = h('input', { type: 'number', step: 0.005, value: 0, class: 'n' }),
      mz = h('input', { type: 'number', step: 0.005, value: 0.06, class: 'n' }),
      mpsi = h('input', { type: 'number', step: 5, value: -90, class: 'n' });
    const tableEl = h('div', { class: 'tblwrap' });
    const idxEl = h('span', { class: 'mono' });
    const nameIn = h('input', { type: 'text', value: L.ikd.exportName, class: 'mono', style: { width: '130px' } });
    nameIn.addEventListener('change', () => {
      L.ikd.exportName = nameIn.value;
      L.ikd.save();
    });
    const hintEl = h('div');
    el.append(
      U.card(
        'Inverse Kinematics Designer',
        hintEl,
        U.hint('Nachbildung des MATLAB-Werkzeugs: Gelenke einstellen, Posen speichern, Abfolge und Kollisionen prüfen, Export in den Workspace. Die eingestellte Pose wird als Geisterbild gezeigt.')
      ),
      U.card('Scene – Add Collision Object', U.hint('collisionBox-Objekte aus dem Workspace (Ini-Skript). Angehakte Objekte werden bei „Check Collisions“ geprüft.'), objList),
      U.card(
        'States – Joint Position',
        jl,
        tcpEl,
        U.row(U.btn('Aktuelle Roboterpose übernehmen', () => L.ikd.snapToRobot(), 'small'), U.btn('Default-Pose', () => HOMEQ(), 'small')),
        h('details', null, h('summary', null, 'Marker Pose Target (TCP-Ziel → Gelenkwinkel)'), U.row(U.field('x [m]', mx), U.field('y [m]', my), U.field('z [m]', mz), U.field('ψ [°]', mpsi)), U.row(U.btn('IK lösen', () => {
          const r = L.ikd.solve(+mx.value, +my.value, +mz.value, +mpsi.value);
          if (r.err) BS.toast(r.err, 'warn');
          else if (r.psi !== +mpsi.value) BS.toast(`ψ = ${mpsi.value}° nicht erreichbar – gelöst mit ψ = ${r.psi}°`, 'warn');
        }, 'small')), U.hint('ψ = Werkzeugwinkel gegenüber der Horizontalen (0 = waagrecht nach außen, −90 = senkrecht nach unten). Gelöst mit der geometrischen IK des Simulators (TCP = Greifmitte, ohne den Versatz von −6 mm).'))
      ),
      U.card(
        'Configurations Panel',
        U.row(
          U.btn('◀', () => L.ikd.go(L.ikd.idx - 1), 'small icon', 'vorherige Konfiguration'),
          idxEl,
          U.btn('▶', () => L.ikd.go(L.ikd.idx + 1), 'small icon', 'nächste Konfiguration'),
          U.btn('Store Configuration', () => L.ikd.store(), 'small primary'),
          U.btn('Snap to Configuration', () => (L.ikd.idx >= 0 ? L.ikd.update(L.ikd.idx) : BS.toast('Keine Konfiguration ausgewählt', 'warn')), 'small', 'Ausgewählte Konfiguration mit der aktuellen Pose überschreiben'),
          U.btn('▶ Abspielen', () => playConfigs(), 'small', 'Abfolge als Geisterbild durchlaufen')
        ),
        tableEl,
        U.row(U.btn('Check Collisions', () => L.ikd.checkAll(), 'small'), h('span', { class: 'grow' }), U.field('Export als', nameIn), U.btn('Export', () => L.ikd.export(), 'small primary')),
        U.row(U.btn('Als MATLAB-Code kopieren', () => U.copy(ikdCode()), 'small'), U.btn('Alle löschen', () => {
          if (!L.ikd.configs.length || !confirm('Alle gespeicherten Konfigurationen löschen?')) return;
          L.ikd.configs = [];
          L.ikd.idx = -1;
          L.ikd.save();
          renderIkd();
        }, 'small danger'))
      )
    );
    ikdEls = { objList, jointRows, tcpEl, tableEl, idxEl, hintEl };
  }
  function HOMEQ() {
    L.ikd.q = [0, 90, 0, 90, 0].map((v) => v * D2R);
    L.ikd.save();
    renderIkd();
  }
  let playTimer = null;
  function playConfigs() {
    clearInterval(playTimer);
    if (!L.ikd.configs.length) return;
    let k = 0;
    L.ikd.go(0);
    playTimer = setInterval(() => {
      k++;
      if (k >= L.ikd.configs.length) return clearInterval(playTimer);
      L.ikd.go(k);
    }, 900);
  }
  function ikdCode() {
    const rows = L.ikd.configs.map((c) => '    ' + c.q.map((v) => v.toFixed(4)).join(' ')).join('\n');
    return `${L.ikd.exportName} = [ ...\n${rows}];  % Konfigurationen (Zeilen), q1..q5 in rad`;
  }
  function renderIkd() {
    if (!ikdEls) return;
    const E = ikdEls;
    const lim = L.ikd.limits();
    E.jointRows.forEach((r, i) => {
      r.range.min = (lim[i][0] * R2D).toFixed(1);
      r.range.max = (lim[i][1] * R2D).toFixed(1);
      const v = (L.ikd.q[i] * R2D).toFixed(1);
      if (document.activeElement !== r.range) r.range.value = v;
      if (document.activeElement !== r.numEl) r.numEl.value = v;
    });
    const fk = L.refFK(L.ikd.q);
    const col = L.ikd.collide(L.ikd.q);
    E.tcpEl.innerHTML = `TCP (Referenz-FK, Labor-KS): x = ${fk.tcp[0].toFixed(4)} m · y = ${fk.tcp[1].toFixed(4)} m · z = ${fk.tcp[2].toFixed(4)} m<br>Konfiguration [rad]: [${L.ikd.q.map((v) => v.toFixed(4)).join(' ')}] · <span class="${col.status === 'kollisionsfrei' ? 'ok' : 'warn'}">${col.status}${col.msg ? ': ' + col.msg : ''}</span>`;
    const objs = L.ikd.collisionObjects();
    E.objList.innerHTML = '';
    if (!objs.length) E.objList.appendChild(U.hint('Keine collisionBox-Objekte im Workspace – zuerst Braccio_kinematics.m ausführen.'));
    for (const o of objs)
      E.objList.appendChild(
        U.chk(o.name, L.ikd.scene.has(o.name), (v) => {
          if (v) L.ikd.scene.add(o.name);
          else L.ikd.scene.delete(o.name);
          L.ikd.save();
          renderIkd();
        })
      );
    E.idxEl.textContent = `${L.ikd.idx + 1} / ${L.ikd.configs.length}`;
    const t = h('table', { class: 'tbl compact' }, h('tr', null, h('th', null, '#'), h('th', null, 'Value [°]'), h('th', null, 'Collision Status'), h('th', { class: 'num' }, 'TCP x / z [m]'), h('th')));
    L.ikd.configs.forEach((c, k) => {
      const f = L.refFK(c.q);
      t.appendChild(
        h(
          'tr',
          { class: 'hov' + (k === L.ikd.idx ? ' sel' : ''), onclick: () => L.ikd.go(k) },
          h('td', null, k + 1),
          h('td', { class: 'mono' }, '[' + c.q.map((v) => (v * R2D).toFixed(1)).join(' ') + ']'),
          h('td', { class: c.status === 'kollisionsfrei' ? 'ok' : c.status === 'NOT RUN' ? 'muted' : 'err', title: c.msg }, c.status + (c.msg ? ' – ' + c.msg : '')),
          h('td', { class: 'num mono' }, f.tcp[0].toFixed(3) + ' / ' + f.tcp[2].toFixed(3)),
          h(
            'td',
            null,
            U.btn('↑', (e) => (e.stopPropagation(), L.ikd.move(k, -1)), 'small icon'),
            U.btn('↓', (e) => (e.stopPropagation(), L.ikd.move(k, 1)), 'small icon'),
            U.btn('✕', (e) => (e.stopPropagation(), L.ikd.remove(k)), 'small icon')
          )
        )
      );
    });
    E.tableEl.innerHTML = '';
    E.tableEl.appendChild(L.ikd.configs.length ? t : U.hint('Noch keine Konfigurationen gespeichert („Store Configuration“).'));
    E.hintEl.innerHTML = L.running ? '<p class="hint warn">Die Simulation läuft – das Geisterbild zeigt die Pose des Designers, der Roboter folgt dem Modell.</p>' : '';
  }
  BS.on('lab2:ikd', () => sub === 'ikd' && renderIkd());

  // ------------------------------------------------------------------
  // Simulink: Modellübersicht, Blockparameter, Scope
  // ------------------------------------------------------------------
  let diagEl, scopePlot, scopeMode = 'tcp';
  function buildSimulink(el) {
    diagEl = h('div', { class: 'diag' });
    el.append(U.card('Modell Braccio_CtrlSimMdl (Signalfluss)', diagEl, U.hint('Hervorgehoben: aktiver Pfad. Blöcke mit „m“ sind MATLAB-Function-Blöcke – Klick öffnet den Code.')));
    const P = L.params;
    const txt = (key, w) => {
      const i = h('input', { type: 'text', value: P[key], class: 'mono', style: { width: w || '100%' } });
      i.addEventListener('change', () => {
        P[key] = i.value;
        L.saveParams();
      });
      return i;
    };
    const chk = (key, label) => U.chk(label, P[key], (v) => ((P[key] = v), L.saveParams()));
    const method = U.sel(
      [
        ['quintic', 'Quintic Polynomial'],
        ['cubic', 'Cubic Polynomial'],
      ],
      P.pnpMethod,
      (v) => ((P.pnpMethod = v), L.saveParams())
    );
    el.append(
      U.card(
        'Blockparameter – PickAndPlace',
        U.hint('Polynomial Trajectory: Eingänge Waypoints (5×N, rad) und TimePoints (1×N, s). Ausdrücke werden beim Start im Workspace ausgewertet.'),
        U.field('Waypoints', txt('pnpWaypoints')),
        U.field('TimePoints', txt('pnpTimePoints')),
        U.row(U.field('Method', method), U.field('Wiederholen ab t ≥', txt('pnpLoopT', '70px')), chk('pnpLoop', 'loop')),
        U.field('Greifer: Zeitpunkte [s]', txt('pnpGripT')),
        U.field('Greifer: Werte q6 [°]', txt('pnpGripV')),
        U.hint('Hinweis Simulator: ein 20-mm-Würfel wird erst ab q6 ≈ 55° fest gegriffen.')
      ),
      U.card('Blockparameter – CartesianPath', U.hint('Sollwerte aus <code>cartesianPath</code> (Ini-Skript), Lookup-Tabellen über der Zeit.'), U.row(U.field('maxIter', txt('cartMaxIter', '80px')), U.field('weights', txt('cartWeights', '130px')), chk('cartLoop', 'loop'))),
      U.card(
        'Blockparameter – simpleDynamics & Optionen',
        U.row(U.field('Rate Limiter [rad/s]', txt('rateLimit', '70px'))),
        chk('autoIni', 'Braccio_kinematics.m vor dem Start ausführen, falls der Workspace leer ist'),
        U.row(U.btn('Parameter zurücksetzen', () => {
          if (!confirm('Alle Blockparameter auf die Werte des Originalmodells zurücksetzen?')) return;
          L.resetParams();
          el.innerHTML = '';
          buildSimulink(el);
        }, 'small'))
      )
    );
    const cv = h('canvas', { class: 'plot tall' });
    scopePlot = new BS.Plot(cv);
    const modeSel = U.sel(
      [
        ['tcp', 'TCPvec (Ihre FK) – x, y, z [m]'],
        ['q', 'Gelenkwinkel q1…q5 [°]'],
      ],
      scopeMode,
      (v) => (scopeMode = v)
    );
    el.append(
      U.card('Scope', U.row(modeSel, h('span', { class: 'grow' }), U.btn('Ergebnisse → out', () => (L.writeOut(), BS.toast("Variable 'out' aktualisiert", 'ok')), 'small')), cv, U.hint('Gestrichelt: Referenz-Vorwärtskinematik. Beim Stoppen werden ConfigSim, TCPpos und CtrlMode in <code>out</code> geschrieben.')),
      U.card('Signale', (simSigEl = h('div', { class: 'sigs' })))
    );
  }
  let simSigEl;
  function diagHtml() {
    const m = L.hmi.mode,
      S = L.st,
      run = L.running;
    const box = (id, label, active, isM) =>
      `<div class="blk ${active ? 'act' : ''} ${isM ? 'mfun' : ''}" ${isM ? `data-file="${isM}"` : ''}>${label}${isM ? '<sup>m</sup>' : ''}</div>`;
    const chart = run ? S.chart : '–';
    return `<div class="drow">${box('hmi', 'Bedienpanel', true)}<span class="arr">→</span>${box('chart', 'Chart<br><small>' + chart + '</small>', run)}<span class="arr">→</span><div class="dcol">
      ${box('m1', '1 Manual (Slider)', m === 1)}
      ${box('m2', '2 PushBtnCtrl', m === 2)}
      ${box('m3', '3 Replay ← TeachIn', m === 3)}
      ${box('m4', "4 PickAndPlace", m === 4)}
      ${box('m5', '5 CartesianPath', m === 5)}
      </div></div>
      <div class="drow sub">${box('st', 'TeachIn/StoreWaypoints', m === 2 || m === 3, 'StoreWaypoints')}${box('ik', 'CartesianPath/inverseKinematics3dof', m === 5, 'inverseKinematics3dof')}</div>
      <div class="drow"><span class="arr">→</span>${box('jl', 'JointLimits', true)}<span class="arr">→</span>${box('dyn', 'simpleDynamics' + (run && S.chart !== 'Run' ? '<br><small>hält</small>' : ''), true)}<span class="arr">→</span>${box('fk', 'ForwardKinematics', true, 'ForwardKinematics')}<span class="arr">→</span>${box('mon', 'Arbeitsraum&shy;überwachung' + (run && S.alarm ? '<br><small class="err">ALARM</small>' : ''), run && S.alarm)}</div>`;
  }
  function updateSimulink() {
    const html = diagHtml();
    if (diagEl._h !== html) {
      diagEl.innerHTML = html;
      diagEl._h = html;
      diagEl.querySelectorAll('[data-file]').forEach((b) => b.addEventListener('click', () => openFile(b.dataset.file)));
    }
    if (simSigEl) simSigEl.innerHTML = sigHtml();
    const S = L.st;
    if (!S || !scopePlot) return;
    const lg = S.log;
    const n = lg.t.length;
    const t1 = n ? lg.t[n - 1] : 0,
      t0 = t1 - 10;
    let i0 = n - 1;
    while (i0 > 0 && lg.t[i0 - 1] >= t0) i0--;
    const step = Math.max(1, Math.floor((n - i0) / 500));
    const idx = [];
    for (let i = Math.max(0, i0); i < n; i += step) idx.push(i);
    const series = [];
    const cols = ['#ff6b6b', '#51cf66', '#4dabf7', '#ffd43b', '#b197fc'];
    if (scopeMode === 'tcp') {
      ['x', 'y', 'z'].forEach((nm, k) => {
        series.push({ name: nm, color: cols[k], data: idx.map((i) => [lg.t[i], lg.tcp[i][k]]) });
        series.push({ color: cols[k], dash: [4, 3], width: 1, data: idx.map((i) => [lg.t[i], lg.ref[i][k]]) });
      });
    } else for (let k = 0; k < 5; k++) series.push({ name: 'q' + (k + 1), color: cols[k], data: idx.map((i) => [lg.t[i], lg.q[i][k] * R2D]) });
    scopePlot.draw({ series, xRange: [Math.max(0, t0), Math.max(t1, 10)], xLabel: 't [s]' });
  }

  // ------------------------------------------------------------------
  // Registrierung
  // ------------------------------------------------------------------
  U.register('lab2', {
    title: 'Labor Ü2',
    build(el) {
      buildHead(el);
      for (const [k] of SUBS) {
        panes[k] = h('div', { class: 'subpane' });
        el.appendChild(panes[k]);
      }
      buildTasks(panes.aufgabe);
      buildMatlab(panes.matlab);
      buildPanel(panes.panel);
      buildIkd(panes.ikd);
      buildSimulink(panes.simulink);
      showSub(sub);
      if (L.lastError) showError(L.lastError);
    },
    enter() {
      document.querySelector('.layout').classList.add('wide');
      setTimeout(() => {
        BS.view.resize();
        if (cm) cm.refresh();
      }, 30);
      if (L.ensureScene()) BS.toast('Laborszene der Übung 2 geladen', 'ok');
      if (sub === 'aufgabe') renderTasks();
    },
    leave() {
      document.querySelector('.layout').classList.remove('wide');
      setTimeout(() => BS.view.resize(), 30);
      clearInterval(playTimer);
    },
    update() {
      const run = L.running;
      startBtn.disabled = run;
      stopBtn.disabled = !run;
      const chart = run ? L.st.chart : 'gestoppt';
      statePill.className = 'pill ' + (!run ? '' : chart === 'Run' ? 'run' : chart === 'Notaus' ? 'err' : 'warn');
      statePill.textContent = run ? `Simulink: ${chart} · CtrlMode ${L.hmi.mode}` : 'Simulink: gestoppt';
      timeEl.textContent = L.st ? `t = ${L.st.t.toFixed(2)} s` : '';
      skelBtn.style.display = L.skeleton ? '' : 'none';
      if (!L.lastError && errBox.style.display !== 'none') showError(null);
      // Geisterbild: IK-Designer bzw. show(…, config)
      const V = BS.view;
      if (sub === 'ikd') V.ghostPose = L.qToM(L.ikd.q, 10 * D2R);
      else if (L.ghostQ && !run) V.ghostPose = L.qToM(L.ghostQ, BS.sim.cmd[5] * D2R);
      else V.ghostPose = null;
      if (sub === 'panel') updatePanel();
      else if (sub === 'simulink') updateSimulink();
      if (sub === 'aufgabe') renderTasks();
    },
  });
})();
