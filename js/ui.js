/* Braccio-Simulator – UI-Grundgerüst: Kopfzeile, HUD, Scope, Tabs, Hilfsfunktionen */
(function () {
  'use strict';
  const BS = window.BS;
  const h = BS.h;
  const C = BS.config;

  const U = (BS.ui = { tabs: [], active: null });

  // ------------------------------------------------------------------
  // Bausteine
  // ------------------------------------------------------------------
  U.btn = (label, onclick, cls, title) => h('button', { class: 'btn ' + (cls || ''), onclick, title, type: 'button' }, label);
  U.num = (val, onchange, o) => {
    o = o || {};
    const el = h('input', { type: 'number', value: val, step: o.step != null ? o.step : 1, class: o.cls || '', title: o.title });
    if (o.min != null) el.min = o.min;
    if (o.max != null) el.max = o.max;
    el.addEventListener('change', () => {
      const v = parseFloat(el.value);
      if (Number.isFinite(v)) onchange(v, el);
    });
    return el;
  };
  U.sel = (opts, val, onchange) => {
    const el = h('select', { onchange: () => onchange(el.value) }, opts.map(([v, l]) => h('option', { value: v }, l)));
    el.value = val;
    return el;
  };
  U.chk = (label, checked, onchange, title) => {
    const inp = h('input', { type: 'checkbox' });
    inp.checked = !!checked;
    inp.addEventListener('change', () => onchange(inp.checked));
    const el = h('label', { class: 'chk', title }, inp, h('span', null, label));
    el.input = inp;
    return el;
  };
  U.card = (title, ...children) => h('div', { class: 'card' }, title ? h('h3', null, title) : null, ...children);
  U.row = (...c) => h('div', { class: 'row' }, ...c);
  U.field = (label, input) => h('label', { class: 'field' }, h('span', null, label), input);
  U.hint = (text) => h('p', { class: 'hint', html: text });
  U.matrix = (rows, digits, rowLabels, colLabels) => {
    const t = h('table', { class: 'matrix' });
    if (colLabels) t.appendChild(h('tr', null, rowLabels ? h('td') : null, colLabels.map((c) => h('td', { class: 'lbl', style: { textAlign: 'right' } }, c))));
    rows.forEach((r, i) =>
      t.appendChild(
        h(
          'tr',
          null,
          rowLabels ? h('td', { class: 'lbl' }, rowLabels[i]) : null,
          r.map((v) => h('td', { class: 'v' }, typeof v === 'number' ? v.toFixed(digits == null ? 2 : digits) : v))
        )
      )
    );
    return t;
  };
  U.copy = (text) => {
    try {
      navigator.clipboard.writeText(text);
      BS.toast('In die Zwischenablage kopiert', 'ok');
    } catch (e) {
      BS.toast('Kopieren nicht möglich', 'err');
    }
  };
  /** Gedrückt halten = wiederholen */
  U.repeatBtn = (label, fn, cls) => {
    const b = U.btn(label, null, cls);
    let t1 = null,
      t2 = null;
    const stop = () => {
      clearTimeout(t1);
      clearInterval(t2);
    };
    b.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      fn();
      t1 = setTimeout(() => (t2 = setInterval(fn, 90)), 350);
    });
    ['pointerup', 'pointerleave', 'pointercancel'].forEach((ev) => b.addEventListener(ev, stop));
    return b;
  };

  U.register = function (id, def) {
    def.id = id;
    U.tabs.push(def);
  };
  U.show = function (id) {
    for (const t of U.tabs) {
      const on = t.id === id;
      t.tabEl.classList.toggle('active', on);
      t.pane.classList.toggle('active', on);
    }
    const prev = U.active;
    U.active = id;
    if (prev !== id) {
      const p = U.tabs.find((t) => t.id === prev);
      if (p && p.leave) p.leave();
      BS.view.ghostPose = null;
      const t = U.tabs.find((t) => t.id === id);
      if (t && t.enter) t.enter();
    }
    BS.store.set('tab', id);
  };

  // ------------------------------------------------------------------
  // Kopfzeile
  // ------------------------------------------------------------------
  let elStatus, elTime, elPause;
  function buildTopbar() {
    const tb = document.getElementById('topbar');
    const logo =
      '<svg viewBox="0 0 24 24" fill="none" stroke="#19b6ae" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M4 21h10"/><path d="M9 21v-4"/><circle cx="9" cy="15" r="2"/><path d="M10.4 13.6 15 7"/><circle cx="15.5" cy="6" r="1.6"/><path d="M17 6.5l3 2.5"/><path d="M20 9l1.5-1.5M20 9l1 2"/></svg>';
    elStatus = h('span', { class: 'pill ok' }, 'Bereit');
    elTime = h('span', { class: 'simtime' }, 't = 0.00 s');
    elPause = U.btn('⏸', () => {
      BS.sim.paused = !BS.sim.paused;
    }, 'icon', 'Simulation anhalten/fortsetzen');
    const speed = U.sel(
      [
        ['0.25', '0.25×'],
        ['0.5', '0.5×'],
        ['1', '1×'],
        ['2', '2×'],
        ['5', '5×'],
        ['10', '10×'],
      ],
      '1',
      (v) => (BS.sim.speed = +v)
    );
    speed.title = 'Simulationsgeschwindigkeit';
    const toggles = [
      ['frames', 'KS', 'DH-Koordinatensysteme anzeigen'],
      ['trace', 'Spur', 'TCP-Spur anzeigen'],
      ['path', 'Pfad', 'Geplante Trajektorie anzeigen'],
      ['zones', 'Zonen', 'Sperrzonen/Arbeitsbereiche anzeigen'],
      ['markers', 'Marker', 'Kalibriermarker anzeigen'],
    ].map(([k, l, t]) => {
      const b = U.btn(l, () => {
        BS.view.show[k] = !BS.view.show[k];
        b.classList.toggle('on', BS.view.show[k]);
        BS.view.applyShow();
      }, 'chip' + (BS.view.show[k] ? ' on' : ''), t);
      b.dataset.show = k;
      return b;
    });
    U.syncToggles = () => toggles.forEach((b) => b.classList.toggle('on', !!BS.view.show[b.dataset.show]));
    const cam = U.sel(
      [
        ['iso', 'Ansicht: Iso'],
        ['vorne', 'Ansicht: Vorne'],
        ['seite', 'Ansicht: Seite'],
        ['oben', 'Ansicht: Oben'],
        ['hinten', 'Ansicht: Hinten'],
      ],
      'iso',
      (v) => BS.view.setView(v)
    );
    tb.append(
      h('div', { class: 'brand', html: logo + 'Braccio-Simulator <small>Robotik</small>' }),
      h('span', { class: 'sep' }),
      elStatus,
      elTime,
      elPause,
      speed,
      U.btn('↺ Reset', () => {
        BS.sim.reset();
        BS.toast('Simulation zurückgesetzt');
      }, '', 'Simulation, Roboter und Szenario zurücksetzen'),
      h('span', { class: 'sep' }),
      ...toggles,
      U.btn('Spur löschen', () => BS.view.clearTrace(), 'chip'),
      h('span', { class: 'grow' }),
      cam
    );
  }

  // ------------------------------------------------------------------
  // HUD
  // ------------------------------------------------------------------
  let hudTcp, hudJoints, hudMon, hudGrip, hudEstop;
  function buildHud() {
    const hud = document.getElementById('hud');
    hudTcp = h('div', { class: 'box' });
    hudJoints = h('div', { class: 'box' });
    hud.append(hudTcp, hudJoints);
    const hr = document.getElementById('hudRight');
    hudEstop = h('div', { class: 'estop' }, h('span', null, '⛔ NOT-HALT'), h('span', { class: 'reason', style: { fontWeight: 500 } }), U.btn('Quittieren', () => BS.sim.ackEstop(), 'small'));
    hudMon = h('div', { class: 'box monbox' });
    hudGrip = h('div', { class: 'box' });
    hr.append(hudEstop, hudMon, hudGrip);
    BS.on('estop', () => {
      hudEstop.classList.toggle('show', BS.sim.estop);
      hudEstop.querySelector('.reason').textContent = BS.sim.estopReason;
    });
  }
  function updateHud() {
    const sim = BS.sim;
    const T = sim.F[5];
    const p = BS.kin.tcp(sim.trueAngles());
    hudTcp.innerHTML = `<b>TCP</b>&nbsp; x ${BS.fmt(T[3])} &nbsp;y ${BS.fmt(T[7])} &nbsp;z ${BS.fmt(T[11])} mm<br><b>ψ</b> ${BS.fmt(p.psi)}° &nbsp;<b>r</b> ${BS.fmt(Math.hypot(T[3], T[7]))} mm &nbsp;<b>v</b> ${BS.fmt(sim.tcpVel, 0)} mm/s`;
    let rows = '<tr><td></td><td><b>Soll</b></td><td><b>Ist</b></td></tr>';
    for (let i = 0; i < 6; i++) rows += `<tr><td style="color:${C.colors[i]}">${C.short[i]}</td><td>${BS.fmt(sim.cmd[i])}</td><td>${BS.fmt(sim.act[i])}</td></tr>`;
    hudJoints.innerHTML = `<table>${rows}</table>`;
    const mon = BS.monitor;
    if (!mon.enabled) {
      hudMon.className = 'box monbox';
      hudMon.innerHTML = '<span class="muted">Überwachung aus</span>';
    } else {
      hudMon.className = 'box monbox ' + (mon.state === 'ok' ? '' : mon.state);
      hudMon.innerHTML =
        mon.state === 'ok'
          ? '<span class="ok">● Arbeitsraum OK</span>'
          : mon.issues
              .slice(0, 4)
              .map((i) => `<div class="${i.level === 'viol' ? 'err' : 'warn'}">● ${i.msg}</div>`)
              .join('');
    }
    const w = sim.jawWidth();
    hudGrip.innerHTML = `<b>Greifer</b> ${BS.fmt(w, 0)} mm${sim.held ? ` · <span class="ok">hält ${sim.held.obj.name}</span>` : ''}`;
  }

  function updateTopbar() {
    const sim = BS.sim;
    elTime.textContent = 't = ' + sim.t.toFixed(2) + ' s';
    elPause.textContent = sim.paused ? '▶' : '⏸';
    let cls = 'pill ok',
      txt = 'Manuell';
    if (sim.estop) (cls = 'pill err'), (txt = 'NOT-HALT');
    else if (sim.owner === 'program') (cls = 'pill run'), (txt = 'Programm läuft');
    else if (sim.owner === 'traj') (cls = 'pill run'), (txt = sim.play ? sim.play.label : 'Trajektorie');
    if (sim.paused) txt += ' · pausiert';
    elStatus.className = cls;
    elStatus.textContent = txt;
  }

  // ------------------------------------------------------------------
  // Scope (Zeitverläufe Soll/Ist)
  // ------------------------------------------------------------------
  let scopePlot,
    scopeEl,
    scopeState = BS.store.get('scope', { j: [true, true, true, true, false, false], soll: true, ist: true, v: false, win: 10, open: true });
  function buildScope() {
    scopeEl = document.getElementById('scope');
    const cv = h('canvas');
    scopePlot = new BS.Plot(cv);
    const save = () => BS.store.set('scope', scopeState);
    const toggle = U.btn(scopeState.open ? '▾ Scope' : '▸ Scope', () => {
      scopeState.open = !scopeState.open;
      toggle.textContent = scopeState.open ? '▾ Scope' : '▸ Scope';
      scopeEl.classList.toggle('collapsed', !scopeState.open);
      save();
      BS.view.resize();
    }, 'small');
    const head = h(
      'div',
      { class: 'head' },
      toggle,
      C.short.map((n, i) =>
        U.chk(h('span', { style: { color: C.colors[i], fontWeight: 600 } }, n), scopeState.j[i], (v) => {
          scopeState.j[i] = v;
          save();
        })
      ),
      h('span', { class: 'sep' }),
      U.chk('Soll (gestrichelt)', scopeState.soll, (v) => {
        scopeState.soll = v;
        save();
      }),
      U.chk('Ist', scopeState.ist, (v) => {
        scopeState.ist = v;
        save();
      }),
      U.chk('v_TCP [mm/s]', scopeState.v, (v) => {
        scopeState.v = v;
        save();
      }),
      h('span', { class: 'grow' }),
      U.sel(
        [
          ['5', '5 s'],
          ['10', '10 s'],
          ['30', '30 s'],
          ['60', '60 s'],
        ],
        String(scopeState.win),
        (v) => {
          scopeState.win = +v;
          save();
        }
      )
    );
    scopeEl.append(head, cv);
    scopeEl.classList.toggle('collapsed', !scopeState.open);
  }
  function updateScope() {
    if (!scopeState.open) return;
    const sim = BS.sim;
    const t1 = sim.t,
      t0 = t1 - scopeState.win;
    const data = sim.scope.filter((s) => s.t >= t0);
    const series = [];
    if (scopeState.v) series.push({ name: 'v_TCP', color: '#e9ecef', data: data.map((s) => [s.t, s.v]) });
    else
      for (let i = 0; i < 6; i++) {
        if (!scopeState.j[i]) continue;
        if (scopeState.ist) series.push({ name: C.short[i], color: C.colors[i], data: data.map((s) => [s.t, s.act[i]]) });
        if (scopeState.soll) series.push({ name: scopeState.ist ? null : C.short[i], color: C.colors[i], dash: [4, 3], width: 1, data: data.map((s) => [s.t, s.cmd[i]]) });
      }
    scopePlot.draw({ series, xRange: [Math.max(0, t0), Math.max(t1, scopeState.win)], xLabel: 't [s]', yRange: scopeState.v ? null : [-5, 185] });
  }

  // ------------------------------------------------------------------
  // Init & Update
  // ------------------------------------------------------------------
  U.init = function () {
    buildTopbar();
    buildHud();
    buildScope();
    const tabs = document.getElementById('tabs'),
      panes = document.getElementById('panes');
    for (const t of U.tabs) {
      t.tabEl = h('div', { class: 'tab', onclick: () => U.show(t.id) }, t.title);
      t.pane = h('div', { class: 'pane' });
      tabs.appendChild(t.tabEl);
      panes.appendChild(t.pane);
      t.build(t.pane);
    }
    const st = BS.store.get('tab', 'joints');
    U.show(U.tabs.some((t) => t.id === st) ? st : 'joints');
  };

  let lastUi = 0,
    lastScope = 0;
  U.update = function (now) {
    if (now - lastUi > 80) {
      lastUi = now;
      updateTopbar();
      updateHud();
      const t = U.tabs.find((t) => t.id === U.active);
      if (t && t.update) t.update();
    }
    if (now - lastScope > 50) {
      lastScope = now;
      updateScope();
    }
  };
})();
