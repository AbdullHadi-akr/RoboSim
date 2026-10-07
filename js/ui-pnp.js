/* Tab „Pick'n'Place“: Szenarien, Objekte, Ziele, Förderband */
(function () {
  'use strict';
  const BS = window.BS,
    U = BS.ui,
    h = BS.h,
    C = BS.config;

  let objEl, tgtEl, gripEl, selEl, convCard, convEl, scoreEl, scenSel;
  const newObj = { size: 30, color: '#e03131', x: 150, y: 200 };
  const COLORS = [
    ['#e03131', 'rot'],
    ['#2f9e44', 'grün'],
    ['#1c7ed6', 'blau'],
    ['#f59f00', 'gelb'],
    ['#7048e8', 'violett'],
    ['#f76707', 'orange'],
  ];

  BS.on('pickObject', (o) => {
    BS.view.selected = o;
    if (U.active === 'pnp') renderSel();
  });
  BS.on('objects', () => objEl && renderObjects());
  BS.on('scenario', () => {
    if (scenSel) scenSel.value = BS.sim.scenario;
    if (tgtEl) {
      renderTargets();
      renderConv();
    }
  });

  /** Optimale Handrotation M5 für das Objekt bei aktueller Armstellung */
  function bestM5(o) {
    const m = BS.sim.cmd.slice();
    let best = null;
    for (let a = 0; a <= 180; a += 1) {
      m[4] = a;
      const T = BS.kin.frames(m)[5];
      const w = 2 * BS.sim.halfExtent(o, [T[0], T[4], T[8]]);
      if (!best || w < best.w - 1e-6) best = { a, w };
    }
    return best;
  }

  function status(o) {
    const sim = BS.sim;
    if (o.held) return '<span class="badge ok">gegriffen</span>';
    const t = sim.targets.find((t) => sim.inTarget(o, t));
    if (t) {
      const ok = !t.accept || t.accept === o.color || t.accept === o.tag;
      return ok ? `<span class="badge ok">✓ ${t.name}</span>` : `<span class="badge err">✗ ${t.name}</span>`;
    }
    return o.resting ? '<span class="badge">liegt</span>' : '<span class="badge warn">fällt</span>';
  }

  function renderObjects() {
    const sim = BS.sim;
    objEl.innerHTML = '';
    const t = h('table', { class: 'tbl' });
    t.appendChild(h('tr', null, h('th', null, 'Objekt'), h('th', { class: 'num' }, 'Kante'), h('th', { class: 'num' }, 'x'), h('th', { class: 'num' }, 'y'), h('th', { class: 'num' }, 'z'), h('th', null, 'Status'), h('th')));
    for (const o of sim.objects) {
      const tr = h(
        'tr',
        { class: 'hov' + (BS.view.selected === o ? ' sel' : '') },
        h('td', null, h('span', { class: 'swatch', style: { background: o.color } }), o.name),
        h('td', { class: 'num' }, o.size[0].toFixed(0)),
        h('td', { class: 'num', 'data-k': 'x' }),
        h('td', { class: 'num', 'data-k': 'y' }),
        h('td', { class: 'num', 'data-k': 'z' }),
        h('td', { 'data-k': 's' }),
        h(
          'td',
          { style: { whiteSpace: 'nowrap' } },
          U.btn('→ IK', (e) => {
            e.stopPropagation();
            BS.emit('setIkTarget', { x: o.M[3], y: o.M[7], z: o.M[11] });
          }, 'small', 'Objektmitte als IK-Ziel übernehmen'),
          U.btn('✕', (e) => {
            e.stopPropagation();
            sim.removeObject(o);
          }, 'small icon')
        )
      );
      tr.addEventListener('click', () => {
        BS.view.selected = o;
        renderObjects();
        renderSel();
      });
      tr._obj = o;
      t.appendChild(tr);
    }
    objEl.appendChild(t);
    if (!sim.objects.length) objEl.appendChild(U.hint('Keine Objekte.'));
    updateObjects();
  }
  function updateObjects() {
    for (const tr of objEl.querySelectorAll('tr')) {
      const o = tr._obj;
      if (!o) continue;
      tr.querySelector('[data-k=x]').textContent = o.M[3].toFixed(1);
      tr.querySelector('[data-k=y]').textContent = o.M[7].toFixed(1);
      tr.querySelector('[data-k=z]').textContent = o.M[11].toFixed(1);
      tr.querySelector('[data-k=s]').innerHTML = status(o);
    }
  }

  function renderTargets() {
    const sim = BS.sim;
    tgtEl.innerHTML = sim.targets.length
      ? '<table class="tbl"><tr><th>Ziel</th><th class="num">x</th><th class="num">y</th><th class="num">Größe</th><th>akzeptiert</th></tr>' +
        sim.targets
          .map(
            (t) =>
              `<tr><td><span class="swatch" style="background:${t.color}"></span>${t.name}</td><td class="num">${t.x}</td><td class="num">${t.y}</td><td class="num">${t.w}×${t.h}</td><td class="muted">${t.accept ? (t.accept.startsWith('#') ? (COLORS.find((c) => c[0] === t.accept) || [0, t.accept])[1] : t.accept) : 'alles'}</td></tr>`
          )
          .join('') +
        '</table>'
      : '<span class="muted">Keine Zielbereiche.</span>';
  }

  function renderSel() {
    const o = BS.view.selected;
    if (!o || !BS.sim.objects.includes(o)) {
      selEl.innerHTML = '<span class="muted">Würfel im 3D-Fenster oder in der Liste anklicken.</span>';
      return;
    }
    const yaw = (Math.atan2(o.M[4], o.M[0]) * BS.R2D).toFixed(1);
    const b = bestM5(o);
    const ik = BS.kin.ikAuto({ x: o.M[3], y: o.M[7], z: o.M[11] }, C.geom, { psiPref: -90, cur: BS.sim.cmd });
    selEl.innerHTML =
      `<div class="kv"><b>Objekt</b><span><span class="swatch" style="background:${o.color}"></span>${o.name} (${o.size[0]} mm, ${o.mass} g${o.tag ? ', ' + o.tag : ''})</span>` +
      `<b>Mitte</b><span class="mono">(${o.M[3].toFixed(1)}, ${o.M[7].toFixed(1)}, ${o.M[11].toFixed(1)}) mm</span>` +
      `<b>Gierwinkel</b><span class="mono">${yaw}°</span>` +
      `<b>IK zur Mitte</b><span class="mono">${ik ? `ψ = ${ik.psi}°, M1..M4 = ${ik.m.map((v) => v.toFixed(0)).join(', ')}` : '<span class="err">nicht erreichbar</span>'}</span>` +
      `<b>M5 (aktuelle Pose)</b><span class="mono">${b.a}° → Greifbreite ${b.w.toFixed(1)} mm</span></div>`;
  }

  function renderConv() {
    const cv = BS.sim.conveyor;
    convCard.style.display = cv ? '' : 'none';
    if (!cv) return;
    convEl.innerHTML = '';
    convEl.append(
      U.row(
        U.chk('Band manuell an', cv.manual, (v) => (cv.manual = v)),
        U.field('v', U.num(cv.speed, (v) => (cv.speed = Math.max(0, v)), { cls: 'w44' })),
        h('span', { class: 'muted' }, 'mm/s')
      ),
      U.row(
        U.chk('Automatisch Boxen nachlegen', cv.autoSpawn, (v) => (cv.autoSpawn = v)),
        U.field('alle', U.num(cv.interval, (v) => (cv.interval = Math.max(1, v)), { cls: 'w44' })),
        h('span', { class: 'muted' }, 's')
      ),
      U.row(
        U.btn('+ kleine Box', () => spawnBox(false), 'small'),
        U.btn('+ große Box', () => spawnBox(true), 'small')
      ),
      U.hint('Pins: <b>D2</b> Lichtschranke unten (jede Box), <b>D4</b> Lichtschranke oben (nur große Box), <b>D7</b> Bandmotor (HIGH = läuft). Kleine Box 28 mm, große Box 46 mm. Anschlag bei y = 140 mm.')
    );
  }
  function spawnBox(big) {
    const cv = BS.sim.conveyor;
    const s = big ? 46 : 28;
    BS.sim.addObject({ size: [s, s, s], color: big ? '#1c7ed6' : '#f76707', mass: big ? 60 : 20, pos: [cv.x, cv.yStart - s / 2 - 5, cv.height + s / 2 + 2], yaw: 0, tag: big ? 'groß' : 'klein', name: big ? 'Box groß' : 'Box klein' });
  }

  U.register('pnp', {
    title: "Pick'n'Place",
    build(el) {
      const sim = BS.sim;
      scenSel = U.sel(
        Object.entries(sim.scenarios).map(([k, s]) => [k, s.name]),
        sim.scenario || 'sortieren',
        (v) => {
          sim.loadScenario(v);
          BS.store.set('scenario', v);
        }
      );
      scoreEl = h('div', { style: { marginTop: '6px' } });
      el.appendChild(
        U.card(
          'Szenario',
          U.row(scenSel, U.btn('Neu laden', () => sim.loadScenario(scenSel.value))),
          scoreEl,
          U.hint('Würfel werden gegriffen, wenn sie zwischen den Fingern liegen und der Greifer schließt (M6 → 73). Beim Öffnen fallen sie auf die Unterlage (Tisch, Band oder anderer Würfel).')
        )
      );
      gripEl = h('div');
      el.appendChild(
        U.card(
          'Greifer',
          gripEl,
          U.row(
            U.btn('Öffnen (M6 = 10)', () => sim.setManual([null, null, null, null, null, 10])),
            U.btn('Schließen (M6 = 73)', () => sim.setManual([null, null, null, null, null, 73]))
          )
        )
      );
      selEl = h('div');
      el.appendChild(U.card('Auswahl', selEl));
      objEl = h('div', { class: 'tblwrap' });
      const colSel = U.sel(COLORS, newObj.color, (v) => (newObj.color = v));
      el.appendChild(
        U.card(
          'Objekte',
          objEl,
          U.row(
            U.field('Kante', U.num(newObj.size, (v) => (newObj.size = BS.clamp(v, 10, 65)), { cls: 'w44' })),
            U.field('x', U.num(newObj.x, (v) => (newObj.x = v), { cls: 'w52', step: 10 })),
            U.field('y', U.num(newObj.y, (v) => (newObj.y = v), { cls: 'w52', step: 10 })),
            colSel,
            U.btn('+ Würfel', () => {
              const nm = (COLORS.find((c) => c[0] === newObj.color) || [0, ''])[1];
              const s = newObj.size;
              sim.addObject({ size: [s, s, s], color: newObj.color, name: 'Würfel ' + nm, pos: [newObj.x, newObj.y, s / 2 + 1], yaw: (Math.atan2(newObj.y, newObj.x) * BS.R2D) });
            })
          ),
          U.hint('Neue Würfel werden zum Roboter ausgerichtet (Gierwinkel = Azimut).')
        )
      );
      tgtEl = h('div');
      el.appendChild(U.card('Zielbereiche', tgtEl));
      convEl = h('div');
      convCard = U.card('Förderband & Lichtschranken', convEl);
      el.appendChild(convCard);
      renderObjects();
      renderTargets();
      renderConv();
      renderSel();
    },
    enter() {
      renderObjects();
      renderTargets();
      renderConv();
      renderSel();
    },
    update() {
      const sim = BS.sim;
      updateObjects();
      gripEl.innerHTML = `<div class="kv"><b>Öffnung</b><span class="mono">${sim.jawWidth().toFixed(1)} mm (M6 Ist ${sim.act[5].toFixed(1)}°)</span><b>Objekt</b><span>${sim.held ? `<span class="ok">${sim.held.obj.name}</span> – Greifbreite ${sim.held.gripW.toFixed(1)} mm, ${sim.held.obj.mass} g` : '<span class="muted">keins</span>'}</span></div>`;
      if (sim.targets.length) {
        let ok = 0,
          need = 0;
        for (const o of sim.objects) {
          const t = sim.targets.find((t) => sim.inTarget(o, t));
          if (t && (!t.accept || t.accept === o.color || t.accept === o.tag)) ok++;
        }
        need = sim.scenario === 'band' ? null : sim.objects.length;
        scoreEl.innerHTML = `Richtig abgelegt: <b class="${need && ok === need ? 'ok' : ''}">${ok}${need ? ' / ' + need : ''}</b>${need && ok === need ? ' 🎉' : ''}`;
      } else scoreEl.innerHTML = '';
      if (BS.view.selected) renderSel();
      if (sim.conveyor) {
        const cv = sim.conveyor;
        const led = (on, txt, g) => `<span class="led ${g ? 'green' : ''} ${on ? 'on' : ''}">${txt}</span>`;
        let st = convEl.querySelector('.convstate');
        if (!st) {
          st = h('div', { class: 'convstate', style: { margin: '6px 0' } });
          convEl.prepend(st);
        }
        st.innerHTML = led(cv.running, 'D7 Band läuft', true) + led(cv.lsLowState, 'D2 LS unten') + led(cv.lsHighState, 'D4 LS oben');
      }
    },
  });
})();
