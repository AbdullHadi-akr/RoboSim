/* Tab „Kalibrierung“: Fertigungsfehler, Messung, Least-Squares-Identifikation, Kompensation */
(function () {
  'use strict';
  const BS = window.BS,
    U = BS.ui,
    h = BS.h,
    C = BS.config;

  let rows = BS.store.get('calibRows', []);
  let sigma = 0.5,
    est = null,
    estJoints = [true, true, true, true],
    showGhost = true;
  let liveEl, nearEl, tableEl, revealEl, estEl, compInputs, markerSel;
  const save = () => BS.store.set('calibRows', rows);

  const randn = () => {
    let u = 0,
      v = 0;
    while (!u) u = Math.random();
    while (!v) v = Math.random();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  };
  const servoIn = () => BS.sim.cmd.map((c, i) => c + (BS.sim.compOn ? BS.sim.comp[i] : 0));
  const tcpTrue = () => {
    const T = BS.sim.F[5];
    return [T[3], T[7], T[11]];
  };
  function nearest() {
    const p = tcpTrue();
    let best = null;
    for (const mk of BS.sim.markers) {
      const d = Math.hypot(p[0] - mk.p[0], p[1] - mk.p[1], p[2] - mk.p[2]);
      if (!best || d < best.d) best = { mk, d, dx: p[0] - mk.p[0], dy: p[1] - mk.p[1], dz: p[2] - mk.p[2] };
    }
    return best;
  }

  function addRow(src, p) {
    if (BS.sim.owner !== 'manual' || BS.sim.vel.some((v) => Math.abs(v) > 1)) BS.toast('Hinweis: Roboter steht nicht still', 'warn');
    rows.push({ src, m: servoIn().map((v) => +v.toFixed(2)), p: p.map((v) => +v.toFixed(2)) });
    save();
    est = null;
    renderTable();
    renderEst();
  }

  function renderTable() {
    tableEl.innerHTML = '';
    tableEl.appendChild(
      h('tr', null, h('th', null, '#'), h('th', null, 'Quelle'), C.short.slice(0, 4).map((n) => h('th', { class: 'num' }, n)), ['x', 'y', 'z'].map((n) => h('th', { class: 'num' }, n)), h('th', { class: 'num', title: 'Residuum nach Identifikation' }, 'Res.'), h('th'))
    );
    rows.forEach((r, k) => {
      const res = est ? Math.hypot(est.residuals[3 * k], est.residuals[3 * k + 1], est.residuals[3 * k + 2]) : null;
      tableEl.appendChild(
        h(
          'tr',
          null,
          h('td', { class: 'muted' }, k + 1),
          h('td', null, r.src),
          r.m.slice(0, 4).map((v) => h('td', { class: 'num' }, v.toFixed(1))),
          r.p.map((v) => h('td', { class: 'num' }, v.toFixed(1))),
          h('td', { class: 'num' }, res == null ? '–' : res.toFixed(2)),
          h(
            'td',
            null,
            U.btn('✕', () => {
              rows.splice(k, 1);
              save();
              est = null;
              renderTable();
              renderEst();
            }, 'small icon')
          )
        )
      );
    });
    if (!rows.length) tableEl.appendChild(h('tr', null, h('td', { colspan: 11, class: 'muted' }, 'Noch keine Messpunkte.')));
  }

  function renderReveal() {
    const sim = BS.sim;
    if (!sim.errRevealed) {
      revealEl.innerHTML = '<span class="muted">Wahre Offsets verborgen.</span>';
      return;
    }
    revealEl.innerHTML =
      '<table class="tbl"><tr><th></th>' +
      C.short.slice(0, 5).map((n) => `<th class="num">${n}</th>`).join('') +
      '</tr><tr><td class="muted">wahrer Offset [°]</td>' +
      sim.errOff.slice(0, 5).map((v) => `<td class="num">${v.toFixed(2)}</td>`).join('') +
      '</tr></table>';
  }

  function renderEst() {
    if (!est) {
      estEl.innerHTML = rows.length < 3 ? '<span class="muted">Mindestens 3 (besser ≥ 6 verteilte) Messpunkte aufnehmen.</span>' : '';
      return;
    }
    const sim = BS.sim;
    let s =
      '<table class="tbl"><tr><th></th>' +
      C.short.slice(0, 4).map((n) => `<th class="num">${n}</th>`).join('') +
      '</tr><tr><td class="muted">geschätzt δ̂ [°]</td>' +
      est.delta.map((v, i) => `<td class="num">${estJoints[i] ? v.toFixed(2) : '–'}</td>`).join('') +
      '</tr>';
    if (sim.errRevealed)
      s +=
        '<tr><td class="muted">wahr [°]</td>' +
        sim.errOff.slice(0, 4).map((v) => `<td class="num">${v.toFixed(2)}</td>`).join('') +
        '</tr><tr><td class="muted">Fehler [°]</td>' +
        est.delta.map((v, i) => `<td class="num">${estJoints[i] ? (v - sim.errOff[i]).toFixed(2) : '–'}</td>`).join('') +
        '</tr>';
    s += `</table><div class="kv" style="margin-top:6px"><b>RMS vorher</b><span class="mono">${est.rmsBefore.toFixed(2)} mm</span><b>RMS nachher</b><span class="mono ok">${est.rmsAfter.toFixed(2)} mm</span></div>`;
    estEl.innerHTML = s;
  }

  function exportCsv() {
    let s = 'quelle;M1;M2;M3;M4;M5;M6;x_mm;y_mm;z_mm\n';
    for (const r of rows) s += [r.src, ...r.m, ...r.p].join(';') + '\n';
    BS.download('braccio_kalibrierung.csv', s, 'text/csv');
  }

  U.register('calib', {
    title: 'Kalibrierung',
    build(el) {
      const sim = BS.sim;
      liveEl = h('div');
      revealEl = h('div');
      el.appendChild(
        U.card(
          '1 · Fertigungsfehler des „realen“ Roboters',
          U.hint('Bei realen Robotern stimmen die Servo-Nullstellungen nie exakt (Montage der Servohörner, Getriebespiel). Hier werden zufällige, <b>verborgene</b> Gelenk-Offsets erzeugt: <span class="mono">q_real = q_soll + δ</span>.'),
          U.row(
            U.chk('Offsets aktiv', sim.errOn, (v) => {
              sim.errOn = v;
              if (v && sim.errOff.every((x) => x === 0)) sim.randomizeErrors();
            }),
            U.field('max. ±', U.num(sim.errMax, (v) => (sim.errMax = Math.abs(v)), { cls: 'w44', step: 0.5 })),
            h('span', { class: 'muted' }, '°'),
            U.btn('Neue Fehler würfeln', () => {
              sim.randomizeErrors();
              est = null;
              renderEst();
              renderReveal();
              BS.toast('Neue verborgene Offsets erzeugt', 'ok');
            }),
            U.btn('Aufdecken', () => {
              sim.errRevealed = !sim.errRevealed;
              renderReveal();
              renderEst();
            })
          ),
          U.row(
            U.chk('Ideal-Modell als Geist zeigen', showGhost, (v) => (showGhost = v)),
            U.chk('Marker K1–K6 zeigen', BS.view.show.markers, (v) => {
              BS.view.show.markers = v;
              U.syncToggles();
            })
          ),
          liveEl,
          revealEl
        )
      );

      nearEl = h('div', { class: 'mono', style: { fontSize: '12px' } });
      markerSel = U.sel(
        sim.markers.map((m) => [String(m.id), `${m.name} (${m.p.join(', ')})`]),
        '1',
        () => {}
      );
      tableEl = h('table', { class: 'tbl' });
      el.appendChild(
        U.card(
          '2 · Messen',
          U.hint('<b>a) Teach-in:</b> TCP mit Slidern/Jog genau auf eine gelbe Kugel fahren und „Marker-Punkt aufnehmen“. <b>b) Lasertracker:</b> misst die reale TCP-Position (mit Rauschen σ).'),
          U.row(
            U.field('Marker', markerSel),
            U.btn('Anfahren (Modell-IK)', () => {
              const mk = sim.markers.find((m) => String(m.id) === markerSel.value);
              const s = BS.kin.ikAuto({ x: mk.p[0], y: mk.p[1], z: mk.p[2] }, C.geom, { psiPref: -90, cur: sim.cmd });
              if (!s) return BS.toast('Marker nicht erreichbar', 'err');
              BS.view.show.markers = true;
              U.syncToggles();
              sim.moveTo([s.m[0], s.m[1], s.m[2], s.m[3]].map((v) => Math.round(v * 10) / 10));
            }, '', 'Fährt den Marker mit dem Nominalmodell an – mit Fertigungsfehlern landet der TCP daneben')
          ),
          nearEl,
          U.row(
            U.btn('Marker-Punkt aufnehmen', () => {
              const n = nearest();
              if (n.d > 15) return BS.toast(`TCP ist ${n.d.toFixed(1)} mm von ${n.mk.name} entfernt – erst genauer ausrichten`, 'warn');
              addRow(n.mk.name, n.mk.p);
            }, 'primary'),
            U.field('σ', U.num(sigma, (v) => (sigma = Math.max(0, v)), { cls: 'w44', step: 0.1 })),
            U.btn('TCP messen (Lasertracker)', () => {
              const p = tcpTrue().map((v) => v + randn() * sigma);
              addRow('Tracker', p);
            })
          ),
          h('div', { class: 'tblwrap' }, tableEl),
          U.row(
            U.btn('Alle löschen', () => {
              rows = [];
              save();
              est = null;
              renderTable();
              renderEst();
            }, 'small'),
            U.btn('CSV exportieren', exportCsv, 'small')
          ),
          U.hint('Gespeichert wird der <b>an den Servo gesendete</b> Winkel (Soll + ggf. Kompensation) zusammen mit der gemessenen Position.')
        )
      );

      estEl = h('div');
      el.appendChild(
        U.card(
          '3 · Identifikation (Least Squares)',
          U.hint('Gesucht sind die Offsets δ, die <span class="mono">Σ‖f_dir(q_k + δ) − p_k‖²</span> minimieren (Levenberg-Marquardt, Jacobi-Matrix aus Kap. 3).'),
          U.row(
            ...C.short.slice(0, 4).map((n, i) => U.chk(n, estJoints[i], (v) => (estJoints[i] = v))),
            U.btn('Offsets schätzen', () => {
              const js = [0, 1, 2, 3].filter((i) => estJoints[i]);
              if (rows.length < 2 || !js.length) return BS.toast('Zu wenige Messpunkte', 'warn');
              est = BS.kin.estimateOffsets(rows, js, C.geom);
              renderEst();
              renderTable();
            }, 'primary')
          ),
          estEl,
          U.row(
            U.btn('Als Kompensation übernehmen', () => {
              if (!est) return;
              for (let i = 0; i < 4; i++) if (estJoints[i]) sim.comp[i] = +(sim.comp[i] - est.delta[i]).toFixed(2);
              sim.compOn = true;
              compChk.input.checked = true;
              compInputs.forEach((inp, i) => (inp.value = sim.comp[i]));
              BS.store.set('comp', sim.comp);
              BS.toast('Kompensation aktiv', 'ok');
            })
          )
        )
      );

      compInputs = [];
      const compChk = U.chk('Kompensation aktiv', sim.compOn, (v) => (sim.compOn = v));
      const compRow = U.row();
      for (let i = 0; i < 5; i++) {
        const inp = U.num(sim.comp[i], (v) => {
          sim.comp[i] = v;
          BS.store.set('comp', sim.comp);
        }, { cls: 'w52', step: 0.1 });
        compInputs.push(inp);
        compRow.appendChild(U.field(C.short[i], inp));
      }
      el.appendChild(
        U.card(
          '4 · Kompensation',
          U.hint('Servo-Winkel = Soll + Kompensation (entspricht <span class="mono">Soll − δ̂</span>). Im Arduino-Code wird das z. B. über ein <span class="mono">OFFSET[]</span>-Array umgesetzt (Beispiel „Kalibrierung“).'),
          compRow,
          U.row(
            compChk,
            U.btn('Zurücksetzen', () => {
              sim.comp = [0, 0, 0, 0, 0, 0];
              compInputs.forEach((inp) => (inp.value = 0));
              BS.store.set('comp', sim.comp);
            }, 'small')
          )
        )
      );
      renderTable();
      renderReveal();
      renderEst();
    },
    leave() {
      BS.view.ghostPose = null;
    },
    update() {
      const sim = BS.sim;
      const real = tcpTrue();
      const model = BS.kin.tcp(sim.cmd);
      const d = Math.hypot(real[0] - model.x, real[1] - model.y, real[2] - model.z);
      liveEl.innerHTML = `<div class="kv"><b>Abweichung TCP real ↔ Modell</b><span class="mono ${d > 2 ? 'warn' : 'ok'}">${d.toFixed(2)} mm</span><b>Status</b><span>${sim.errOn ? '<span class="warn">Fehler aktiv</span>' : 'ideal (keine Fehler)'}${sim.compOn ? ' · Kompensation aktiv' : ''}</span></div>`;
      const n = nearest();
      nearEl.innerHTML = `Nächster Marker <b>${n.mk.name}</b>: |d| = <b class="${n.d < 1 ? 'ok' : n.d < 5 ? 'warn' : ''}">${n.d.toFixed(2)} mm</b> (dx ${n.dx.toFixed(1)}, dy ${n.dy.toFixed(1)}, dz ${n.dz.toFixed(1)})`;
      BS.view.ghostPose = showGhost && sim.errOn ? sim.cmd.slice() : null;
    },
  });
})();
