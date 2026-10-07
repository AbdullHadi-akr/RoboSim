/* Tab „Arbeitsraum“: Visualisierung, Überwachung, Zonen, Ereignisprotokoll */
(function () {
  'use strict';
  const BS = window.BS,
    U = BS.ui,
    h = BS.h,
    C = BS.config;

  let ws = null,
    wsMode = 'free',
    wsPsi = -90,
    canvas,
    hoverEl,
    statusEl,
    zonesEl,
    logEl,
    hover = null;

  const EXAMPLE_ZONES = [
    { name: 'Bedienerplatz', type: 'forbidden', c: [-330, 120, 150], s: [160, 300, 300] },
    { name: 'Säule', type: 'forbidden', c: [180, 330, 120], s: [60, 60, 240] },
    { name: 'Arbeitszelle', type: 'allowed', c: [0, 150, 250], s: [900, 600, 600] },
  ];

  function saveZones() {
    BS.store.set('zones', BS.sim.zones);
  }

  function compute() {
    ws = BS.kin.workspace({ mode: wsMode, psi: wsPsi, g: C.geom });
    BS.view.setWorkspace(ws);
    draw();
  }

  function draw() {
    if (!canvas) return;
    const dpr = window.devicePixelRatio || 1;
    const W = canvas.clientWidth,
      H = canvas.clientHeight;
    if (!W) return;
    canvas.width = W * dpr;
    canvas.height = H * dpr;
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = '#0f1216';
    ctx.fillRect(0, 0, W, H);
    const r0 = -500,
      r1 = 500,
      z0 = -150,
      z1 = 560;
    const sc = Math.min(W / (r1 - r0), H / (z1 - z0));
    const ox = (W - (r1 - r0) * sc) / 2,
      oy = (H - (z1 - z0) * sc) / 2;
    const X = (r) => ox + (r - r0) * sc,
      Y = (z) => oy + (z1 - z) * sc;
    canvas._map = { X, Y, sc, ox, oy, r0, z1 };
    // Raster
    ctx.strokeStyle = '#232a33';
    ctx.lineWidth = 1;
    for (let r = -500; r <= 500; r += 100) {
      ctx.beginPath();
      ctx.moveTo(X(r), Y(z0));
      ctx.lineTo(X(r), Y(z1));
      ctx.stroke();
    }
    for (let z = -100; z <= 500; z += 100) {
      ctx.beginPath();
      ctx.moveTo(X(r0), Y(z));
      ctx.lineTo(X(r1), Y(z));
      ctx.stroke();
    }
    // Arbeitsraum
    if (ws) {
      const cw = Math.max(1, ws.cell * sc);
      for (let iz = 0; iz < ws.nz; iz++)
        for (let ir = 0; ir < ws.nr; ir++) {
          const v = ws.grid[iz * ws.nr + ir];
          if (!v) continue;
          ctx.fillStyle = v === 1 ? 'rgba(64,192,87,0.75)' : 'rgba(253,126,20,0.6)';
          ctx.fillRect(X(ws.r0 + ir * ws.cell), Y(ws.z0 + (iz + 1) * ws.cell), cw + 0.5, cw + 0.5);
        }
    }
    // Tisch & Basis
    ctx.fillStyle = 'rgba(231,227,218,0.15)';
    ctx.fillRect(X(r0), Y(0), (r1 - r0) * sc, (0 - z0) * sc);
    ctx.strokeStyle = '#e7e3da';
    ctx.beginPath();
    ctx.moveTo(X(r0), Y(0));
    ctx.lineTo(X(r1), Y(0));
    ctx.stroke();
    ctx.fillStyle = '#3a404a';
    ctx.fillRect(X(-75), Y(29), 150 * sc, 29 * sc);
    // Arm (Ideal-Ebene, aktueller Ist-Zustand)
    const p = BS.kin.planar(BS.sim.trueAngles(), C.geom);
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 3;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(X(0), Y(29));
    ctx.lineTo(X(0), Y(p.shoulder.z));
    ctx.lineTo(X(p.elbow.r), Y(p.elbow.z));
    ctx.lineTo(X(p.wrist.r), Y(p.wrist.z));
    ctx.lineTo(X(p.tip.r), Y(p.tip.z));
    ctx.stroke();
    ctx.fillStyle = '#ff7a00';
    ctx.beginPath();
    ctx.arc(X(p.tcp.r), Y(p.tcp.z), 4, 0, 7);
    ctx.fill();
    for (const q of [p.shoulder, p.elbow, p.wrist]) {
      ctx.fillStyle = '#19b6ae';
      ctx.beginPath();
      ctx.arc(X(q.r), Y(q.z), 3.5, 0, 7);
      ctx.fill();
    }
    // Achsen
    ctx.fillStyle = '#8f9aab';
    ctx.font = '11px system-ui';
    ctx.fillText('r [mm] →  (Richtung M1)', X(150), Y(-120));
    ctx.fillText('z [mm]', X(-490), Y(540));
    for (let r = -400; r <= 400; r += 200) ctx.fillText(String(r), X(r) - 8, Y(-140) + 10);
    for (let z = 0; z <= 500; z += 100) ctx.fillText(String(z), X(-498), Y(z) - 2);
    if (hover) {
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.arc(X(hover.r), Y(hover.z), 5, 0, 7);
      ctx.stroke();
    }
  }

  function renderZones() {
    const sim = BS.sim;
    zonesEl.innerHTML = '';
    if (!sim.zones.length) zonesEl.appendChild(U.hint('Keine Zonen definiert.'));
    for (const z of sim.zones) {
      const upd = () => {
        saveZones();
      };
      const nm = h('input', { type: 'text', value: z.name, style: { width: '120px' } });
      nm.addEventListener('change', () => {
        z.name = nm.value;
        upd();
      });
      const typ = U.sel(
        [
          ['forbidden', 'Sperrzone'],
          ['allowed', 'Arbeitsbereich (TCP muss innen bleiben)'],
        ],
        z.type,
        (v) => {
          z.type = v;
          upd();
        }
      );
      const nums = (arr, lbls) => lbls.map((l, i) => U.field(l, U.num(arr[i], (v) => ((arr[i] = v), upd()), { cls: 'w52', step: 10 })));
      zonesEl.appendChild(
        h(
          'div',
          { style: { borderTop: '1px solid var(--border)', padding: '6px 0' } },
          U.row(nm, typ, h('span', { class: 'grow' }), U.btn('✕', () => {
            sim.zones = sim.zones.filter((x) => x !== z);
            saveZones();
            renderZones();
          }, 'small icon')),
          U.row(h('span', { class: 'muted', style: { width: '52px' } }, 'Mitte'), ...nums(z.c, ['x', 'y', 'z'])),
          U.row(h('span', { class: 'muted', style: { width: '52px' } }, 'Größe'), ...nums(z.s, ['Δx', 'Δy', 'Δz']))
        )
      );
    }
  }
  function addZone(z) {
    const sim = BS.sim;
    sim.zones.push(Object.assign({ id: sim.nextId++ }, JSON.parse(JSON.stringify(z))));
    saveZones();
    renderZones();
  }

  function renderLog() {
    const mon = BS.monitor;
    logEl.innerHTML = mon.log.length
      ? mon.log
          .slice(0, 60)
          .map((e) => `<div><span class="mono muted">${e.t.toFixed(2)} s</span> <span class="${e.level === 'viol' ? 'err' : 'warn'}">${e.level === 'viol' ? '■' : '▲'}</span> ${e.msg}</div>`)
          .join('')
      : '<span class="muted">Keine Ereignisse.</span>';
  }
  BS.on('monlog', () => logEl && U.active === 'workspace' && renderLog());

  U.register('workspace', {
    title: 'Arbeitsraum',
    build(el) {
      const mon = BS.monitor;
      canvas = h('canvas', { class: 'plot tall', style: { cursor: 'crosshair' } });
      hoverEl = h('div', { class: 'mono', style: { fontSize: '12px', minHeight: '18px' } });
      canvas.addEventListener('mousemove', (e) => {
        const m = canvas._map;
        if (!m) return;
        const rect = canvas.getBoundingClientRect();
        const r = (e.clientX - rect.left - m.ox) / m.sc + m.r0;
        const z = m.z1 - (e.clientY - rect.top - m.oy) / m.sc;
        hover = { r, z };
        let st = '–';
        if (ws) {
          const ir = Math.floor((r - ws.r0) / ws.cell),
            iz = Math.floor((z - ws.z0) / ws.cell);
          const v = ir >= 0 && ir < ws.nr && iz >= 0 && iz < ws.nz ? ws.grid[iz * ws.nr + ir] : 0;
          st = v === 1 ? '<span class="ok">erreichbar</span>' : v === 2 ? '<span class="warn">unzulässig (Kollision)</span>' : '<span class="muted">unerreichbar</span>';
        }
        hoverEl.innerHTML = `r = ${r.toFixed(0)} mm, z = ${z.toFixed(0)} mm → ${st} &nbsp;<span class="muted">(Klick: als IK-Ziel)</span>`;
        draw();
      });
      canvas.addEventListener('mouseleave', () => {
        hover = null;
        draw();
      });
      canvas.addEventListener('click', () => {
        if (!hover) return;
        const a = BS.sim.cmd[0] * BS.D2R;
        BS.emit('setIkTarget', { x: hover.r * Math.cos(a), y: hover.r * Math.sin(a), z: hover.z });
      });
      const psiIn = U.num(wsPsi, (v) => {
        wsPsi = v;
        if (wsMode === 'fixed') compute();
      }, { cls: 'w52' });
      el.appendChild(
        U.card(
          'Arbeitsraum (Schnitt in der Armebene)',
          U.row(
            U.field(
              'Werkzeugwinkel',
              U.sel(
                [
                  ['free', 'ψ beliebig'],
                  ['fixed', 'ψ fest'],
                ],
                wsMode,
                (v) => {
                  wsMode = v;
                  compute();
                }
              )
            ),
            U.field('ψ [°]', psiIn),
            U.btn('Berechnen', compute, 'primary')
          ),
          U.row(
            U.chk('Schnitt im 3D', BS.view.show.wsPlane, (v) => {
              BS.view.show.wsPlane = v;
              if (v && !ws) compute();
            }),
            U.chk('Hülle im 3D', BS.view.show.wsHull, (v) => {
              BS.view.show.wsHull = v;
              if (v && !ws) compute();
            })
          ),
          canvas,
          hoverEl,
          U.hint('<span class="ok">■</span> erreichbar · <span class="warn">■</span> erreichbar, aber unzulässig (Tisch- oder Selbstkollision) · leer: unerreichbar. Negative r: Arm lehnt sich nach hinten (M2 &lt; 90°). Der Arbeitsraum ist rotationssymmetrisch um die z-Achse.')
        )
      );

      statusEl = h('div', { style: { margin: '6px 0' } });
      el.appendChild(
        U.card(
          'Arbeitsraumüberwachung',
          U.row(
            U.chk('Überwachung aktiv', mon.enabled, (v) => (mon.enabled = v)),
            U.field(
              'Reaktion',
              U.sel(
                [
                  ['warn', 'nur warnen'],
                  ['slow', 'Geschwindigkeit reduzieren (25 %)'],
                  ['stop', 'Not-Halt (Programm stoppen)'],
                ],
                mon.reaction,
                (v) => {
                  mon.reaction = v;
                  BS.store.set('monReaction', v);
                }
              )
            )
          ),
          U.row(
            U.field('Sicherheitsabstand', U.num(mon.margin, (v) => (mon.margin = Math.max(0, v)), { cls: 'w44' })),
            h('span', { class: 'muted' }, 'mm'),
            U.chk('Tischkollision', mon.table, (v) => (mon.table = v)),
            U.chk('Selbstkollision Basis', mon.base, (v) => (mon.base = v))
          ),
          U.row(U.chk('TCP-Geschwindigkeit begrenzen', mon.vLimitOn, (v) => (mon.vLimitOn = v)), U.num(mon.vLimit, (v) => (mon.vLimit = v), { cls: 'w52', step: 10 }), h('span', { class: 'muted' }, 'mm/s')),
          statusEl,
          U.row(U.btn('Not-Halt quittieren', () => BS.sim.ackEstop())),
          U.hint('Geprüft werden Punkte entlang Oberarm, Unterarm, Greifer und Fingern gegen alle Zonen (mit Sicherheitsabstand), gegen den Tisch (z &lt; 0) und gegen die Basis.')
        )
      );
      zonesEl = h('div');
      el.appendChild(
        U.card(
          'Zonen',
          U.row(
            U.btn('+ Sperrzone', () => addZone({ name: 'Sperrzone', type: 'forbidden', c: [200, 200, 100], s: [100, 100, 200] })),
            U.btn('+ Arbeitsbereich', () => addZone({ name: 'Arbeitsbereich', type: 'allowed', c: [0, 200, 200], s: [600, 400, 400] })),
            U.btn('Beispielzonen', () => {
              BS.sim.zones = [];
              EXAMPLE_ZONES.forEach(addZone);
              BS.view.show.zones = true;
              U.syncToggles();
            }),
            U.btn('Alle löschen', () => {
              BS.sim.zones = [];
              saveZones();
              renderZones();
            })
          ),
          zonesEl
        )
      );
      logEl = h('div', { style: { maxHeight: '180px', overflow: 'auto', fontSize: '12px' } });
      el.appendChild(
        U.card(
          h('span', null, 'Ereignisprotokoll'),
          logEl,
          U.row(
            U.btn('Leeren', () => {
              BS.monitor.log = [];
              renderLog();
            }, 'small')
          )
        )
      );
      renderZones();
      renderLog();
    },
    enter() {
      if (!ws) compute();
      renderLog();
      setTimeout(draw, 20);
    },
    update() {
      const mon = BS.monitor;
      draw();
      statusEl.innerHTML = !mon.enabled
        ? '<span class="badge">aus</span>'
        : mon.state === 'ok'
          ? '<span class="badge ok">OK</span>'
          : mon.issues.map((i) => `<div><span class="badge ${i.level === 'viol' ? 'err' : 'warn'}">${i.level === 'viol' ? 'Verletzung' : 'Warnung'}</span> ${i.msg}</div>`).join('');
      if (BS.sim.estop) statusEl.innerHTML += `<div style="margin-top:4px"><span class="badge err">NOT-HALT</span> ${BS.sim.estopReason}</div>`;
    },
  });
})();
