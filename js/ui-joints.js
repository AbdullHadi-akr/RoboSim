/* Tab „Gelenke“: Servo-Slider, Vorwärtskinematik, kartesisches Verfahren */
(function () {
  'use strict';
  const BS = window.BS,
    U = BS.ui,
    h = BS.h,
    C = BS.config;

  let rows = [],
    lockNote,
    tcpBox,
    codeLine,
    jogStep = 5;

  function codeFor(m) {
    return `Braccio.ServoMovement(20, ${m.map((v) => Math.round(v)).join(', ')});`;
  }

  function jog(axis, dir) {
    const sim = BS.sim;
    if (!sim.canManual()) return BS.toast('Programm/Trajektorie aktiv – manuelles Verfahren gesperrt', 'warn');
    const cur = sim.cmd.slice();
    const p = BS.kin.tcp(cur);
    const t = { x: p.x, y: p.y, z: p.z };
    let psi = p.psi;
    if (axis === 'psi') psi += dir * jogStep;
    else t[axis] += dir * jogStep;
    const r = BS.kin.cartMove(cur, t, psi, C.geom);
    if (r.frac > 0) sim.setManual(r.m.slice(0, 4));
    if (r.ok) return;
    if (BS.kin.ikBest(t, psi, C.geom, cur))
      BS.toast('Singularität: Weiterfahren erfordert einen Konfigurationswechsel (vorne/hinten) – kartesisch nicht stetig möglich. Bitte per PTP (Tab IK) umorientieren.', 'warn');
    else {
      const any = BS.kin.ik(t, psi, C.geom, { m1: cur[0] }).find((x) => x.reachable);
      BS.toast(any ? 'Nicht zulässig: ' + any.reasons[0] : 'Außerhalb des Arbeitsraums', 'warn');
    }
  }

  U.register('joints', {
    title: 'Gelenke',
    build(el) {
      const sim = BS.sim;
      lockNote = U.hint('');
      const jcard = U.card('Gelenkwinkel (Servo-Sollwerte)', lockNote);
      rows = C.names.map((name, i) => {
        const [lo, hi] = C.limits[i];
        const range = h('input', { type: 'range', min: lo, max: hi, step: 1, value: sim.cmd[i] });
        const num = U.num(sim.cmd[i], (v) => {
          const val = BS.clamp(Math.round(v), lo, hi);
          if (!sim.setManual(Object.assign([], { [i]: val }))) BS.toast('Programm läuft – bitte zuerst stoppen', 'warn');
        }, { min: lo, max: hi, cls: 'w52' });
        range.addEventListener('input', () => {
          if (!sim.setManual(Object.assign([], { [i]: +range.value }))) BS.toast('Programm läuft – bitte zuerst stoppen', 'warn');
        });
        const ist = h('span', { class: 'jist' });
        jcard.appendChild(h('div', { class: 'jrow' }, h('span', { class: 'jname', style: { '--c': C.colors[i] } }, name), range, num, ist));
        return { range, num, ist };
      });
      jcard.appendChild(
        U.row(
          U.btn('Aufrecht (90°)', () => sim.moveTo(C.home)),
          U.btn('Begin-Pose', () => sim.moveTo(C.beginPose)),
          U.btn('Greifer auf', () => sim.setManual([null, null, null, null, null, 10])),
          U.btn('Greifer zu', () => sim.setManual([null, null, null, null, null, 73])),
          U.btn('■ Stopp', () => sim.stopMotion(), 'danger')
        )
      );
      jcard.appendChild(U.hint('Die Sollwerte gehen direkt an die Servos (wie <span class="mono">Servo.write()</span>). Die Ist-Werte folgen mit begrenzter Servo-Geschwindigkeit.'));
      el.appendChild(jcard);

      tcpBox = h('div');
      el.appendChild(U.card('Vorwärtskinematik – TCP-Pose', tcpBox, U.hint('<b>Real</b>: tatsächliche Pose (mit Fertigungsfehlern, falls aktiv). <b>Modell</b>: <span class="mono">f_dir(Soll)</span> mit Nenngeometrie.')));

      const stepSel = U.sel(
        [
          ['1', '1 mm / 1°'],
          ['5', '5 mm / 5°'],
          ['10', '10 mm / 10°'],
          ['25', '25 mm / 25°'],
        ],
        '5',
        (v) => (jogStep = +v)
      );
      const grid = h('div', { style: { display: 'grid', gridTemplateColumns: 'repeat(4, auto)', gap: '6px', justifyContent: 'start' } });
      for (const [ax, l] of [
        ['x', 'X'],
        ['y', 'Y'],
        ['z', 'Z'],
        ['psi', 'ψ'],
      ]) {
        grid.append(U.repeatBtn(l + ' −', () => jog(ax, -1)), U.repeatBtn(l + ' +', () => jog(ax, +1)));
      }
      el.appendChild(
        U.card(
          'Kartesisch verfahren (inverse Kinematik)',
          U.row(U.field('Schrittweite', stepSel)),
          grid,
          U.hint('Gedrückt halten zum Wiederholen. Die IK hält den Werkzeugwinkel ψ konstant und wählt die Lösung, die der aktuellen Pose am nächsten liegt.')
        )
      );

      codeLine = h('pre', { class: 'code' });
      el.appendChild(
        U.card(
          'Arduino-Code für die aktuelle Pose',
          codeLine,
          U.row(
            U.btn('Kopieren', () => U.copy(codeFor(BS.sim.cmd))),
            U.btn('+ Als Wegpunkt', () => {
              BS.emit('addWaypoint', BS.sim.cmd.slice());
              BS.toast('Wegpunkt hinzugefügt (Tab Trajektorie)', 'ok');
            })
          )
        )
      );
    },
    update() {
      const sim = BS.sim;
      const manual = sim.canManual();
      lockNote.innerHTML = manual
        ? ''
        : sim.owner === 'program'
          ? '<span class="warn">Arduino-Programm aktiv – Slider zeigen die Sollwerte des Programms.</span>'
          : '<span class="warn">Trajektorie aktiv – „Stopp“ beendet sie.</span>';
      rows.forEach((r, i) => {
        r.range.disabled = sim.owner === 'program';
        if (document.activeElement !== r.range) r.range.value = sim.cmd[i];
        if (document.activeElement !== r.num) r.num.value = Math.round(sim.cmd[i] * 10) / 10;
        r.ist.textContent = 'Ist ' + sim.act[i].toFixed(1) + '°';
      });
      const real = BS.kin.tcp(sim.trueAngles());
      const model = BS.kin.tcp(sim.cmd);
      const d = Math.hypot(real.x - model.x, real.y - model.y, real.z - model.z);
      const r = (lbl, p) =>
        `<tr><td>${lbl}</td><td class="num">${BS.fmt(p.x)}</td><td class="num">${BS.fmt(p.y)}</td><td class="num">${BS.fmt(p.z)}</td><td class="num">${BS.fmt(p.psi)}</td></tr>`;
      tcpBox.innerHTML =
        `<table class="tbl"><tr><th></th><th class="num">x [mm]</th><th class="num">y [mm]</th><th class="num">z [mm]</th><th class="num">ψ [°]</th></tr>` +
        r('Real', real) +
        r('Modell', model) +
        `</table><div class="hint">Abweichung Real ↔ Modell: <b class="${d > 2 ? 'warn' : 'ok'}">${d.toFixed(2)} mm</b></div>`;
      codeLine.textContent = codeFor(sim.cmd);
    },
  });
})();
