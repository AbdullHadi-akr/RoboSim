/* Tab „Inverse Kinematik“: geometrische Lösung, numerische Verfahren, Jacobi-Matrix */
(function () {
  'use strict';
  const BS = window.BS,
    U = BS.ui,
    h = BS.h,
    C = BS.config;

  const st = BS.store.get('ik', {
    x: 0,
    y: 260,
    z: 40,
    psi: -70,
    psiAuto: true,
    m5: 90,
    m6: 73,
    method: 'dls',
    alpha: 1.5e-6,
    lambda: 20,
    kmax: 500,
    eps: 1e-7,
    usePsi: true,
    start: 'cur',
  });
  let inX, inY, inZ, inPsi, solBox, numBox, jacBox, convPlot, dragChk, clickChk, liveChk;
  let numRes = null,
    anim = null,
    hoverPose = null,
    autoPsi = null,
    sols = [];
  const save = () => BS.store.set('ik', st);

  function target() {
    return { x: st.x, y: st.y, z: st.z };
  }
  function setTarget(t, silent) {
    st.x = Math.round(t.x * 10) / 10;
    st.y = Math.round(t.y * 10) / 10;
    st.z = Math.round(t.z * 10) / 10;
    if (inX) {
      inX.value = st.x;
      inY.value = st.y;
      inZ.value = st.z;
    }
    save();
    solve();
    if (!silent && liveChk && liveChk.input.checked) follow();
  }
  BS.on('setIkTarget', (t) => {
    setTarget(t);
    U.show('ik');
  });
  BS.on('ikTargetDrag', (t) => setTarget(t));
  BS.on('pickTable', (p) => {
    if (U.active === 'ik' && clickChk && clickChk.input.checked) setTarget({ x: p.x, y: p.y, z: st.z });
  });

  function full(m) {
    return [m[0], m[1], m[2], m[3], st.m5, st.m6];
  }

  function solve() {
    if (!solBox) return;
    let psi = st.psi;
    autoPsi = null;
    if (st.psiAuto) {
      const a = BS.kin.ikAuto(target(), C.geom, { psiPref: -90, cur: BS.sim.cmd });
      if (a) psi = autoPsi = a.psi;
    }
    sols = BS.kin.ik(target(), psi, C.geom, { m1: BS.sim.cmd[0] });
    BS.view.setIkTarget(target(), psi, true);
    const t = h('table', { class: 'tbl' });
    t.appendChild(h('tr', null, h('th', null, 'Lösung'), C.short.slice(0, 4).map((n, i) => h('th', { class: 'num', style: { color: C.colors[i] } }, n)), h('th', null, 'Status'), h('th')));
    for (const s of sols) {
      const name = `${s.side > 0 ? 'vorne' : 'hinten'}${s.m ? ', Ellb. ' + (s.elbowUp ? 'oben' : 'unten') : ''}`;
      if (!s.reachable) {
        t.appendChild(h('tr', null, h('td', null, name), h('td', { colspan: 4, class: 'muted' }, '—'), h('td', null, h('span', { class: 'badge' }, 'unerreichbar')), h('td')));
        continue;
      }
      const badge = s.valid ? h('span', { class: 'badge ok' }, 'gültig') : s.limitsOk ? h('span', { class: 'badge warn', title: s.reasons.join('\n') }, 'unzulässig') : h('span', { class: 'badge err', title: s.reasons.join('\n') }, 'Gelenkgrenze');
      const tr = h(
        'tr',
        { class: 'hov', title: s.reasons.join('\n') || 'Zeile überfahren: Vorschau als Geist' },
        h('td', null, name),
        s.m.map((v, i) => h('td', { class: 'num ' + (v < C.limits[i][0] - 1e-6 || v > C.limits[i][1] + 1e-6 ? 'err' : '') }, v.toFixed(1))),
        h('td', null, badge),
        h('td', null, U.btn('Anfahren', () => go(s.m), 'small' + (s.valid ? '' : ''), s.valid ? '' : 'Achtung: Pose ist nicht gültig'))
      );
      tr.addEventListener('mouseenter', () => (hoverPose = full(s.m)));
      tr.addEventListener('mouseleave', () => (hoverPose = null));
      t.appendChild(tr);
    }
    solBox.innerHTML = '';
    solBox.append(
      t,
      U.hint(
        (st.psiAuto ? (autoPsi != null ? `ψ automatisch gewählt: <b>${autoPsi}°</b> (nächster zulässiger Winkel ab −90°). ` : '<span class="err">Für kein ψ gültig erreichbar.</span> ') : '') +
          '<b>unerreichbar</b>: außerhalb des Arbeitsraums · <b>Gelenkgrenze</b>: Servowinkel außerhalb des zulässigen Bereichs · <b>unzulässig</b>: erreichbar, aber Kollision (Tisch/Basis).'
      )
    );
  }

  function go(m) {
    const sim = BS.sim;
    const v = BS.kin.inLimits(m, 4);
    if (v.length) return BS.toast('Gelenkgrenze verletzt – nicht anfahrbar', 'err');
    sim.moveTo(full(m.map((x) => Math.round(x * 10) / 10)));
  }
  function follow() {
    const s = sols.find((x) => x.valid) && BS.kin.ikBest(target(), autoPsi != null ? autoPsi : st.psi, C.geom, BS.sim.cmd);
    if (s) BS.sim.setManual(full(s.m));
  }

  function runNumeric() {
    const q0 = st.start === 'cur' ? BS.sim.cmd.slice(0, 4) : st.start === 'home' ? [90, 90, 90, 90] : [90, 120, 120, 120];
    const psi = autoPsi != null && st.psiAuto ? autoPsi : st.psi;
    numRes = BS.kin.ikNumeric(target(), {
      q0,
      method: st.method,
      alpha: st.alpha,
      lambda: st.lambda,
      kmax: Math.round(st.kmax),
      eps: st.eps,
      usePsi: st.usePsi,
      psi,
    });
    const r = numRes;
    numBox.innerHTML =
      `<div class="kv"><b>Ergebnis</b><span>${r.converged ? '<span class="ok">konvergiert</span>' : '<span class="warn">nicht konvergiert</span>'} – ${r.reason}</span>` +
      `<b>Iterationen</b><span class="mono">${r.iters}</span>` +
      `<b>Positionsfehler</b><span class="mono">${r.ep.toFixed(3)} mm</span>` +
      (r.epsi != null ? `<b>Winkelfehler ψ</b><span class="mono">${r.epsi.toFixed(3)}°</span>` : '') +
      `<b>Rechenzeit</b><span class="mono">${r.ms.toFixed(2)} ms</span>` +
      `<b>q*</b><span class="mono">${r.m.map((v) => v.toFixed(2)).join(' · ')}</span></div>`;
    const data = r.hist.map((x, k) => [k, Math.max(x.ep, 1e-6)]);
    const series = [{ name: '‖e_pos‖ [mm]', color: '#19b6ae', data }];
    if (st.usePsi) series.push({ name: '|e_ψ| [°]', color: '#ffa94d', data: r.hist.map((x, k) => [k, Math.max(x.epsi, 1e-6)]) });
    convPlot.draw({ series, logY: true, xLabel: 'Iteration k', title: 'Konvergenz (logarithmisch)' });
  }

  function renderJac() {
    const m = BS.sim.cmd;
    const J = BS.kin.jacobian(m, C.geom, true);
    const info = BS.kin.jacobianInfo(m, C.geom);
    jacBox.innerHTML = '';
    jacBox.append(
      U.matrix(
        J.map((r) => r.map((v) => v)),
        1,
        ['∂x', '∂y', '∂z', '∂ψ'],
        C.short.slice(0, 4).map((n) => '∂' + n)
      ),
      h('div', {
        class: 'kv',
        html:
          `<b>Einheiten</b><span>mm/rad (Zeilen x, y, z), rad/rad (ψ)</span>` +
          `<b>Singulärwerte J_pos</b><span class="mono">${info.sv.map((v) => v.toFixed(1)).join(' · ')}</span>` +
          `<b>Manipulierbarkeit w</b><span class="mono">${info.w.toExponential(3)} = √det(J·Jᵀ)</span>` +
          `<b>Konditionszahl</b><span class="mono ${info.cond > 50 ? 'warn' : ''}">${Number.isFinite(info.cond) ? info.cond.toFixed(1) : '∞'}</span>` +
          (info.cond > 50 ? '<b></b><span class="warn">Nahe einer Singularität (z. B. gestreckter Arm)</span>' : ''),
      })
    );
  }

  U.register('ik', {
    title: 'Inverse Kinematik',
    build(el) {
      inX = U.num(st.x, (v) => setTarget({ x: v, y: st.y, z: st.z }), { cls: 'w52' });
      inY = U.num(st.y, (v) => setTarget({ x: st.x, y: v, z: st.z }), { cls: 'w52' });
      inZ = U.num(st.z, (v) => setTarget({ x: st.x, y: st.y, z: v }), { cls: 'w52' });
      inPsi = U.num(st.psi, (v) => {
        st.psi = v;
        save();
        solve();
      }, { cls: 'w52' });
      dragChk = U.chk('Ziel im 3D verschieben', false, (v) => BS.view.setTargetDrag(v));
      clickChk = U.chk('Klick auf Tisch setzt Ziel (x, y)', true, () => {});
      liveChk = U.chk('Arm folgt live', false, (v) => v && follow());
      el.appendChild(
        U.card(
          'Zielpose des TCP',
          U.row(U.field('x', inX), U.field('y', inY), U.field('z', inZ), h('span', { class: 'muted' }, 'mm')),
          U.row(
            U.field('ψ [°]', inPsi),
            U.chk('ψ automatisch', st.psiAuto, (v) => {
              st.psiAuto = v;
              save();
              solve();
            }),
            U.field('M5', U.num(st.m5, (v) => ((st.m5 = BS.clamp(v, 0, 180)), save()), { cls: 'w44' })),
            U.field('M6', U.num(st.m6, (v) => ((st.m6 = BS.clamp(v, 10, 73)), save()), { cls: 'w44' }))
          ),
          U.row(
            U.btn('Aktuelle TCP-Pose übernehmen', () => {
              const p = BS.kin.tcp(BS.sim.cmd);
              st.psi = Math.round(p.psi);
              inPsi.value = st.psi;
              setTarget(p);
            }),
            U.btn('Gültige Lösung anfahren', () => {
              const s = BS.kin.ikBest(target(), autoPsi != null && st.psiAuto ? autoPsi : st.psi, C.geom, BS.sim.cmd);
              if (s) go(s.m);
              else BS.toast('Keine gültige Lösung', 'err');
            }, 'primary')
          ),
          U.row(dragChk, clickChk, liveChk),
          U.hint('ψ = Winkel der Werkzeugachse gegenüber der Horizontalen (−90° = senkrecht nach unten), bezogen auf die radial nach außen zeigende Richtung.')
        )
      );
      solBox = h('div');
      el.appendChild(U.card('Geometrische (analytische) Lösung', solBox));

      numBox = h('div');
      const cv = h('canvas', { class: 'plot' });
      convPlot = new BS.Plot(cv);
      const methodSel = U.sel(
        [
          ['gradient', 'Gradientenverfahren  q += α·Jᵀ·e'],
          ['pinv', 'Pseudoinverse  q += J#·e'],
          ['dls', 'Damped Least Squares  q += Jᵀ(JJᵀ+λ²I)⁻¹·e'],
        ],
        st.method,
        (v) => {
          st.method = v;
          save();
        }
      );
      el.appendChild(
        U.card(
          'Numerische Lösung (Kap. 3)',
          U.row(U.field('Verfahren', methodSel)),
          U.row(
            U.field('α', U.num(st.alpha, (v) => ((st.alpha = v), save()), { step: 'any', cls: 'w84' })),
            U.field('λ', U.num(st.lambda, (v) => ((st.lambda = v), save()), { step: 1, cls: 'w44' })),
            U.field('k_max', U.num(st.kmax, (v) => ((st.kmax = Math.max(1, v)), save()), { step: 100, cls: 'w52' })),
            U.field('ε', U.num(st.eps, (v) => ((st.eps = v), save()), { step: 'any', cls: 'w84' }))
          ),
          U.row(
            U.chk('ψ als 4. Zielgröße', st.usePsi, (v) => {
              st.usePsi = v;
              save();
            }),
            U.field(
              'Startwert',
              U.sel(
                [
                  ['cur', 'aktuelle Pose'],
                  ['home', 'aufrecht (90°)'],
                  ['bent', '90/120/120/120'],
                ],
                st.start,
                (v) => {
                  st.start = v;
                  save();
                }
              )
            )
          ),
          U.row(
            U.btn('Berechnen', runNumeric, 'primary'),
            U.btn('Iterationen animieren', () => {
              if (numRes) anim = { t0: performance.now(), res: numRes };
            }),
            U.btn('Anfahren', () => numRes && go(numRes.m))
          ),
          numBox,
          cv,
          U.hint('Projektion auf die Gelenkgrenzen nach jedem Schritt: q = max(q_min, min(q_max, q)). Ohne ψ ist der Arm redundant (3 Gleichungen, 4 Gelenke). Ein zu großes α führt zum Oszillieren, ein zu kleines zu langsamer Konvergenz.')
        )
      );
      jacBox = h('div');
      el.appendChild(U.card('Jacobi-Matrix der aktuellen Soll-Pose', jacBox));
      solve();
    },
    enter() {
      solve();
    },
    leave() {
      BS.view.setIkTarget(null, null, false);
      BS.view.setTargetDrag(false);
      if (dragChk) dragChk.input.checked = false;
    },
    update() {
      renderJac();
      let ghost = hoverPose;
      if (anim) {
        const hist = anim.res.hist;
        const dur = Math.min(3000, 300 + hist.length * 15);
        const f = (performance.now() - anim.t0) / dur;
        if (f >= 1) anim = null;
        else {
          const k = Math.min(hist.length - 1, Math.floor(f * hist.length));
          ghost = full(hist[k].m);
        }
      }
      BS.view.ghostPose = ghost;
    },
  });
})();
