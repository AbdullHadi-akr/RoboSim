/* Tab „Modell“: Geometrie, DH-Parameter, Transformationsmatrizen, Servos, Statik, MATLAB-Export */
(function () {
  'use strict';
  const BS = window.BS,
    U = BS.ui,
    h = BS.h,
    C = BS.config;

  let dhEl, tEl, tSel, statEl;
  const G = 9.81;

  function staticTorques() {
    const sim = BS.sim;
    const p = BS.kin.planar(sim.trueAngles(), C.geom);
    const ms = C.masses;
    const payload = sim.held ? sim.held.obj.mass / 1000 : 0;
    // Schwerpunkte in der Armebene (r)
    const upper = (p.shoulder.r + p.elbow.r) / 2;
    const fore = (p.elbow.r + p.wrist.r) / 2;
    const hand = p.wrist.r + (p.tcp.r - p.wrist.r) * 0.4;
    const bodies = [
      { m: ms.upper, r: upper, from: 1 },
      { m: ms.fore, r: fore, from: 2 },
      { m: ms.hand, r: hand, from: 3 },
      { m: payload, r: p.tcp.r, from: 3 },
    ];
    const axis = [null, p.shoulder.r, p.elbow.r, p.wrist.r];
    const tau = [0, 0, 0, 0];
    for (let j = 1; j <= 3; j++) for (const b of bodies) if (b.from >= j) tau[j] += (b.m * G * (b.r - axis[j])) / 1000; // N·m
    return { tau, payload };
  }

  function matlabExport() {
    const g = C.geom;
    return `%% Braccio-Modell – exportiert aus dem Braccio-Simulator
% Konvention (Servowinkel M1..M5 in Grad):
%   DH:  i | theta_i      | d_i  | a_i | alpha_i
%        1 | M1           | d1   | 0   | 90°
%        2 | 180° - M2    | 0    | a2  | 0
%        3 |  90° - M3    | 0    | a3  | 0
%        4 | 180° - M4    | 0    | 0   | 90°
%        5 | M5           | lTcp | 0   | 0
clear; clc;
P.d1 = ${g.d1}; P.a2 = ${g.a2}; P.a3 = ${g.a3}; P.lTcp = ${g.lTcp};   % [mm]

m = [90 120 140 160 90];            % Servowinkel M1..M5 [deg]
T = braccio_fk(m, P);
fprintf('TCP = [%.1f %.1f %.1f] mm\\n', T(1:3,4));

q = braccio_ik([0 260 40], -70, P);  % Ziel [mm], Werkzeugwinkel psi [deg]
disp(q)
J = braccio_jacobian(m, P)

%% ---------------- Funktionen ----------------
function T = dh(theta, d, a, alpha)
  T = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
       sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
       0           sin(alpha)             cos(alpha)            d;
       0           0                      0                     1];
end

function [T, Ts] = braccio_fk(m, P)
  th = deg2rad([m(1), 180 - m(2), 90 - m(3), 180 - m(4), m(5)]);
  DH = [th(1) P.d1 0    pi/2;
        th(2) 0    P.a2 0;
        th(3) 0    P.a3 0;
        th(4) 0    0    pi/2;
        th(5) P.lTcp 0  0];
  T = eye(4); Ts = cell(1, 5);
  for i = 1:5
    T = T * dh(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
    Ts{i} = T;
  end
end

function m = braccio_ik(p, psi, P)
  % Geometrische IK (vorne, Ellbogen oben). p = [x y z] in mm, psi in Grad.
  m1 = atan2d(p(2), p(1));
  r  = hypot(p(1), p(2));
  rw = r - P.lTcp * cosd(psi);
  zw = p(3) - P.d1 - P.lTcp * sind(psi);
  D  = (rw^2 + zw^2 - P.a2^2 - P.a3^2) / (2 * P.a2 * P.a3);
  if abs(D) > 1, error('Ziel außerhalb des Arbeitsraums'); end
  t3 = -acos(D);
  e2 = atan2(zw, rw) - atan2(P.a3 * sin(t3), P.a2 + P.a3 * cos(t3));
  e3 = e2 + t3;
  m2 = 180 - rad2deg(e2);
  m3 = 270 - m2 - rad2deg(e3);
  m4 = 360 - m2 - m3 - psi;
  m  = [m1 m2 m3 m4];
end

function J = braccio_jacobian(m, P)
  % d[x y z psi]/d[M1..M4] (Gelenkwinkel in rad)
  e2 = deg2rad(180 - m(2)); e3 = deg2rad(270 - m(2) - m(3)); e4 = deg2rad(360 - m(2) - m(3) - m(4));
  r  = P.a2*cos(e2) + P.a3*cos(e3) + P.lTcp*cos(e4);
  dr = [0, P.a2*sin(e2) + P.a3*sin(e3) + P.lTcp*sin(e4), P.a3*sin(e3) + P.lTcp*sin(e4), P.lTcp*sin(e4)];
  dz = [0, -(P.a2*cos(e2) + P.a3*cos(e3) + P.lTcp*cos(e4)), -(P.a3*cos(e3) + P.lTcp*cos(e4)), -P.lTcp*cos(e4)];
  c = cosd(m(1)); s = sind(m(1));
  J = [-r*s, c*dr(2:4);
        r*c, s*dr(2:4);
        dz;
        0 -1 -1 -1];
end
`;
  }

  U.register('model', {
    title: 'Modell',
    build(el) {
      const g = C.geom;
      const inputs = {};
      const geomRow = U.row();
      for (const [k, l] of [
        ['d1', 'd1'],
        ['a2', 'a2'],
        ['a3', 'a3'],
        ['d5', 'd5'],
        ['lTcp', 'L_TCP'],
        ['lTip', 'L_Spitze'],
      ]) {
        inputs[k] = U.num(g[k], () => {}, { cls: 'w52', step: 0.5 });
        geomRow.appendChild(U.field(l, inputs[k]));
      }
      el.appendChild(
        U.card(
          'Geometrie [mm]',
          geomRow,
          U.row(
            U.chk('STL-Geometrie anzeigen (sonst vereinfachtes Modell)', BS.view.useStl, (v) => {
              BS.view.useStl = v;
              BS.store.set('useStl', v);
              BS.view.rebuildRobot();
            })
          ),
          U.row(
            U.btn('Übernehmen', () => {
              for (const k in inputs) {
                const v = parseFloat(inputs[k].value);
                if (Number.isFinite(v) && v > 0) g[k] = v;
              }
              BS.store.set('geom', g);
              BS.view.rebuildRobot();
              BS.toast('Geometrie übernommen', 'ok');
            }, 'primary'),
            U.btn('Standardwerte', () => {
              Object.assign(g, { d1: 71.5, a2: 125, a3: 125, d5: 130, lTcp: 180, lTip: 186 });
              for (const k in inputs) inputs[k].value = g[k];
              BS.store.set('geom', g);
              BS.view.rebuildRobot();
            })
          ),
          U.hint('d1: Tisch → Schulterachse · a2: Schulter → Ellbogen · a3: Ellbogen → Handgelenk · d5: Handgelenk → Greiferflansch · L_TCP: Handgelenk → Greifmitte (TCP) · L_Spitze: Handgelenk → Fingerspitze. Die Werte am realen Roboter nachmessen und hier eintragen.')
        )
      );
      el.appendChild(
        U.card(
          'Winkelkonvention',
          h('div', {
            class: 'kv mono',
            html:
              '<b>φ</b><span>= M1 (Azimut, 0° = +x, 90° = +y)</span>' +
              '<b>e2</b><span>= 180° − M2 (Oberarm-Elevation)</span>' +
              '<b>e3</b><span>= 270° − M2 − M3 (Unterarm)</span>' +
              '<b>ψ = e4</b><span>= 360° − M2 − M3 − M4 (Werkzeug)</span>' +
              '<b>r</b><span>= a2·cos e2 + a3·cos e3 + L·cos e4</span>' +
              '<b>z</b><span>= d1 + a2·sin e2 + a3·sin e3 + L·sin e4</span>' +
              '<b>x, y</b><span>= r·cos φ, r·sin φ</span>',
          }),
          U.hint('Alle Servos auf 90° → Arm senkrecht nach oben. M5 = 90° → Greiferbacken öffnen quer zur Armebene. Diese Zuordnung entspricht dem Bibliotheksbeispiel (z. B. M2 = 90°, M3 = 180°, M4 = 180° → Greifer senkrecht nach unten auf den Tisch).')
        )
      );
      dhEl = h('div');
      el.appendChild(U.card('DH-Parameter (aktuelle Soll-Pose)', dhEl, U.hint('Konvention wie im Skript: T = Rot_z(θ)·Trans_z(d)·Trans_x(a)·Rot_x(α). KS0 liegt in der Basismitte auf dem Tisch, KS5 im TCP (Schalter „KS“ oben blendet die Koordinatensysteme ein).')));
      tSel = U.sel(
        [1, 2, 3, 4, 5].map((i) => [String(i), `T₀^${i}`]),
        '5',
        () => {}
      );
      tEl = h('div');
      el.appendChild(U.card(h('span', null, 'Homogene Transformation ', tSel), tEl));

      const servoRow = U.row();
      C.short.forEach((n, i) => servoRow.appendChild(U.field(n, U.num(C.servo.vmax[i], (v) => (C.servo.vmax[i] = Math.max(1, v)), { cls: 'w44', step: 10 }))));
      el.appendChild(
        U.card(
          'Servo-Modell',
          h('div', { class: 'muted', style: { fontSize: '12px' } }, 'max. Geschwindigkeit [°/s]'),
          servoRow,
          U.row(
            U.field('Zeitkonstante τ [s]', U.num(C.servo.tau, (v) => (C.servo.tau = BS.clamp(v, 0.005, 1)), { cls: 'w52', step: 0.01 })),
            U.chk('Sollwerte auf 1° runden (wie Servo.write)', BS.sim.roundCmd, (v) => (BS.sim.roundCmd = v))
          ),
          U.hint('Jeder Servo ist lagegeregelt: PT1-Verhalten mit Zeitkonstante τ und Geschwindigkeitsbegrenzung (Stellgrößenbeschränkung). Braccio: M1–M3 SR431, M4–M6 SR311.')
        )
      );
      const massRow = U.row();
      for (const [k, l] of [
        ['upper', 'Oberarm'],
        ['fore', 'Unterarm'],
        ['hand', 'Hand+Greifer'],
      ])
        massRow.appendChild(U.field(l + ' [g]', U.num(C.masses[k] * 1000, (v) => (C.masses[k] = Math.max(0, v) / 1000), { cls: 'w52', step: 10 })));
      statEl = h('div');
      el.appendChild(
        U.card(
          'Statik – Haltemomente durch Gewichtskraft',
          massRow,
          statEl,
          U.hint('τ_j = g · Σ m_k · (r_k − r_j) über alle Körper hinter Gelenk j (Schwerpunkte vereinfacht). Nutzlast = Masse des gegriffenen Objekts. Herstellerangabe: 150 g Traglast bei 32 cm Reichweite.')
        )
      );
      el.appendChild(
        U.card(
          'Export',
          U.row(U.btn('MATLAB-Skript (FK, IK, Jacobi)', () => BS.download('braccio_modell.m', matlabExport()))),
          U.hint('Erzeugt ein lauffähiges MATLAB-Skript mit DH-Vorwärtskinematik, geometrischer IK und Jacobi-Matrix mit der aktuell eingestellten Geometrie.')
        )
      );
    },
    update() {
      const sim = BS.sim;
      const m = sim.cmd;
      const rows = BS.kin.dhRows(m, C.geom);
      const f = ['M1', '180° − M2', '90° − M3', '180° − M4', 'M5'];
      dhEl.innerHTML =
        '<table class="tbl"><tr><th>i</th><th>θ_i</th><th class="num">θ_i [°]</th><th class="num">d_i</th><th class="num">a_i</th><th class="num">α_i</th></tr>' +
        rows
          .map(
            (r, i) =>
              `<tr><td>${i + 1}</td><td class="mono">${f[i]}</td><td class="num">${(r.theta * BS.R2D).toFixed(1)}</td><td class="num">${r.d}</td><td class="num">${r.a}</td><td class="num">${(r.alpha * BS.R2D).toFixed(0)}°</td></tr>`
          )
          .join('') +
        '</table>';
      const F = BS.kin.frames(m, C.geom);
      const T = F[+tSel.value];
      tEl.innerHTML = '';
      tEl.appendChild(U.matrix([T.slice(0, 4), T.slice(4, 8), T.slice(8, 12), T.slice(12, 16)], 3));
      const st = staticTorques();
      statEl.innerHTML =
        '<table class="tbl"><tr><th>Gelenk</th><th class="num">τ [N·m]</th><th class="num">Grenze</th><th>Auslastung</th></tr>' +
        [1, 2, 3]
          .map((j) => {
            const u = Math.abs(st.tau[j]) / C.stall[j];
            const col = u > 1 ? 'var(--err)' : u > 0.7 ? 'var(--warn)' : 'var(--ok)';
            return `<tr><td>${C.short[j]} (${C.servoType[j]})</td><td class="num">${st.tau[j].toFixed(3)}</td><td class="num">${C.stall[j].toFixed(2)}</td><td><div class="bar"><i style="width:${Math.min(100, u * 100).toFixed(0)}%;background:${col}"></i></div></td></tr>`;
          })
          .join('') +
        `</table><div class="hint">Nutzlast: ${(st.payload * 1000).toFixed(0)} g</div>`;
    },
  });
})();
