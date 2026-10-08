/* Braccio-Simulator – Detailgeometrie der 3D-Darstellung (Three.js r147, Einheiten mm, z nach oben)
 *  - Schrauben: Achsbolzen an den Gelenklagern, Befestigungsschrauben der Basis, Gelenkstifte im Greifer
 *  - Steuerung: Arduino Uno mit Braccio-Shield neben der Basis
 *  - Servokabel (3-adrig: braun/rot/orange) von jedem Servo entlang des Arms zum Shield, folgen der Armbewegung
 * Maße der Lagerstellen sind aus den STL-Meshes (js/meshes.js) abgelesen.
 */
(function () {
  'use strict';
  const BS = window.BS;
  const D = (BS.details = {});

  // Lage der Steuerung auf dem Tisch (hinter der Basis, Stiftleisten zum Roboter hin)
  const BOARD = { x: 0, y: -150 };
  const UNO = { w: 68.6, h: 53.3, t: 1.6, standoff: 3 };
  const SHIELD_Z = UNO.standoff + UNO.t + 11; // Unterkante Shield-Platine
  const HEADER_Y = 19; // Servo-Stiftleisten (Board-Koordinaten)
  const HEADER_X = (k) => -22.5 + 9 * k; // M1..M6
  const CONN_H = 12; // Höhe der aufgesteckten Servostecker

  let MAT = null;
  function mats() {
    if (MAT) return MAT;
    const S = (c, o) => new THREE.MeshStandardMaterial(Object.assign({ color: c, roughness: 0.5, metalness: 0, envMapIntensity: 0.5 }, o || {}));
    MAT = {
      steel: S(0xc9ccd1, { metalness: 1, roughness: 0.28, envMapIntensity: 1 }),
      recess: S(0x101113, { roughness: 0.8 }),
      pcbUno: S(0x00878f, { roughness: 0.55 }),
      black: S(0x18191b, { roughness: 0.6 }),
      gold: S(0xd8ab4c, { metalness: 1, roughness: 0.3, envMapIntensity: 1 }),
      nylon: S(0xf2efe6, { roughness: 0.7 }),
      chip: S(0x222326, { roughness: 0.4 }),
      green: S(0x1f8f3a, { roughness: 0.55 }),
      led: S(0x40ff70, { emissive: 0x30d060, emissiveIntensity: 1.2 }),
      capBody: S(0x1d2b52, { roughness: 0.4 }),
      wire: new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.5, envMapIntensity: 0.5 }),
    };
    return MAT;
  }

  // ------------------------------------------------------------------
  // Schrauben (Achse = lokale +y, Kopfunterseite bei y = 0)
  // ------------------------------------------------------------------
  let GEO = null;
  function geos() {
    if (GEO) return GEO;
    const V2 = (x, y) => new THREE.Vector2(x, y);
    GEO = {
      // Zylinderkopf mit Innensechskant (ISO 4762 M3: d = 5,5, k = 3)
      cap: new THREE.LatheGeometry([V2(0, 0), V2(2.75, 0), V2(2.75, 2.7), V2(2.45, 3), V2(0, 3)], 28),
      hex: new THREE.CylinderGeometry(1.45, 1.45, 0.3, 6),
      // Linsenkopf mit Kreuzschlitz (Achsbolzen)
      pan: new THREE.LatheGeometry([V2(0, 0), V2(3.6, 0), V2(3.6, 0.7), V2(3.3, 1.6), V2(2.4, 2.2), V2(1.2, 2.45), V2(0, 2.5)], 28),
      slot: new THREE.BoxGeometry(3.4, 0.5, 0.7),
      // kleiner Gelenkstift im Greifer
      pin: new THREE.LatheGeometry([V2(0, 0), V2(2.3, 0), V2(2.3, 0.6), V2(1.9, 1.3), V2(0, 1.5)], 20),
    };
    return GEO;
  }
  const Y = new THREE.Vector3(0, 1, 0);
  function screw(kind, scale) {
    const G = geos(),
      M = mats();
    const g = new THREE.Group();
    if (kind === 'cap') {
      g.add(new THREE.Mesh(G.cap, M.steel));
      const h = new THREE.Mesh(G.hex, M.recess);
      h.position.y = 2.9;
      g.add(h);
    } else if (kind === 'pan') {
      g.add(new THREE.Mesh(G.pan, M.steel));
      for (const r of [0, Math.PI / 2]) {
        const s = new THREE.Mesh(G.slot, M.recess);
        s.position.y = 2.3;
        s.rotation.y = r;
        g.add(s);
      }
    } else g.add(new THREE.Mesh(G.pin, M.steel));
    if (scale) g.scale.setScalar(scale);
    return g;
  }
  function place(parent, obj, pos, n) {
    obj.position.set(pos[0], pos[1], pos[2]);
    obj.quaternion.setFromUnitVectors(Y, new THREE.Vector3(n[0], n[1], n[2]).normalize());
    parent.add(obj);
    return obj;
  }

  // ------------------------------------------------------------------
  // Steuerung: Arduino Uno + Braccio-Shield
  // ------------------------------------------------------------------
  function silkscreen() {
    const s = 16; // px/mm
    const c = document.createElement('canvas');
    c.width = Math.round(UNO.w * s);
    c.height = Math.round(UNO.h * s);
    const g = c.getContext('2d');
    g.fillStyle = '#123f6b';
    g.fillRect(0, 0, c.width, c.height);
    const X = (x) => (x + UNO.w / 2) * s,
      Yp = (y) => (UNO.h / 2 - y) * s;
    // Leiterbahnen
    g.strokeStyle = 'rgba(120,170,220,0.35)';
    g.lineWidth = 0.5 * s;
    for (let k = 0; k < 6; k++) {
      g.beginPath();
      g.moveTo(X(HEADER_X(k)), Yp(HEADER_Y - 3));
      g.lineTo(X(HEADER_X(k)), Yp(4 - k));
      g.lineTo(X(20 + k * 1.6), Yp(-6 - k * 1.2));
      g.stroke();
    }
    g.beginPath();
    g.moveTo(X(-28), Yp(-17));
    g.lineTo(X(-28), Yp(10));
    g.lineTo(X(25), Yp(10));
    g.stroke();
    // Bestückungsdruck
    g.fillStyle = '#f4f6f8';
    g.strokeStyle = '#f4f6f8';
    g.lineWidth = 0.25 * s;
    g.textAlign = 'center';
    g.textBaseline = 'middle';
    g.font = `700 ${4.2 * s}px system-ui, sans-serif`;
    g.fillText('BRACCIO SHIELD', X(4), Yp(-2));
    g.font = `500 ${2.2 * s}px system-ui, sans-serif`;
    g.fillText('TinkerKit · Servo-Shield V4', X(4), Yp(-7.5));
    g.font = `700 ${2.6 * s}px system-ui, sans-serif`;
    for (let k = 0; k < 6; k++) {
      g.fillText('M' + (k + 1), X(HEADER_X(k)), Yp(HEADER_Y - 5));
      g.strokeRect(X(HEADER_X(k) - 4.3), Yp(HEADER_Y + 1.8), 8.6 * s, 3.6 * s);
    }
    g.font = `600 ${2 * s}px system-ui, sans-serif`;
    g.fillText('5V', X(-28), Yp(-23));
    g.fillText('ON', X(-14), Yp(-24.5));
    g.fillText('SOFT-START', X(14), Yp(-23.5));
    // Montagebohrungen
    g.fillStyle = '#c9a24a';
    for (const [x, y] of [
      [-31, 23],
      [31, 23],
      [-31, -23],
      [31, -23],
    ]) {
      g.beginPath();
      g.arc(X(x), Yp(y), 1.8 * s, 0, 2 * Math.PI);
      g.fill();
    }
    const tex = new THREE.CanvasTexture(c);
    tex.encoding = THREE.sRGBEncoding;
    tex.anisotropy = 4;
    return tex;
  }

  /** Steuerung auf dem Tisch aufbauen; liefert die Steckerpositionen (Welt) der Servokabel M1..M6 */
  D.buildController = function (parent) {
    const M = mats();
    const grp = new THREE.Group();
    grp.position.set(BOARD.x, BOARD.y, 0);
    parent.add(grp);
    const box = (w, h, d, mat, x, y, z, shadow) => {
      const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
      m.position.set(x, y, z);
      m.castShadow = shadow !== false;
      m.receiveShadow = true;
      grp.add(m);
      return m;
    };
    const cyl = (r, h, mat, x, y, z, seg) => {
      const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, seg || 20), mat);
      m.rotation.x = Math.PI / 2;
      m.position.set(x, y, z);
      m.castShadow = true;
      grp.add(m);
      return m;
    };
    const zU = UNO.standoff,
      zS = SHIELD_Z,
      top = zS + UNO.t;
    // Abstandshalter + Arduino Uno
    for (const [x, y] of [
      [-31, 23],
      [31, 23],
      [-31, -23],
      [31, -23],
    ]) {
      cyl(2.6, zU, M.nylon, x, y, zU / 2);
      cyl(2.2, zS - zU - UNO.t, M.nylon, x, y, zU + UNO.t + (zS - zU - UNO.t) / 2);
      place(grp, screw('cap', 0.8), [x, y, top], [0, 0, 1]);
    }
    box(UNO.w, UNO.h, UNO.t, M.pcbUno, 0, 0, zU + UNO.t / 2);
    const zt = zU + UNO.t;
    box(16, 12, 11, M.steel, -UNO.w / 2 + 6, 12, zt + 5.5); // USB-B
    box(14, 9, 11, M.black, -UNO.w / 2 + 5, -17, zt + 5.5); // Hohlstecker
    box(35, 7.5, 3.5, M.chip, 12, -8, zt + 1.75); // ATmega328P
    box(46, 2.5, zS - zt, M.black, 9, UNO.h / 2 - 2.5, zt + (zS - zt) / 2); // Buchsenleisten
    box(36, 2.5, zS - zt, M.black, 14, -UNO.h / 2 + 2.5, zt + (zS - zt) / 2);
    // Shield
    const sh = new THREE.Mesh(new THREE.BoxGeometry(UNO.w, UNO.h, UNO.t), [
      M.black,
      M.black,
      M.black,
      M.black,
      new THREE.MeshStandardMaterial({ map: silkscreen(), roughness: 0.5 }),
      M.black,
    ]);
    sh.position.set(0, 0, zS + UNO.t / 2);
    sh.castShadow = sh.receiveShadow = true;
    grp.add(sh);
    // Servo-Stiftleisten mit aufgesteckten Steckern
    const conn = [];
    for (let k = 0; k < 6; k++) {
      const x = HEADER_X(k);
      box(7.62, 2.54, 2.5, M.black, x, HEADER_Y, top + 1.25);
      for (const dx of [-2.54, 0, 2.54]) box(0.64, 0.64, 3, M.gold, x + dx, HEADER_Y, top + 4, false);
      box(7.8, 2.9, CONN_H, M.black, x, HEADER_Y, top + 4 + CONN_H / 2);
      conn.push([BOARD.x + x, BOARD.y + HEADER_Y, top + 4 + CONN_H]);
    }
    // Versorgung, Schalter, LED, Elko
    box(10, 7.5, 9, M.green, -28, -17, top + 4.5);
    for (const dx of [-2.5, 2.5]) place(grp, screw('cap', 0.55), [-28 + dx, -17, top + 9], [0, 0, 1]);
    box(8, 4, 4.5, M.black, -14, -20, top + 2.25);
    box(1.6, 1.6, 4, M.steel, -14, -20, top + 6);
    cyl(1.5, 2.2, M.led, -5, -21, top + 1.1, 14);
    cyl(4, 10, M.capBody, 14, -15, top + 5, 24);
    cyl(3.6, 0.4, M.steel, 14, -15, top + 10.2, 24);
    D.controller = { conn, lat: [1, 0, 0] };
    return D.controller;
  };

  // ------------------------------------------------------------------
  // Kabel: dünne Röhren entlang Catmull-Rom-Kurven, Geometrie wird in-place aktualisiert
  // ------------------------------------------------------------------
  const WIRE_COLORS = [0x6b3e1f, 0xc0281f, 0xf08a1c]; // GND, +5 V, Signal
  const WIRE_OFF = [-1.35, 0, 1.35];
  const RING = 6,
    WIRE_R = 0.62;

  function wireMesh(nWires, nPts) {
    const nv = nWires * nPts * RING;
    const pos = new Float32Array(nv * 3),
      nor = new Float32Array(nv * 3),
      col = new Float32Array(nv * 3);
    const idx = [];
    const c = new THREE.Color();
    for (let w = 0; w < nWires; w++) {
      c.setHex(WIRE_COLORS[w % 3]).convertSRGBToLinear();
      const base = w * nPts * RING;
      for (let i = 0; i < nPts * RING; i++) col.set([c.r, c.g, c.b], (base + i) * 3);
      for (let i = 0; i < nPts - 1; i++)
        for (let r = 0; r < RING; r++) {
          const a = base + i * RING + r,
            b = base + i * RING + ((r + 1) % RING);
          idx.push(a, a + RING, b, b, a + RING, b + RING);
        }
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3).setUsage(THREE.DynamicDrawUsage));
    g.setAttribute('normal', new THREE.BufferAttribute(nor, 3).setUsage(THREE.DynamicDrawUsage));
    g.setAttribute('color', new THREE.BufferAttribute(col, 3));
    g.setIndex(nv > 65535 ? new THREE.Uint32BufferAttribute(idx, 1) : new THREE.Uint16BufferAttribute(idx, 1));
    const m = new THREE.Mesh(g, mats().wire);
    m.castShadow = true;
    m.frustumCulled = false;
    return m;
  }

  const _t = new THREE.Vector3(),
    _n = new THREE.Vector3(),
    _b = new THREE.Vector3(),
    _q = new THREE.Vector3();
  /** Röhre für Draht w aus Punktfolge P (parallel transportierte Normalen) */
  function writeTube(geo, w, P) {
    const pos = geo.attributes.position.array,
      nor = geo.attributes.normal.array;
    const n = P.length,
      base = w * n * RING;
    _n.set(0, 0, 0);
    for (let i = 0; i < n; i++) {
      _t.subVectors(P[Math.min(n - 1, i + 1)], P[Math.max(0, i - 1)]).normalize();
      if (i === 0) {
        _n.set(0, 0, 1);
        if (Math.abs(_t.z) > 0.9) _n.set(1, 0, 0);
      }
      _n.addScaledVector(_t, -_t.dot(_n)).normalize();
      _b.crossVectors(_t, _n);
      for (let r = 0; r < RING; r++) {
        const a = (r / RING) * Math.PI * 2,
          ca = Math.cos(a),
          sa = Math.sin(a);
        _q.set(_n.x * ca + _b.x * sa, _n.y * ca + _b.y * sa, _n.z * ca + _b.z * sa);
        const k = (base + i * RING + r) * 3;
        pos[k] = P[i].x + _q.x * WIRE_R;
        pos[k + 1] = P[i].y + _q.y * WIRE_R;
        pos[k + 2] = P[i].z + _q.z * WIRE_R;
        nor[k] = _q.x;
        nor[k + 1] = _q.y;
        nor[k + 2] = _q.z;
      }
    }
  }

  /*
   * Kabelführung: Stützpunkte [Frame, x, y, z, Querachse, Bündelversatz]
   * Frame 0..5 = KS0..KS5, 'fl' = Greiferflansch, 'w' = Welt. Querachse: Richtung, in der die Adern
   * (und die Kabel des Bündels) nebeneinander liegen. Arm: auf der Flanschfläche y = −18,6 des Doppel-T-Profils.
   */
  const P = (f, x, y, z, lat, o) => ({ f, p: [x, y, z], lat, o: !!o });
  const ROUTE = {
    wrist: [P(4, -21, -30.5, 6, 'x', 1)],
    fore: [P(3, 8, -18.6, 0, 'z', 1), P(3, -60, -18.6, 0, 'z', 1), P(3, -106, -18.6, 0, 'z', 1)],
    upper: [P(2, 10, -18.6, 0, 'z', 1), P(2, -60, -18.6, 0, 'z', 1), P(2, -106, -18.6, 0, 'z', 1)],
    turret: [P(1, -30, -30, 0, 'z', 1), P(1, -47, -51.5, 0, 'z', 1), P(1, -60, -60, 0, 'z', 1)],
    base: [P('w', 0, -72, 4, 'x', 1), P('w', 0, -98, 1.2, 'x', 1)],
  };
  function cableAnchors(k) {
    const R = ROUTE;
    const start = [
      [P('w', 0, -56, 6, 'x')], // M1: Basisservo
      [P(1, 18, -42, 0, 'z'), P(1, -10, -48, 0, 'z', 1)], // M2: Schulterservo im Drehturm
      [P(2, -40, -11, 0, 'z'), P(2, -50, -18.6, 0, 'z', 1)], // M3: Ellbogenservo im Oberarm
      [P(3, -40, -11, 0, 'z'), P(3, -50, -18.6, 0, 'z', 1)], // M4: Handgelenkservo im Unterarm
      [P(4, -25, 0, 33, 'x'), P(4, -23, -30.5, 28, 'x', 1)], // M5: Handrotation
      [P('fl', 12, 6, -71, 'x'), P(4, -22, -30.5, 46, 'x', 1)], // M6: Greiferservo
    ][k];
    const via = [
      R.base,
      R.turret.slice(1).concat(R.base),
      R.upper.slice(2).concat(R.turret, R.base),
      R.fore.slice(2).concat(R.upper, R.turret, R.base),
      R.wrist.concat(R.fore, R.upper, R.turret, R.base),
      R.wrist.concat(R.fore, R.upper, R.turret, R.base),
    ][k];
    return start.concat(via);
  }

  /** Details am Roboter: Schrauben + Kabel. ctx = {root, F, flange, carrier, link} */
  D.buildRobotDetails = function (ctx) {
    const { root, F, flange, carrier, link: L } = ctx;
    // Befestigung der Basis (6 Laschen, r = 60 mm, Laschenstärke 3 mm)
    for (let k = 0; k < 6; k++) {
      const a = ((30 + 60 * k) * Math.PI) / 180;
      place(F[0], screw('cap'), [60 * Math.cos(a), 60 * Math.sin(a), 3], [0, 0, 1]);
    }
    // Achsbolzen an den Lagerstellen der Gabeln (Ober-/Unterarm: Schulter- bzw. Ellbogenachse)
    for (const i of [2, 3]) {
      place(F[i], screw('pan'), [-125, 0, 28.5], [0, 0, 1]);
      place(F[i], screw('pan'), [-125, 0, -34.3], [0, 0, -1]);
    }
    place(F[4], screw('pan'), [0, 28.6, 0], [0, 1, 0]);
    place(F[4], screw('pan'), [0, -28.4, 0], [0, -1, 0]);
    // Gelenkstifte im Greifergetriebe (Seitenplatten bei x = ±7,3)
    const pinAt = (parent, yz, origin) => {
      for (const s of [1, -1]) place(parent, screw('pin'), [s * 7.35, yz[0] - origin[0], yz[1] - origin[1]], [s, 0, 0]);
    };
    for (const k of ['gL', 'aL', 'aR']) pinAt(flange, L[k], [0, 0]);
    pinAt(carrier.L, L.cL, L.cL);
    pinAt(carrier.L, L.rL, L.cL);
    pinAt(carrier.R, L.cR, L.cR);
    pinAt(carrier.R, L.rR, L.cR);

    // Servokabel
    const ctrl = D.controller;
    const SEG = 9;
    const cables = [];
    const group = new THREE.Group();
    root.add(group);
    if (ctrl) {
      for (let k = 0; k < 6; k++) {
        const a = cableAnchors(k);
        const c = ctrl.conn[k];
        // zum Stecker auf dem Shield: über den Tisch, in Steckerhöhe heran, dann senkrecht von oben einstecken
        a.push(P('w', c[0], c[1] + 42, 2, 'x'), P('w', c[0], c[1] + 14, c[2] + 5, 'x'), P('w', c[0], c[1] + 2, c[2] + 7, 'x'), P('w', c[0], c[1], c[2], 'x'));
        const nPts = (a.length - 1) * SEG + 1;
        const mesh = wireMesh(3, nPts);
        group.add(mesh);
        cables.push({ a, nPts, mesh, off: (k - 2.5) * 4.4, pts: [0, 1, 2].map(() => a.map(() => new THREE.Vector3())) });
      }
    }
    const last = new Float64Array(16 * 7).fill(NaN);
    const v = new THREE.Vector3();
    const axis = { x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] };
    function update() {
      if (!cables.length || !group.visible) return;
      // nur neu berechnen, wenn sich eine Gelenkstellung geändert hat
      let changed = false;
      const ms = F.map((f) => f.matrix).concat([flange.matrix]);
      ms.forEach((m, j) => {
        for (let i = 0; i < 16; i++)
          if (last[j * 16 + i] !== m.elements[i]) {
            last[j * 16 + i] = m.elements[i];
            changed = true;
          }
      });
      if (!changed) return;
      for (const cb of cables) {
        cb.a.forEach((an, i) => {
          const m = an.f === 'w' ? null : an.f === 'fl' ? flange.matrix : F[an.f].matrix;
          for (let w = 0; w < 3; w++) {
            const o = WIRE_OFF[w] + (an.o ? cb.off : 0);
            const ax = axis[an.lat];
            v.set(an.p[0] + ax[0] * o, an.p[1] + ax[1] * o, an.p[2] + ax[2] * o);
            if (m) v.applyMatrix4(m);
            cb.pts[w][i].copy(v);
          }
        });
        for (let w = 0; w < 3; w++) {
          const curve = new THREE.CatmullRomCurve3(cb.pts[w], false, 'centripetal');
          writeTube(cb.mesh.geometry, w, curve.getPoints(cb.nPts - 1));
        }
        cb.mesh.geometry.attributes.position.needsUpdate = true;
        cb.mesh.geometry.attributes.normal.needsUpdate = true;
      }
    }
    return { update, cables: group };
  };
})();
