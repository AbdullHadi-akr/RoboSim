/* Braccio-Simulator – 3D-Ansicht (Three.js r147, z-Achse nach oben, Einheiten in mm) */
(function () {
  'use strict';
  const BS = window.BS;
  const C = BS.config;

  const V = (BS.view = {
    show: { frames: false, trace: true, ghost: false, wsPlane: false, wsHull: false, zones: true, markers: false, tcp: true, path: true, labels: true },
    selected: null,
    ghostPose: null,
    ghostReason: '',
  });

  let renderer, scene, camera, controls, tctrl, container;
  let robot, ghost;
  const objMeshes = new Map(),
    targetMeshes = new Map(),
    zoneMeshes = new Map();
  let markerGroup, conveyorGroup, traceLine, pathLine, wsPlane, wsHull, ikTarget, worldGroup;

  // ------------------------------------------------------------------
  // Hilfen
  // ------------------------------------------------------------------
  function std(color, opts) {
    return new THREE.MeshStandardMaterial(Object.assign({ color, roughness: 0.65, metalness: 0.08 }, opts || {}));
  }
  function mesh(geo, mat, parent, pos, rot, shadow) {
    const m = new THREE.Mesh(geo, mat);
    if (pos) m.position.set(pos[0], pos[1], pos[2]);
    if (rot) m.rotation.set(rot[0], rot[1], rot[2]);
    if (shadow !== false) {
      m.castShadow = true;
      m.receiveShadow = true;
    }
    parent.add(m);
    return m;
  }
  function box(w, h, d, mat, parent, pos, rot, shadow) {
    return mesh(new THREE.BoxGeometry(w, h, d), mat, parent, pos, rot, shadow);
  }
  function label(text, color, size) {
    const c = document.createElement('canvas');
    const ctx = c.getContext('2d');
    const fs = 44;
    ctx.font = `600 ${fs}px system-ui, sans-serif`;
    const w = Math.ceil(ctx.measureText(text).width) + 20;
    c.width = w;
    c.height = fs + 16;
    ctx.font = `600 ${fs}px system-ui, sans-serif`;
    ctx.fillStyle = 'rgba(20,24,30,0.72)';
    const r = 12;
    ctx.beginPath();
    ctx.moveTo(r, 0);
    ctx.arcTo(w, 0, w, c.height, r);
    ctx.arcTo(w, c.height, 0, c.height, r);
    ctx.arcTo(0, c.height, 0, 0, r);
    ctx.arcTo(0, 0, w, 0, r);
    ctx.fill();
    ctx.fillStyle = color || '#fff';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, 10, c.height / 2 + 2);
    const tex = new THREE.CanvasTexture(c);
    tex.encoding = THREE.sRGBEncoding;
    const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
    const s = size || 18;
    sp.scale.set((s * w) / c.height, s, 1);
    sp.renderOrder = 10;
    return sp;
  }
  function setMatrixRowMajor(obj, M) {
    obj.matrix.set(M[0], M[1], M[2], M[3], M[4], M[5], M[6], M[7], M[8], M[9], M[10], M[11], M[12], M[13], M[14], M[15]);
    obj.matrixWorldNeedsUpdate = true;
  }
  V.label = label;

  // ------------------------------------------------------------------
  // Roboter-Modell (Geometrie in den DH-Koordinatensystemen)
  // ------------------------------------------------------------------
  function buildRobot(isGhost) {
    const g = C.geom;
    const root = new THREE.Group();
    const ghostMat = new THREE.MeshBasicMaterial({ color: 0x4dabf7, transparent: true, opacity: 0.22, depthWrite: false });
    const M = isGhost
      ? { base: ghostMat, link: ghostMat, servo: ghostMat, horn: ghostMat, pad: ghostMat, up: ghostMat, fore: ghostMat, hand: ghostMat }
      : {
          base: std(0x2c3036),
          servo: std(0x121417, { roughness: 0.5 }),
          horn: std(0xe9ecef, { roughness: 0.4 }),
          pad: std(0x5c636c, { roughness: 0.9 }),
          up: std(0x26292e),
          fore: std(0x26292e),
          hand: std(0x26292e),
          ring: std(0xf08c00, { roughness: 0.5 }),
        };
    const sh = !isGhost;
    const F = [];
    for (let i = 0; i < 6; i++) {
      const grp = new THREE.Group();
      grp.matrixAutoUpdate = false;
      root.add(grp);
      F.push(grp);
      if (!isGhost) {
        const ax = new THREE.AxesHelper(i === 5 ? 45 : 55);
        ax.visible = false;
        ax.renderOrder = 5;
        ax.material.depthTest = false;
        grp.add(ax);
        grp.userData.axes = ax;
        const lb = label(i === 0 ? 'KS0' : i === 5 ? 'KS5 (TCP)' : 'KS' + i, '#fff', 13);
        lb.position.set(14, 14, 14);
        lb.visible = false;
        grp.add(lb);
        grp.userData.label = lb;
      }
    }
    const cylZ = (r1, r2, h, mat, parent, pos, seg) => mesh(new THREE.CylinderGeometry(r1, r2, h, seg || 40), mat, parent, pos, [Math.PI / 2, 0, 0], sh);
    const cylY = (r1, r2, h, mat, parent, pos, seg) => mesh(new THREE.CylinderGeometry(r1, r2, h, seg || 40), mat, parent, pos, null, sh);
    // Frame 0: feste Basis
    cylZ(76, 80, 26, M.base, F[0], [0, 0, 13], 56);
    if (!isGhost) cylZ(77, 77, 3, M.ring, F[0], [0, 0, 27.5], 56);
    // Frame 1: Drehteller + Schulterlager (y1 = oben, z1 = Schulterachse)
    const yb = 29 - g.d1;
    cylY(60, 62, 14, M.base, F[1], [0, yb + 7, 0], 48);
    box(56, 48, 5, M.base, F[1], [0, yb + 14 + 24 - 10, 31], null, sh);
    box(56, 48, 5, M.base, F[1], [0, yb + 14 + 24 - 10, -31], null, sh);
    box(22, 44, 40, M.servo, F[1], [0, -14, 0], null, sh);
    cylZ(11, 11, 3, M.horn, F[1], [0, 0, 35]);
    cylZ(11, 11, 3, M.horn, F[1], [0, 0, -35]);
    // Frame 2: Oberarm (x2 zeigt von der Schulter zum Ellbogen, Ursprung im Ellbogen)
    box(g.a2 + 28, 26, 5, M.up, F[2], [-g.a2 / 2, 0, 27], null, sh);
    box(g.a2 + 28, 26, 5, M.up, F[2], [-g.a2 / 2, 0, -27], null, sh);
    box(10, 18, 50, M.up, F[2], [-g.a2 * 0.55, 0, 0], null, sh);
    box(42, 22, 40, M.servo, F[2], [-10, 0, 0], null, sh);
    cylZ(10, 10, 3, M.horn, F[2], [0, 0, 31]);
    cylZ(10, 10, 3, M.horn, F[2], [0, 0, -31]);
    // Frame 3: Unterarm
    box(g.a3 + 24, 22, 4, M.fore, F[3], [-g.a3 / 2, 0, 22], null, sh);
    box(g.a3 + 24, 22, 4, M.fore, F[3], [-g.a3 / 2, 0, -22], null, sh);
    box(10, 16, 40, M.fore, F[3], [-g.a3 * 0.55, 0, 0], null, sh);
    box(32, 17, 34, M.servo, F[3], [-8, 0, 0], null, sh);
    cylZ(8, 8, 3, M.horn, F[3], [0, 0, 25]);
    cylZ(8, 8, 3, M.horn, F[3], [0, 0, -25]);
    // Frame 4: Handgelenk (z4 = Werkzeugachse, y4 = Handgelenkachse)
    box(24, 4, 62, M.hand, F[4], [0, 19, 22], null, sh);
    box(24, 4, 62, M.hand, F[4], [0, -19, 22], null, sh);
    box(22, 30, 34, M.servo, F[4], [0, 0, 64], null, sh);
    // Frame 5: Greifer (rotiert mit M5), Ursprung = TCP
    const zb = -g.lTcp + 88;
    box(78, 36, 10, M.hand, F[5], [0, 0, zb], null, sh);
    box(30, 18, 30, M.servo, F[5], [22, 24, zb - 18], null, sh);
    const tipZ = g.lTip - g.lTcp;
    const fz0 = zb + 5,
      fl = tipZ - fz0;
    const fingers = [];
    for (const s of [-1, 1]) {
      const fg = new THREE.Group();
      F[5].add(fg);
      box(6, 22, fl, M.hand, fg, [0, 0, fz0 + fl / 2], null, sh);
      box(2.5, 18, 34, M.pad, fg, [-s * 4.2, 0, tipZ - 19], null, sh);
      fingers.push({ grp: fg, s });
    }
    let tcpMarker = null;
    if (!isGhost) {
      tcpMarker = mesh(new THREE.SphereGeometry(4.5, 18, 12), new THREE.MeshBasicMaterial({ color: 0xff7a00, depthTest: false }), F[5], [0, 0, 0], null, false);
      tcpMarker.renderOrder = 6;
    }
    return {
      root,
      F,
      fingers,
      tcpMarker,
      mats: M,
      setPose(Fr, jaw) {
        for (let i = 0; i < 6; i++) setMatrixRowMajor(F[i], Fr[i]);
        for (const f of fingers) f.grp.position.x = f.s * (jaw / 2 + 3);
      },
      highlight(state) {
        if (isGhost) return;
        const col = (s) => (s === 'viol' ? 0x9c0000 : s === 'warn' ? 0x7a4a00 : 0x000000);
        M.up.emissive.setHex(col(state['Oberarm']));
        M.fore.emissive.setHex(col(state['Unterarm']));
        M.hand.emissive.setHex(col(state['Greifer'] || state['Greiferfinger']));
      },
    };
  }

  V.rebuildRobot = function () {
    if (robot) worldGroup.remove(robot.root);
    if (ghost) worldGroup.remove(ghost.root);
    robot = buildRobot(false);
    ghost = buildRobot(true);
    ghost.root.visible = false;
    worldGroup.add(robot.root);
    worldGroup.add(ghost.root);
    applyFrameVisibility();
  };
  function applyFrameVisibility() {
    for (const f of robot.F) {
      f.userData.axes.visible = V.show.frames;
      f.userData.label.visible = V.show.frames;
    }
  }

  // ------------------------------------------------------------------
  // Szene
  // ------------------------------------------------------------------
  V.init = function (el) {
    container = el;
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.outputEncoding = THREE.sRGBEncoding;
    el.appendChild(renderer.domElement);
    scene = new THREE.Scene();
    worldGroup = new THREE.Group();
    scene.add(worldGroup);
    camera = new THREE.PerspectiveCamera(40, 1, 5, 20000);
    camera.up.set(0, 0, 1);
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.14;
    V.setView('iso');

    scene.add(new THREE.HemisphereLight(0xffffff, 0x6b7380, 0.75));
    const dl = new THREE.DirectionalLight(0xffffff, 0.85);
    dl.position.set(350, -250, 900);
    dl.castShadow = true;
    dl.shadow.mapSize.set(2048, 2048);
    const sc = dl.shadow.camera;
    sc.left = -650;
    sc.right = 650;
    sc.top = 650;
    sc.bottom = -650;
    sc.near = 100;
    sc.far = 2000;
    dl.shadow.bias = -0.0004;
    dl.target.position.set(0, 150, 0);
    scene.add(dl, dl.target);
    const fill = new THREE.DirectionalLight(0xdbe4ff, 0.3);
    fill.position.set(-600, 500, 400);
    scene.add(fill);

    // Tisch
    const table = mesh(new THREE.BoxGeometry(1400, 1000, 20), std(0xe7e3da, { roughness: 0.9 }), worldGroup, [0, 150, -10.05], null, false);
    table.receiveShadow = true;
    table.name = 'table';
    V.table = table;
    const grid = new THREE.GridHelper(1300, 26, 0x9aa1aa, 0xc5c9cf);
    grid.rotation.x = Math.PI / 2;
    grid.position.set(0, 150, 0.15);
    grid.material.transparent = true;
    grid.material.opacity = 0.7;
    worldGroup.add(grid);
    // Weltachsen
    const axes = new THREE.Group();
    const arr = (dir, col, txt) => {
      const a = new THREE.ArrowHelper(new THREE.Vector3(...dir), new THREE.Vector3(0, 0, 0.5), 140, col, 18, 9);
      axes.add(a);
      const l = label(txt, '#' + col.toString(16).padStart(6, '0'), 16);
      l.position.set(dir[0] * 160, dir[1] * 160, dir[2] * 160 + 4);
      axes.add(l);
    };
    arr([1, 0, 0], 0xe03131, 'x');
    arr([0, 1, 0], 0x2f9e44, 'y');
    arr([0, 0, 1], 0x1c7ed6, 'z');
    axes.position.set(-560, -250, 0);
    worldGroup.add(axes);
    // Maßstab-Beschriftung am Raster
    for (const r of [100, 200, 300, 400]) {
      const l = label(r + ' mm', '#dfe3e8', 11);
      l.position.set(r, -8, 2);
      l.material.opacity = 0.75;
      worldGroup.add(l);
    }

    V.rebuildRobot();

    // Spur
    const tg = new THREE.BufferGeometry();
    tg.setAttribute('position', new THREE.BufferAttribute(new Float32Array(4000 * 3), 3));
    tg.setDrawRange(0, 0);
    traceLine = new THREE.Line(tg, new THREE.LineBasicMaterial({ color: 0xff7a00, transparent: true, opacity: 0.85 }));
    traceLine.frustumCulled = false;
    worldGroup.add(traceLine);
    // Trajektorienvorschau
    const pg = new THREE.BufferGeometry();
    pg.setAttribute('position', new THREE.BufferAttribute(new Float32Array(20000 * 3), 3));
    pg.setDrawRange(0, 0);
    pathLine = new THREE.Line(pg, new THREE.LineDashedMaterial({ color: 0x15aabf, dashSize: 6, gapSize: 4 }));
    pathLine.frustumCulled = false;
    worldGroup.add(pathLine);

    // IK-Ziel
    ikTarget = new THREE.Group();
    const sph = mesh(new THREE.SphereGeometry(7, 20, 14), new THREE.MeshStandardMaterial({ color: 0xd6336c, emissive: 0x5a0f2c }), ikTarget, [0, 0, 0], null, false);
    sph.name = 'ikTargetSphere';
    ikTarget.userData.arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(0, 0, 0), 60, 0xd6336c, 14, 8);
    ikTarget.add(ikTarget.userData.arrow);
    ikTarget.visible = false;
    worldGroup.add(ikTarget);
    tctrl = new THREE.TransformControls(camera, renderer.domElement);
    tctrl.setSize(0.8);
    tctrl.addEventListener('dragging-changed', (e) => (controls.enabled = !e.value));
    tctrl.addEventListener('objectChange', () => {
      const p = ikTarget.position;
      BS.emit('ikTargetDrag', { x: p.x, y: p.y, z: p.z });
    });
    scene.add(tctrl);

    markerGroup = new THREE.Group();
    worldGroup.add(markerGroup);
    V.rebuildMarkers();

    // Auswahl per Klick
    let down = null;
    renderer.domElement.addEventListener('pointerdown', (e) => (down = [e.clientX, e.clientY]));
    renderer.domElement.addEventListener('pointerup', (e) => {
      if (!down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 4 || tctrl.dragging) return;
      const rect = renderer.domElement.getBoundingClientRect();
      const ndc = new THREE.Vector2(((e.clientX - rect.left) / rect.width) * 2 - 1, -((e.clientY - rect.top) / rect.height) * 2 + 1);
      const rc = new THREE.Raycaster();
      rc.setFromCamera(ndc, camera);
      const objs = [...objMeshes.values()];
      const hit = rc.intersectObjects(objs, false)[0];
      if (hit) {
        BS.emit('pickObject', hit.object.userData.obj);
        return;
      }
      const th = rc.intersectObject(table, false)[0];
      if (th) BS.emit('pickTable', { x: th.point.x, y: th.point.y, z: 0, shift: e.shiftKey });
    });

    new ResizeObserver(() => V.resize()).observe(el);
    V.resize();
    V.setView('iso');
  };

  V.resize = function () {
    if (!renderer) return;
    const w = container.clientWidth,
      h = container.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h, false);
    renderer.domElement.style.width = w + 'px';
    renderer.domElement.style.height = h + 'px';
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  };

  V.setView = function (name) {
    const views = {
      iso: [
        [760, -560, 600],
        [0, 140, 130],
      ],
      vorne: [
        [0, 1250, 260],
        [0, 100, 180],
      ],
      seite: [
        [1250, 120, 260],
        [0, 120, 180],
      ],
      oben: [
        [0, 149, 1500],
        [0, 150, 0],
      ],
      hinten: [
        [0, -900, 450],
        [0, 120, 150],
      ],
    };
    const v = views[name] || views.iso;
    // bei schmalen Fenstern weiter herauszoomen, damit Roboter und Tisch sichtbar bleiben
    const f = Math.max(1, 1.6 / Math.max(0.3, camera.aspect || 1.6));
    camera.position.set(v[1][0] + (v[0][0] - v[1][0]) * f, v[1][1] + (v[0][1] - v[1][1]) * f, v[1][2] + (v[0][2] - v[1][2]) * f);
    controls.target.set(...v[1]);
    controls.update();
  };

  // ------------------------------------------------------------------
  // Synchronisation mit der Simulation
  // ------------------------------------------------------------------
  V.sync = function () {
    const sim = BS.sim;
    if (!sim.F) sim.updateKinematics();
    robot.setPose(sim.F, sim.jawWidth());
    robot.highlight(BS.monitor.linkState || {});
    robot.tcpMarker.visible = V.show.tcp;
    // Ghost
    if (V.ghostPose) {
      ghost.root.visible = true;
      const m = V.ghostPose;
      ghost.setPose(BS.kin.frames(m), sim.jawFromAngle(m[5] != null ? m[5] : sim.cmd[5]));
    } else ghost.root.visible = false;
    // Spur
    traceLine.visible = V.show.trace;
    if (sim.traceDirty) {
      sim.traceDirty = false;
      const a = traceLine.geometry.attributes.position;
      const n = Math.min(sim.trace.length, 4000);
      for (let i = 0; i < n; i++) a.setXYZ(i, sim.trace[i][0], sim.trace[i][1], sim.trace[i][2]);
      a.needsUpdate = true;
      traceLine.geometry.setDrawRange(0, n);
      traceLine.geometry.computeBoundingSphere();
    }
    pathLine.visible = V.show.path;
    syncObjects();
    syncTargets();
    syncZones();
    syncConveyor();
    markerGroup.visible = V.show.markers;
    if (wsPlane) {
      wsPlane.visible = V.show.wsPlane;
      const a = sim.act[0] * BS.D2R + (sim.errOn ? sim.errOff[0] * BS.D2R : 0);
      wsPlane.rotation.set(0, 0, a);
    }
    if (wsHull) wsHull.visible = V.show.wsHull;
  };

  function syncObjects() {
    const sim = BS.sim;
    const seen = new Set();
    for (const o of sim.objects) {
      seen.add(o.id);
      let m = objMeshes.get(o.id);
      const key = o.size.join('x') + o.color;
      if (m && m.userData.key !== key) {
        worldGroup.remove(m);
        objMeshes.delete(o.id);
        m = null;
      }
      if (!m) {
        m = new THREE.Mesh(new THREE.BoxGeometry(o.size[0], o.size[1], o.size[2]), std(o.color, { roughness: 0.55 }));
        m.castShadow = m.receiveShadow = true;
        m.matrixAutoUpdate = false;
        m.userData = { obj: o, key };
        const edges = new THREE.LineSegments(new THREE.EdgesGeometry(m.geometry), new THREE.LineBasicMaterial({ color: 0xffffff }));
        edges.visible = false;
        m.add(edges);
        m.userData.edges = edges;
        worldGroup.add(m);
        objMeshes.set(o.id, m);
      }
      m.userData.obj = o;
      setMatrixRowMajor(m, o.M);
      m.userData.edges.visible = V.selected === o;
    }
    for (const [id, m] of objMeshes)
      if (!seen.has(id)) {
        worldGroup.remove(m);
        objMeshes.delete(id);
      }
  }

  function syncTargets() {
    const sim = BS.sim;
    const seen = new Set();
    for (const t of sim.targets) {
      seen.add(t.id);
      let g = targetMeshes.get(t.id);
      const key = [t.x, t.y, t.w, t.h, t.color, t.name].join();
      if (g && g.userData.key !== key) {
        worldGroup.remove(g);
        g = null;
      }
      if (!g) {
        g = new THREE.Group();
        g.userData.key = key;
        const pl = new THREE.Mesh(new THREE.PlaneGeometry(t.w, t.h), new THREE.MeshBasicMaterial({ color: t.color, transparent: true, opacity: 0.22, depthWrite: false }));
        pl.position.set(t.x, t.y, 0.4);
        g.add(pl);
        const pts = [
          [-1, -1],
          [1, -1],
          [1, 1],
          [-1, 1],
          [-1, -1],
        ].map(([a, b]) => new THREE.Vector3(t.x + (a * t.w) / 2, t.y + (b * t.h) / 2, 0.6));
        g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: t.color })));
        const lb = label(t.name, '#fff', 13);
        lb.position.set(t.x, t.y - t.h / 2 - 12, 4);
        g.add(lb);
        worldGroup.add(g);
        targetMeshes.set(t.id, g);
      }
    }
    for (const [id, g] of targetMeshes)
      if (!seen.has(id)) {
        worldGroup.remove(g);
        targetMeshes.delete(id);
      }
  }

  function syncZones() {
    const sim = BS.sim;
    const seen = new Set();
    for (const z of sim.zones) {
      seen.add(z.id);
      let g = zoneMeshes.get(z.id);
      const key = [...z.c, ...z.s, z.type, z.name].join();
      if (g && g.userData.key !== key) {
        worldGroup.remove(g);
        g = null;
      }
      if (!g) {
        g = new THREE.Group();
        g.userData.key = key;
        const geo = new THREE.BoxGeometry(z.s[0], z.s[1], z.s[2]);
        const mat = new THREE.MeshBasicMaterial({ color: z.type === 'allowed' ? 0x2f9e44 : 0xe03131, transparent: true, opacity: z.type === 'allowed' ? 0.04 : 0.14, depthWrite: false });
        const m = new THREE.Mesh(geo, mat);
        const e = new THREE.LineSegments(new THREE.EdgesGeometry(geo), new THREE.LineBasicMaterial({ color: z.type === 'allowed' ? 0x2f9e44 : 0xe03131 }));
        g.add(m, e);
        g.position.set(...z.c);
        const lb = label(z.name, z.type === 'allowed' ? '#8ce99a' : '#ffa8a8', 13);
        lb.position.set(0, 0, z.s[2] / 2 + 12);
        g.add(lb);
        g.userData.mat = mat;
        g.userData.edge = e.material;
        worldGroup.add(g);
        zoneMeshes.set(z.id, g);
      }
      g.visible = V.show.zones;
      const st = BS.monitor.zoneState[z.id];
      const base = z.type === 'allowed' ? 0x2f9e44 : 0xe03131;
      const col = st === 'viol' ? 0xff0000 : st === 'warn' ? 0xf59f00 : base;
      g.userData.mat.color.setHex(col);
      g.userData.mat.opacity = st ? 0.3 : z.type === 'allowed' ? 0.04 : 0.14;
      g.userData.edge.color.setHex(col);
    }
    for (const [id, g] of zoneMeshes)
      if (!seen.has(id)) {
        worldGroup.remove(g);
        zoneMeshes.delete(id);
      }
  }

  let convKey = null;
  function syncConveyor() {
    const cv = BS.sim.conveyor;
    const key = cv ? [cv.x, cv.yStart, cv.yEnd, cv.width, cv.height].join() : null;
    if (key !== convKey) {
      if (conveyorGroup) worldGroup.remove(conveyorGroup);
      conveyorGroup = null;
      convKey = key;
      if (cv) {
        const g = new THREE.Group();
        const len = cv.yStart - cv.yEnd + 30;
        const cy = (cv.yStart + cv.yEnd) / 2 + 15;
        box(cv.width + 16, len, cv.height - 4, std(0x868e96), g, [cv.x, cy, (cv.height - 4) / 2]);
        box(cv.width, len, 4, std(0x343a40, { roughness: 0.95 }), g, [cv.x, cy, cv.height - 2]);
        box(cv.width + 20, 8, cv.height + 22, std(0xfab005), g, [cv.x, cv.yEnd - 4, (cv.height + 22) / 2]);
        const postMat = std(0x495057);
        const beams = [];
        for (const side of [-1, 1]) {
          const px = cv.x + side * (cv.width / 2 + 16);
          mesh(new THREE.CylinderGeometry(4, 4, cv.lsHigh + 14, 16), postMat, g, [px, cv.lsY, (cv.lsHigh + 14) / 2], [Math.PI / 2, 0, 0]);
        }
        for (const z of [cv.lsLow, cv.lsHigh]) {
          const pts = [new THREE.Vector3(cv.x - cv.width / 2 - 16, cv.lsY, z), new THREE.Vector3(cv.x + cv.width / 2 + 16, cv.lsY, z)];
          const ln = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: 0x40c057 }));
          g.add(ln);
          beams.push(ln);
        }
        const lb = label('Förderband', '#fff', 13);
        lb.position.set(cv.x, cv.yStart + 30, cv.height + 10);
        g.add(lb);
        const l2 = label('D2 / D4', '#fff', 11);
        l2.position.set(cv.x + cv.width / 2 + 40, cv.lsY, cv.lsHigh + 18);
        g.add(l2);
        g.userData.beams = beams;
        worldGroup.add(g);
        conveyorGroup = g;
      }
    }
    if (conveyorGroup && cv) {
      conveyorGroup.userData.beams[0].material.color.setHex(cv.lsLowState ? 0xff2020 : 0x40c057);
      conveyorGroup.userData.beams[1].material.color.setHex(cv.lsHighState ? 0xff2020 : 0x40c057);
    }
  }

  V.rebuildMarkers = function () {
    markerGroup.clear();
    const pinMat = std(0x868e96),
      ballMat = new THREE.MeshStandardMaterial({ color: 0xfab005, emissive: 0x4a3200 });
    for (const mk of BS.sim.markers) {
      const [x, y, z] = mk.p;
      mesh(new THREE.CylinderGeometry(2, 2, z - 5, 10), pinMat, markerGroup, [x, y, (z - 5) / 2], [Math.PI / 2, 0, 0]);
      mesh(new THREE.SphereGeometry(5, 18, 12), ballMat, markerGroup, [x, y, z]);
      const lb = label(mk.name, '#ffe066', 13);
      lb.position.set(x, y, z + 16);
      markerGroup.add(lb);
    }
  };

  // ------------------------------------------------------------------
  // IK-Ziel, Pfadvorschau, Arbeitsraum
  // ------------------------------------------------------------------
  V.setIkTarget = function (t, psi, visible) {
    ikTarget.visible = !!visible;
    if (!t) return;
    ikTarget.position.set(t.x, t.y, t.z);
    if (psi != null) {
      const az = Math.atan2(t.y, t.x);
      const p = psi * BS.D2R;
      const dir = new THREE.Vector3(Math.cos(p) * Math.cos(az), Math.cos(p) * Math.sin(az), Math.sin(p));
      ikTarget.userData.arrow.setDirection(dir.normalize());
      ikTarget.userData.arrow.visible = true;
    } else ikTarget.userData.arrow.visible = false;
  };
  V.setTargetDrag = function (on) {
    if (on) {
      ikTarget.visible = true;
      tctrl.attach(ikTarget);
    } else tctrl.detach();
  };

  V.setPath = function (xyz) {
    const a = pathLine.geometry.attributes.position;
    const n = xyz ? Math.min(xyz.length, 20000) : 0;
    for (let i = 0; i < n; i++) a.setXYZ(i, xyz[i][0], xyz[i][1], xyz[i][2]);
    a.needsUpdate = true;
    pathLine.geometry.setDrawRange(0, n);
    pathLine.computeLineDistances();
    pathLine.geometry.computeBoundingSphere();
  };

  V.clearTrace = function () {
    BS.sim.trace = [];
    BS.sim.traceDirty = true;
  };

  /** Arbeitsraum-Schnitt als Textur in der Armebene + Rotationshülle */
  V.setWorkspace = function (ws) {
    if (wsPlane) worldGroup.remove(wsPlane);
    if (wsHull) worldGroup.remove(wsHull);
    wsPlane = wsHull = null;
    if (!ws) return;
    const c = document.createElement('canvas');
    c.width = ws.nr;
    c.height = ws.nz;
    const ctx = c.getContext('2d');
    const img = ctx.createImageData(ws.nr, ws.nz);
    for (let iz = 0; iz < ws.nz; iz++)
      for (let ir = 0; ir < ws.nr; ir++) {
        const v = ws.grid[iz * ws.nr + ir];
        const o = ((ws.nz - 1 - iz) * ws.nr + ir) * 4;
        if (v === 1) {
          img.data[o] = 64;
          img.data[o + 1] = 192;
          img.data[o + 2] = 87;
          img.data[o + 3] = 150;
        } else if (v === 2) {
          img.data[o] = 253;
          img.data[o + 1] = 126;
          img.data[o + 2] = 20;
          img.data[o + 3] = 120;
        }
      }
    ctx.putImageData(img, 0, 0);
    const tex = new THREE.CanvasTexture(c);
    tex.magFilter = THREE.NearestFilter;
    const W = ws.r1 - ws.r0,
      H = ws.z1 - ws.z0;
    const pl = new THREE.Mesh(new THREE.PlaneGeometry(W, H), new THREE.MeshBasicMaterial({ map: tex, transparent: true, side: THREE.DoubleSide, depthWrite: false }));
    // Ebene: lokales x = radial, lokales y = z (Welt)
    pl.rotation.x = Math.PI / 2;
    pl.position.set((ws.r0 + ws.r1) / 2, 0, (ws.z0 + ws.z1) / 2);
    wsPlane = new THREE.Group();
    wsPlane.add(pl);
    worldGroup.add(wsPlane);
    // Hülle: äußere Reichweite je Höhe (vorne)
    const prof = [];
    for (let iz = 0; iz < ws.nz; iz++) {
      let rmax = -1;
      for (let ir = ws.nr - 1; ir >= 0; ir--) {
        const r = ws.r0 + (ir + 0.5) * ws.cell;
        if (r < 0) break;
        if (ws.grid[iz * ws.nr + ir] === 1) {
          rmax = r;
          break;
        }
      }
      if (rmax > 0) prof.push(new THREE.Vector2(rmax, ws.z0 + (iz + 0.5) * ws.cell));
    }
    if (prof.length > 2) {
      const pts = [new THREE.Vector2(0, prof[0].y), ...prof, new THREE.Vector2(0, prof[prof.length - 1].y)];
      const geo = new THREE.LatheGeometry(pts, 72);
      const m = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: 0x40c057, transparent: true, opacity: 0.08, side: THREE.DoubleSide, depthWrite: false }));
      m.rotation.x = Math.PI / 2;
      const wire = new THREE.LineSegments(new THREE.WireframeGeometry(new THREE.LatheGeometry(pts.filter((_, i) => i % 6 === 0), 36)), new THREE.LineBasicMaterial({ color: 0x2f9e44, transparent: true, opacity: 0.25 }));
      wire.rotation.x = Math.PI / 2;
      wsHull = new THREE.Group();
      wsHull.add(m, wire);
      worldGroup.add(wsHull);
    }
  };

  V.applyShow = function () {
    applyFrameVisibility();
  };

  V.render = function () {
    controls.update();
    renderer.render(scene, camera);
  };
})();
