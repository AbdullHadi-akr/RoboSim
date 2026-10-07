/* Braccio-Simulator – Start und Hauptschleife */
(function () {
  'use strict';
  const BS = window.BS;

  function fatal(msg) {
    const vp = document.getElementById('viewport');
    vp.appendChild(BS.h('div', { class: 'fatal', html: msg }));
  }

  window.addEventListener('DOMContentLoaded', () => {
    if (!window.THREE || !THREE.OrbitControls) {
      fatal('<div><b>Three.js konnte nicht geladen werden.</b><br>Der Simulator lädt die 3D-Bibliothek aus dem Internet (cdn.jsdelivr.net).<br>Bitte Internetverbindung prüfen und die Seite neu laden.</div>');
      return;
    }
    const sim = BS.sim;
    // gespeicherte Einstellungen
    Object.assign(BS.config.geom, BS.store.get('geom', {}));
    sim.zones = BS.store.get('zones', []);
    for (const z of sim.zones) sim.nextId = Math.max(sim.nextId, (z.id || 0) + 1);
    sim.comp = BS.store.get('comp', [0, 0, 0, 0, 0, 0]);
    BS.monitor.reaction = BS.store.get('monReaction', 'warn');
    BS.monitor.reset();

    BS.view.init(document.getElementById('viewport'));
    sim.loadScenario(BS.store.get('scenario', 'sortieren'));
    sim.updateKinematics();
    BS.ui.init();

    let last = performance.now();
    async function frame(now) {
      const dt = Math.min(0.05, Math.max(0, (now - last) / 1000));
      last = now;
      try {
        await sim.frame(dt);
        BS.view.sync();
        BS.view.render();
        BS.ui.update(now);
      } catch (e) {
        console.error(e);
      }
      requestAnimationFrame(frame);
    }
    requestAnimationFrame(frame);
  });
})();
