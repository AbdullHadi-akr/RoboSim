# Braccio-Simulator

3D-Simulation des Roboterarms Arduino/TinkerKit **Braccio** im Browser.
Arduino-Sketches für den Braccio lassen sich damit ohne Hardware schreiben, ausführen und prüfen.
Dazu kommen Werkzeuge für Kinematik, Trajektorienplanung, Kalibrierung, Arbeitsraumüberwachung und Pick'n'Place.

## Starten

- **macOS:** `start.command` doppelklicken. Das Skript startet einen lokalen Webserver und öffnet den Browser.
  Falls macOS warnt: Rechtsklick → *Öffnen*.
- **Alternativ:** `index.html` direkt in Chrome, Edge oder Firefox öffnen.

Keine Installation nötig. Eine Internetverbindung braucht es trotzdem, weil Three.js (3D) und CodeMirror (Editor) per CDN geladen werden.
Code, Wegpunkte, Zonen und Einstellungen werden im Browser gespeichert (localStorage).

## Funktionen

| Bereich | Was man damit machen kann |
|---|---|
| **Gelenke** | Servowinkel M1–M6 per Slider einstellen, TCP-Pose ablesen (real und Modell), TCP kartesisch in x/y/z/ψ verfahren, passende Code-Zeile kopieren |
| **Arduino-Code** | Sketch schreiben oder öffnen, in der Simulation ausführen, serieller Monitor, Fehlermeldungen mit Zeilennummer, 7 Beispielprogramme |
| **Trajektorie** | Wegpunkte anlegen; Trapezprofil, Polynom 3. oder 5. Ordnung; PTP (Gelenkraum) oder LIN (Gerade); mit oder ohne Halt; Plots von q, q̇, q̈; Export als CSV oder Arduino-Sketch |
| **Inverse Kinematik** | alle geometrischen Lösungen (vorne/hinten, Ellbogen oben/unten) mit Status *gültig / Gelenkgrenze / unzulässig / unerreichbar*; numerisch per Gradientenverfahren, Pseudoinverse oder Damped Least Squares mit Konvergenzplot; Jacobi-Matrix, Singulärwerte, Manipulierbarkeit; Ziel per Klick oder Ziehen im 3D |
| **Kalibrierung** | verborgene Gelenk-Offsets simulieren, Messpunkte an Referenzmarkern oder per „Lasertracker“ aufnehmen, Offsets per Least Squares schätzen, Kompensation anwenden |
| **Arbeitsraum** | erreichbaren und unzulässigen Arbeitsraum als 2D-Schnitt und in 3D anzeigen; Sperrzonen und Arbeitsbereiche; Tisch- und Selbstkollision; Geschwindigkeitsgrenze; Reaktion *warnen / verlangsamen / Not-Halt*; Ereignisprotokoll |
| **Pick'n'Place** | Szenarien: Einzelwürfel, Farbsortierung, Stapeln, Förderband mit Lichtschranken; Greifen, Ablegen, Stapeln; Kontrolle der Zielbereiche |
| **Modell** | Geometrie anpassen, DH-Tabelle, Transformationsmatrizen T₀ⁱ, Servo-Parameter, Haltemomente, Export der Kinematik als MATLAB-Skript, Umschalten zwischen STL-Geometrie und vereinfachtem Modell |
| **Labor Ü2** | Laborübung 2 (Simulink-Modell `Braccio_CtrlSimMdl.slx`) im Browser: MATLAB-Dateien der Übung bearbeiten und ausführen, Bedienpanel wie im Labor, IK-Designer, Blockparameter, Scope – siehe unten |

Unten zeigt ein **Scope** Soll- und Ist-Werte der Gelenke sowie die TCP-Geschwindigkeit.
Oben links blendet ein HUD TCP-Pose und Gelenkwinkel ein.
Simulationsgeschwindigkeit (0,25×–10×), Pause, Kameraansichten und die Anzeige von Koordinatensystemen, TCP-Spur und Zonen sind oben in der Leiste einstellbar.

## Arduino-Unterstützung

- `Braccio.begin()`, `Braccio.ServoMovement(stepDelay, M1..M6)`: wie die Bibliothek mit 1°-Schritten und Winkelgrenzen (M2 15–165°, M6 10–73°)
- `Servo`: `attach`, `write`, `writeMicroseconds`, `read`
  - Pins: 11 Basis, 10 Schulter, 9 Ellbogen, 5 Handgelenk, 6 Handrotation, 3 Greifer
- `delay`, `millis`, `micros` laufen in Simulationszeit
- `Serial.print/println`, `pinMode`, `digitalRead/Write`
- Mathematik: `sin`, `cos`, `atan2`, `sqrt`, `pow`, `map`, `constrain`, `random` u. a.
- Sprachumfang:
  - eigene Funktionen, globale und lokale Variablen
  - Arrays (auch 2D), `#define` (auch mit Parametern), Casts
  - `for`, `while`, `do`, `switch`

Förderband-Szenario: **D2** Lichtschranke unten (jede Box), **D4** Lichtschranke oben (nur große Box), **D7** Bandmotor.

**Nicht unterstützt:**
- Zeiger, Structs, Klassen
- `static` wird ignoriert.
- Ganzzahl-Division ist nur bei der Initialisierung von `int`-Variablen exakt.
- `Serial`-Eingaben liefern nichts.

## Laborübung 2 (Reiter „Labor Ü2“)

Der Reiter bildet die Übung `Labore/Uebung2_Braccio` nach: das Simulink-Modell `Braccio_CtrlSimMdl.slx` mit der Bibliothek `Braccio_lib.slx`, die Szene (Plattform, Pick-up-Marker, zwei Würfel, Hochregal, Sortierbox, virtuelle Wand) und die MATLAB-Dateien.

- **MATLAB:** `Braccio_kinematics.m`, `Braccio_plotConfig.m` und `Braccio_plotSimResults.m` im Original, außerdem der Code der MATLAB-Function-Blöcke `ForwardKinematics`, `TeachIn/StoreWaypoints` und `CartesianPath/inverseKinematics3dof`.
  - Alle Dateien lassen sich bearbeiten. Skripte laufen ganz oder abschnittsweise (Strg+Enter) in einem Basis-Workspace.
  - Dazu gibt es Befehlsfenster, Workspace-Ansicht und Abbildungen (`figure`, `subplot`, `plot` …).
  - `show(braccioRobot)` zeichnet die Körper-KS des `rigidBodyTree` als DH-Skelett ins 3D-Fenster.
  - Prüfknöpfe vergleichen die DH-Parameter und die eigene Vorwärtskinematik mit der Referenz.
- **Simulation:** feste Abtastzeit 10 ms, wie im Modell:
  - Chart (Pause/Run/Notaus) → CtrlMode 1–5 (Slider, Push Buttons, Replay, Pick'n'Place, CartesianPath) → JointLimits → simpleDynamics (Ratenbegrenzer 1 rad/s) → ForwardKinematics → Arbeitsraumüberwachung.
  - Die Blöcke rechnen mit dem bearbeiteten MATLAB-Code. Die Marker TCP, B und C zeigen dessen Ergebnis.
  - Bei x ≤ `efence1_coord.x` leuchten die Wand und „Lim X“ rot.
  - Nach dem Stoppen liegt `out` (ConfigSim, TCPpos, CtrlMode) im Workspace.
- **Bedienpanel:** INIT, NOTAUS, RUN/PAUSE, Mode, Axis, TEACH-IN POSITION/RESET, −/+ und die Lampen wie in `Braccio_HMI.png`.
- **IK-Designer:** Ersatz für den Inverse Kinematics Designer.
  - Gelenke einstellen, „Store Configuration“, Abfolge abspielen.
  - „Check Collisions“ prüft gegen die `collisionBox`-Objekte im Workspace, „Export“ legt `waypointData` (N×5, rad) an.
- **Blockparameter:** Waypoints und TimePoints des Polynomial-Trajectory-Blocks, Greifer-Lookup-Tabelle, Ratenbegrenzung und weitere. Sie werden beim Start im Workspace ausgewertet.

Koordinaten: Labor-KS0 in m mit x nach vorne. Im Simulator gilt x_sim = −y_lab, y_sim = x_lab.
Gelenkwinkel: M1 = q1 + 90°, M2 = 180° − q2, M3 = 90° − q3, M4 = 180° − q4, M5 = q5 + 90°.
Die Slider des Bedienpanels geben Servo-Winkel φ vor: q2 = φ2 + 90°, q4 = φ4 + 90°, sonst q = φ.

Nicht nachgebildet: Unreal-Visualisierung, STL-Import (`stlread` & Co. haben keine Wirkung), Videoexport in `Braccio_plotSimResults.m`.
Den Inhalt des Blocks `Replay` (Bibliothek `Braccio_lib_loesung`) enthält die Übung nicht. Er fährt hier die Teach-In-Posen mit Quintic-Polynomen im 2-s-Raster ab.

## Modell und Konvention

```
φ  = M1                 e2 = 180° − M2          e3 = 270° − M2 − M3
ψ  = 360° − M2 − M3 − M4 (Werkzeugwinkel ggü. Horizontale, −90° = senkrecht nach unten)
r  = a2·cos e2 + a3·cos e3 + L·cos ψ,   z = d1 + a2·sin e2 + a3·sin e3 + L·sin ψ
```

DH-Parameter (T = Rot_z(θ)·Trans_z(d)·Trans_x(a)·Rot_x(α)):

| i | θ_i | d_i | a_i | α_i |
|---|---|---|---|---|
| 1 | M1 | d1 | 0 | 90° |
| 2 | 180° − M2 | 0 | a2 | 0 |
| 3 | 90° − M3 | 0 | a3 | 0 |
| 4 | 180° − M4 | 0 | 0 | 90° |
| 5 | M5 | L_TCP | 0 | 0 |

Standardgeometrie: d1 = 71,5 mm, a2 = a3 = 125 mm, d5 = 130 mm (Greiferflansch), L_TCP = 180 mm (Greifmitte), L_Spitze = 186 mm.
Die Werte lassen sich im Bereich *Modell* an einen konkreten Roboter anpassen.

Die 3D-Darstellung nutzt die STL-Geometrie des Braccio (reduzierte Meshes in `js/meshes.js`, je Glied im zugehörigen DH-Koordinatensystem; der Greifer sitzt auf KS5 mit d5 = 130 mm, die Finger öffnen und schließen mit M6). Der Greifer ist als Getriebe animiert: Servo-Zahnrad und Gegenrad (1:1, gegensinnig) treiben je Seite ein Viergelenk aus Kurbel (30 mm), Schwinge (30 mm) und Fingerträger (25 mm); die Getriebestellung wird aus der Greiferöffnung berechnet (`BS.kin.gripLinkage`).

Die Meshes entsprechen den Visuals des MATLAB-Modells `Braccio_robot.mat` (`rigidBodyTree`, `*_reduced.stl`). Für die Darstellung werden die Normalen mit Kantenwinkel geglättet, Servos (schwarz) und Abtriebswellen (weiß) sind als eigene Teile abgetrennt. `js/details.js` ergänzt Achsbolzen, Basisschrauben, Gelenkstifte im Greifer, die Steuerung (Arduino Uno mit Braccio-Shield hinter der Basis) und die 3-adrigen Servokabel, die entlang des Arms zu den Steckern M1–M6 laufen und der Bewegung folgen. Die Zusatzteile sind rein visuell und gehen nicht in Kollisions- oder Arbeitsraumprüfung ein.
Alle Servos auf 90° heißt: Arm steht senkrecht. M5 = 90° heißt: Die Greiferbacken öffnen quer zur Armebene.

**Grenzen des Modells:**
- Simuliert werden Kinematik und ein vereinfachtes Servo-Modell (PT1 mit Geschwindigkeitsbegrenzung), keine Mehrkörperdynamik.
- Das Greifen ist vereinfacht: Würfel richten sich beim Schließen an den Backen aus.

## Dateien

```
index.html          Einstieg
start.command       Startskript (macOS)
css/style.css       Gestaltung
js/core.js          Konfiguration, Hilfsfunktionen, Plot
js/kinematics.js    DH, Vorwärts-/inverse Kinematik, Jacobi, Arbeitsraum, Kalibrier-Schätzung
js/trajectory.js    Trajektorienplanung
js/sim.js           Simulationskern: Zeit, Servos, Greifen, Objekte, Förderband, Überwachung
js/arduino.js       Arduino→JavaScript-Übersetzer, Laufzeit, Beispielprogramme
js/matlab.js        MATLAB-Interpreter (Teilmenge) für die Laborübung
js/lab2.js          Laborübung 2: Simulink-Modell, Szene, IK-Designer
js/lab2-files.js    Originaldateien der Übung (m-Skripte, Code der MATLAB-Function-Blöcke)
js/view3d.js        3D-Darstellung
js/meshes.js        STL-Geometrie des Roboters (Base, Link 1–4, Greifer; Servos und Greifergetriebe als eigene Teile)
js/details.js       Zusatzdetails der 3D-Darstellung (Schrauben, Steuerung, Servokabel)
js/ui*.js           Oberfläche (je Bereich eine Datei)
```
