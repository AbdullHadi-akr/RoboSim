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
| **Machine Learning** | inverse Kinematik mit einem neuronalen Netz: Trainingsdaten in der Simulation erzeugen (Nennmodell oder „realer“ Roboter mit Fertigungsfehlern und Messrauschen), Netz konfigurieren und im Web Worker trainieren, Loss-Kurven, Fehlerstatistik, Histogramm und Fehlerkarte im 3D; Vergleich mit der analytischen IK, Verfeinerung per DLS, Kreisbahn abfahren; Modell als JSON oder Arduino-Header exportieren |

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

## Machine Learning: inverse Kinematik mit einem neuronalen Netz

Das Netz lernt die Abbildung Zielpose (x, y, z, ψ) → Servowinkel M1–M4.

- **Daten:** Die Trainingsdaten entstehen ohne IK-Formel.
  - Zufällige Gelenkwinkel werden angefahren und die erreichte TCP-Pose „gemessen“ (Vorwärtskinematik). Alternativ werden Zielposen gleichverteilt im Arbeitsraum gezogen und über die analytische IK beschriftet.
  - Eine Pose hat bis zu vier Lösungen (vorne/hinten × Ellbogen oben/unten). Deshalb wird auf die Konfiguration „vorne“ und eine Ellbogenlage eingeschränkt.
  - Mit der Quelle *realer Roboter* kommen die verborgenen Gelenk-Offsets aus dem Tab *Kalibrierung* und Messrauschen dazu. Das Netz lernt dann die tatsächliche Kinematik, die die analytische IK des Nennmodells nicht kennt.
- **Netz:** mehrschichtiges Perzeptron (Schichten frei wählbar, tanh/ReLU/Leaky ReLU), normierte Ein- und Ausgänge. Merkmale kartesisch (x, y, z, cos ψ, sin ψ) oder zylindrisch (r, cos φ, sin φ, z, cos ψ, sin ψ).
- **Training:** Adam, Mini-Batches, konstante Lernrate oder Kosinus-Abfall, optional L2. Das Training läuft in einem Web Worker, auch bei `file://`.
  - *Gelenkwinkel-Loss* (überwacht): mittlerer quadratischer Fehler der normierten Servowinkel.
  - *Vorwärtskinematik-Loss* (selbstüberwacht): Fehler f(q̂) − p, Gradient über die Jacobi-Matrix (Jᵀ·e), dazu ein Strafterm für die Gelenkgrenzen. Braucht keine Labels und bleibt auch bei mehrdeutigen Daten eindeutig.
- **Richtwerte:** Mit 20 000 Beispielen und einem Netz 6 → 64 → 64 → 4 liegt der mittlere Positionsfehler nach 80 Epochen unter 1 mm. Das Training dauert etwa eine halbe Minute.
- **Export:** `nnIK(x, y, z, ψ, m)` als C-Header mit den Gewichten im Flash (PROGMEM). Ein Netz 6 → 32 → 32 → 4 belegt auf dem Uno rund 9 KB Flash.

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
js/view3d.js        3D-Darstellung
js/meshes.js        STL-Geometrie des Roboters (Base, Link 1–4, Greifer; Servos und Greifergetriebe als eigene Teile)
js/details.js       Zusatzdetails der 3D-Darstellung (Schrauben, Steuerung, Servokabel)
js/nn.js            Neuronales Netz: MLP, Backpropagation, Adam, FK-Loss, Web Worker, Export
js/ui*.js           Oberfläche (je Bereich eine Datei)
```
