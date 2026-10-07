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
| **Modell** | Geometrie anpassen, DH-Tabelle, Transformationsmatrizen T₀ⁱ, Servo-Parameter, Haltemomente, Export der Kinematik als MATLAB-Skript |

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

Standardgeometrie: d1 = 71,5 mm, a2 = a3 = 125 mm, L_TCP = 175 mm (Greifmitte), L_Spitze = 192,5 mm.
Die Werte sind Richtwerte und lassen sich im Bereich *Modell* an einen konkreten Roboter anpassen.
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
js/view3d.js        3D-Darstellung
js/ui*.js           Oberfläche (je Bereich eine Datei)
```
