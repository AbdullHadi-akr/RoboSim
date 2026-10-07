#!/bin/bash
# Startet den Braccio-Simulator über einen lokalen Webserver und öffnet ihn im Browser.
cd "$(dirname "$0")"
PORT=8000
( sleep 1; open "http://localhost:$PORT/index.html" ) &
echo "Braccio-Simulator läuft auf http://localhost:$PORT  –  Beenden mit Ctrl+C"
python3 -m http.server $PORT --directory "$(pwd)"
