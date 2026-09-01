# HyperLED-Slave

Firmware für Slave-Boards im Master/Slave-Verbund des [HyperLED](https://github.com/KaelanTesseract/HyperLED)-Projekts. Ein Slave übernimmt keine eigene Steuerlogik oder Web-Oberfläche – er wird vollständig über den Master konfiguriert und liefert im Gegenzug exakt synchrones LED-Bild, egal ob als einfacher LED-Streifen oder als HUB75-Matrixpanel.

## Funktionsweise

- **Automatische Verbindungserkennung:** Ein Slave erkennt selbstständig, ob er kabelgebunden (UART) oder kabellos (ESP-NOW) mit dem Master verbunden ist, und rastet dauerhaft in diesem Modus ein. Bricht die Verbindung länger aus, wird automatisch wieder in den Erkennungsmodus zurückgewechselt.
- **Daisy-Chaining:** Kabelgebundene Slaves können weitere Slaves an ihrem eigenen Downlink-Port weiterreichen, sodass sich mehrere Boards in Reihe verketten lassen.
- **Fernkonfiguration:** LED-Typ, Pinbelegung, LED-Anzahl bzw. Matrixgröße und Name werden vollständig über die Web-Oberfläche des Masters eingestellt – am Slave selbst ist nichts einzurichten.
- **Over-the-Air-Updates:** Der Master kann ein Firmware-Update aus der Ferne anstoßen; der Slave verbindet sich dafür kurzzeitig eigenständig mit dem WLAN.

## Unterstützte LED-Typen

Digitale LEDs (WS281x-Familie, SK6812 u. a.), SPI-LEDs (APA102 u. a.), analoge/PWM-LEDs sowie HUB75-Scan-Matrix-Panels.

## Hardware

Die Firmware läuft auf dem **ESP32-S3** (getestet auf dem Waveshare ESP32-S3-Zero). Die feste Pinbelegung für HUB75 und die HyperBus-Verbindung ist in `include/Config.h` dokumentiert.

## Lizenz

HyperLED-Slave steht unter der [European Union Public Licence v1.2 (EUPL-1.2)](LICENSE). Hinweise zu verwendeten Drittanbieter-Bibliotheken finden sich am Ende der Lizenzdatei.
