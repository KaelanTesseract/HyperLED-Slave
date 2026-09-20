<p align="center">
  <img width="320" alt="HyperLED" src="https://github.com/user-attachments/assets/9cf6acca-d60d-4139-bd9b-425fdc38ad3c" />
</p>

<h1 align="center">HyperLED-Slave</h1>

<p align="center">
  Firmware für die Slave-Boards im Master/Slave-Verbund von
  <a href="https://github.com/KaelanTesseract/HyperLED">HyperLED</a> –
  <br>ein Board mehr für LEDs, eingerichtet und gesteuert vom Master.
</p>

<p align="center">
  <a href="https://github.com/KaelanTesseract/HyperLED-Slave/releases/latest"><img alt="Neueste Version" src="https://img.shields.io/github/v/release/KaelanTesseract/HyperLED-Slave?label=Version"></a>
  <a href="LICENSE"><img alt="Lizenz" src="https://img.shields.io/badge/Lizenz-EUPL--1.2-blue"></a>
  <img alt="Plattform" src="https://img.shields.io/badge/Plattform-ESP32--S3-informational">
  <a href="https://github.com/KaelanTesseract/HyperLED/blob/main/docs/de/07_Master_Slave_Architektur.md"><img alt="Dokumentation" src="https://img.shields.io/badge/Doku-Wiki-success"></a>
</p>

---

## Überblick

Ein Slave hat keine eigene Weboberfläche und keine eigene Steuerung. Er wird vollständig vom Master eingerichtet und zeigt dafür ein exakt synchrones Bild – als LED-Streifen oder als HUB75-Panel.

So lässt sich eine Installation über mehrere Boards verteilen: LEDs dort anschließen, wo sie hingehören, und alles bleibt über den Master an einer Stelle bedienbar.

## Funktionsweise

- **Verbindung erkennt sich selbst:** Der Slave merkt, ob er per Kabel (UART) oder per Funk (ESP-NOW) mit dem Master spricht, und bleibt dann dabei. Fällt die Verbindung längere Zeit aus, sucht er von selbst wieder neu.
- **Einrichtung über den Master:** LED-Typ, Pins, LED-Anzahl bzw. Panelgröße und Name kommen aus der Weboberfläche des Masters. Am Slave selbst ist nichts einzustellen.
- **Rechnet selbst:** Effekte, Uhr, Datum, Text, Lauftext, Wetter, Bild und der Hintergrund-Effekt werden auf dem Slave gezeichnet. Der Master schickt nur die Einstellungen, nicht jedes Bild – das hält die Funkstrecke frei.
- **Reihenschaltung:** Ein Slave am Kabel reicht Nachrichten an seinem Downlink weiter, sodass sich mehrere Boards hintereinander hängen lassen.
- **Update aus der Ferne:** Der Master stößt ein Firmware-Update an; der Slave holt es sich selbst aus dem WLAN. Die Zugangsdaten bekommt er dabei **verschlüsselt** (ab 0.2.008). Unverschlüsselte Zugangsdaten nimmt er nur noch über das Kabel an.
- **Status-LED** an Bord zeigt, ob er eingerichtet und verbunden ist.

## Unterstützte LED-Typen

Digitale LEDs (WS281x-Familie, SK6812 und weitere), SPI-LEDs (APA102 und weitere), analoge PWM-Kanäle sowie HUB75-Scan-Matrix-Panels.

## Hardware

Entwickelt und getestet auf dem **ESP32-S3** (Waveshare ESP32-S3-Zero). Welche anderen ESP32-Chips in Frage kommen, steht im [Wiki des Hauptprojekts](https://github.com/KaelanTesseract/HyperLED/blob/main/docs/de/03_Hardware_Setup.md).

Feste Pins (in `include/Config.h` dokumentiert):

| Zweck | Pins |
|---|---|
| HyperBus zum Master (Uplink) | RX 16, TX 17 |
| HyperBus zum nächsten Slave (Downlink) | RX 18, TX 38 |
| LED-Streifen (Standard, änderbar über den Master) | 4 |
| HUB75-Panel | 1, 2, 4–15 |
| Status-LED an Bord | 21 |

> [!IMPORTANT]
> Der ESP32 darf die LEDs nicht mit Strom versorgen. Immer ein passendes Netzteil verwenden und nur Datenleitung und gemeinsame Masse mit dem Controller verbinden. Master und Slave brauchen für die Kabelverbindung eine gemeinsame Masse.

## Installation

Gebaut und geflasht wird mit **PlatformIO** (Erweiterung für Visual Studio Code). Ein Dateisystem wie beim Master braucht der Slave nicht.

```bash
git clone https://github.com/KaelanTesseract/HyperLED-Slave.git
cd HyperLED-Slave

pio run -t upload      # Firmware auf den ESP32 schreiben
```

**Danach**
1. Slave mit Strom versorgen. Per Kabel: Master TX 17 an Slave RX 16, dazu gemeinsame Masse. Per Funk: nichts weiter zu tun – der Slave sucht den Master selbst über die WLAN-Kanäle.
2. In der Weboberfläche des Masters unter *Geräte → Verbundene Geräte* erscheint der neue Slave.
3. Dort LED-Typ, Anzahl bzw. Panelgröße und Namen einstellen und speichern. Anschließend lässt sich dem Slave ein Segment zuweisen.

Spätere Updates laufen über den Master: *Einstellungen → System → Geräte jetzt aktualisieren*.

> [!NOTE]
> Die Datei in den [Releases](https://github.com/KaelanTesseract/HyperLED-Slave/releases) ist für dieses Update aus der Ferne gedacht. Die Erstinstallation läuft über den Quellcode.

## Dokumentation

Die Slave-Firmware ist Teil von HyperLED; die Doku liegt im Hauptprojekt:

- 🇩🇪 [Master/Slave-Architektur](https://github.com/KaelanTesseract/HyperLED/blob/main/docs/de/07_Master_Slave_Architektur.md) · [Hardware und Pinbelegung](https://github.com/KaelanTesseract/HyperLED/blob/main/docs/de/03_Hardware_Setup.md) · [Deutsches Wiki](https://github.com/KaelanTesseract/HyperLED/blob/main/docs/de/01_Home.md)
- 🇬🇧 [English Wiki](https://github.com/KaelanTesseract/HyperLED/blob/main/docs/en/01_Home.md)

## Lizenz

[European Union Public Licence v1.2 (EUPL-1.2)](LICENSE). Die verwendeten Drittanbieter-Bibliotheken sind am Ende der Lizenzdatei aufgeführt.
