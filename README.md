# JC8012P4A1C_I_W_Y – Funkkamera (Freenove ESP32-S3 CAM)

Das Bild einer **Freenove ESP32-S3-WROOM CAM (FNK0085)** kommt drahtlos auf das Display
des **Guition JC8012P4A1C_I_W_Y**. Es ist kein fremdes WLAN und es sind keine Zugangsdaten
nötig: Die Kamera öffnet ihr eigenes WLAN, das Display verbindet sich damit.

```
Freenove ESP32-S3 + Kamera
  ├─ OV2640/OV3660/OV5640: JPEG direkt vom Sensor (Standard 800x600)
  └─ GC0308: YUV422 640x480, JPEG-Kompression auf dem S3 (esp_new_jpeg)
  └─ eigenes WLAN „JC8012-CAM“ (SoftAP, Kanal 1)
  └─ HTTP-MJPEG-Stream  http://192.168.4.1/stream
        │  WLAN
        ▼
JC8012P4A1C_I_W_Y
  └─ ESP32-C6 (ESP-Hosted über SDIO) verbindet sich mit „JC8012-CAM“
  └─ ESP32-P4 liest den Stream → Hardware-JPEG-Decoder → PPA (skalieren, drehen)
  └─ Display 1280x800 (Querformat)
```

Die Kamera-Firmware erkennt den Sensor selbst. Gemessen:

| Sensor | Bild | Bildrate | Datenrate |
|---|---|---|---|
| OV2640 | 800x600, JPEG vom Sensor | **25 fps** (volle Sensorrate) | ~5,4 Mbit/s |
| GC0308 | 640x480, JPEG auf dem S3 | 11–12 fps (Grenze: Software-Encoder) | ~2,2 Mbit/s |

Nach dem Verbinden dauert es etwa 20 s, bis die WLAN-Strecke ihre Übertragungsrate
hochgeregelt hat. In dieser Zeit sind es weniger Bilder pro Sekunde.

Den Stream kann man auch am Laptop oder Handy ansehen: mit dem WLAN **JC8012-CAM**
(Passwort `jc8012cam`) verbinden und http://192.168.4.1 öffnen.

## Projekte

| Ordner | Gerät | Inhalt |
|---|---|---|
| `cam_s3/` | Freenove ESP32-S3-WROOM CAM | Kamera, ggf. JPEG-Encoder, WLAN-Access-Point, HTTP-Server |
| `display_p4/` | JC8012P4A1C_I_W_Y (ESP32-P4) | WLAN über den C6, MJPEG-Empfang, Hardware-Decoder, Anzeige |

## Voraussetzungen

- **ESP-IDF 6.1**
- Auf dem ESP32-C6 des JC8012 muss **ESP-Hosted 2.11.7** laufen (der Host ist auf diese Version
  festgelegt). Ab Werk ist dort 2.3.2 installiert; das Update erledigt einmalig
  [JC8012P4A1C_I_W_Y_ESP_Hosted](https://github.com/Meisterschulen-am-Ostbahnhof-Munchen/JC8012P4A1C_I_W_Y_ESP_Hosted).

## Bauen und Flashen

Freenove-Kamera: der **rechte USB-C-Anschluss** (Blick auf die Kamera), also der
UART-Port mit CH343.

```sh
. ~/.espressif/tools/activate_idf_v6.1.sh

cd cam_s3
idf.py set-target esp32s3            # nur beim ersten Mal
idf.py -p /dev/ttyACMx flash monitor

cd ../display_p4
idf.py set-target esp32p4            # nur beim ersten Mal
idf.py -p /dev/ttyUSB0 flash monitor
```

Die Reihenfolge ist egal: Das Display wartet auf das Kamera-WLAN und verbindet sich neu,
wenn die Kamera neu startet.

## Einstellungen (`idf.py menuconfig`)

`cam_s3`, Menü **Freenove camera**:

- WLAN-Name, Passwort, Kanal
- Bildgröße für Sensoren mit JPEG (OV2640 …): 800x600, 1024x768, 1280x720 oder 1600x1200.
  Beim OV2640 laufen 800x600 mit 25 fps; die großen Formate liefert er langsamer.
- JPEG-Qualität des Sensors (0–63, kleiner = besser, Standard 12)
- Bildgröße und Encoder-Qualität für Sensoren ohne JPEG (GC0308)
- Bild um 180° drehen (Standard: an, im Sensor, kostet keine Rechenzeit)

`display_p4`, Menü **Wireless camera**: WLAN-Name, Passwort und Stream-URL. Diese müssen
zur Kamera passen. Angezeigt werden Bilder bis 1600x1200.

## Log (OV2640)

Kamera:
```
camera: Detected OV2640 camera
cam: Camera sensor: OV2640, JPEG from the sensor
cam: Wi-Fi AP "JC8012-CAM" on channel 1, stream at http://192.168.4.1/stream
cam: Sensor: 25.0 fps, 26 KB/frame
cam: Stream: 25.0 fps sent, 5424 kbit/s
```

Display:
```
wcam: C6 runs ESP-Hosted 2.11.7
wcam: Connected to "JC8012-CAM", IP 192.168.4.2
wcam: Streaming from http://192.168.4.1/stream (HTTP 200)
wcam: Showing 800x600 MJPEG
wcam: 25.0 fps displayed, 5370 kbit/s
```

## Was es dafür gebraucht hat

- **Nagle-Algorithmus aus.** Der Stream-Handler schickt pro Bild einen kleinen Header und
  dann die Bilddaten. Ohne `TCP_NODELAY` hielt TCP den Header zurück, bis die vorigen Daten
  bestätigt waren. Der Empfänger bestätigt aber verzögert, also kamen beim OV2640 nur
  2–14 statt 25 fps an. Dazu gehen Trennzeile und Header als ein Stück raus.
- **Größere Netzwerkpuffer auf dem S3.** Mit dem Standard von 5,7 KB TCP-Sendepuffer kamen
  nur etwa 2 fps an. `cam_s3/sdkconfig.defaults` vergrößert TCP- und WLAN-Puffer und legt sie
  in den PSRAM.
- **Kanal 1** statt 6: Kanal 6 war hier von mehreren WLANs belegt.
- **Sensor-Erkennung.** Die FNK0085 wird mit verschiedenen Kameras geliefert. Die Firmware
  versucht zuerst JPEG vom Sensor. Kann der Sensor das nicht (GC0308), nimmt sie YUV422 auf
  und komprimiert mit `esp_new_jpeg` in einem eigenen Task auf Core 1. Drei Puffer sorgen
  dafür, dass Komprimieren und Senden parallel laufen.
- **Bild auf dem Kopf.** Das OV2640-Modul sitzt anders herum auf dem Flexkabel; der Sensor
  dreht das Bild per `vflip` + `hmirror` um 180°.

## Pins der Freenove-Kamera

XCLK 15, SCCB SDA 4 / SCL 5, D0–D7 11/9/8/10/12/18/17/16, VSYNC 6, HREF 7, PCLK 13
(wie ESP32-S3-EYE; aus den Freenove-Beispielen).

## Lizenz

Apache-2.0, siehe [LICENSE](LICENSE). `display_p4/main/jc8012_lcd_init.c` stammt aus dem
Hersteller-BSP (Espressif, Apache-2.0).
