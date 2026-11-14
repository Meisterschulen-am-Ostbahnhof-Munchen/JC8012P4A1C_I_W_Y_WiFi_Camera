# JC8012P4A1C_I_W_Y – Funkkamera (Freenove ESP32-S3 CAM)

Das Bild einer **Freenove ESP32-S3-WROOM CAM (FNK0085)** kommt drahtlos auf das Display
des **Guition JC8012P4A1C_I_W_Y**. Es ist kein fremdes WLAN und es sind keine Zugangsdaten
nötig: Die Kamera öffnet ihr eigenes WLAN, das Display verbindet sich damit.

```
Freenove ESP32-S3 + GC0308 (640x480, YUV422)
  └─ JPEG-Kompression auf dem S3 (esp_new_jpeg)
  └─ eigenes WLAN „JC8012-CAM“ (SoftAP, Kanal 1)
  └─ HTTP-MJPEG-Stream  http://192.168.4.1/stream
        │  WLAN
        ▼
JC8012P4A1C_I_W_Y
  └─ ESP32-C6 (ESP-Hosted über SDIO) verbindet sich mit „JC8012-CAM“
  └─ ESP32-P4 liest den Stream → Hardware-JPEG-Decoder → PPA (skalieren, drehen)
  └─ Display 1280x800 (Querformat)
```

Gemessen: **11–12 fps** bei 640x480, etwa 2,2 Mbit/s. Die Grenze ist die
Software-JPEG-Kompression auf dem S3, nicht die Funkstrecke.

Den Stream kann man auch am Laptop oder Handy ansehen: mit dem WLAN **JC8012-CAM**
(Passwort `jc8012cam`) verbinden und http://192.168.4.1 öffnen.

## Projekte

| Ordner | Gerät | Inhalt |
|---|---|---|
| `cam_s3/` | Freenove ESP32-S3-WROOM CAM | Kamera, JPEG-Encoder, WLAN-Access-Point, HTTP-Server |
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

- `cam_s3`, Menü **Freenove camera**: WLAN-Name, Passwort, Kanal, Bildgröße (VGA/QVGA),
  JPEG-Qualität (Standard 60)
- `display_p4`, Menü **Wireless camera**: WLAN-Name, Passwort und Stream-URL. Diese müssen
  zur Kamera passen.

## Log

Kamera:
```
camera: Detected GC0308 camera
cam: Camera sensor: GC0308, 640x480 YUV422
cam: Wi-Fi AP "JC8012-CAM" on channel 1, stream at http://192.168.4.1/stream
cam: Client dc:1e:d5:9b:7f:88 joined
cam: Encoder: 11.5 fps, 27 KB/frame
```

Display:
```
wcam: C6 runs ESP-Hosted 2.11.7
wcam: Connected to "JC8012-CAM", IP 192.168.4.2
wcam: Streaming from http://192.168.4.1/stream (HTTP 200)
wcam: Showing 640x480 MJPEG
wcam: 11.6 fps displayed, 2121 kbit/s
```

## Was es dafür gebraucht hat

- **Der Sensor ist ein GC0308**, kein OV2640. Er liefert nur 640x480 und kein JPEG. Der S3
  nimmt deshalb YUV422 auf und komprimiert es mit `esp_new_jpeg`. Der Encoder läuft als
  eigener Task auf Core 1 mit drei JPEG-Puffern, sodass Kompression und Senden parallel
  laufen.
- **Die Standard-Netzwerkpuffer des S3 sind für Video zu klein.** Mit 5,7 KB TCP-Sendepuffer
  kamen nur etwa 2 fps (500 kbit/s) an. `cam_s3/sdkconfig.defaults` vergrößert TCP- und
  WLAN-Puffer und legt sie in den PSRAM.
- **Kanal 1** statt 6: Kanal 6 war hier von mehreren WLANs belegt.

## Mögliche Erweiterung: OV2640 statt GC0308

Ein OV2640 (oder OV5640) im Kamera-Steckplatz liefert JPEG direkt vom Sensor und bis
1600x1200. Dann entfällt die Kompression auf dem S3, und 800x600 oder mehr mit 20 fps und
mehr wäre realistisch. Dafür muss `cam_s3/main/main.c` bei diesen Sensoren auf
`PIXFORMAT_JPEG` umschalten.

## Pins der Freenove-Kamera

XCLK 15, SCCB SDA 4 / SCL 5, D0–D7 11/9/8/10/12/18/17/16, VSYNC 6, HREF 7, PCLK 13
(wie ESP32-S3-EYE; aus den Freenove-Beispielen).

## Lizenz

Apache-2.0, siehe [LICENSE](LICENSE). `display_p4/main/jc8012_lcd_init.c` stammt aus dem
Hersteller-BSP (Espressif, Apache-2.0).
