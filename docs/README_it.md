# Rover Luna su Arduino UNO Q

## 1. Importa e avvia

1. In Arduino App Lab seleziona il tuo UNO Q. Ferma Camera Basics con **Stop**.
2. Se il servizio SwarmFlow è tornato attivo, fermalo temporaneamente dal terminale UNO Q con il comando che hai già usato:

   `sudo systemctl --user --machine=arduino@.host stop swarmflow-vision-worker.service`

   Puoi verificare che la camera sia libera con `sudo fuser -v /dev/video0`.
3. In App Lab apri **Apps → + → Import or install App**, seleziona **Rover-Luna-colori.zip** senza estrarlo e conferma con **Open**.
4. Apri **Rover Luna** e premi **Run**. Il primo avvio può scaricare OpenCV e U8g2. Apri la pagina web proposta da App Lab. Se non compare, controlla l'indirizzo indicato nella scheda **Console → Python**.

## 2. Prova le immagini

1. Sotto **Reference map** premi **Choose File** e seleziona la foto completa in JPG o PNG. Per il primo test usa un foglio piano fotografato dall'alto, ben illuminato. Per inviare la posa all'altro sito, carica la stessa immagine usata dal sito e premi **Start live tracking**.
2. Punta la camera su una parte della stessa superficie e premi **Scatta con Arduino**. In alternativa carica una seconda foto sotto **Fotogramma da file**, utile per provare anche con due scatti del telefono.
3. Premi **Localizza sulla mappa**. Il contorno verde indica l'area inquadrata e la freccia punta verso il bordo superiore del fotogramma. Il centro è in pixel della mappa, con origine in alto a sinistra. L'angolo è in gradi nel piano dell'immagine, positivo in senso orario a partire dall'alto.

Il prototipo usa SIFT e un'omografia stimata con RANSAC per una superficie piana. L'angolo misura la direzione proiettata nel quadro di riferimento fotografico. Per ottenere posizione in centimetri e yaw fisico al touchdown servono coordinate metriche sulla superficie, calibrazione e montaggio della camera, e un evento o un fotogramma che identifichi l'atterraggio. Puoi usare lo scatto manuale o una sequenza breve con il pulsante descritto qui sotto.

## 3. Scarica la mappa dall'altro PC via Wi-Fi

1. Sul PC che ospita la mappa crea una cartella con il solo file `mappa.jpg`. Apri PowerShell in quella cartella e avvia `py -m http.server 8000` (oppure `python -m http.server 8000`). Lascia la finestra aperta mentre fai le prove.
2. Usa `ipconfig` sul PC per trovare il suo IPv4 Wi-Fi e assicurati che anche l'UNO Q sia sulla stessa rete. Se Windows chiede di consentire Python nel firewall, scegli la rete privata.
3. In App Lab premi **Stop**, apri `python/main.py` e imposta ad esempio `MAP_URL = "http://192.168.1.50:8000/mappa.jpg"` sostituendo l'IP con quello del PC. Salva e premi **Run**.
4. Nella pagina premi **Scarica la mappa dal PC**. La mappa resta caricata nella pagina per gli scatti successivi fino al ricaricamento della pagina. Poi scatta e localizza come nel punto 2.

## Se qualcosa si blocca

- **Camera occupata**. Ferma l'app e verifica il processo con `sudo fuser -v /dev/video0`. Camera Basics e SwarmFlow devono lasciare libera la camera per lo scatto. A prova conclusa puoi fermare Rover Luna e riavviare SwarmFlow sostituendo `stop` con `start` nel comando iniziale.
- **Importazione, avvio o sensore**. Copia il primo errore dalla scheda **Console** o il messaggio nella pagina, indicando quale pulsante hai premuto.
- **Corrispondenze insufficienti**. Prova una zona più nitida e riconoscibile con maggiore sovrapposizione. Usa una superficie piana e parti da una vista quasi dall'alto.

[Importare un'app in App Lab](https://docs.arduino.cc/software/app-lab/apps/manage-apps) · [Esempio ufficiale Camera Basics](https://github.com/arduino/app-bricks-examples/tree/main/core-and-foundational/06-camera-basics/01-simple-camera)


## Prova di atterraggio

Con la mappa caricata, premi **Prova atterraggio (5 s)**. Abbassa la camera simulando la discesa e tienila ferma negli ultimi due secondi. L'app acquisisce otto fotogrammi a intervalli di circa mezzo secondo e tenta il matching sugli ultimi. Mostra una stima solo se almeno due fotogrammi finali concordano su centro e direzione. L'ultimo match deve essere vicino alla fine della cattura. Il pulsante simula il momento dell'atterraggio usando la fine della sequenza.

Se la posizione non viene confermata, l'app mostra l'ultimo fotogramma. Controlla che sia nitido e contenga piu dettagli della mappa, poi riprova.


## Correzione dei colori

La correzione ? attiva al primo avvio e si applica ai nuovi fotogrammi Arduino, compresa la sequenza di atterraggio. Per un confronto, togli la spunta da **Correggi i colori della camera** e scatta di nuovo, lasciando fermi camera e illuminazione. Le foto importate restano come sono. Lo scatto singolo lascia circa due secondi alla camera per assestare l'esposizione. La correzione usa quattro tonalit? di grigio misurate sulla carta di riferimento. Il profilo ? specifico per le condizioni della prova e pu? richiedere una nuova calibrazione se cambia l'illuminazione.

## Modulino Movement attitude

Connect the Modulino Movement to the UNO Q Qwiic connector. The firmware checks
addresses 0x6A and 0x6B on Qwiic and the UNO header I²C bus, and retries if the
sensor does not respond.
Keep the rover stationary at startup or press **Calibrate IMU while stationary**.
The app uses 40 stationary samples for gyro bias and accelerometer magnitude
calibration. Earth gravity is set to 9.81 m/s² in python/imu.py. Gravity aids
roll and pitch when the measured acceleration is near the stationary magnitude.
Yaw is relative to calibration and drifts. Set SENSOR_TO_ROVER for the physical
mounting before interpreting the IMU angles as rover angles.

Load a reference map and press **Start live tracking**. The marker shows the
latest camera position in map pixels. Camera fixes anchor heading, and fresh
IMU yaw can update it for up to three seconds after a fix. A position older
than 1.25 seconds is labeled **Last known position**. The timing readout
shows the processing time for the latest map match or optical flow update.

## Pose API

Keep **Start live tracking** running in Rover Luna with the same reference image
used by the map website. Poll `GET http://<uno-q-ip>:7000/api/pose` from the
website or its backend. Polling twice per second is a starting point. For a
website served over HTTPS, use an HTTPS backend or proxy that can reach the
UNO Q on the local network.

Example response:

```json
{
  "map": {"id": "<sha256-of-reference-image-bytes>", "width_px": 1280, "height_px": 960},
  "tracking_status": "stale",
  "x_px": 924.8,
  "y_px": 423.6,
  "x_norm": 0.7225,
  "y_norm": 0.44125,
  "z_m": null,
  "height_source": null,
  "height_age_ms": null,
  "height_calibrated": false,
  "frame_age_ms": 1650,
  "heading_deg": 10.3,
  "heading_source": "camera_imu",
  "imu_estimated": 1
}
```

- Use `x_norm` and `y_norm` to place the marker on the complete reference image.
  They are fractions from 0 to 1, with (0, 0) at the top left and (1, 1) at
  the bottom right. Pixel coordinates remain available as `x_px` and `y_px`.
  Heading is clockwise from image-up.
- `imu_estimated` is 1 when the IMU propagates heading or supplies the current
  or last known height, 0 when
  the available heading or height comes from a camera anchor, and null
  when both heading and height are unavailable. X and Y come from the camera.
- tracking_status is valid for a fresh visual pose, tentative for a
  low-confidence visual pose, stale for the last known visual position,
  waiting before the first usable pose, or inactive when live tracking is off.
  Use this field to style or hide the marker independently of imu_estimated.
- map.id hashes the exact image bytes supplied to live tracking. map is
  null while inactive. Position fields are null while inactive or waiting.


## OLED landing status

Keep the rover still until IMU calibration finishes. The OLED shows READY
until the IMU detects 0.25 seconds of sustained motion, then IN AIR. After
two continuous seconds of low linear acceleration and low angular rate it
shows LANDED. Sustained motion after landing switches it back to IN AIR.
The dashboard reports the same state and offers manual arming for bench tests.

The IMU rule estimates touchdown from motion and settling. Hovering or a
pause in motion can produce the same readings as rest on a surface. Ground
contact confirmation requires another measurement.

### Height above the map

The pose response includes `z_m`, the estimated distance in meters from
the camera center to the mapped surface. The live dashboard shows it directly
under the position. `height_calibrated` indicates whether the selected map
has a matching calibration.
Add a profile to `HEIGHT_CALIBRATIONS` in `python/main.py` using the map ID
from the API as the key, and the map scale in meters per pixel and
`(fx, fy, cx, cy)` as the value. Measure the camera intrinsics at
1280 × 960 in the camera's capture mode. Use a flat, orthorectified map with
uniform scale and a camera with negligible residual lens distortion.
During live tracking, enter the measured lens-to-surface height while
stationary and click **Set IMU starting height** to start estimating before
the first camera match. API clients can send that measurement to
`POST /live/height` as `{"height_m": 0.75}`.
Rover Luna projects IMU acceleration onto the vertical axis and subtracts
gravity. Reliable visual heights correct the inertial prediction and reset
the height anchor. Expiry or a gap in readings holds the last estimate.
`height_source` is `initial`, `camera`, `imu`, `last_initial`,
`last_imu`, `last_camera`, or null. `height_age_ms` reports the age of the
anchor and `height_estimate_age_ms` reports the age of the displayed estimate.
The IMU estimate is limited to four seconds from a measured or visual anchor because
acceleration and tilt errors drift. Live tracking skips nearly black frames.
Subtract the camera's height above the lander reference point
when appropriate for the mounted orientation.

The included bench calibrations apply to the exact `download (7).jpg` and
`download (8).jpg` images. They use a 30 mm wheel diameter spanning about 97
and 55 pixels respectively, and a reference camera height of about 0.75 m.
They assume a near-overhead reference view and a centered principal point.
Verify the scale against a second known height before relying on the measurements.

Live X/Y tracking refreshes its feature points as the camera moves. It
tries to re-match the full map periodically and can keep a geometrically
consistent track for up to 12 seconds after the last map match.

## Bench obstacle check

1. Take the reference photo with the intended tabletop landing area clear.
   Leave the landmarks around it in place so the camera can match the map.
2. Load the reference photo and click **Mark clear landing area**. Click two
   opposite corners of the clear area in the map preview.
3. Set **Landing footprint** to the diameter of the lander and its desired
   margin, as a percentage of the full map width. For example, a 10 cm
   footprint over a 70 cm wide reference is about 14%.
4. Start live tracking, point the camera over the marked area, and put an
   object underneath it. A red outline means blocked. A dashed circle inside
   the marked area suggests the nearest fully visible alternative with enough
   clearance. An unverified result means the footprint could not be checked.

The check compares the registered camera view with the clear reference.
It can detect a new object that changes the appearance of the surface.
Shadows, severe lighting changes, and objects with little visual contrast
can affect it. The suggested site is a candidate for the bench test.

To enable the check via API, supply
`"landing_zone": {"area": [0.3, 0.2, 0.8, 0.75], "footprint": 0.14}`
in `POST /live/start`. The normalized rectangle is left, top, right, bottom.
`GET /api/pose` then includes a `landing_zone` object with `status`
(`clear`, `blocked`, or `unknown`), the marked `area`, the
`footprint`, and `target` and `alternative` points in normalized map
coordinates. Either point can be null. The field is null when the check is off.
