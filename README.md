# Rover Luna su Arduino UNO Q

## 1. Importa e avvia

1. In Arduino App Lab seleziona il tuo UNO Q. Ferma Camera Basics con **Stop**.
2. Se il servizio SwarmFlow è tornato attivo, fermalo temporaneamente dal terminale UNO Q con il comando che hai già usato:

   `sudo systemctl --user --machine=arduino@.host stop swarmflow-vision-worker.service`

   Puoi verificare che la camera sia libera con `sudo fuser -v /dev/video0`.
3. In App Lab apri **Apps → + → Import or install App**, seleziona **Rover-Luna-colori.zip** senza estrarlo e conferma con **Open**.
4. Apri **Rover Luna** e premi **Run**. Il primo avvio può scaricare OpenCV e U8g2. Apri la pagina web proposta da App Lab. Se non compare, controlla l'indirizzo indicato nella scheda **Console → Python**.

## 2. Prova le immagini

1. Carica la foto completa sotto **Mappa di riferimento**. Per il primo test usa un foglio piano fotografato il più possibile dall'alto, ben illuminato.
2. Punta la camera su una parte della stessa superficie e premi **Scatta con Arduino**. In alternativa carica una seconda foto sotto **Fotogramma da file**, utile per provare anche con due scatti del telefono.
3. Premi **Localizza sulla mappa**. Il contorno verde indica l'area inquadrata e la freccia punta verso il bordo superiore del fotogramma. Il centro è in pixel della mappa, con origine in alto a sinistra. L'angolo è in gradi nel piano dell'immagine, positivo in senso orario a partire dall'alto.

Il prototipo usa SIFT e un'omografia stimata con RANSAC per una superficie piana. L'angolo misura la direzione proiettata nel quadro di riferimento fotografico. Per ottenere posizione in centimetri e yaw fisico al touchdown servono coordinate metriche sulla superficie, calibrazione e montaggio della camera, e un evento o un fotogramma che identifichi l'atterraggio. Puoi usare lo scatto manuale o una sequenza breve con il pulsante descritto qui sotto.

## 3. Calibra il sensore su A0

Con la pagina aperta, spegni la lampada e premi **Registra al buio**. Poi accendila e premi **Registra con luce**. Il valore ADC si aggiorna ogni 1,5 secondi e lo stato passa a **LUCE** o **BUIO**. Se resta **Da calibrare**, aumenta la differenza fra le due condizioni e registra di nuovo entrambe. Le letture periodiche dipendono dalla pagina aperta. La calibrazione si perde quando riavvii l'app.

Il codice assume un OLED I²C SSD1306 su `Wire` all'indirizzo predefinito della libreria. Se la pagina mostra valori ma l'OLED resta vuoto, controlla il controller del modulo. Per un SH1106 sostituisci `U8X8_SSD1306_128X64_NONAME_HW_I2C` con `U8X8_SH1106_128X64_NONAME_HW_I2C` in `sketch/sketch.ino`, poi salva e riavvia l'app. Se il modulo usa l'indirizzo 0x3D, aggiungi `display.setI2CAddress(0x3D << 1);` prima di `display.begin();`.

## 4. Scarica la mappa dall'altro PC via Wi-Fi

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

Con la mappa caricata, premi **Prova atterraggio (5 s)**. Abbassa la camera simulando la discesa e tienila ferma negli ultimi due secondi. L'app acquisisce otto fotogrammi a intervalli di circa mezzo secondo e tenta il matching sugli ultimi. Mostra una stima solo se almeno due fotogrammi finali concordano su centro e direzione. L'ultimo match deve essere vicino alla fine della cattura. Il pulsante simula il momento dell'atterraggio usando la fine della sequenza. Non richiede il sensore di luce.

Se la posizione non viene confermata, l'app mostra l'ultimo fotogramma. Controlla che sia nitido e contenga piu dettagli della mappa, poi riprova.


## Correzione dei colori

La correzione ? attiva al primo avvio e si applica ai nuovi fotogrammi Arduino, compresa la sequenza di atterraggio. Per un confronto, togli la spunta da **Correggi i colori della camera** e scatta di nuovo, lasciando fermi camera e illuminazione. Le foto importate restano come sono. Lo scatto singolo lascia circa due secondi alla camera per assestare l'esposizione. La correzione usa quattro tonalit? di grigio misurate sulla carta di riferimento. Il profilo ? specifico per le condizioni della prova e pu? richiedere una nuova calibrazione se cambia l'illuminazione.

## Modulino Movement attitude

Connect the Modulino Movement to the UNO Q Qwiic connector (Wire1, IMU address 0x6A).
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
  "frame_age_ms": 1650,
  "heading_deg": 10.3,
  "heading_source": "camera_imu",
  "imu_estimated": 1
}
```

- Coordinates use the reference image's top-left origin. X increases to the
  right and Y down. For a scaled display, use x_px / width_px and
  y_px / height_px as the marker proportions. Heading is clockwise from image-up.
- imu_estimated is 0 for a camera-anchored heading, 1 when a newer IMU
  sample propagates heading from the last camera anchor, and null when
  heading is unavailable. Position always comes from the camera.
- tracking_status is valid for a fresh visual pose, tentative for a
  low-confidence visual pose, stale for the last known visual position,
  waiting before the first usable pose, or inactive when live tracking is off.
  Use this field to style or hide the marker independently of imu_estimated.
- map.id hashes the exact image bytes supplied to live tracking. map is
  null while inactive. Position fields are null while inactive or waiting.
