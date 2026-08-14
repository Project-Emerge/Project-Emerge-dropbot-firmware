# Calibrazione DWM3000, arena ed EKF

Questa procedura è scritta per l'hardware realmente disponibile:

- **4 ancore di produzione**, indicate come A001, A002, A003 e A004;
- **12 robot di produzione**, ognuno con il proprio DWM3000;
- **nessuna scheda DWM3000 aggiuntiva** e nessuna fixture elettronica dedicata.

Non occorrono altri moduli. La “fixture a tre nodi” è solo una disposizione geometrica temporanea:
si riflashano tre dispositivi di produzione con i ruoli C001, C002 e C003, si eseguono le misure e
poi si ripristina il firmware di produzione. Il DWM3000 deve rimanere montato sul dispositivo finale,
con batteria e meccanica definitive.

> **Nomenclatura importante**
>
> - R01–R12 sono nomi fisici scelti dall'operatore per i robot.
> - A001–A004 sono le identità di produzione delle quattro ancore.
> - C001–C003 sono ruoli radio **temporanei** della fixture. Non identificano la scheda fisica.
> - `508040` negli esempi è il device ID MQTT del robot R01: sostituirlo con quello reale.

La calibrazione ha due fasi distinte:

1. **hardware**, una volta per tutti i 16 dispositivi, per determinare il delay del singolo DWM3000;
2. **di sito**, dopo il montaggio delle quattro ancore nell'arena, per geometria, residui e rumore.

La posizione pubblicata resta riferita al centro del robot. Il filtro ruota automaticamente
`antenna_offset_x_m` e `antenna_offset_y_m` con l'heading per calcolare il punto reale dell'antenna.

## 0. Regole da rispettare

- Misurare sempre la distanza tra i **centri di fase delle antenne**, non tra bordi dei PCB o scocche.
- Durante una sessione fixture tenere spenti tutti i dispositivi che non sono C001, C002 e C003.
- Usare la stessa configurazione PHY della produzione. Una modifica a canale, PRF, preambolo o
  protocol fingerprint invalida la calibrazione.
- Non eseguire un chip erase del robot dopo la calibrazione: il delay è nella partizione NVS. Un
  normale cambio immagine o OTA deve conservarla. Controllare al boot `persisted true` e la
  generazione attesa.
- Una scrittura permanente sul robot è possibile solo con motori arrestati e nella finestra fisica
  di 60 secondi aperta tenendo premuto il pulsante per 1,5–3 secondi.
- Un'ancora di produzione funziona solo se UID STM32, `anchor_id` e delay compilati nel manifest
  coincidono. Un UID vuoto non è un wildcard: il ranging rimane disabilitato.
- Il DS-TWR identifica il delay aggregato del dispositivo. RX e TX ricevono quindi lo stesso valore;
  il TWR non consente di separarli in modo affidabile.

### 0.1 Orientamento delle quattro ancore

Non montare per default le ancore con il PCB verticale mentre il DWM3000 del robot è orizzontale.
Qorvo definisce l'antenna DWM3000 linearmente polarizzata e avverte che due moduli perpendicolari
possono presentare nulli, perdita di range ed errore di localizzazione. La raccomandazione “modulo
verticale” del datasheet vale per un sistema RTLS in cui anche l'altro modulo è verticale; non risolve
il caso di un tag obbligatoriamente orizzontale.

Configurazione iniziale consigliata per questa arena:

- PCB delle quattro ancore **orizzontali e paralleli al pavimento**, come il PCB dei robot;
- stessa faccia del DWM3000 rivolta verso l'alto;
- zona antenna/keepout completamente libera, senza metallo, batteria o piano di massa sopra e sotto;
- ancore ai bordi o agli angoli, non sopra una zona in cui il robot possa passare direttamente sotto
  l'antenna, perché la direzione normale al PCB è sfavorevole;
- azimut delle quattro ancore non tutti uguali: partire con gli assi antenna sfalsati di 0°, 45°, 90°
  e 135°. In questo modo la rotazione del robot non porta tutti i link vicino allo stesso minimo di
  polarizzazione.

Prima del fissaggio definitivo, mettere R01 al centro e acquisire almeno 200 range per ancora agli
otto heading 0°–315°. Per ogni heading controllare tasso di misure valide, deviazione standard e P95.
La metrica decisiva è il **terzo link migliore**: devono restare almeno tre link utilizzabili in ogni
orientamento. Se non accade, variare l'azimut di una o più ancore di 15°–30° e ripetere. Provare le
ancore verticali solo come confronto sperimentale; non adottarle se migliorano un heading ma creano
nulli negli altri.

Nella fixture hardware, appoggiare temporaneamente anche le ancore in orizzontale, alla stessa altezza
del robot e con geometria simmetrica. La calibrazione di sito successiva assorbirà il residuo della
posizione finale, ma la calibrazione del delay non deve essere contaminata intenzionalmente da una
forte perdita di polarizzazione.

## 1. Preparazione

### 1.1 Costruire la CLI

Eseguire tutti i comandi dalla root del repository:

```sh
cargo build --release --manifest-path tools/uwb-calibrate/Cargo.toml
cargo run --release --manifest-path tools/uwb-calibrate/Cargo.toml -- --help
```

`fit-hardware` e `fit-site` generano un JSON, un CSV dei residui e un grafico SVG. Ogni comando che
modifica NVS, manifest o MQTT mostra sempre un'anteprima e richiede di digitare `APPLY`; `--yes`
elimina soltanto la domanda, non l'anteprima.

### 1.2 Creare il registro fisico

Prima di calibrare, preparare una tabella esterna come questa:

| Nome fisico | Tipo | ID di produzione | Device ID/UID | Stato delay | Report |
|---|---|---|---|---|---|
| R01 | robot di riferimento | — | ID MQTT a 6 caratteri | nominale | — |
| A001 | ancora di riferimento | A001 | UID STM32 a 24 hex | nominale | — |
| A002 | ancora bootstrap | A002 | UID STM32 a 24 hex | nominale | — |
| … | … | … | … | … | … |

Leggere l'UID delle quattro ancore dalla riga RTT `physical STM32 UID`. Conservare l'ordine dei 12
byte stampati e convertirli in 24 cifre esadecimali senza separatori.

### 1.3 Build e flash sono due operazioni diverse

I comandi `laze build` riportati sotto compilano l'immagine. Dopo ogni build bisogna flashare il
binario ottenuto sulla scheda fisica indicata, usando la normale procedura ESP/SWD del progetto.

## 2. Calibrazione hardware dei 16 dispositivi

Con quattro ancore e dodici robot la procedura più efficiente è:

1. calibrare insieme R01, A001 e A002 per creare i primi tre dispositivi calibrati;
2. usare poi **R01 come C001** e **A001 come C002** come riferimenti fissi;
3. inserire uno alla volta gli altri 13 dispositivi nel ruolo C003.

R01 deve essere C001 perché è il gateway Wi-Fi/MQTT. Le ancore non hanno Wi-Fi.

### 2.1 Bootstrap: R01 + A001 + A002

#### A. Preparare il triangolo

Disporre i tre dispositivi orizzontali, con la stessa faccia verso l'alto, alla stessa altezza, con
linea di vista e lati preferibilmente tra 2 e 3 m.
Annotare le tre distanze con precisione millimetrica:

| Ruolo fixture | Dispositivo fisico | Distanza da misurare |
|---|---|---|
| C001 | R01 | C001–C002 e C001–C003 |
| C002 | A001 | C002–C003 |
| C003 | A002 | — |

Il robot va appoggiato con batteria e scocca definitive; non usare il solo PCB sul tavolo.

#### B. Compilare e flashare i tre ruoli

R01 come C001:

```sh
laze build -b dropbot -a dropbot-firmware -s fixture-node-1
```

A001 come C002:

```sh
laze -C anchor build -b uwb-anchor -a uwb-anchor-firmware \
  -s anchor-1 -s anchor-fixture-node-2
```

A002 come C003:

```sh
laze -C anchor build -b uwb-anchor -a uwb-anchor-firmware \
  -s anchor-2 -s anchor-fixture-node-3
```

**Compilare e flashare un nodo alla volta.** Le build C002 e C003 scrivono entrambe nello stesso
file `build/bin/uwb-anchor/cargo/thumbv7em-none-eabihf/release/uwb-anchor-firmware`: la seconda build
sovrascrive la prima. Quindi eseguire nell'ordine `build C001 -> flash R01`, `build C002 -> flash
A001`, `build C003 -> flash A002`; non compilare tutte le immagini per poi flasharle.

Al riavvio, tutti e tre i nodi devono mostrare la stessa revisione della schedule. Per la revisione
corrente la riga contiene `schedule rev 2, turnaround 5000 us, report 3000 us`; se uno dei tre
mostra valori diversi, le immagini fixture non sono compatibili e la cattura non è valida.

La selezione `anchor-N` conserva l'identità fisica dell'ancora; `anchor-fixture-node-N` sceglie solo
il ruolo temporaneo C00N. Il firmware fixture ignora il blocco UID di produzione, quindi può essere
usato anche quando il manifest non è ancora provisionato.

#### C. Acquisire almeno 500 misure per coppia

Accendere soltanto i tre nodi, attendere che R01 sia connesso al broker e avviare:

```sh
cargo run --release --manifest-path tools/uwb-calibrate/Cargo.toml -- capture-fixture \
  --gateway-id 508040 --broker 192.168.1.10 --duration-s 180 \
  --pair C001,C002,2.137 --pair C001,C003,2.426 --pair C002,C003,2.284 \
  --output bootstrap-pre.csv
```

Sostituire le tre distanze di esempio con quelle misurate. Il CSV deve contenere tutte le sei
direzioni C001↔C002, C001↔C003 e C002↔C003 e almeno 500 campioni complessivi per coppia non ordinata.

#### D. Calcolare i tre delay

```sh
cargo run --release --manifest-path tools/uwb-calibrate/Cargo.toml -- fit-hardware \
  --input bootstrap-pre.csv --output bootstrap-fit
```

Il fit è valido solo se il report termina con `ACCEPTED`. Il solver usa Huber/IRLS e una partizione
80/20; le soglie sono mediana ≤2 cm, P95 ≤5 cm e deviazione standard ≤5 cm.

#### E. Applicare R01

Ripristinare su R01 il firmware robot di produzione, senza cancellare la NVS:

```sh
laze build -b dropbot -a dropbot-firmware
```

Quindi applicare il valore associato a C001:

```sh
cargo run --release --manifest-path tools/uwb-calibrate/Cargo.toml -- apply-robot-delay \
  --report bootstrap-fit.json --node-id C001 --robot-id 508040 \
  --broker 192.168.1.10
```

Quando richiesto, tenere premuto il pulsante di R01 per 1,5–3 secondi. Il robot arresta e disabilita
i motori, salva il record, lo rilegge e si riavvia.

#### F. Provisionare A001 e A002

Aggiornare il manifest con il valore C002 per A001 e C003 per A002:

```sh
cargo run --release --manifest-path tools/uwb-calibrate/Cargo.toml -- provision-anchor \
  --report bootstrap-fit.json --node-id C002 --anchor-id A001 \
  --uid UID_REALE_DI_A001

cargo run --release --manifest-path tools/uwb-calibrate/Cargo.toml -- provision-anchor \
  --report bootstrap-fit.json --node-id C003 --anchor-id A002 \
  --uid UID_REALE_DI_A002
```

I valori finiscono in [`anchor/calibration.toml`](../anchor/calibration.toml) e quindi nella flash
della specifica immagine ancora. Non scambiare fisicamente A001 e A002 dopo il provisioning.

#### G. Validare a una seconda geometria

Cambiare tutte e tre le lunghezze del triangolo, misurarle nuovamente e riflashare:

- R01 ancora come C001; la sua NVS contiene il delay calibrato;
- A001 come C002 e A002 come C003; ricompilare dopo l'aggiornamento del manifest.

Acquisire un nuovo file, per esempio `bootstrap-post.csv`, usando le nuove distanze, quindi:

```sh
cargo run --release --manifest-path tools/uwb-calibrate/Cargo.toml -- validate-hardware \
  --input bootstrap-post.csv --output bootstrap-validation
```

`validate-hardware` controlla l'errore grezzo dopo il flash e **non** rifà il fit. Procedere solo con
`ACCEPTED`. Eseguire un altro `fit-hardware` al posto della validazione nasconderebbe un delay non
applicato correttamente.

### 2.2 Calibrare a rotazione gli altri 13 dispositivi

Dopo il bootstrap, lasciare questi due riferimenti in fixture:

- R01 calibrato nel ruolo C001, gateway MQTT;
- A001 calibrata nel ruolo C002.

Per ogni DUT, cioè A003, A004 e R02–R12, ripetere la seguente checklist.

#### Checklist per un singolo DUT

1. **Delay iniziale nominale.** Il DUT C003 deve partire da RX/TX 16385. Un robot mai calibrato lo
   usa automaticamente. Per rifare una calibrazione robot già esistente, eseguire prima
   `clear-robot-delay` dal firmware di produzione. Per un'ancora da ricalibrare, riportare
   temporaneamente i suoi `rx_ticks` e `tx_ticks` a 16385 nel manifest.
2. **Flash C003.** Per un robot usare `fixture-node-3`. Per un'ancora usare la sua identità fisica e
   `anchor-fixture-node-3`.
3. **Triangolo.** Posizionare il DUT come C003, misurare nuovamente tutti i tre lati e cambiare il
   nome dei file, per esempio `R02-pre.csv` oppure `A003-pre.csv`.
4. **Cattura e fit.** Usare `capture-fixture`, quindi `fit-hardware`.
5. **Applicare soltanto C003.** Le righe C001 e C002 del nuovo report descrivono il residuo dei due
   riferimenti già calibrati e non devono essere riapplicate. Il solver esprime i risultati rispetto
   al valore nominale, per questo il DUT deve partire da 16385.
6. **Seconda geometria.** Con il nuovo delay attivo, cambiare i tre lati, acquisire il file `-post.csv`
   ed eseguire `validate-hardware`.
7. **Ripristino produzione.** Se la validazione è accettata, flashare il firmware di produzione del
   DUT e aggiornare il registro fisico.

Esempio di build di R02 come DUT C003:

```sh
laze build -b dropbot -a dropbot-firmware -s fixture-node-3
```

Applicazione del suo risultato:

```sh
cargo run --release --manifest-path tools/uwb-calibrate/Cargo.toml -- apply-robot-delay \
  --report R02-fit.json --node-id C003 --robot-id ID_MQTT_DI_R02 \
  --broker 192.168.1.10
```

Esempio di A003 come DUT C003:

```sh
laze -C anchor build -b uwb-anchor -a uwb-anchor-firmware \
  -s anchor-3 -s anchor-fixture-node-3

cargo run --release --manifest-path tools/uwb-calibrate/Cargo.toml -- provision-anchor \
  --report A003-fit.json --node-id C003 --anchor-id A003 \
  --uid UID_REALE_DI_A003
```

Al termine devono risultare calibrati 12/12 robot e 4/4 ancore. Ripristinare anche R01 e A001 con le
immagini di produzione e verificare:

- `persisted true` su tutti i robot dopo reboot;
- UID e delay corretti nel log delle quattro ancore;
- nessuna ancora con `expected_uid = ""` nel manifest;
- ogni ancora risponde soltanto con la propria identità A001–A004.

Rollback robot, se necessario:

```sh
cargo run --release --manifest-path tools/uwb-calibrate/Cargo.toml -- clear-robot-delay \
  --robot-id ID_MQTT --broker 192.168.1.10
```

## 3. Calibrazione dell'arena

Questa fase si esegue dopo aver scelto l'azimut, installato e flashato in produzione tutte le quattro
ancore. L'orientamento deve rimanere invariato dopo questa calibrazione. La
geometria e i parametri delle ancore sono condivisi dalla flotta: usare R01 come robot di riferimento
per definirli una volta, quindi non sovrascriverli durante la regolazione degli altri robot.

### 3.1 Rilievo geometrico

Definire un sistema di riferimento unico e misurare `x`, `y`, `z` del centro di fase di ogni ancora.
Misurare anche:

- altezza del centro di fase dell'antenna del robot;
- `antenna_offset_x_m`, positivo verso il davanti del robot;
- `antenna_offset_y_m`, positivo verso la sinistra del robot.

Esempio minimo di `arena.json`; gli ID JSON sono decimali:

```json
{
  "robot_antenna_height_m": 0.070,
  "anchors": [
    { "anchor_id": 40961, "x": 0.00, "y": 0.00, "z": 0.35 },
    { "anchor_id": 40962, "x": 4.00, "y": 0.00, "z": 0.35 },
    { "anchor_id": 40963, "x": 4.00, "y": 4.00, "z": 0.35 },
    { "anchor_id": 40964, "x": 0.00, "y": 4.00, "z": 0.35 }
  ]
}
```

40961–40964 corrispondono a 0xA001–0xA004. Le coordinate sono solo un esempio e devono essere
sostituite con il rilievo reale.

### 3.2 Campagna completa con R01

Usare cinque posizioni: centro e quattro punti prossimi agli angoli, evitando di mettere il robot
direttamente sotto un'ancora. In ogni punto acquisire otto heading: 0°, 45°, …, 315°. Sono quindi 40
pose. Spostare e ruotare il robot manualmente, con motori arrestati.

Per la prima posa:

```sh
cargo run --release --manifest-path tools/uwb-calibrate/Cargo.toml -- capture-site \
  --robot-id 508040 --broker 192.168.1.10 --anchors arena.json \
  --x-m 2.0 --y-m 2.0 --heading-deg 0 \
  --antenna-offset-x-m 0.022 --antenna-offset-y-m -0.006 \
  --duration-s 60 --output R01-site.csv
```

Per le altre 39 pose ripetere lo stesso comando con coordinate/heading reali e `--append`:

```sh
... --heading-deg 45 --output R01-site.csv --append
```

Le chiamate `--append` riusano automaticamente il session ID della prima riga. Controllare almeno
200 misure valide per posa e una distribuzione ragionevolmente uniforme tra le quattro ancore. Le
misure con clock tracker non convergente restano nel CSV ma sono escluse dal fit.

Calcolare il fit:

```sh
cargo run --release --manifest-path tools/uwb-calibrate/Cargo.toml -- fit-site \
  --input R01-site.csv --output R01-site-fit
```

Il modello stima:

- offset residuo comune di R01;
- offset, `scale_ppm` e `range_sigma_m` di ogni ancora;
- residuo per heading a passi di 45° e per `response_subslot`.

Gli offset delle quattro ancore hanno somma zero. La dipendenza dall'orientamento viene diagnosticata
e incorporata nel sigma, non compensata con una LUT.

Applicare sia R01 sia la configurazione condivisa delle ancore:

```sh
cargo run --release --manifest-path tools/uwb-calibrate/Cargo.toml -- apply-site \
  --report R01-site-fit.json --current-robot-config robot-R01.json \
  --broker 192.168.1.10
```

Questo pubblica retained `/config/robots/{R01_ID}` e `/config/anchors`.

### 3.3 Gli altri undici robot

Per R02–R12 non è necessario ricalibrare fisicamente le ancore. Procedere in due livelli:

1. **Verifica minima obbligatoria:** con la configurazione ancore di R01 già attiva, controllare ogni
   robot in centro e vicino ai quattro angoli, in almeno quattro orientamenti. Verificare errore della
   posa, jitter e numero di link utilizzabili.
2. **Fit individuale, solo se serve:** se un robot mostra un bias comune pur avendo superato la
   calibrazione hardware, ripetere le 40 pose e `fit-site` per quel robot. Applicare esclusivamente la
   sua configurazione usando `--robot-only`:

```sh
cargo run --release --manifest-path tools/uwb-calibrate/Cargo.toml -- apply-site \
  --report R02-site-fit.json --current-robot-config robot-R02.json --robot-only \
  --broker 192.168.1.10
```

Con `--robot-only`, `/config/anchors` rimane invariato. Non usare un normale `apply-site` per R02–R12,
altrimenti l'ultimo robot calibrato sostituirebbe la configurazione condivisa delle ancore.

Misurare comunque l'offset geometrico dell'antenna per ogni robot se le tolleranze di assemblaggio
non garantiscono la stessa posizione; non copiare automaticamente quello di R01.

### 3.4 Validazione finale indipendente

La validazione non deve riutilizzare le stesse 40 pose del fit. Scegliere punti e traiettorie nuovi e
confrontare `/pose/{id}` con posizioni di riferimento misurate. Ripetere prima con un solo robot e poi
con più robot presenti, perché batteria, scocche e corpi degli altri robot introducono NLOS.

Criteri di accettazione:

- jitter statico RMS ≤3 cm;
- errore posizione P95 ≤15 cm nell'arena;
- almeno tre link utilizzabili a ogni orientamento;
- nessun salto significativo della posa con uno spike positivo di 30 cm;
- deviazione standard LOS calibrata ≤5 cm.

Se il residuo dipende fortemente dall'heading o restano meno di tre link, intervenire su altezza,
azimut/polarizzazione delle ancore o ambiente RF prima di irrigidire i gate dell'EKF.

## 4. Topic MQTT di calibrazione

| Topic | Direzione | Uso |
|---|---|---|
| `/calibration/command/{id}` | PC → robot | `start_capture`, `apply_robot_delay`, `clear_robot_delay`. |
| `/calibration/status/{id}` | robot → PC | Stato, rifiuto, `armed`, generazione persistita e reboot. |
| `/calibration/samples/{id}` | R01 fixture → PC | Report DS-TWR con coppia, sequenza e distanza. |

Le scritture permanenti non sono retained. `/config/robots/{id}` e `/config/anchors` sono retained
perché sono configurazioni di sito reversibili.

## 5. Ordine di tuning dell'EKF

Dopo aver concluso hardware e sito, usare il `range_sigma_m` misurato per ancora e regolare in questo
ordine:

1. `gate_long` e `gate_short`, con `gate_long` più stretto per gli spike NLOS positivi;
2. process noise durante il movimento;
3. process noise da fermo;
4. `speed_tau_s`;
5. `full_duty_speed_m_s` del singolo robot.

Non aumentare il process noise per nascondere un range sistematicamente errato. Una modifica al
modello del filtro o all'offset antenna forza correttamente un nuovo bootstrap EKF a tre ancore.
