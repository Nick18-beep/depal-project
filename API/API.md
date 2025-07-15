## 🚀 Riferimento API

Ecco l'elenco completo degli endpoint disponibili.

---

### `GET /`

Endpoint radice che fornisce lo stato del server e una mappa degli altri endpoint disponibili. Utile per verificare che il server sia attivo e funzionante.

* **Metodo**: `GET`
* **Risposta di Successo (`200 OK`)**:
    ```json
    {
      "status": "server_running",
      "message": "Benvenuto nel server di simulazione Isaac Sim.",
      "endpoints": {
        "generate_scene": "POST /generate_scene",
        "regenerate_data": "POST /regenerate_data",
        "list_files": "GET /list_files",
        "get_document": "GET /get_document/<path:filename>"
      }
    }
    ```

---

### `POST /generate_scene`

Avvia una **nuova simulazione da zero**, cancellando i risultati precedenti e potenzialmente caricando una nuova scena. Questo endpoint accetta due tipi di richieste: una semplice richiesta JSON o una richiesta `multipart/form-data` per caricare un file di configurazione.

* **Metodo**: `POST`
* **Logica**:
    * Controlla se un'altra simulazione è in corso.
    * Pulisce la directory di output da dati precedenti.
    * Se viene fornito un file `config.yaml`, lo salva e lo usa per la simulazione.
    * Aggiunge il task `start_simulation` alla coda.
* **Body Richiesta (Caso 1: JSON)**: Per avviare la simulazione con una configurazione di default o pre-caricata.
    ```json
    {
      "options": ["rgb", "depth"]
    }
    ```
* **Body Richiesta (Caso 2: Multipart/Form-Data)**: Per fornire un file di configurazione personalizzato.
    * `config_file`: Il file `config.yaml`.
    * `options`: Una stringa JSON contenente le opzioni.
* **Risposta di Successo (`200 OK`)**:
    ```json
    {
      "status": "success",
      "message": "Comando di avvio simulazione inviato."
    }
    ```
* **Risposte di Errore**:
    * **`409 Conflict`**: Se una simulazione è già in corso.
        ```json
        {"status": "error", "message": "Simulazione già in corso."}
        ```
    * **`500 Internal Server Error`**: In caso di errori durante l'elaborazione.

---

### `POST /regenerate_data`

Avvia un processo di **rigenerazione dei dati** sulla scena attualmente caricata, senza resettare la simulazione. È utile per generare nuovi output (es. punti di presa, bounding box) con parametri diversi.

* **Metodo**: `POST`
* **Logica**:
    * Controlla se un'altra simulazione è in corso.
    * Usa le `options` fornite per configurare gli annotatori o altri processi di generazione.
    * Aggiunge il task `regenerate_data` alla coda.
* **Body Richiesta (`JSON`)**:
    ```json
    {
      "options": ["grip", "pinza"]
    }
    ```
* **Risposta di Successo (`200 OK`)**:
    ```json
    {
      "status": "success",
      "message": "Comando di rigenerazione dati inviato."
    }
    ```
* **Risposte di Errore**:
    * **`409 Conflict`**: Se una simulazione è già in corso.

---

### `GET /list_files`

Restituisce un elenco di tutti i file presenti nella directory di output (`OUTPUT_DIRECTORY`).

* **Metodo**: `GET`
* **Risposta di Successo (`200 OK`)**: I percorsi sono relativi alla directory di output.
    ```json
    {
      "status": "success",
      "files": [
        "active_config.yaml",
        "img1/rgb_0.png",
        "img1/depth_0.tiff",
        "img1/annotations.json"
      ]
    }
    ```
* **Risposte di Errore**:
    * **`404 Not Found`**: Se la directory di output non esiste.

---

### `GET /get_document/<path:filename>`

Permette di scaricare un singolo file dalla directory di output.

* **Metodo**: `GET`
* **Parametri URL**:
    * `filename`: Il percorso relativo del file da scaricare (es. `img1/rgb_0.png`).
* **Risposta di Successo (`200 OK`)**: Il file richiesto viene inviato come allegato (`attachment`).
* **Risposte di Errore**:
    * **`404 Not Found`**: Se il file non viene trovato nel percorso specificato.

---

## 📄 Esempi di Utilizzo (cURL)

1.  **Avviare una nuova simulazione (senza file di config):**
    ```bash
    curl -X POST -H "Content-Type: application/json" \
    -d '{"options": ["rgb", "depth"]}' \
    [http://127.0.0.1:5000/generate_scene](http://127.0.0.1:5000/generate_scene)
    ```

2.  **Avviare una nuova simulazione (CON un file di config):**
    ```bash
    curl -X POST \
    -F "config_file=@/path/to/my_config.yaml" \
    -F "options=[\"rgb\", \"depth\"]" \
    [http://127.0.0.1:5000/generate_scene](http://127.0.0.1:5000/generate_scene)
    ```

3.  **Rigenerare i dati (es. punti di presa):**
    ```bash
    curl -X POST -H "Content-Type: application/json" \
    -d '{"options": ["grip", "pinza"]}' \
    [http://127.0.0.1:5000/regenerate_data](http://127.0.0.1:5000/regenerate_data)
    ```

4.  **Elencare tutti i file di output:**
    ```bash
    curl [http://127.0.0.1:5000/list_files](http://127.0.0.1:5000/list_files)
    ```

5.  **Scaricare un'immagine specifica:**
    ```bash
    # Salva il file come "output_image.png" nella cartella corrente
    curl -o output_image.png [http://127.0.0.1:5000/get_document/img1/rgb_0.png](http://127.0.0.1:5000/get_document/img1/rgb_0.png)
    ```