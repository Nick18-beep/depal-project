# API Reference

All endpoints are served by the Flask app launched from `main.py`. Requests are enqueued asynchronously; the HTTP response only confirms acceptance.

## `GET /`
Returns server status and the list of available endpoints.

**Response 200**
```json
{
  "status": "server_running",
  "message": "Welcome to the Isaac Sim simulation server.",
  "endpoints": {
    "generate_scene": "POST /generate_scene",
    "regenerate_data": "POST /regenerate_data",
    "list_files": "GET /list_files",
    "get_document": "GET /get_document/<path:filename>"
  }
}
```

---

## `POST /generate_scene`
Creates a new scene from scratch. The orchestrator clears previous outputs, optionally loads a user-supplied configuration, and enqueues a `start_simulation` task.

| Field         | Type                   | Description                                          |
|---------------|------------------------|------------------------------------------------------|
| `options`     | `list[str]` (optional) | Extra modules to enable (e.g., `"rgb"`, `"grip"`)    |
| `config_file` | file (optional)        | Custom `config.yaml` sent via multipart form data    |

### Example (JSON)
```bash
curl -X POST \
  -H "Content-Type: application/json" \
  -d '{"options": ["rgb", "depth"]}' \
  http://127.0.0.1:5000/generate_scene
```

### Example (multipart form)
```bash
curl -X POST \
  -F "config_file=@/path/to/my_config.yaml" \
  -F 'options=["rgb", "grip"]' \
  http://127.0.0.1:5000/generate_scene
```

**Response 200**
```json
{"status": "success", "message": "Start simulation command accepted."}
```

**Response 409** - refused because another job is running.

---

## `POST /regenerate_data`
Runs the replicator and optional gripping modules on **the last generated scene** without clearing outputs. Useful to regenerate annotations or grasp attempts.

```json
{
  "options": ["grip"]
}
```

**Response 200**
```json
{"status": "success", "message": "Data regeneration command accepted."}
```

**Response 409** - refused because another job is active.

---

## `GET /list_files`
Lists every file stored under the output directory configured in `main.py`.

**Response 200**
```json
{
  "status": "success",
  "files": [
    "active_config.yaml",
    "img1/left/rgb/rgb_0000.png",
    "img1/left/pick/grasp_results.json"
  ]
}
```

**Response 404** - output directory not found.

---

## `GET /get_document/<path:filename>`
Streams a single artifact from the output directory. The path must be relative (for example `img1/left/rgb/rgb_0000.png`).

**Response 200** - the file is returned as attachment.  
**Response 404** - the file does not exist.

---

## Workflow summary
1. `POST /generate_scene`
   - loads configuration
   - spawns assets and cameras
   - runs replicator + gripping pipelines
2. `POST /regenerate_data`
   - reuses the frozen stage from the previous run
   - regenerates outputs requested in `options`
3. Use `GET /list_files` and `GET /get_document/...` to inspect or download results.

Logs go to stdout; set `DEPAL_LOG_LEVEL` to control verbosity (`DEBUG`, `INFO`, `WARNING`, `ERROR`).
