"""Flask application factory for the Depal simulation server."""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

from flask import Flask, jsonify, request, send_from_directory

from .orchestrator import SimulationOrchestrator


def create_app(orchestrator: SimulationOrchestrator, output_dir: Path) -> Flask:
    app = Flask(__name__)
    output_dir.mkdir(parents=True, exist_ok=True)

    @app.route("/")
    def index():
        return jsonify(
            {
                "status": "server_running",
                "simulation_in_progress": orchestrator.is_running,
                "endpoints": {
                    "generate_scene": "POST /generate_scene",
                    "regenerate_data": "POST /regenerate_data",
                    "list_files": "GET /list_files",
                    "get_document": "GET /get_document/<path:filename>",
                },
            }
        )

    @app.route("/generate_scene", methods=["POST"])
    def generate_scene():
        if orchestrator.is_running:
            return jsonify({"status": "error", "message": "Simulazione già in corso."}), 409

        try:
            if "config_file" in request.files:
                config_file = request.files["config_file"]
                options = _parse_options(request.form.get("options"))
                config_path = output_dir / "active_config.yaml"
                config_file.save(config_path)
                orchestrator.submit_start(options, config_path)
                return jsonify({"status": "success", "message": "Simulazione avviata con config caricata."})

            payload = request.get_json(silent=True) or {}
            options = _parse_options(payload.get("options"))
            orchestrator.submit_start(options, None)
            return jsonify({"status": "success", "message": "Simulazione avviata."})

        except Exception as exc:  # pylint: disable=broad-exception-caught
            return jsonify({"status": "error", "message": str(exc)}), 500

    @app.route("/regenerate_data", methods=["POST"])
    def regenerate_data():
        if orchestrator.is_running:
            return jsonify({"status": "error", "message": "Simulazione già in corso."}), 409

        try:
            payload = request.get_json(silent=True) or {}
            options = _parse_options(payload.get("options"))
            orchestrator.submit_regenerate(options)
            return jsonify({"status": "success", "message": "Rigenerazione dati avviata."})
        except Exception as exc:  # pylint: disable=broad-exception-caught
            return jsonify({"status": "error", "message": str(exc)}), 500

    @app.route("/list_files", methods=["GET"])
    def list_files():
        if not output_dir.exists():
            return jsonify({"status": "error", "message": "Directory dei risultati non trovata."}), 404

        files = []
        for path in output_dir.rglob("*"):
            if path.is_file():
                files.append(str(path.relative_to(output_dir)).replace("\\", "/"))
        return jsonify({"status": "success", "files": files})

    @app.route("/get_document/<path:filename>", methods=["GET"])
    def get_document(filename: str):
        try:
            return send_from_directory(output_dir, filename, as_attachment=True)
        except FileNotFoundError:
            return jsonify({"status": "error", "message": "File non trovato"}), 404

    return app


def _parse_options(raw_options) -> Sequence[str]:
    if raw_options is None:
        return ()
    if isinstance(raw_options, (list, tuple)):
        return tuple(str(opt) for opt in raw_options)
    if isinstance(raw_options, str):
        raw_options = raw_options.strip()
        if not raw_options:
            return ()
        try:
            # Expect JSON encoded list in multipart scenarios.
            import json

            parsed = json.loads(raw_options)
            if isinstance(parsed, list):
                return tuple(str(opt) for opt in parsed)
        except json.JSONDecodeError:
            pass
        return tuple(opt.strip() for opt in raw_options.split(",") if opt.strip())
    return ()
