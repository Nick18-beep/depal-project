"""Entry point for the Depal Isaac Sim application."""

from __future__ import annotations

import threading
from pathlib import Path

from depal.config import ConfigStore
from depal.simulation import SimulationEnvironment, SimulationPipeline
from depal.server import SimulationOrchestrator, create_app

BASE_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = BASE_DIR / "config.yaml"
OUTPUT_DIRECTORY = BASE_DIR / "output"


def main() -> None:
    config_store = ConfigStore(DEFAULT_CONFIG_PATH)

    environment = SimulationEnvironment(config_store.current()["simulation_setup"])
    environment.initialize()

    pipeline = SimulationPipeline(environment, BASE_DIR)
    orchestrator = SimulationOrchestrator(config_store, pipeline, OUTPUT_DIRECTORY)

    flask_app = create_app(orchestrator, OUTPUT_DIRECTORY)

    server_thread = threading.Thread(
        target=lambda: flask_app.run(host="0.0.0.0", port=5000, debug=False),
        daemon=True,
    )
    server_thread.start()

    print("\n--- SERVER FLASK AVVIATO IN BACKGROUND su http://127.0.0.1:5000 ---")
    print("--- AVVIO LOOP DI SIMULAZIONE PRINCIPALE. In attesa di comandi... ---")

    orchestrator.run_forever()


if __name__ == "__main__":
    main()
