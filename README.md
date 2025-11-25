# Depal Project

Pipeline to generate synthetic scenes and datasets in Isaac Sim, with a lightweight Flask server exposing control APIs.

## Requirements
- NVIDIA Omniverse Isaac Sim 4.5 installed (run scripts with the Python that ships with Isaac Sim).
- Python packages installed inside the Isaac environment: `flask`, `pyyaml`, `numpy` (`python -m pip install ...` using the Isaac Python).
- NVIDIA GPU with RTX-enabled drivers.

## Quick setup
1) Install Isaac Sim, open a terminal **inside the Isaac Sim installation folder**, and run in order:  
   - Windows: `.\post_install.bat` then `.\setup_python_env.bat`  
   - Linux: run the equivalent post-install scripts provided with Isaac (if available).  
2) Clone or copy this repository somewhere on your machine (e.g., `C:\path\depal-project`).  
3) Open a terminal using the Isaac Sim Python and `cd` into the project folder.  
4) Install Python dependencies:  
   - Linux: `./isaac-sim.sh --no-window --cmd "python -m pip install flask pyyaml numpy"`  
   - Windows: `.\isaac-sim.bat -p -m pip install flask pyyaml numpy`

## Run
- Start the app with the Isaac Sim Python in headless mode:  
  - Linux: `./isaac-sim.sh --no-window --allow-root --cmd "python /absolute/path/to/main.py"`  
  - Windows: `.\isaac-sim.bat -p C:\absolute\path\to\main.py`
- The Flask server starts in the background on `http://127.0.0.1:5000` and waits for commands.  
- API reference: see `API/API.md`.

## Configuration
- Default configuration file: `config.yaml` (asset paths, textures, camera, lights, number of images).  
- To use a different config, send it as `config_file` to `/generate_scene`; it is saved as `output/active_config.yaml`.  
- Outputs are written under `output/` (rgb, depth, camera parameters, etc.).

## Notes on assets
- Ensure the `texture/` folder and the USD assets referenced in `config.yaml` are reachable (some URLs download Omniverse assets on the fly).  
- Keep the folder structure (`API/`, `depal/`, `pre_build_asset/`, etc.) unchanged.
