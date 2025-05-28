import os
import sys
import omni.usd
import omni.timeline # Aggiunto per controllare la timeline/simulazione
import omni.kit.app

# Assicurati che _get_main_script_dir() e save_stage_with_pause_and_resume
# siano definite come nelle risposte precedenti.

def _get_main_script_dir():
    # ... (implementazione da risposta precedente)
    try:
        main_mod = sys.modules.get("__main__")
        main_file = getattr(main_mod, '__file__', None)
        if main_file:
            return os.path.dirname(os.path.abspath(main_file))
        else:
            print("[_get_main_script_dir] Attenzione: __main__.__file__ non disponibile. Uso os.getcwd() come fallback.")
            return os.getcwd()
    except Exception as e:
        print(f"[_get_main_script_dir] Eccezione durante il tentativo di ottenere la directory principale: {e}. Uso os.getcwd().")
        return os.getcwd()

def save_stage_with_pause_and_resume(output_folder_relative: str, file_name: str = "saved_stage.usd"):
    # ... (implementazione da risposta precedente)
    print(f"[save_stage] Avvio salvataggio stage in '{output_folder_relative}/{file_name}'...")
    tl = omni.timeline.get_timeline_interface()
    is_playing = tl.is_playing()
    if is_playing:
        tl.stop()
        print("[save_stage] Timeline messa in pausa.")

    base_dir = _get_main_script_dir()
    
    full_output_folder = os.path.join(base_dir, output_folder_relative)
    full_output_folder = os.path.normpath(full_output_folder)

    try:
        os.makedirs(full_output_folder, exist_ok=True)
        print(f"[save_stage] Cartella di output assicurata: {os.path.abspath(full_output_folder)}")
    except Exception as e:
        print(f"[save_stage] Errore durante la creazione della cartella '{full_output_folder}': {e}", file=sys.stderr)
        if is_playing:
            tl.play()
        return

    ctx = omni.usd.get_context()
    stage = ctx.get_stage()
    if not stage:
        print("[save_stage] Errore: Nessuno stage attivo da salvare.", file=sys.stderr)
        if is_playing:
            tl.play()
        return
        
    root_layer = stage.GetRootLayer()
    output_path = os.path.join(full_output_folder, file_name)
    abs_path = os.path.abspath(output_path)
    print(f"[save_stage] Salvataggio stage in percorso assoluto: {abs_path}")

    try:
        if not root_layer.Export(abs_path):
             raise RuntimeError(f"root_layer.Export() non riuscito per '{abs_path}'")
        print(f"[save_stage] Stage salvato con successo in: {abs_path}")
    except Exception as e:
        print(f"[save_stage] Errore durante l'esportazione dello stage in '{abs_path}': {e}", file=sys.stderr)
    finally:
        if is_playing:
            tl.play()
            print("[save_stage] Timeline ripresa.")


def load_stage_in_new_stage(file_path_relative_to_main: str):
    """
    Carica lo stage USD da file_path_relative_to_main in uno stage nuovo e vuoto,
    utilizzando direttamente omni.usd.get_context().new_stage().
    Tenta di mettere in pausa la simulazione durante il processo.
    """
    base_dir = _get_main_script_dir()
    abs_file_path = os.path.join(base_dir, file_path_relative_to_main)
    abs_file_path = os.path.normpath(os.path.abspath(abs_file_path))

    print(f"[load_stage] Tentativo di caricare lo stage da: {file_path_relative_to_main}")
    print(f"[load_stage] Percorso assoluto calcolato per il caricamento: {abs_file_path}")

    if not os.path.exists(abs_file_path):
        print(f"[load_stage] ERRORE CRITICO: Il file specificato non esiste: {abs_file_path}", file=sys.stderr)
        raise FileNotFoundError(f"File USD non trovato presso {abs_file_path}.")

    app = omni.kit.app.get_app()
    timeline = omni.timeline.get_timeline_interface()
    was_playing = timeline.is_playing()

    if was_playing:
        print("[load_stage] Timeline in esecuzione, la metto in pausa prima di modificare lo stage.")
        timeline.stop()
        app.update() # Assicurati che la pausa sia effettiva e processata

    # 1. Crea un nuovo stage vuoto
    print("[load_stage] Creazione di un nuovo stage utilizzando omni.usd.get_context().new_stage()...")
    ctx = omni.usd.get_context()
    
    if not ctx.new_stage():
        print("[load_stage] ERRORE: omni.usd.get_context().new_stage() è fallito.", file=sys.stderr)
        if was_playing: timeline.play() # Ripristina se fallisce qui
        raise RuntimeError("[load_stage] Impossibile creare un nuovo stage vuoto.")
    else:
        print("[load_stage] Nuovo stage creato con successo tramite omni.usd.get_context().new_stage().")
    
    app.update() # Permetti all'applicazione di processare la creazione del nuovo stage

    # 2. Apri il file USD nello stage
    print(f"[load_stage] Apertura del file USD '{abs_file_path}' nel nuovo stage...")
    try:
        if not ctx.open_stage(abs_file_path):
            usd_error_log = ctx.get_error_log()
            error_msg = f"omni.usd.get_context().open_stage() ha restituito False per '{abs_file_path}'."
            if usd_error_log:
                error_msg += f"\nLog errori USD:\n{usd_error_log}"
            # if was_playing: timeline.play() # Considera se ripristinare in caso di fallimento
            raise RuntimeError(error_msg)
        
        app.update() # Permetti all'applicazione di processare l'apertura dello stage
        print(f"[load_stage] Stage caricato con successo da: {abs_file_path}")

    except Exception as e:
        print(f"[load_stage] ERRORE CRITICO durante il caricamento dello stage da '{abs_file_path}': {e}", file=sys.stderr)
        if was_playing: timeline.play() # Ripristina se fallisce qui
        raise RuntimeError(f"Impossibile aprire lo stage {abs_file_path} nel nuovo contesto.") from e
    
    if was_playing:
        print("[load_stage] Ripresa della timeline.")
        timeline.play()
        app.update() # Assicurati che la ripresa sia effettiva


