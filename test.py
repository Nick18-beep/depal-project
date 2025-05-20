# Moduli standard Python
import os
import asyncio
import traceback

# Classe SimulationApp di Isaac Sim
from isaacsim import SimulationApp

# Variabili globali
kit_app = None
asset_converter_manager = None
usd_context = None
asset_converter_module_ref = None
Gf_global = None
UsdGeom_global = None
Usd_global = None

# --- Configurazione Utente ---
input_stl_file = r"C:\Users\cm03696\Desktop\depal project\pre_build_asset\Cube.stl"
output_usd_file = r"C:\Users\cm03696\Desktop\depal project\pre_build_asset\result2_centered.usd"
prim_path_in_stage = "/World/Imported/CenteredObject"
simulation_app_options = {"headless": False}
initial_scale_vec = (1.0, 1.0, 1.0)
# --- Fine Configurazione Utente ---

async def convert_asset_to_usd_impl(source_path, destination_path):
    global asset_converter_manager, asset_converter_module_ref
    if asset_converter_manager is None or asset_converter_module_ref is None:
        print("ERRORE INTERNO: Asset Converter non inizializzato.")
        return False
    if not os.path.exists(source_path):
        print(f"Errore: File di input non trovato: {source_path}")
        return False

    print(f"Avvio conversione da '{source_path}' a '{destination_path}'...")
    AssetConverterContext = asset_converter_module_ref.AssetConverterContext
    converter_options_context = AssetConverterContext()
    converter_options_context.source_path = source_path
    converter_options_context.output_path = destination_path
    task = asset_converter_manager.create_converter_task(source_path, destination_path, converter_options_context)

    success = False
    try:
        success = await task.wait_until_finished()
        if success:
            print(f"Conversione completata: {destination_path}")
        else:
            print(f"Conversione fallita. Dettagli: {task.get_detailed_status() or 'N/A'}")
    except Exception as e:
        print(f"Errore task di conversione: {e}")
        traceback.print_exc()
    return success

def load_usd_in_stage_impl(usd_file_path, prim_render_path, scale_tuple):
    global usd_context, Gf_global, UsdGeom_global, Usd_global
    if not all([usd_context, Gf_global, UsdGeom_global, Usd_global]):
        print("ERRORE INTERNO: Moduli PXR o USD Context non inizializzati.")
        return False
    if not os.path.exists(usd_file_path):
        print(f"Errore: File USD da caricare non trovato: {usd_file_path}")
        return False

    print(f"Caricamento e centratura di '{usd_file_path}' in '{prim_render_path}'...")
    stage = usd_context.get_stage()
    if not stage:
        print("Errore: Stage USD non trovato.")
        return False

    parent_path_str, _ = os.path.split(prim_render_path)
    if parent_path_str and not stage.GetPrimAtPath(parent_path_str):
        UsdGeom_global.Xform.Define(stage, parent_path_str)

    xform_prim_schema = UsdGeom_global.Xform.Define(stage, prim_render_path)
    ref_prim_usd = xform_prim_schema.GetPrim()
    if not ref_prim_usd.IsValid():
        print(f"Errore: Impossibile definire prim a '{prim_render_path}'.")
        return False

    ref_prim_usd.GetReferences().AddReference(assetPath=usd_file_path)
    xformable = UsdGeom_global.Xformable(ref_prim_usd)
    xformable.ClearXformOpOrder()

    scale_val = Gf_global.Vec3f(*scale_tuple)
    scale_op = xformable.AddScaleOp(UsdGeom_global.XformOp.PrecisionFloat)
    scale_op.Set(scale_val)
    print(f"'xformOp:scale' impostato su {scale_val}.")

    imageable = UsdGeom_global.Imageable(ref_prim_usd)
    time_code = Usd_global.TimeCode.Default()
    purpose_token = UsdGeom_global.Tokens.default_
    world_bbox = imageable.ComputeWorldBound(time_code, purpose_token)

    calculated_translation_vec = Gf_global.Vec3d(0.0, 0.0, 0.0)
    # --- CORREZIONE QUI ---
    if world_bbox.GetRange().IsEmpty():
    # --- FINE CORREZIONE ---
        print(f"ATTENZIONE: Bounding box per '{prim_render_path}' è vuota. Posizionato a (0,0,0).")
    else:
        bbox_range= world_bbox.GetRange()
        bbox_center_world = (bbox_range.GetMin() + bbox_range.GetMax()) / 2.0
        calculated_translation_vec = -bbox_center_world
        print(f"Centro BBox: {bbox_center_world}, Traslazione per centrare: {calculated_translation_vec}")

    translate_op = xformable.AddTranslateOp(UsdGeom_global.XformOp.PrecisionDouble)
    translate_op.Set(calculated_translation_vec)
    print(f"'xformOp:translate' impostato su {calculated_translation_vec} per centrare.")

    print(f"Trasformazioni (scala e centratura) applicate a '{prim_render_path}'.")
    return True

async def main_logic_task():
    global kit_app, asset_converter_manager, usd_context, asset_converter_module_ref
    global Gf_global, UsdGeom_global, Usd_global

    is_success_overall = False
    try:
        print("Task principale avviato.")
        await asyncio.sleep(0.1)

        print("Importazione moduli PXR e Kit...")
        try:
            from pxr import UsdGeom, Gf, Usd
            Gf_global, UsdGeom_global, Usd_global = Gf, UsdGeom, Usd
            import omni.kit.app
            import omni.usd
            usd_context = omni.usd.get_context()
            print("Moduli PXR e Kit importati.")
        except ImportError as e:
            print(f"ERRORE CRITICO: Impossibile importare moduli essenziali: {e}")
            traceback.print_exc()
            return

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        asset_converter_ext_id = "omni.kit.asset_converter"
        if not ext_manager.is_extension_enabled(asset_converter_ext_id):
            print(f"Abilitazione estensione '{asset_converter_ext_id}'...")
            ext_manager.set_extension_enabled_immediate(asset_converter_ext_id, True)
            loaded = False
            for i in range(20):
                await asyncio.sleep(0.1)
                if ext_manager.is_extension_loaded(asset_converter_ext_id):
                    loaded = True; print(f"Estensione '{asset_converter_ext_id}' caricata."); break
            if not loaded: print(f"ATTENZIONE: Estensione '{asset_converter_ext_id}' potrebbe non essersi caricata.")
        else:
            print(f"Estensione '{asset_converter_ext_id}' già abilitata.")

        try:
            import omni.kit.asset_converter
            asset_converter_module_ref = omni.kit.asset_converter
            asset_converter_manager = asset_converter_module_ref.get_instance()
            if asset_converter_manager is None:
                print("ERRORE CRITICO: Impossibile ottenere Asset Converter Manager.")
                return
            print("Asset Converter Manager ottenuto.")
        except Exception as e:
            print(f"ERRORE CRITICO ottenendo Asset Converter Manager: {e}")
            traceback.print_exc()
            return

        conversion_success = await convert_asset_to_usd_impl(input_stl_file, output_usd_file)
        if not conversion_success:
            print("Conversione fallita. Uscita."); return

        await asyncio.sleep(0.5)

        load_success = load_usd_in_stage_impl(output_usd_file, prim_path_in_stage, initial_scale_vec)
        if load_success:
            print("Asset importato, scalato e centrato con successo!")
            is_success_overall = True
            if not simulation_app_options["headless"]:
                print("Applicazione attiva. Chiudere la finestra per terminare.")
        else:
            print("Caricamento/centratura dell'asset USD fallito.")

    except Exception as e:
        print(f"ERRORE CATASTROFICO nel task principale: {e}")
        traceback.print_exc()
    finally:
        if simulation_app_options["headless"] and is_success_overall:
            print("Operazione headless completata. Chiusura app.")
            if kit_app and kit_app.is_running(): kit_app.close()
        elif not is_success_overall:
            print("Errore o operazione non riuscita. Chiusura app.")
            if kit_app and kit_app.is_running(): kit_app.close()

if __name__ == "__main__":
    print("Avvio script Python...")
    try:
        kit_app = SimulationApp(launch_config=simulation_app_options)
        print("Omniverse Kit Application Inizializzata.")
        for _ in range(5): kit_app.update()

        asyncio.ensure_future(main_logic_task())
        while kit_app.is_running():
            kit_app.update()
        print("Loop principale Kit Application terminato.")
    except KeyboardInterrupt:
        print("KeyboardInterrupt rilevato, chiusura...")
    except Exception as e:
        print(f"Errore fatale in __main__: {e}")
        traceback.print_exc()
    finally:
        if kit_app is not None and kit_app.is_running():
            print("Chiusura Omniverse Kit Application (finally __main__)...")
            kit_app.close()
        print("Script terminato.")