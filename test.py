# Moduli standard Python
import os
import asyncio
import traceback # Per una migliore diagnostica degli errori

# Classe SimulationApp di Isaac Sim
from isaacsim import SimulationApp 

# Variabili globali che verranno inizializzate dopo l'avvio di Kit
kit_app = None
asset_converter_manager = None 
usd_context = None             
asset_converter_module_ref = None 

# Variabili globali per i moduli/classi PXR, da popolare dopo l'init di Kit
Gf_global = None
UsdGeom_global = None

# --- Configurazione Utente ---
input_stl_file = r"C:\Users\cm03696\Desktop\depal project\pre_build_asset\Baby_Bunny_FREE.stl"
output_usd_file = r"C:\Users\cm03696\Desktop\depal project\pre_build_asset\Baby_Bunny_FREE.usd"
prim_path_in_stage = "/World/Imported/BabyBunny"
simulation_app_options = {"headless": False} 
initial_position_vec = (0.0, 0.0, 0.0) 
initial_scale_vec = (1.0, 1.0, 1.0)   
# --- Fine Configurazione Utente ---

async def convert_asset_to_usd_impl(source_path, destination_path):
    global asset_converter_manager, asset_converter_module_ref 

    if asset_converter_manager is None or asset_converter_module_ref is None:
        print("ERRORE INTERNO: Asset Converter Manager o il suo modulo non sono stati inizializzati correttamente.")
        return False

    print(f"Avvio conversione da '{source_path}' a '{destination_path}'...")
    if not os.path.exists(source_path):
        print(f"Errore: File di input non trovato: {source_path}")
        return False

    AssetConverterContext = asset_converter_module_ref.AssetConverterContext
    converter_options_context = AssetConverterContext()
    converter_options_context.source_path = source_path 
    converter_options_context.output_path = destination_path
    
    print(f"Chiamata a create_converter_task con source_path, destination_path, e context separati.")
    task = asset_converter_manager.create_converter_task(source_path, destination_path, converter_options_context)
    
    success = False
    try:
        success = await task.wait_until_finished() 
        if success:
            print(f"Conversione completata con successo: {destination_path}")
        else:
            detailed_status = task.get_detailed_status()
            print(f"Conversione fallita. Dettagli: {detailed_status if detailed_status else 'N/A'}")
    except Exception as e:
        print(f"Errore durante l'attesa del task di conversione: {e}")
        traceback.print_exc()
    return success    

def load_usd_in_stage_impl(usd_file_path, prim_render_path, position_tuple, scale_tuple):
    global usd_context, Gf_global, UsdGeom_global

    if usd_context is None:
        print("ERRORE INTERNO: USD Context (omni.usd) non è stato inizializzato correttamente.")
        return False
    if Gf_global is None or UsdGeom_global is None:
        print("ERRORE INTERNO: Moduli PXR (Gf, UsdGeom) non inizializzati correttamente.")
        return False
        
    if not os.path.exists(usd_file_path):
        print(f"Errore: File USD da caricare non trovato: {usd_file_path}")
        return False
    
    print(f"Caricamento di '{usd_file_path}' in '{prim_render_path}'...")
    stage = usd_context.get_stage()
    if not stage:
        print("Errore: Stage USD non trovato nel contesto.")
        return False

    position = Gf_global.Vec3d(*position_tuple)
    scale = Gf_global.Vec3f(*scale_tuple)

    parent_path_str, _ = os.path.split(prim_render_path)
    if parent_path_str and not stage.GetPrimAtPath(parent_path_str):
        UsdGeom_global.Xform.Define(stage, parent_path_str)

    ref_prim_usd = UsdGeom_global.Xform.Define(stage, prim_render_path).GetPrim()
    if not ref_prim_usd.IsValid():
        print(f"Errore: Impossibile definire o ottenere un prim valido a '{prim_render_path}'.")
        return False
        
    ref_prim_usd.GetReferences().AddReference(assetPath=usd_file_path)
    print(f"Riferimento a '{usd_file_path}' aggiunto al prim '{prim_render_path}'.")

    # --- USA XformCommonAPI PER IMPOSTARE LE TRASFORMAZIONI ---
    xform_common_api = UsdGeom_global.XformCommonAPI(ref_prim_usd)
    
    xform_common_api.SetTranslate(position) 
    print(f"'xformOp:translate' (o equivalente) impostato su {position} usando XformCommonAPI.")
    
    xform_common_api.SetScale(scale)
    print(f"'xformOp:scale' (o equivalente) impostato su {scale} usando XformCommonAPI.")
    
    # Opzionale: per resettare la rotazione a identità se XformCommonAPI ne crea una
    # default_rotation_euler_xyz = Gf_global.Vec3f(0,0,0) 
    # xform_common_api.SetRotate(default_rotation_euler_xyz, UsdGeom_global.XformCommonAPI.RotationOrderXYZ)
    # print(f"Rotazione impostata su {default_rotation_euler_xyz} usando XformCommonAPI.")

    print(f"Trasformazioni applicate a '{prim_render_path}' usando XformCommonAPI.")
    return True
    # --- FINE MODIFICA ---

async def main_logic_task(): 
    global kit_app, asset_converter_manager, usd_context, asset_converter_module_ref
    global Gf_global, UsdGeom_global

    is_success_overall = False
    try:
        print("Task principale (main_logic_task) avviato.")
        
        for _ in range(10): 
            await asyncio.sleep(0.01) 

        print("Importazione moduli PXR e Kit specifici...")
        try:
            from pxr import UsdGeom, Gf 
            Gf_global = Gf             
            UsdGeom_global = UsdGeom   
            print("Moduli PXR (UsdGeom, Gf) importati con successo.")

            import omni.kit.app 
            import omni.usd     
            usd_context = omni.usd.get_context()
            print("Moduli 'omni.kit.app' e 'omni.usd' importati e contesto USD ottenuto.")

        except ImportError as e:
            print(f"ERRORE CRITICO: Impossibile importare moduli PXR o Kit essenziali: {e}")
            traceback.print_exc()
            return 

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        asset_converter_ext_id = "omni.kit.asset_converter"

        if not ext_manager.is_extension_enabled(asset_converter_ext_id):
            print(f"Abilitazione estensione '{asset_converter_ext_id}'...")
            ext_manager.set_extension_enabled_immediate(asset_converter_ext_id, True)
            print(f"Attesa caricamento estensione '{asset_converter_ext_id}'...")
            loaded = False
            for i in range(20): 
                await asyncio.sleep(0.1) 
                if ext_manager.is_extension_loaded(asset_converter_ext_id):
                    loaded = True
                    print(f"Estensione '{asset_converter_ext_id}' caricata dopo { (i+1)*0.1:.1f} secondi.")
                    break
            if not loaded:
                print(f"ATTENZIONE: Estensione '{asset_converter_ext_id}' potrebbe non essersi caricata completamente.")
        else:
            print(f"Estensione '{asset_converter_ext_id}' già abilitata.")
        
        try:
            import omni.kit.asset_converter 
            asset_converter_module_ref = omni.kit.asset_converter 
            asset_converter_manager = asset_converter_module_ref.get_instance() 

            if asset_converter_manager is None:
                print(f"ERRORE CRITICO: Impossibile ottenere l'istanza di Asset Converter Manager ('{asset_converter_ext_id}').")
                return 
            print(f"Modulo '{asset_converter_ext_id}' importato e istanza del manager ottenuta.")
        except ImportError as e:
            print(f"ERRORE CRITICO: Impossibile importare il modulo '{asset_converter_ext_id}': {e}")
            traceback.print_exc()
            return
        except Exception as e: 
            print(f"ERRORE CRITICO: Durante l'ottenimento di Asset Converter Manager: {e}")
            traceback.print_exc()
            return
        
        conversion_success = await convert_asset_to_usd_impl(input_stl_file, output_usd_file)
        if not conversion_success:
            print("Conversione fallita. Uscita dal task.")
            return

        for _ in range(5): 
            await asyncio.sleep(0.1) 

        load_success = load_usd_in_stage_impl(output_usd_file, prim_path_in_stage, 
                                               initial_position_vec, initial_scale_vec)
        if load_success:
            print("Asset STL importato e caricato con successo!")
            is_success_overall = True 
            if not simulation_app_options["headless"]:
                print("L'applicazione resterà attiva (gestita dal loop esterno in __main__).")
        else:
            print("Caricamento dell'asset USD fallito.")
        
    except Exception as e:
        print(f"ERRORE CATASTROFICO nel task principale (main_logic_task): {e}")
        traceback.print_exc()
    finally:
        if simulation_app_options["headless"] and is_success_overall:
            print("Operazione headless completata con successo. Chiusura dell'app dal task.")
            if kit_app and kit_app.is_running(): kit_app.close()
        elif not is_success_overall and kit_app and kit_app.is_running(): 
            print("Errore nel task o operazione non riuscita, chiusura dell'app dal task.")
            if kit_app and kit_app.is_running(): kit_app.close()


if __name__ == "__main__":
    print("Avvio script da VSCode (o altra console Python)...")
    
    try:
        kit_app = SimulationApp(launch_config=simulation_app_options)
        print("Omniverse Kit Application Inizializzata (da __main__).")
        
        for _ in range(5): 
             kit_app.update() 

        asyncio.ensure_future(main_logic_task())
        
        while kit_app.is_running():
            kit_app.update() 

        print("Il loop principale di Kit Application è terminato (kit_app.is_running() è False).")

    except KeyboardInterrupt:
        print("Rilevato KeyboardInterrupt (Ctrl+C), chiusura...")
    except Exception as e: 
        print(f"Errore fatale durante l'inizializzazione o nel loop principale di __main__: {e}")
        traceback.print_exc()
    finally:
        if kit_app is not None and kit_app.is_running():
            print("Chiusura Omniverse Kit Application (dal blocco finally di __main__)...")
            kit_app.close()
        print("Script terminato.")