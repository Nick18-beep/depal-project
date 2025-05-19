# scene_creator.py

try:
    from isaacsim.core.api.objects import GroundPlane
    from isaacsim.core.api.materials import VisualMaterial 
    from isaacsim.core.utils.semantics import add_update_semantics 
except ImportError as e:
    print(f"ERRORE CRITICO in scene_creator.py: Impossibile importare classi da 'isaacsim.core.api'. Errore: {e}")
    GroundPlane = None
    VisualMaterial = None
import traceback


def aggiungi_pavimento_con_materiale_esistente(stage, prim_path: str = "/World/PhysicsFloor", 
                                            pbr_material_components_da_applicare=None):
    if GroundPlane is None or VisualMaterial is None:
        print("scene_creator.py: Classi GroundPlane o VisualMaterial non importate. Impossibile creare pavimento.")
        return None

    print(f"scene_creator.py: Tentativo di aggiungere un pavimento con fisica a '{prim_path}'...")
    
    visual_material_python_object = None 
    
    if pbr_material_components_da_applicare:
        pbr_mat_prim, pbr_shader_prims_list, pbr_mat_schema = pbr_material_components_da_applicare
        
        if pbr_mat_prim and pbr_mat_prim.IsValid() and \
           pbr_mat_schema and isinstance(pbr_shader_prims_list, list) :
            
            # Definisci un NUOVO e UNIVOCO percorso USD per il prim che l'oggetto VisualMaterial gestirà.
            # Questo prim VisualMaterial sarà configurato per usare il materiale PBR esistente.
            # È buona norma metterlo sotto un scope "Looks" del prim a cui sarà applicato (il pavimento).
            nuovo_prim_path_per_visual_material = f"{prim_path}/Looks/PBRMaterialReference" 

            print(f"scene_creator.py: Tentativo di creare un oggetto Python VisualMaterial.")
            print(f"  Il VisualMaterial creerà/gestirà un prim a: '{nuovo_prim_path_per_visual_material}'")
            print(f"  e sarà configurato per usare il materiale PBR esistente: '{pbr_mat_prim.GetPath()}'")
            
            try:
                # Ora usiamo tutti e 5 gli argomenti richiesti dal costruttore:
                # prim_path: dove il VisualMaterial creerà/gestirà il suo prim.
                # name: nome dell'oggetto Python.
                # prim: il Usd.Prim del materiale PBR sorgente.
                # shaders_list: la lista dei Usd.Prim shader del PBR.
                # material: lo schema UsdShade.Material del PBR.
                visual_material_python_object = VisualMaterial(
                    prim_path=nuovo_prim_path_per_visual_material,    # NUOVO: il percorso per il prim del VisualMaterial
                    name=f"{pbr_mat_prim.GetName()}_VMObjectForFloor",# Nome per l'oggetto Python
                    prim=pbr_mat_prim,                                # Prim del materiale PBR esistente
                    shaders_list=pbr_shader_prims_list,               # Lista dei prim shader del PBR
                    material=pbr_mat_schema                           # Schema UsdShade.Material del PBR
                )
                # L'oggetto VisualMaterial Python ora esiste e gestisce il prim a 'nuovo_prim_path_per_visual_material',
                # configurato per riflettere l'aspetto del materiale PBR originale.
                print(f"scene_creator.py: Oggetto Python VisualMaterial ('{visual_material_python_object.name}') creato. "
                      f"Gestisce il prim '{visual_material_python_object.prim_path}' che usa gli shader di '{pbr_mat_prim.GetPath()}'.")
            except TypeError as te_vm_ctor:
                 print(f"scene_creator.py: ERRORE DI TIPO durante l'istanza di VisualMaterial: {te_vm_ctor}")
                 traceback.print_exc()
                 visual_material_python_object = None
            except Exception as e_vm_general:
                print(f"scene_creator.py: ERRORE generico durante l'istanza di VisualMaterial: {e_vm_general}")
                traceback.print_exc()
                visual_material_python_object = None
        else:
            print("scene_creator.py: Componenti del materiale PBR forniti non validi o mancanti. "
                  "Impossibile creare l'oggetto VisualMaterial per il pavimento.")
    else:
        print("scene_creator.py: Nessun componente materiale PBR fornito per il pavimento.")
            
    # Crea l'oggetto GroundPlane
    try:
        floor_isaac_object = GroundPlane(
            prim_path=prim_path,
            name=f"{prim_path.split('/')[-1]}_isaac_obj",
            z_position=0.0,
            visual_material=visual_material_python_object # Assegna l'oggetto Python VisualMaterial
        )
        # --- Applica Semantica ---
        
        
        add_update_semantics(
                prim=floor_isaac_object.prim,
                semantic_label="floor",
                type_label="class"
            )
        
        print(f"scene_creator.py: Oggetto GroundPlane '{floor_isaac_object.prim_path}' aggiunto/creato con successo.")

        if visual_material_python_object: 
             # Il prim gestito da visual_material_python_object (a nuovo_prim_path_per_visual_material)
             # è quello che viene effettivamente assegnato e configurato.
             print(f"scene_creator.py: Al pavimento '{floor_isaac_object.prim_path}' è stato assegnato il VisualMaterial "
                   f"'{visual_material_python_object.prim_path}', che riflette l'aspetto del materiale PBR '{pbr_mat_prim.GetPath()}'.")
        elif pbr_material_components_da_applicare and not visual_material_python_object:
            print(f"scene_creator.py: AVVISO - Si è tentato di assegnare l'aspetto del materiale PBR, "
                  "ma la creazione dell'oggetto VisualMaterial è fallita. Il pavimento userà il suo aspetto di default.")
        else:
            print(f"scene_creator.py: Nessun materiale PBR specifico da assegnare al pavimento '{floor_isaac_object.prim_path}'. Userà il suo aspetto di default.")
            
        return floor_isaac_object
    except Exception as e_gp_create:
        print(f"scene_creator.py: ERRORE CRITICO durante la creazione dell'oggetto GroundPlane: {e_gp_create}")
        traceback.print_exc()
        return None