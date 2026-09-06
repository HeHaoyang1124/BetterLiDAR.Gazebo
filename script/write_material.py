from pygltflib import GLTF2


def write_material_extras(glb_path):
    gltf = GLTF2().load(glb_path)

    # material 0: cone body
    mat_body = gltf.materials[0]
    mat_body.extras = {
        "lidar_schlick": {
            "Clambda": 3,
            "r": 0.75,
            "p": 1.0,
            "is_transparent": False,
            "enable_retro": False,
            "k_retro": 0.0,
            "r_retro": 0.02,
            "p_retro": 2.0,
            "f0_retro": 0.95
        }
    }

    # material 1: cone sticker
    mat_sticker = gltf.materials[1]
    mat_sticker.extras = {
        "lidar_schlick": {
            "Clambda": 3,
            "r": 0.35,
            "p": 1.0,
            "is_transparent": False,
            "enable_retro": True,
            "k_retro": 0.45,
            "r_retro": 0.3,
            "p_retro": 2.0,
            "f0_retro": 0.95
        }
    }
    gltf.save_binary(glb_path)
    print(f"written {glb_path}")

if __name__ == "__main__":
    write_material_extras("yellow_cone.glb")
    write_material_extras("blue_cone.glb")