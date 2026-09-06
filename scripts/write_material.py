from pygltflib import GLTF2


def write_material_extras(glb_path):
    gltf = GLTF2().load(glb_path)

    # material 0: cone body (high reflectance)
    mat_body = gltf.materials[0]
    mat_body.extras = {
        "lidar_lambert": {
            "reflectance": 0.14
        }
    }

    # material 1: cone sticker (low reflectance)
    mat_sticker = gltf.materials[1]
    mat_sticker.extras = {
        "lidar_lambert": {
            "reflectance": 0.57
        }
    }
    gltf.save_binary(glb_path)
    print(f"written {glb_path}")

if __name__ == "__main__":
    write_material_extras("yellow_cone.glb")
    write_material_extras("blue_cone.glb")