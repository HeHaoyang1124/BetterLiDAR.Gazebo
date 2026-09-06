from pygltflib import GLTF2

def clear_extras(glb_path):
    gltf = GLTF2().load(glb_path)
    for m in gltf.materials:
        m.extras = {}
    gltf.save_binary(glb_path)
    print(f"{glb_path} cleared")

if __name__ == "__main__":
    clear_extras("yellow_cone.glb")
    clear_extras("blue_cone.glb")
