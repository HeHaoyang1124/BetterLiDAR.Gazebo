from pygltflib import GLTF2


def check_extras(glb_path):
    print(f"\n=== {glb_path} ===")
    gltf = GLTF2().load(glb_path)
    for idx, m in enumerate(gltf.materials):
        print(f"  material [{idx}] {m.name}")
        if m.extras:
            lidar = m.extras.get("lidar_lambert", {})
            print(f"    reflectance: {lidar.get('reflectance')}")
        else:
            print("    （empty）")


if __name__ == "__main__":
    check_extras("yellow_cone.glb")
    check_extras("blue_cone.glb")