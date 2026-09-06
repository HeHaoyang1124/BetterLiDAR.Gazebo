from pygltflib import GLTF2


def check_extras(glb_path):
    print(f"\n=== {glb_path} ===")
    gltf = GLTF2().load(glb_path)
    for idx, m in enumerate(gltf.materials):
        print(f"  material [{idx}] {m.name}")
        if m.extras:
            lidar = m.extras.get("lidar_schlick", {})
            print(f"    Clambda:        {lidar.get('Clambda')}")
            print(f"    r:              {lidar.get('r')}")
            print(f"    is_transparent: {lidar.get('is_transparent')}")
            print(f"    enable_retro:   {lidar.get('enable_retro')}")
            print(f"    k_retro:        {lidar.get('k_retro')}")
            print(f"    r_retro:        {lidar.get('r_retro')}")
        else:
            print("    （empty）")


if __name__ == "__main__":
    check_extras("yellow_cone.glb")
    check_extras("blue_cone.glb")