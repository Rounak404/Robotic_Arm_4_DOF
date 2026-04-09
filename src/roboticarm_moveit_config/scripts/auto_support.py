#!/usr/bin/env python3
import subprocess
import math
import sys
import time

SUPPORT_SDF = "/home/rounak/robot_models/support.sdf"
BOX_SDF = "/home/rounak/robot_models/box.sdf"


def spawn_support_stack(x, y, box_z, box_size=0.04, unit_height=0.01):
    """
    Spawn stacked supports under a box using 1cm blocks.
    """

    half_box = box_size / 2
    box_bottom = box_z - half_box

    # Skip if box already on ground
    if box_bottom <= 0.01:
        print("✅ Box already on ground → no support needed")
        return

    # Number of blocks required
    n = math.ceil(box_bottom / unit_height)

    print(f"🔧 Spawning {n} support blocks (0.01m each)")

    for i in range(n):
        z = (unit_height / 2) + i * unit_height

        subprocess.run([
            "ros2", "run", "ros_gz_sim", "create",
            "-name", f"support_{i}",
            "-x", str(x),
            "-y", str(y),
            "-z", str(z),
            "-file", SUPPORT_SDF
        ])

        time.sleep(0.02)  # smoother stacking


def spawn_box_above(x, y, box_z):
    """
    Spawn box slightly above to allow settling.
    """

    spawn_height = box_z + 0.2

    print(f"📦 Spawning box at z={spawn_height:.2f}")

    subprocess.run([
        "ros2", "run", "ros_gz_sim", "create",
        "-name", "my_box",
        "-x", str(x),
        "-y", str(y),
        "-z", str(spawn_height),
        "-file", BOX_SDF
    ])


def main():
    if len(sys.argv) != 4:
        print("\nUsage:")
        print("python3 auto_support.py <x> <y> <z>")
        print("\nExample:")
        print("python3 auto_support.py 0.5 0.0 0.6\n")
        return

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])

    print(f"\n📍 Target position: ({x}, {y}, {z})\n")

    # Step 1: spawn supports
    spawn_support_stack(x, y, z)

    # Step 2: spawn box
    time.sleep(0.5)
    spawn_box_above(x, y, z)

    print("\n✅ Stable stacking complete\n")


if __name__ == "__main__":
    main()