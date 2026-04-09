# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from moveit_msgs.msg import CollisionObject
# from moveit_msgs.srv import GetPlanningScene
# from shape_msgs.msg import SolidPrimitive
# from geometry_msgs.msg import Pose
# from std_msgs.msg import Header
# from gazebo_msgs.srv import SpawnEntity
# import os
# import time

# class SceneManager(Node):
#     def __init__(self):
#         super().__init__('scene_manager')

#         # Publisher to MoveIt planning scene
#         self.scene_pub = self.create_publisher(
#             CollisionObject, '/collision_object', 10)

#         # Gazebo spawn client
#         self.spawn_client = self.create_client(
#             SpawnEntity, '/spawn_entity')
#         self.spawn_client.wait_for_service(timeout_sec=5.0)

#     def add_box_to_scene(self, name, x, y, z, size_x=0.05, size_y=0.05, size_z=0.05):
#         """Add a box to BOTH Gazebo and MoveIt planning scene."""

#         # --- 1. Spawn in Gazebo ---
#         sdf = f"""
#         <sdf version='1.6'>
#           <model name='{name}'>
#             <static>true</static>
#             <link name='link'>
#               <visual name='visual'>
#                 <geometry><box><size>{size_x} {size_y} {size_z}</size></box></geometry>
#                 <material><ambient>1 0 0 1</ambient></material>
#               </visual>
#               <collision name='collision'>
#                 <geometry><box><size>{size_x} {size_y} {size_z}</size></box></geometry>
#               </collision>
#             </link>
#           </model>
#         </sdf>"""
#         req = SpawnEntity.Request()
#         req.name = name
#         req.xml = sdf
#         req.initial_pose.position.x = x
#         req.initial_pose.position.y = y
#         req.initial_pose.position.z = z
#         future = self.spawn_client.call_async(req)
#         rclpy.spin_until_future_complete(self, future)

#         if future.result() is not None:
#             self.get_logger().info(f'✅ Spawned {name} in Gazebo')
#         else:
#             self.get_logger().error(f'❌ Failed to spawn {name}')

#         # --- 2. Add to MoveIt planning scene ---
#         co = CollisionObject()
#         co.header = Header()
#         co.header.frame_id = 'world'
#         co.id = name

#         box = SolidPrimitive()
#         box.type = SolidPrimitive.BOX
#         box.dimensions = [size_x, size_y, size_z]

#         pose = Pose()
#         pose.position.x = x
#         pose.position.y = y
#         pose.position.z = z
#         pose.orientation.w = 1.0

#         co.primitives = [box]
#         co.primitive_poses = [pose]
#         co.operation = CollisionObject.ADD

#         self.scene_pub.publish(co)
#         self.get_logger().info(f'Added {name} at ({x}, {y}, {z})')

#     def add_table(self):
#         """Add a table as a flat box."""
#         self.add_box_to_scene('table', x=0.5, y=0.0, z=0.4,
#                                size_x=0.6, size_y=0.8, size_z=0.02)

#     def add_target_object(self):
#         """Add the object to be picked."""
#         self.add_box_to_scene('target_box', x=0.5, y=0.0, z=0.445,
#                                size_x=0.05, size_y=0.05, size_z=0.05)


# def main():
#     rclpy.init()
#     node = SceneManager()
#     node.add_table()
#     time.sleep(1.0)
#     node.add_target_object()
#     rclpy.spin_once(node, timeout_sec=2.0)
#     node.destroy_node()
#     rclpy.shutdown()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import time


class SceneManager(Node):
    def __init__(self):
        super().__init__('scene_manager')

        # Publisher to MoveIt planning scene
        self.scene_pub = self.create_publisher(
            CollisionObject, '/collision_object', 10)

    def add_box_to_scene(self, name, x, y, z,
                         size_x=0.05, size_y=0.05, size_z=0.05):
        """Add a box ONLY to MoveIt planning scene."""

        co = CollisionObject()
        co.header = Header()

        # ✅ IMPORTANT: match your robot frame
        co.header.frame_id = 'base_link'

        co.id = name

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [size_x, size_y, size_z]

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        co.primitives = [box]
        co.primitive_poses = [pose]
        co.operation = CollisionObject.ADD

        # Publish multiple times (important for reliability)
        for _ in range(5):
            self.scene_pub.publish(co)
            time.sleep(0.1)

        self.get_logger().info(
            f'📦 Added {name} at ({x:.2f}, {y:.2f}, {z:.2f})')

    def add_table(self):
        """Add a table."""
        self.add_box_to_scene(
            'table',
            x=0.5, y=0.0, z=0.4,
            size_x=0.6, size_y=0.8, size_z=0.02
        )

    def add_target_object(self):
        """Add pick object."""
        self.add_box_to_scene(
            'target_box',
            x=0.5, y=0.0, z=0.445,
            size_x=0.05, size_y=0.05, size_z=0.05
        )


def main():
    rclpy.init()
    node = SceneManager()

    # Give time for MoveIt to initialize
    time.sleep(2.0)

    node.add_table()
    time.sleep(0.5)

    node.add_target_object()
    time.sleep(0.5)

    node.get_logger().info("✅ Scene ready!")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()