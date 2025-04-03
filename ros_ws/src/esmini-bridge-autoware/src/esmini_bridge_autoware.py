import rclpy
from rclpy.executors import MultiThreadedExecutor

from .World import World


def main():
    rclpy.init()

    world = World()

    executor = MultiThreadedExecutor()
    executor.add_node(world)
    executor.add_node(world.ego_controller)

    try:
        executor.spin()
    finally:
        world.ego_controller.destroy_node()
        world.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
