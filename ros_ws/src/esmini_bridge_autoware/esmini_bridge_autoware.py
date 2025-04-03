import rclpy

from .World import World


def main():
    rclpy.init()
    world = World()
    rclpy.spin(world)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
