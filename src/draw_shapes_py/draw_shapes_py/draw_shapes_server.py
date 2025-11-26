import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from draw_shapes.srv import DrawShape

class ShapeDrawer(Node):
    def __init__(self):
        super().__init__('shape_drawer_service')
        self.srv = self.create_service(DrawShape, 'draw_shape', self.draw_shape_callback)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info("Shape drawer service ready")

    def stop(self):
        self.pub.publish(Twist())

    def move_straight(self, distance, speed=1.5):
        msg = Twist()
        msg.linear.x = speed
        duration = distance / speed
        start = time.time()
        while time.time() - start < duration:
            self.pub.publish(msg)
            time.sleep(0.02)
        self.stop()

    def rotate(self, angle_rad, angular_speed=1.5):
        msg = Twist()
        msg.angular.z = angular_speed
        duration = abs(angle_rad) / angular_speed
        start = time.time()
        while time.time() - start < duration:
            self.pub.publish(msg)
            time.sleep(0.02)
        self.stop()

    def draw_circle(self, radius):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 1.0 / radius
        duration = 2 * 3.14159 * radius
        start = time.time()
        while time.time() - start < duration:
            self.pub.publish(msg)
            time.sleep(0.02)
        self.stop()
        return [radius]

    def draw_square(self, side):
        for _ in range(4):
            self.move_straight(side)
            self.rotate(1.5708)
        return [side]

    def draw_rectangle(self, w, h):
        for i in range(4):
            self.move_straight(w if i % 2 == 0 else h)
            self.rotate(1.5708)
        return [w, h]

    def draw_triangle(self, side):
        for _ in range(3):
            self.move_straight(side)
            self.rotate(2.0944)
        return [side]

    def draw_shape_callback(self, request, response):
        shape = request.shape.lower()
        params = request.params

        try:
            if shape == "circle":
                dims = self.draw_circle(params[0])
            elif shape == "square":
                dims = self.draw_square(params[0])
            elif shape == "rectangle":
                dims = self.draw_rectangle(params[0], params[1])
            elif shape == "triangle":
                dims = self.draw_triangle(params[0])
            else:
                raise ValueError("Unknown shape")

            response.success = True
            response.shape_name = shape
            response.dimensions = dims
            response.message = f"Drew {shape} with {dims}"
        except Exception as e:
            response.success = False
            response.message = str(e)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ShapeDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

