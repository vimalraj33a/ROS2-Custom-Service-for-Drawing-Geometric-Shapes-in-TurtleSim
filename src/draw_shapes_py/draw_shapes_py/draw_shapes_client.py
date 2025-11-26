import rclpy
from rclpy.node import Node
from draw_shapes.srv import DrawShape
import sys

class ShapeClient(Node):
    def __init__(self):
        super().__init__('shape_client')
        self.cli = self.create_client(DrawShape, 'draw_shape')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service...")

    def send_request(self, shape, params):
        req = DrawShape.Request()
        req.shape = shape
        req.params = params
        return self.cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    client = ShapeClient()

    if len(sys.argv) < 2:
        print("Usage: draw_shapes_client <shape> <params...>")
        return

    shape = sys.argv[1]
    params = [float(p) for p in sys.argv[2:]]

    future = client.send_request(shape, params)
    rclpy.spin_until_future_complete(client, future)

    if future.result():
        r = future.result()
        print("Response:")
        print(" Success:", r.success)
        print(" Shape:", r.shape_name)
        print(" Dimensions:", r.dimensions)
        print(" Message:", r.message)
    else:
        print("Service call failed")

    client.destroy_node()
    rclpy.shutdown()

