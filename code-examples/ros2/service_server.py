import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalServiceServer(Node):

    def __init__(self):
        super().__init__('minimal_service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service_server = MinimalServiceServer()

    rclpy.spin(minimal_service_server)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_service_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()