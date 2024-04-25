import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen, Spawn, Kill
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios
import time

class DriverNode(Node):
    def __init__(self):
        super().__init__('driver_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')

        self.pen_r = 255
        self.pen_g = 210
        self.pen_b = 20
        self.pen_width = 15
        self.pen_off = False

        self.spawn_turtle()
        self.set_initial_pen_settings()

    def spawn_turtle(self):
        self.get_logger().info('Aguardando serviço de Spawn...')
        self.spawn_client.wait_for_service()
        spawn_request = Spawn.Request()
        spawn_request.x = 20.0
        spawn_request.y = 20.0
        spawn_request.theta = 0.0
        spawn_request.name = 'turtle2'
        future = self.spawn_client.call_async(spawn_request)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            self.turtle_name = response.name
            self.get_logger().info(f'Tartura de nome: {self.turtle_name} spawnada')
        except Exception as e:
            self.get_logger().error('Falha na hora de spawnar: %r' % (e,))

    def timer_callback(self):
        msg = Twist()
        linear_vel = 1.3
        radius = 0.7
        msg.linear.x = linear_vel
        msg.linear.y = 0.0
        msg.angular.z = linear_vel / radius
        self.publisher_.publish(msg)

    def set_initial_pen_settings(self):
        self.get_logger().info('Aguardando serviço de SetPen...')
        self.pen_client.wait_for_service()
        req = SetPen.Request()
        req.r = self.pen_r
        req.g = self.pen_g
        req.b = self.pen_b
        req.width = self.pen_width
        req.off = self.pen_off
        future = self.pen_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            future.result()
            self.get_logger().info('Configurações da caneta setadas com sucesso.')
            self.get_logger().info('Para matar a tartaruga, basta pressionar a tecla "Q"')
        except Exception as e:
            self.get_logger().error('Setagem da caneta falhada: %r' % (e,))

    def kill_turtle(self):
        if rclpy.ok():
            self.get_logger().info('Esperando serviço de Kill...')
            self.kill_client.wait_for_service()
            kill_request = Kill.Request()
            kill_request.name = self.turtle_name
            future = self.kill_client.call_async(kill_request)
            rclpy.spin_until_future_complete(self, future)
            try:
                future.result()
                self.get_logger().info(f'Tartaruga nome {self.turtle_name} foi morta. Fechando este terminal em 15 segundos')
            except Exception as e:
                self.get_logger().error('Falha na morte: %r' % (e,))
        else:
            self.get_logger().info('ROS está fechando, sem necessidade de matar a tartaruga')

def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    shutdown_called = False
    original_settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0)[0]:
                if sys.stdin.read(1) == 'q':
                    node.get_logger().info('Tecla Q pressionada, matando a tartaruga...')
                    node.kill_turtle()
                    time.sleep(15)
                    break
            rclpy.spin_once(node, timeout_sec=0.1) 
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
        if not shutdown_called:
            shutdown_called = True
            rclpy.shutdown()

if __name__ == '__main__':
    main()
