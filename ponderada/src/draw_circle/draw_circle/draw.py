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
    # Classe que controla a tartaruga
    def __init__(self):
        super().__init__('driver_node')
        # Criação dos clientes para os serviços
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')

        # Variáveis de controle da caneta
        self.pen_r = 255
        self.pen_g = 210
        self.pen_b = 20
        self.pen_width = 15
        self.pen_off = False

        # Spawna a tartaruga
        self.spawn_turtle()

    # Método para spawnar a tartaruga
    def spawn_turtle(self):
        self.get_logger().info('Aguardando serviço de Spawn...')
        self.spawn_client.wait_for_service()
        # Criação do request para o serviço de spawn
        spawn_request = Spawn.Request()
        # Configuração da posição inicial da tartaruga
        spawn_request.x = 2.0
        spawn_request.y = 2.0
        spawn_request.theta = 0.0
        spawn_request.name = 'turtle2'
        future = self.spawn_client.call_async(spawn_request)
        rclpy.spin_until_future_complete(self, future)
        try:
            # Verifica se o serviço foi executado com sucesso
            response = future.result()
            self.turtle_name = response.name
            self.get_logger().info(f'Tartaruga de nome: {self.turtle_name} spawnada')
            self.setup_turtle_controls()
            # Se o serviço falhar, printa o erro
        except Exception as e:
            self.get_logger().error('Falha na hora de spawnar: %r' % (e,))

    # Método para configurar os controles da tartaruga
    def setup_turtle_controls(self):
        self.publisher_ = self.create_publisher(Twist, f'/{self.turtle_name}/cmd_vel', 10)
        self.pen_client = self.create_client(SetPen, f'/{self.turtle_name}/set_pen')
        self.timer = self.create_timer(1, self.timer_callback)
        self.set_initial_pen_settings()

    # Método para controlar a tartaruga
    def timer_callback(self):
        msg = Twist()
        linear_vel = 1.3
        radius = 0.7
        msg.linear.x = linear_vel
        msg.linear.y = 0.0
        msg.angular.z = linear_vel / radius
        self.publisher_.publish(msg)

    # Método para setar as configurações iniciais da caneta
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
            self.get_logger().info('Para matar a tartaruga, pressione a tecla (Q)')
        except Exception as e:
            self.get_logger().error('Setagem da caneta falhada: %r' % (e,))

    # Método para matar a tartaruga
    def kill_turtle(self):
        self.get_logger().info('Esperando serviço de Kill...')
        self.kill_client.wait_for_service()
        kill_request = Kill.Request()
        kill_request.name = self.turtle_name
        future = self.kill_client.call_async(kill_request)
        rclpy.spin_until_future_complete(self, future)
        try:
            future.result()
            self.get_logger().info(f'Tartaruga de nome {self.turtle_name} foi morta. Fechando este terminal em 15 segundos')
        except Exception as e:
            self.get_logger().error(f'Falha na morte: {e}')

# Função principal
def main(args=None):
    # Inicialização do ROS
    rclpy.init(args=args)
    # Criação do nó
    node = DriverNode()
    original_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            # Verifica se a tecla Q foi pressionada
            if select.select([sys.stdin], [], [], 0)[0]:
                # Se sim, mata a tartaruga
                if sys.stdin.read(1) == 'q':
                    node.get_logger().info('Tecla Q pressionada, matando a tartaruga...')
                    node.kill_turtle()
                    time.sleep(15)
                    break
            rclpy.spin_once(node, timeout_sec=0.1)
            # Se a tartaruga for morta, fecha o terminal
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
