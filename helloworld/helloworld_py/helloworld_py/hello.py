""""
  需求：编写程序，控制机器狗打招呼
  流程：
  1.创建发布对象
  2.创建定时器
  3.定时器中，调用发布对象发布速度指令

"""


# 1.导包；
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request

# 3.自定义节点类；
class Go2Hello(Node):
    def __init__(self):
        super().__init__("go2_hello_node.py")
        self.req_puber = self.create_publisher(Request,"/api/sport/request",10)
        self.timer = self.create_timer(1.0,self.hello)

    def hello(self):
        request = Request()
        request.header.identity.api_id = 1016
        self.req_puber.publish(request)

def main():
    # 2.初始化ROS2客户端；
    rclpy.init()
    # 4.调用spin函数，并传入节点对象；
    rclpy.spin(Go2Hello())
    # 5.资源释放。
    rclpy.shutdown()

if __name__ == '__main__':
    main()
