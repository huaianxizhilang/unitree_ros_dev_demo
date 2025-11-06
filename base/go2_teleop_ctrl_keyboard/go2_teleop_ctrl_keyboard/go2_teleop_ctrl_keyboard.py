
import sys
import threading
import time
from unitree_api.msg import Request
import rclpy
import json

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

# 视频版写法

# from rclpy.node import Node


msg = """
This node takes keypresses from the keyboard and publishes them
as unitree_api/msg/Request messages. It works best with a US keyboard layout.
---------------------------
Moving around:
  q     w     e
  a     x     d
  z     s     c

For Holonomic mode (strafing), hold down the shift key:
---------------------------
  Q     W     E
  A     X     D
  Z     S     C

anything else : stop

r/t : increase/decrease max speeds by 10%
f/g : increase/decrease only linear speed by 10%
v/b : increase/decrease only angular speed by 10%

h: Greet
j: Front Jump
k: Stretch
n: Sit Down
m: Stand Up from Sitting
y: Dance 1
u: Dance 2

CTRL-C to quit
"""
# class TeleopCtrlKeyboardNode(Node):
#     def __init__(self):
#         super().__init__('teleop_ctrl_keyboard')




# 定义常量字典
# 定义常量字典
ROBOT_SPORT_API_IDS = {
    "DAMP": 1001,          # 阻尼控制
    "BALANCESTAND": 1002, # 平衡站立
    "STOPMOVE": 1003,     # 停止运动
    "STANDUP": 1004,      # 站立
    "STANDDOWN": 1005,    # 站立下降
    "RECOVERYSTAND": 1006,# 恢复站立
    "EULER": 1007,        # 欧拉角控制
    "MOVE": 1008,         # 移动
    "SIT": 1009,          # 坐下
    "RISESIT": 1010,      # 从坐下恢复站立
    "SWITCHGAIT": 1011,   # 切换步态
    "TRIGGER": 1012,      # 触发
    "BODYHEIGHT": 1013,   # 身体高度调整
    "FOOTRAISEHEIGHT": 1014, # 脚抬起高度调整
    "SPEEDLEVEL": 1015,   # 速度级别调整
    "HELLO": 1016,        # 打招呼
    "STRETCH": 1017,      # 伸展
    "TRAJECTORYFOLLOW": 1018, # 轨迹跟随
    "CONTINUOUSGAIT": 1019,   # 连续步态
    "CONTENT": 1020,      # 内容
    "WALLOW": 1021,       # 打滚
    "DANCE1": 1022,       # 舞蹈1
    "DANCE2": 1023,       # 舞蹈2
    "GETBODYHEIGHT": 1024,# 获取身体高度
    "GETFOOTRAISEHEIGHT": 1025, # 获取脚抬起高度
    "GETSPEEDLEVEL": 1026,# 获取速度级别
    "SWITCHJOYSTICK": 1027, # 切换操纵杆
    "POSE": 1028,         # 姿态
    "SCRAPE": 1029,       # 刮擦
    "FRONTFLIP": 1030,    # 前空翻
    "FRONTJUMP": 1031,    # 前跳
    "FRONTPOUNCE": 1032   # 前扑
}

sportMode = {
    'h': ROBOT_SPORT_API_IDS["HELLO"],
    'j': ROBOT_SPORT_API_IDS["FRONTJUMP"],
    'k': ROBOT_SPORT_API_IDS["STRETCH"],
    'n': ROBOT_SPORT_API_IDS["SIT"],
    'm': ROBOT_SPORT_API_IDS["RISESIT"],
    'y': ROBOT_SPORT_API_IDS["DANCE1"],
    'u': ROBOT_SPORT_API_IDS["DANCE2"]
}


moveBindings = {
    'w': (1, 0, 0, 0),
    'e': (1, 0, 0, -1),
    'a': (0, 0, 0, 1),
    'd': (0, 0, 0, -1),
    'q': (1, 0, 0, 1),
    's': (-1, 0, 0, 0),
    'c': (-1, 0, 0, 1),
    'z': (-1, 0, 0, -1),
    'E': (1, -1, 0, 0),
    'W': (1, 0, 0, 0),
    'A': (0, 1, 0, 0),
    'D': (0, -1, 0, 0),
    'Q': (1, 1, 0, 0),
    'S': (-1, 0, 0, 0),
    'C': (-1, -1, 0, 0),
    'Z': (-1, 1, 0, 0),
}

speedBindings = {
    'r': (1.1, 1.1),
    't': (.9, .9),
    'f': (1.1, 1),
    'g': (.9, 1),
    'v': (1, 1.1),
    'b': (1, .9),
}

def getkey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s' % (speed, turn)

def publish_request(pub, api_id, x=0.0, y=0.0, z=0.0):
    req = Request()
    js = {"x": x, "y": y, "z": z}
    req.parameter = json.dumps(js)
    req.header.identity.api_id = api_id
    pub.publish(req)




def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('go2_teleop_ctrl_keyboard')
    node.declare_parameter('speed', 0.2)
    node.declare_parameter('turn', 0.4)
    pub = node.create_publisher(Request, '/api/sport/request', 10)
    
    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    speed = node.get_parameter('speed').value
    turn = node.get_parameter('turn').value
    x = 0.0
    y = 0.0
    api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"]
    th = 0.0
    status = 0.0
    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            api_id = ROBOT_SPORT_API_IDS["MOVE"]
            key = getkey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
            
                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key in sportMode.keys():
                api_id = sportMode[key]
            else:
                x = 0.0
                y = 0.0
                th = 0.0
                api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"]
                if (key == '\x03'):
                    break
            publish_request(pub, api_id, x * speed, y * speed, th * turn)
    except Exception as e:
        print(e)
    finally:
        publish_request(pub, ROBOT_SPORT_API_IDS["BALANCESTAND"])
        time.sleep(1)  # 等待1秒以确保消息发送完成，但是无法做到平衡站立
        print("已关闭，但是无法做到平衡站立...")
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
