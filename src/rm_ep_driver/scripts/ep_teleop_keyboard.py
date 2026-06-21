#!/usr/bin/env python3
"""
RoboMaster EP 键盘遥控节点

麦轮全向移动控制（以 k 为中心的九宫格）:
  u  i  o       前左  前  前右
  j  k  l   =>  左   停  右
  m  >  .       后左  后  后右

空格: 急停  r: 切换速度档位
"""

import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist


# 按键 -> (vx, vy, vz) 映射，麦轮全向控制
KEY_BINDINGS = {
    # 九宫格方向
    'u': ( 1,  1,  0),
    'i': ( 1,  0,  0),
    'o': ( 1, -1,  0),
    'j': ( 0,  1,  0),
    'k': ( 0,  0,  0),
    'l': ( 0, -1,  0),
    'm': (-1, -1,  0),
    '>': (-1,  0,  0),
    '.': (-1,  1,  0),
}

SPEED_LEVELS = [
    (0.2, 0.2, 0.4),   # 低速
    (0.5, 0.5, 0.8),   # 中速
    (0.8, 0.8, 1.2),   # 高速
]


def get_key():
    """读取单个按键（非阻塞）"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        # 处理方向键等 ESC 序列，直接丢弃
        if ch == '\x1b':
            sys.stdin.read(2)
            return ''
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rospy.init_node('ep_teleop_keyboard', anonymous=False)

    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    speed_level = rospy.get_param('~speed_level', 1)
    speed_level = max(0, min(len(SPEED_LEVELS) - 1, speed_level))

    # 按键超时自动停车
    cmd_timeout = rospy.get_param('~cmd_timeout', 0.5)

    rate = rospy.Rate(10)
    last_key_time = rospy.Time.now()

    rospy.loginfo("=" * 40)
    rospy.loginfo("EP 键盘遥控已启动")
    rospy.loginfo("当前档位: %d (%.1f m/s, %.1f m/s, %.1f rad/s)",
                  speed_level, *SPEED_LEVELS[speed_level])
    rospy.loginfo("  u  i  o      前左 前 前右")
    rospy.loginfo("  j  k  l  =>  左  停 右")
    rospy.loginfo("  m  >  .      后左 后 后右")
    rospy.loginfo("空格: 急停  r: 换挡")
    rospy.loginfo("=" * 40)

    while not rospy.is_shutdown():
        key = get_key()

        if key == '\x03':  # Ctrl+C
            break

        if key == ' ':
            # 急停
            cmd_pub.publish(Twist())
            rospy.loginfo("急停")
            continue

        if key == 'r':
            speed_level = (speed_level + 1) % len(SPEED_LEVELS)
            rospy.loginfo("档位: %d (%.1f m/s, %.1f m/s, %.1f rad/s)",
                          speed_level, *SPEED_LEVELS[speed_level])
            continue

        if key in KEY_BINDINGS:
            vx_dir, vy_dir, vz_dir = KEY_BINDINGS[key]
            max_vx, max_vy, max_vz = SPEED_LEVELS[speed_level]

            twist = Twist()
            twist.linear.x = vx_dir * max_vx
            twist.linear.y = vy_dir * max_vy
            twist.angular.z = vz_dir * max_vz

            cmd_pub.publish(twist)
            last_key_time = rospy.Time.now()
        elif key:
            # 未识别按键，不处理
            continue

        # 超时自动停车
        if (rospy.Time.now() - last_key_time).to_sec() > cmd_timeout:
            cmd_pub.publish(Twist())

        rate.sleep()

    # 退出时停车
    cmd_pub.publish(Twist())


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
