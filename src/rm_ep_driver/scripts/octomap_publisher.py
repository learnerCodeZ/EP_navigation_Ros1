#!/usr/bin/env python3
"""
从 octomap_server_static 服务获取地图，发布到话题供 RViz 消费
"""
import rospy
from octomap_msgs.srv import GetOctomap

def main():
    rospy.init_node('octomap_republisher')
    pub = rospy.Publisher('/octomap_full', rospy.AnyMsg, queue_size=1, latch=True)

    rospy.loginfo("等待 octomap_server_static 服务...")
    rospy.wait_for_service('/octomap_full')
    rospy.loginfo("服务就绪，获取地图...")

    get_octomap = rospy.ServiceProxy('/octomap_full', GetOctomap)
    try:
        resp = get_octomap()
        pub.publish(resp.map)
        rospy.loginfo("地图已发布到 /octomap_full (%d bytes)，按 Ctrl+C 退出", len(resp.map.data))
    except Exception as e:
        rospy.logerr("获取地图失败: %s", e)
        return

    rospy.spin()

if __name__ == '__main__':
    main()
