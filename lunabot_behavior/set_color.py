import rospy

from lunabot_msgs.msg import Color

rospy.init_node('color_node')

colorpub = rospy.Publisher("/errors", Color, queue_size=1)
color_msg = Color()
color_msg.color = Color.RED
colorpub.publish(color_msg)