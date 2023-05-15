#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
kutu_x = 0
kutu_y = 0
kutu_z = 0.5

def kutu_spawn_et(kutu_x, kutu_y, kutu_z):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    initial_pose = Pose()
    initial_pose.position.x = kutu_x
    initial_pose.position.y = kutu_y
    initial_pose.position.z = kutu_z
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        model_name = "kutu_" + str(rospy.get_time())
        model_path = "/home/zarquon/Project/Hexapod-ROS1/src/hex/models/box/box.sdf"
        with open(model_path, "r") as f:
            model_xml = f.read()
        spawn_model(model_name, model_xml, "robotos_name_space",initial_pose, "world")
        

    except rospy.ServiceException as e:
        rospy.logerr("Spawn servis çağrısı başarısız oldu: %s" % e)

def spawn_urdf(name, description_xml, pose, reference_frame):
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf(name, description_xml, "/", pose, reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e)) 

def konum_callback(msg):
    kutu_x = msg.twist.twist.linear.x + 6
    kutu_y = msg.twist.twist.linear.y + 6
    kutu_z = msg.twist.twist.linear.z
    kutu_spawn_et(kutu_x, kutu_y, kutu_z)
    print("Kutu spawn edildi")

rospy.init_node('kutu_spawner')
sub = rospy.Subscriber('/odom', Odometry, konum_callback)
print("Kutu spawner başlatıldı")
rospy.spin()
