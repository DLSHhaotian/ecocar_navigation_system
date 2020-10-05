#!/usr/bin/env python

import rospy, tf, random
import tf_conversions
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *

if __name__ == '__main__':
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    print("Got it.")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    with open("/home/dlsh/catkin_soft/src/hello_ros/urdf/cube.urdf", "r") as f:
        product_xml = f.read()

    orient = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., 0.0, 0.785398))
    rospy.set_param("cube_bucket_orientRPY",[0.0,0.0,0.785398])#set param
    num_of_cubes = random.randint(2,6)
    rospy.set_param("cube_num",num_of_cubes)#set param
    for num in xrange(0,num_of_cubes):
        bin_y   =   random.uniform(0,0.5)
        bin_x   =   random.uniform(0,0.5)
        item_name   =   "cube{}".format(num)
        print("Spawning model:%s", item_name)
        item_pose   =   Pose(Point(x=bin_x, y=bin_y,    z=1),   orient)
        spawn_model(item_name, product_xml, "", item_pose, "world")
        rospy.set_param(item_name+"_XYZ",[bin_x,bin_y,1.0])

    with open("/home/dlsh/catkin_soft/src/hello_ros/urdf/bucket.urdf", "r") as f:
        product_xml = f.read()

    item_pose   =   Pose(Point(x=0.53, y=-0.23,    z=0.78),   orient)
    print("Spawning model:%s", "bucket")
    spawn_model("bucket", product_xml, "", item_pose, "world")
    rospy.set_param("bucket_XYZ",[0.53,-0.23,0.78])
