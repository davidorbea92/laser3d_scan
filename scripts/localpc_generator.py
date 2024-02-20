#!/usr/bin/env python3

import rospy, math
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud
from sensor_msgs.msg import ChannelFloat32
from geometry_msgs.msg import Point32, Pose
from std_msgs.msg import Float64, Int8
import math
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler


scan_distance = 0.005  # Desplazamiento vertical para la generacion en metros
distance_limit=3.0 #Distancia maxima para almacenamiento de nubes de puntos


# Los angulos se consederan en radianes con 0 en el centro y a la derecha/izquierda positivos y negativos
# El rango va desde -2.3 --- 0 ---- 2.09 segun el rango del laser
#start_angle = -0.3  # Ángulo de inicio
#end_angle = 0.2  # Ángulo de finalización
start_angle=-3.1416/4
end_angle=3.1416/4
list_pcd = []

rate=0.3

class laser3d_reconstruction(object):
    def __init__(self):
        self.pc_pub = rospy.Publisher('robot/pointcloud', PointCloud, queue_size=1)
        self.scan_state=0
        self.accumulated_pc=PointCloud()
        self.count_pc=0
        self.last_scan_pos=0
        self.listener=tf.TransformListener()
        self.pose_base=Pose()
        self.list_pcd = []

    def save3d_callback(self,msg):
        np_point_cloud = np.asarray(self.list_pcd)        
        o3d_point_cloud = o3d.geometry.PointCloud()
        o3d_point_cloud.points = o3d.utility.Vector3dVector(np_point_cloud)
        o3d.io.write_point_cloud(f"~/catkin_wine/src/laser3d_scan/pcd/vid_{self.count_pc}.pcd", o3d_point_cloud, write_ascii=True)
        rospy.loginfo("3d_pc_saved")
        self.count_pc += 1

    def clear3d_callback(self,msg):
        self.accumulated_pc=PointCloud()
        self.list_pcd=[]
    
    def statescan_callback(self,msg):
        self.scan_state=msg.data

    def pc_base_callback(self,msg):
        self.pose_base=msg

    def scan_callback(self,data):
        ranges = data.ranges
        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_increment = data.angle_increment

        valid_ranges = [distance for i, distance in enumerate(ranges) if not math.isnan(distance) and start_angle <= angle_min + i * angle_increment <= end_angle]

        if self.scan_state==1:
            try:
                (trans,rot)=self.listener.lookupTransform('gripperMover','world',rospy.Time(0))     

                if (abs(self.last_scan_pos-trans[2])>=scan_distance):
                    if valid_ranges:
                        for i, distance in enumerate(valid_ranges):
                            if distance<=distance_limit:
                                angle = angle_min + i * angle_increment
                                point = Point32()
                                orientation_list = [self.pose_base.orientation.x, self.pose_base.orientation.y,self.pose_base.orientation.z,self.pose_base.orientation.w]
                                (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
                                point.x = self.pose_base.position.x+distance * math.cos(angle+yaw)
                                point.y = self.pose_base.position.y+distance * math.sin(angle+yaw)
                                point.z = -trans[2]
                                self.accumulated_pc.points.append(point)
                                self.list_pcd.append([point.x, point.y, point.z])
                        rospy.loginfo("scan_added")
                        self.last_scan_pos=trans[2]
                        channel = ChannelFloat32()
                        channel.name = "intensities"
                        channel.values = valid_ranges
                        self.accumulated_pc.channels.append(channel)
                        
                    
                    self.accumulated_pc.header = data.header
                    self.accumulated_pc.header.frame_id = "map"
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
                rospy.loginfo(str(err))
                return False
            else:
                return False

    def pc_timer(self,event=None):
        self.pc_pub.publish(self.accumulated_pc)

    def start(self):
        rospy.Subscriber('/robot/z1_scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/robot/state_scan', Int8, self.statescan_callback) # 1 para inicio, 0 para parar
        rospy.Subscriber('/robot/3dscan_save', Int8, self.save3d_callback) # topico para guardar la nube de puntos
        rospy.Subscriber('/robot/3dscan_clear', Int8, self.clear3d_callback) # topic para limpiar la nube de puntos
        rospy.Subscriber('/robot/pose', Pose, self.pc_base_callback) # punto sobre el que empezara el escaneo
        rospy.Timer(rospy.Duration(1/rate), self.pc_timer)

        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('localpc_generator')
    main=laser3d_reconstruction()
    try:
        main.start()
    except rospy.ROSInterruptException:
        pass