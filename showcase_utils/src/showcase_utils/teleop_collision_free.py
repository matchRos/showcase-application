#!/usr/bin/env python
from ps4_controller.PlaystationHandler import PlayStationHandler
from rospy import rostime
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import Twist,TwistStamped, TransformStamped
from sensor_msgs.msg import LaserScan
import rospy
import laser_geometry.laser_geometry as lg
import tf
#import ros_numpy
import math


class PlayStationDiffDrive(PlayStationHandler):
    def __init__(self,message_type):
        PlayStationHandler.__init__(self)
        
        self.listener = tf.TransformListener()
        self.lp = lg.LaserProjection()
        self.rate=rospy.Rate(rospy.get_param("~rate",10))
        self.active_robot = 0
        self.transform = TransformStamped()
        self.speed_translation = rospy.get_param("~translation",0.1)
        self.speed_rotation =  rospy.get_param("~rotation",0.2)
        self.trans_incr=rospy.get_param("~trans_incr",0.1)
        self.rot_incr=rospy.get_param("~rot_incr",0.1)
        self.robotnames = rospy.get_param("~robot_names","")
        print("test"+self.robotnames)
        self.cmd_vel_topic_prefix = rospy.get_param("~cmd_vel_topic_prefix","")
        self.callbackList=[   self.decreaseTrans,
                                self.increaseRot,
                                self.decreaseRot,
                                self.increaseTrans,
                                self.dummy,
                                self.dummy,
                                self.dummy,
                                self.changeRobot
                                ]

        self.translation = float()
        self.rotation = float()
        self.initialized = False
        self.lower_position_reached = False
        
        self.publishFunction=None        
            
        if message_type==Twist:
            print("Publishing as Twist")
            self.publishFunction=self.publishTwist
        elif  message_type==TwistStamped:
            print("Publishing as TwistStamped")
            self.publishFunction=self.publishTwistStamped
        
        self.publisher_stack = []  
        self.f_scan_subscriber_stack = []
        if self.robotnames == "":
            self.publisher_stack.append(rospy.Publisher(self.cmd_vel_topic_prefix + "/cmd_vel",message_type,queue_size= 10))
            self.f_scan_subscriber_stack.append(rospy.Subscriber("f_scan" , LaserScan, self.f_scan_cb))
            self.f_scan_subscriber_stack.append(rospy.Subscriber("b_scan" , LaserScan, self.b_scan_cb))
        else:
            for i in self.robotnames: 
                self.publisher_stack.append(rospy.Publisher(i+"/" + self.cmd_vel_topic_prefix + "/cmd_vel",message_type,queue_size= 10))
                self.f_scan_subscriber_stack.append(rospy.Subscriber(i+"/f_scan" , LaserScan, self.f_scan_cb))
                self.f_scan_subscriber_stack.append(rospy.Subscriber(i+"/b_scan" , LaserScan, self.b_scan_cb))
                
                
        self.listener.waitForTransform("base_link", "front_laser_link", rospy.Time(), rospy.Duration(4.0))
        (self.trans,self.rot) = self.listener.lookupTransform("base_link","front_laser_link", rostime.Time(0))
        (self.trans_back,self.rot_back) = self.listener.lookupTransform("base_link","back_laser_link", rostime.Time(0))
        #euler = tf.transformations.euler_from_quaternion(self.rot)
        #print(self.trans,euler[2]/math.pi*180)
                

    def dummy(self):
        pass

    def increaseRot(self):
        print("Increasing rot")
        self.speed_rotation=self.speed_rotation * (1+self.rot_incr)
        
      
    def increaseTrans(self):
        print("Increasing trans")
        self.speed_translation=self.speed_translation * (1+self.trans_incr)
    
    def decreaseRot(self):
        print("Decreasing rot")
        self.speed_rotation=self.speed_rotation* (1- self.rot_incr) 
        if self.speed_rotation<0.0:
            self.speed_rotation=0.0
      
    def decreaseTrans(self):
        print("Decreasing trans")
        self.speed_translation=self.speed_translation * (1-self.trans_incr)
        if  self.speed_translation<0.0:
            self.speed_translation=0.0
    
    def changeRobot(self):
        self.active_robot = (self.active_robot + 1) % len(self.robotnames)
        
    def f_scan_cb(self,data):
        
        T = tf.transformations.quaternion_matrix([self.rot[0],self.rot[1],self.rot[2],self.rot[3]])
        T[0][3] = self.trans[0]
        T[1][3] = self.trans[1]
        T[2][3] = self.trans[2]
        
        
        
        angle_min =  data.angle_min
        angle_increment = data.angle_increment
        x = [9999999] * len(data.ranges)
        y = [9999999] * len(data.ranges)
        d = [9999999] * len(data.ranges)
        x_transformed = [9999999] * len(data.ranges)
        y_transformed = [9999999] * len(data.ranges)
        d_transformed = [9999999] * len(data.ranges)
        
        for i in range(0,len(data.ranges)):
            if data.ranges[i] > 0.15:
                phi = angle_min + data.angle_increment*i
                x[i] = abs(math.cos(phi) * data.ranges[i])
                y[i] = abs(math.sin(phi) * data.ranges[i])
                d[i] = math.sqrt(x[i]**2 + y[i] **2)
                x_transformed[i] = abs(T[0][0] * x[i] + T[0][1] * y[i] + T[0][3] )
                y_transformed[i] = abs(T[1][0] * x[i] + T[1][1] * y[i] + T[1][3] )
                
                d_transformed[i] = math.sqrt(x_transformed[i]**2 + y_transformed[i] **2)
            
        
        self.front_dist_min = min(d_transformed)
            
    def b_scan_cb(self,data):

        T = tf.transformations.quaternion_matrix([self.rot_back[0],self.rot_back[1],self.rot_back[2],self.rot_back[3]])
        T[0][3] = self.trans_back[0]
        T[1][3] = self.trans_back[1]
        T[2][3] = self.trans_back[2]
        
        angle_min =  data.angle_min
        angle_increment = data.angle_increment
        x = [9999999] * len(data.ranges)
        y = [9999999] * len(data.ranges)
        d = [9999999] * len(data.ranges)
        x_transformed = [9999999] * len(data.ranges)
        y_transformed = [9999999] * len(data.ranges)
        d_transformed = [9999999] * len(data.ranges)
        
        for i in range(0,len(data.ranges)):
            if data.ranges[i] > 0.15:
                phi = angle_min + data.angle_increment*i
                x[i] = abs(math.cos(phi) * data.ranges[i])
                y[i] = abs(math.sin(phi) * data.ranges[i])
                d[i] = math.sqrt(x[i]**2 + y[i] **2)
                x_transformed[i] = abs(T[0][0] * x[i] + T[0][1] * y[i] + T[0][3] )
                y_transformed[i] = abs(T[1][0] * x[i] + T[1][1] * y[i] + T[1][3] )
                
                d_transformed[i] = math.sqrt(x_transformed[i]**2 + y_transformed[i] **2)
            
        
        self.back_dist_min = min(d_transformed)
        #print(self.back_dist_min)
    
        
        


    def publishTwist(self):
        msg=Twist()
        msg.linear.x=self.translation
        msg.angular.z=self.rotation
        
        self.publisher_stack[self.active_robot].publish(msg)

    def publishTwistStamped(self):
        msg=TwistStamped()
        msg.header.stamp=rospy.Time.now()
        msg.twist.linear.x=self.translation
        msg.twist.angular.z=self.rotation
        
        self.publisher_stack[self.active_robot].publish(msg)

    def run(self):
        while not rospy.is_shutdown(): 
            for i,edge in enumerate(self._edges):
                if edge:
                    self._edges[i] = 0
                    try:
                        self.callbackList[i]()
                    except Exception as ex:
                        print(ex)
                        pass
            
            
            if self.initialized == True:
                self.translation = (abs(self._axes[5] - 1) - abs(self._axes[2] - 1)) *self.speed_translation #data.axes[1] + data.axes[4]
                self.rotation = (self._axes[0] + self._axes[3])*self.speed_rotation
                
                if self.front_dist_min < 0.8 or self.back_dist_min < 0.8:
                    self.translation = self.translation * 0.1
                    self.rotation = self.rotation * 0.1
                    
                
                self.publishFunction()
            else:
                rospy.loginfo_throttle(5,"Controller is not initialized. Press and release both shoulder buttons simultaneously")
                if self._axes[2] == -1.0 and self._axes[5] == -1.0:
                    self.lower_position_reached = True
                    rospy.loginfo_once("lower position reached")
                if self.lower_position_reached == True and self._axes[2] == 1.0 and self._axes[5] == 1.0:
                    self.initialized = True
                    rospy.loginfo_once("initilization complete")
            self.rate.sleep()            


if __name__=="__main__":
    rospy.init_node("ps4_collision_free")
    ps4=PlayStationDiffDrive(Twist)
    ps4.run()
    rospy.spin()