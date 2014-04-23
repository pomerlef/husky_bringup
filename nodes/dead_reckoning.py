#!/usr/bin/env python
import roslib; roslib.load_manifest('husky_bringup')
#import roslib.rosenv
import rospy
import PyKDL
import copy

from math import sin,cos,pi
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse
from clearpath_base.msg import Encoders

# Dynamic Reconfigure
import dynamic_reconfigure.server
from husky_bringup.cfg import HuskyConfig

ODOM_POSE_COVAR_MOTION = [1e-3,     0,   0,    0,    0,   0, 
                             0, 1e-3,    0,    0,    0,   0,
                             0,     0, 1e6,    0,    0,   0,
                             0,     0,   0,  1e6,    0,   0,
                             0,     0,   0,    0,  1e6,   0,
                             0,     0,   0,    0,    0, 1e-2]

ODOM_POSE_COVAR_NOMOVE = [1e-9,    0,    0,   0,   0,    0, 
                             0, 1e-3,    0,   0,   0,    0,
                             0,    0,  1e6,   0,   0,    0,
                             0,    0,    0, 1e6,   0,    0,
                             0,    0,    0,   0, 1e6,    0,
                             0,    0,    0,   0,   0, 1e-9]

ODOM_TWIST_COVAR_MOTION = [1e-3, 0, 0, 0, 0, 0, 
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e6]
ODOM_TWIST_COVAR_NOMOVE = [1e-9, 0, 0, 0, 0, 0, 
                          0, 1e-3, 0, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9]


class DeadReckoning(object):

    def InitOdometry(self):
        self.odom = Odometry()
        self.odom.header.frame_id = "odom_combined"
        self.odom.child_frame_id = "base_footprint" 
        self.last_encoder = []


    def __init__(self):
        rospy.init_node('dead_reckoning')
        # Parameters
        self.width = rospy.get_param('~width',0.5)
        self.gyro_scale = rospy.get_param('~gyro_scale_correction',1.0)
        # a steerign efficiency of 0.48 is typical for indoor floors
        self.steering_efficiency= rospy.get_param('~steering_efficiency',0.48)
        
        # Initialize odometry message
        self.InitOdometry()

        # Set up publishers/subscribers
        self.pub_imu = rospy.Publisher('imu_data',Imu)
        self.pub_enc_odom = rospy.Publisher('encoder',Odometry);
        rospy.Subscriber('imu/data', Imu, self.HandleIMU)
        rospy.Subscriber('husky/data/encoders', Encoders, self.HandleEncoder)

        # Set up services
        rospy.Service('~start_steering_calib', Empty, self.HandleStartCalibSteering)
        rospy.Service('~stop_steering_calib', Empty, self.HandleStopCalibSteering)
        rospy.Service('~reset_odometry', Empty, self.HandleResetOdom)

        self.calibStarted = False

                
        dynamic_reconfigure.server.Server(HuskyConfig, self.Reconfigure)
    
    
    def HandleStartCalibSteering(self, req):
        rospy.loginfo("Starting steering calibration");
        rospy.loginfo("Rotate the robot by a full turn");
        self.fullTurn = 0.0
        self.calibStarted = True
        return EmptyResponse()

    def HandleStopCalibSteering(self, req):

        if not self.calibStarted:
          rospy.loginfo("You need to start the calibration first");
        else:
          self.calibStarted = False
          if not self.fullTurn == 0.0:
            correction = abs(2.0*pi/self.fullTurn)
            self.steering_efficiency = self.steering_efficiency * correction
            rospy.loginfo("The new steering efficiency is %.3f" % self.steering_efficiency);
          else:
            rospy.loginfo("The robot didn't turned. Not modification to the steering efficiency will be applied");
        return EmptyResponse()

    def HandleResetOdom(self, req):
        self.InitOdometry()
        return EmptyResponse()


    def Reconfigure(self, config, level):
        # Handler for dynamic_reconfigure
        self.gyro_scale = config['gyro_scale_correction']
        return config

    def HandleIMU(self,data):
        # Correct IMU data
        # Right now, gyro scale only
        # TODO: Evaluate necessity of adding drift correction 
        imu_corr = copy.deepcopy(data)
        imu_corr.header.frame_id = "base_link"
        imu_corr.angular_velocity.z = data.angular_velocity.z * self.gyro_scale
        self.pub_imu.publish(imu_corr)
   
    def HandleEncoder(self,data):
        # Initialize encoder state

        if not self.last_encoder:
            self.last_encoder = data
            return
        # Calculate deltas
        dr = ((data.encoders[0].travel - self.last_encoder.encoders[0].travel) +
              (data.encoders[1].travel - self.last_encoder.encoders[1].travel)) / 2;
        da = ((data.encoders[1].travel - self.last_encoder.encoders[1].travel) - 
              (data.encoders[0].travel - self.last_encoder.encoders[0].travel)) * \
              (self.steering_efficiency / self.width);
        ddr = (data.encoders[0].speed + data.encoders[1].speed)/2;
        dda = (data.encoders[1].speed - data.encoders[0].speed) * \
              (self.steering_efficiency / self.width);
        self.last_encoder = data

        # Integrate for calibration
        if self.calibStarted:
          self.fullTurn = self.fullTurn + da

        # FP: this is a patch for encoder msg reseting
        # It is not a solution, the problem needs to be solved at a lower level
        
        rospy.loginfo("dr: %f" % dr)

        if(abs(dr) < 2.0):
          # Update data
          o = self.odom.pose.pose.orientation
          cur_heading = PyKDL.Rotation.Quaternion(o.x,o.y,o.z,o.w).GetEulerZYX()
          self.odom.pose.pose.position.x += dr * cos(cur_heading[0]) # FP: TODO confirm that
          self.odom.pose.pose.position.y += dr * sin(cur_heading[0]) 
          quat = PyKDL.Rotation.RotZ(cur_heading[0] + da).GetQuaternion()
          self.odom.pose.pose.orientation = Quaternion(quat[0],quat[1],quat[2],quat[3])
          self.odom.twist.twist.linear.x = ddr
          self.odom.twist.twist.angular.z = dda
          
          self.odom.header.stamp = rospy.Time.now()

          # If our wheels aren't moving, we're likely not moving at all.
          # Adjust covariance appropriately
          if data.encoders[0].speed == 0 and data.encoders[1].speed == 0:
              self.odom.pose.covariance = ODOM_POSE_COVAR_NOMOVE
              self.odom.twist.covariance = ODOM_TWIST_COVAR_NOMOVE
          else:
              self.odom.pose.covariance = ODOM_POSE_COVAR_MOTION
              self.odom.twist.covariance = ODOM_TWIST_COVAR_MOTION

          self.pub_enc_odom.publish(self.odom)


if __name__ == "__main__":
    obj = DeadReckoning()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
