import os
from math import sin, cos, pi
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped,Quaternion
# import tf_transformations

class BaseController:
    # def __init__(self, node,arduino, base_frame, name="base_controllers"):
    def __init__(self, node, name="base_controllers"):
        self.node = node
        self.arduino = node.arduino
        self.name = name
        self.base_frame = node.get_parameter("base_frame").value
        self.odom_frame = node.get_parameter("odom_frame").value
        self.rate = float(node.get_parameter("base_controller_rate").value)
        self.timeout = node.get_parameter("base_controller_timeout").value
        self.stopped = False

        pid_params = dict()
        pid_params['wheel_diameter'] = node.get_parameter("wheel_diameter").value
        pid_params['wheel_track'] = node.get_parameter("wheel_track").value
        pid_params['encoder_resolution'] = node.get_parameter("encoder_resolution").value
        pid_params['gear_reduction'] = node.get_parameter("gear_reduction").value
        pid_params['Kp'] = node.get_parameter("Kp").value
        pid_params['Kd'] = node.get_parameter("Kd").value
        pid_params['Ki'] = node.get_parameter("Ki").value
        pid_params['Ko'] = node.get_parameter("Ko").value
        
        self.accel_limit = node.get_parameter("accel_limit").value
        self.motors_reversed = node.get_parameter("motors_reversed").value

        self.max_vel_x = node.get_parameter("max_vel_x").value
        self.min_vel_x = node.get_parameter("min_vel_x").value
        self.max_vel_th = node.get_parameter("max_vel_th").value
        self.min_vel_th = node.get_parameter("min_vel_th").value

        # 设置pid
        self.setup_pid(pid_params)

        # How many encoder ticks are there per meter?
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction  / (self.wheel_diameter * pi)
        
        # What is the maximum acceleration we will tolerate when changing wheel speeds?
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate

        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0   

        # now = rospy.Time.now()  
        now = self.node.get_clock().now()  
        self.then = now # time for determining dx/dy
        self.t_delta = Duration(seconds=(1.0 / self.rate))
        self.t_next = now + self.t_delta      
        
        # Internal data        
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0.0                      # position in xy plane
        self.y = 0.0
        self.th = 0.0                     # rotation in radians
        self.v_left = 0.0
        self.v_right = 0.0
        self.v_des_left = 0.0             # cmd_vel setpoint
        self.v_des_right = 0.0
        self.last_cmd_vel = now

        self.vel_sub = self.node.create_subscription(Twist,"cmd_vel",self.cmdVelCallback,10)

        # Clear any old odometry info
        self.arduino.reset_encoders()

        # Set up the odometry broadcaster
        self.odomPub = self.node.create_publisher(Odometry,"odom",10)
        self.odomBroadcaster = TransformBroadcaster(self.node)

        self.node.get_logger().info("Started base controller for a base of " + str(self.wheel_track) + "m wide with " + str(self.encoder_resolution) + " ticks per rev")
        self.node.get_logger().info("Publishing odometry data at: " + str(self.rate) + " Hz using " + str(self.base_frame) + " as base frame")
        

    def setup_pid(self, pid_params):
        # Check to see if any PID parameters are missing
        missing_params = False
        for param in pid_params:
            if pid_params[param] == "":
                print("*** PID Parameter " + param + " is missing. ***")
                missing_params = True
        
        if missing_params:
            os._exit(1)
                
        self.wheel_diameter = pid_params['wheel_diameter']
        self.wheel_track = pid_params['wheel_track']
        self.encoder_resolution = pid_params['encoder_resolution']
        self.gear_reduction = pid_params['gear_reduction']
        
        self.Kp = pid_params['Kp']
        self.Kd = pid_params['Kd']
        self.Ki = pid_params['Ki']
        self.Ko = pid_params['Ko']
        
        self.arduino.update_pid(self.Kp, self.Kd, self.Ki, self.Ko)

    def poll(self):
        now = self.node.get_clock().now()  
        if now > self.t_next:
            # Read the encoders
            try:
                left_enc, right_enc = self.arduino.get_encoder_counts()
            except Exception as e:
                self.bad_encoder_count += 1
                self.node.get_logger().error("Encoder exception count: " + str(self.bad_encoder_count))
                return
                            
            dt = now - self.then
            self.then = now
            dt = dt.nanoseconds / 1000000000.0
            

            # Calculate odometry
            if self.enc_left == None:
                dright = 0
                dleft = 0
            else:
                dright = (right_enc - self.enc_right) / self.ticks_per_meter
                dleft = (left_enc - self.enc_left) / self.ticks_per_meter

            self.enc_right = right_enc
            self.enc_left = left_enc
            
            dxy_ave = (dright + dleft) / 2.0
            dth = (dright - dleft) / self.wheel_track
            vxy = dxy_ave / dt
            vth = dth / dt
                
            if (dxy_ave != 0):
                dx = cos(dth) * dxy_ave
                dy = -sin(dth) * dxy_ave
                self.x += (cos(self.th) * dx - sin(self.th) * dy)
                self.y += (sin(self.th) * dx + cos(self.th) * dy)
    
            if (dth != 0):
                self.th += dth 
            
            
            quaternion = Quaternion()
            quaternion.x = 0.0 
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2.0)
            quaternion.w = cos(self.th / 2.0)

            t = TransformStamped()

            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame

            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0

            t.transform.rotation.x = quaternion.x
            t.transform.rotation.y = quaternion.y
            t.transform.rotation.z = quaternion.z
            t.transform.rotation.w = quaternion.w
        
            self.odomBroadcaster.sendTransform(t)
    
            odom = Odometry()
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame
            odom.header.stamp = now.to_msg()
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = vxy
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = vth

            self.odomPub.publish(odom)
            
            if now > (self.last_cmd_vel + Duration(seconds=self.timeout)):
                self.v_des_left = 0.0
                self.v_des_right = 0.0
                
            if self.v_left < self.v_des_left:
                self.v_left += self.max_accel
                if self.v_left > self.v_des_left:
                    self.v_left = self.v_des_left
            else:
                self.v_left -= self.max_accel
                if self.v_left < self.v_des_left:
                    self.v_left = self.v_des_left
            
            if self.v_right < self.v_des_right:
                self.v_right += self.max_accel
                if self.v_right > self.v_des_right:
                    self.v_right = self.v_des_right
            else:
                self.v_right -= self.max_accel
                if self.v_right < self.v_des_right:
                    self.v_right = self.v_des_right
            
            # Set motor speeds in encoder ticks per PID loop
            if not self.stopped:
                self.arduino.drive(self.v_left, self.v_right)
                
            self.t_next = now + self.t_delta

    def stop(self):
        self.stopped = True
        self.arduino.drive(0, 0)

    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        self.last_cmd_vel = self.node.get_clock().now()  
        
        x = req.linear.x         # m/s
        if x >= self.max_vel_x:
            x = self.max_vel_x
        elif x <= self.min_vel_x:
            x = self.min_vel_x

        th = req.angular.z       # rad/s
        if th >= self.max_vel_th:
            th = self.max_vel_th
        elif th <= self.min_vel_th:
            th = self.min_vel_th

        if x == 0:
            # Turn in place
            right = th * self.wheel_track  * self.gear_reduction / 2.0
            left = -right
        elif th == 0:
            # Pure forward/backward motion
            left = right = x
        else:
            # Rotation about a point in space
            left = x - th * self.wheel_track  * self.gear_reduction / 2.0
            right = x + th * self.wheel_track  * self.gear_reduction / 2.0
            
        self.v_des_left = int(left * self.ticks_per_meter / self.arduino.PID_RATE)
        self.v_des_right = int(right * self.ticks_per_meter / self.arduino.PID_RATE)
