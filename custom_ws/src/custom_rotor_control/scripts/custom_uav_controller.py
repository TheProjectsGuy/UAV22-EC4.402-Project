#!/usr/bin/env python3
"""
    A custom UAV controller using PD control on position and 
    orientation.
"""

# Import everything
import rospy
import tf
# from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
import numpy as np
from threading import Lock
import time
import sys
import traceback


# Main controller class
class MAVController:
    """
        Multirotor UAV controller class
        The constructor assumes that the parameter `config` is passed.
        See the `custom_map_controller.yaml` file for the definitions.
    """
    def __init__(self) -> None:
        # --- Parameters ---
        # Initialize parameters
        confs = rospy.get_param(f"{rospy.get_name()}/config")
        rospy.loginfo(f"All configs: {confs}")
        # Load MAV and environment parameters
        self.m = confs.get("mav_m", 2.0)    # Mass
        # Inertia tensor
        ixx, ixy= confs["J"]["ixx"], confs["J"]["ixy"]
        ixz, iyy = confs["J"]["ixz"], confs["J"]["iyy"]
        iyz, izz = confs["J"]["iyz"], confs["J"]["izz"]
        self.J = np.array([ # 3x3 Inertia tensor
            [ixx, ixy, ixz],
            [ixy, iyy, iyz],    # Symmetric by nature!
            [ixz, iyz, izz]
        ])
        self.L = confs.get("L", 0.21)   # Arm length
        self.env_g = confs.get("env_g", 9.8)    # Gravity
        # Coefficients of propeller
        self.kT = float(confs["prop"]["thrust_coeff"])
        self.kt = float(confs["prop"]["torque_coeff"])
        # Limits
        self.max_rpm = confs["limits"]["max_prop_rpm"]
        # Limit on control action - keep the small angle assumption!
        max_phi = np.deg2rad(confs["limits"]["b_phi_d"])
        max_theta = np.deg2rad(confs["limits"]["b_theta_d"])
        max_psi = np.deg2rad(confs["limits"]["b_psi_d"])
        self.max_ang = np.array([max_phi, max_theta, max_psi])
        self.min_ang = -self.max_ang
        # --- State variables ---
        # Desired
        self.pos_d: np.ndarray = None   # [x, y, z] position
        self.ang_d: np.ndarray = None   # RPY desired angles
        self.vel_d: np.ndarray = None   # [x, y, z] velocity
        self.angvel_d: np.ndarray = None    # RPY ang. velocity
        # Current
        self.cpos: np.ndarray = None    # [x, y, z] position
        self.cvel: np.ndarray = None    # [x, y, z] velocity
        self.cang: np.array = None      # RPY angle
        self.cangvel: np.array = None   # Angular velocities (RPY)
        # --- Controller ---
        # Position PD gains
        self.Kp_pos = np.array(confs["kp_pos"])
        self.Kd_pos = np.array(confs["kd_pos"])
        # Angle PD gains
        self.Kp_ang = np.array(confs["kp_ang"])
        self.Kd_ang = np.array(confs["kd_ang"])
        # --- Topics ---
        # Subscribe for sensor data
        odom_topic = confs["topics"]["sensing"]["odom"]
        self.odom_sens_obj = rospy.Subscriber(odom_topic, Odometry,
            lambda odom_msg: self._update_odom_data(odom_msg), 
            queue_size=10)  # Odometry data (to update current state)
        # Motor speeds
        ms_topic = confs["topics"]["pub"]["motor_c"]
        self.ms_pub_obj = rospy.Publisher(ms_topic, Actuators, 
            queue_size=10)  # Motor speed publisher
        # --- Multi-threading stuff ---
        # Locks for the motor speed calculations
        self.lock_odom_ms = Lock()

    # Odometry subscriber callback
    def _update_odom_data(self, odom_msg: Odometry):
        # Get current position
        cx = odom_msg.pose.pose.position.x
        cy = odom_msg.pose.pose.position.y
        cz = odom_msg.pose.pose.position.z
        # Get current orientation
        cqx = odom_msg.pose.pose.orientation.x
        cqy = odom_msg.pose.pose.orientation.y
        cqz = odom_msg.pose.pose.orientation.z
        cqw = odom_msg.pose.pose.orientation.w
        # Convert to euler
        (r, p, y) = tf.transformations.euler_from_quaternion(
            [cqx, cqy, cqz, cqw])   # Roll, Pitch, Yaw
        # Get current linear velocity
        cvx = odom_msg.twist.twist.linear.x
        cvy = odom_msg.twist.twist.linear.y
        cvz = odom_msg.twist.twist.linear.z
        # Get current angular velocity
        cavx = odom_msg.twist.twist.angular.x
        cavy = odom_msg.twist.twist.angular.y
        cavz = odom_msg.twist.twist.angular.z
        # Update the local variables
        self.lock_odom_ms.acquire()
        self.cpos = np.array([cx, cy, cz])  # Position
        self.cang = np.array([r, p, y]) # Angle
        self.cvel = np.array([cvx, cvy, cvz])   # Linear vel.
        self.cangvel = np.array([cavx, cavy, cavz]) # Angular vel.
        self.lock_odom_ms.release()
    
    # Using desired thrust and angular accelerations, get n_i**2
    def _thr_angaccs_to_motor_speeds(self, des_thrust: float, 
            ang_accs: np.ndarray):
        # Shorthand names
        kT, kt = self.kT, self.kt   # Thrust, torque constants
        J = self.J  # Inertia tensor
        # Inverse of relation matrix
        M_inv = np.array([
            [0.25/kT, 0, -0.5/kt, 0.25/kt],
            [0.25/kT, 0.5/kt, 0, -0.25/kt],
            [0.25/kT, 0, 0.5/kt, 0.25/kt],
            [0.25/kT, -0.5/kt, 0, -0.25/kt]
        ])
        # Tau (body torque - inertial)
        tau: np.ndarray = J @ ang_accs.reshape(3, 1)    # tx, ty, tz
        # rospy.loginfo(f"Tau: {tau.flatten().tolist()}")   # FIXME:
        # Force (effort) tensor: [T, tx, ty, tz]
        f_vect = np.array([des_thrust, *tau.flatten().tolist()]).\
            reshape(4, 1)   # Convert this to 'n_i' using 'M_inv'
        # Get (n_i)**2 values using 'M_inv' (n_i in RPM)
        ms_sq = M_inv @ f_vect
        ms_sq[ms_sq < 0] = 0    # This should probably NOT happen!
        ms_unclipped = ms_sq ** (0.5)   # Rotor speeds (unclipped)
        # Clip n_i (RPM) values
        ms_clipped = np.clip(ms_unclipped, 
            np.zeros_like(ms_unclipped), 
            np.array([4*[self.max_rpm]]).reshape(4, 1))
        return ms_clipped.flatten() # As (4,)

    # Run single loop of control and get motor speeds
    def run_once(self):
        # Short hand
        pos_d, cpos = self.pos_d, self.cpos
        vel_d, cvel = self.vel_d, self.cvel
        Kp_pos, Kd_pos = self.Kp_pos, self.Kd_pos
        m, g = self.m, self.env_g
        Kp_ang, Kd_ang = self.Kp_ang, self.Kd_ang
        cang, cangvel = self.cang, self.cangvel
        angvel_d = self.angvel_d
        self.lock_odom_ms.acquire() # Don't change odom readings here
        # Main loop
        pos_err = pos_d - cpos    # Position error
        vel_err = vel_d - cvel    # Velocity error
        # Desired acceleration - (PD controller)
        des_acc = Kp_pos * pos_err + Kd_pos * vel_err
        # rospy.loginfo(f"Des acc: {des_acc.tolist()}")   # FIXME:
        # Counter gravity (in the home frame)
        des_acc[2] = (des_acc[2] + g)/\
            (np.cos(cang[0])*np.cos(cang[1]))
        # Thrust needed (in +Z) - The MAV can only give this!
        des_thrust: float = m * des_acc[2]
        # Calculate angle desired (from spherical to cartesian)
        des_acc_mag = np.linalg.norm(des_acc)
        if des_acc_mag == 0:
            des_acc_mag = 1.0   # If no acceleration vector needed
        des_ang = np.array([ # Desired angles in the inertial frame
            # Invert 1*sin(phi)*cos(theta) = -acc_y_hat (unit vect.)
            np.arcsin(np.clip(
                -des_acc[1] / des_acc_mag / np.cos(cang[1]),
                -0.95, 0.95)),    # asin needs only [-1, 1]
            # Invert sin(theta) = acc_x_hat (unit vect.)
            np.arcsin(des_acc[0] / des_acc_mag),
            # Psi is always desired to be zero
            0]
        )  # Desired phi, theta, psi calculated
        # rospy.loginfo(f"Ang: {np.rad2deg(des_ang)}")    # FIXME:
        # Threshold these angles
        des_ang_clip = np.clip(des_ang, self.min_ang, self.max_ang)
        # Get angle action (angle error -> controller)
        ang_err = des_ang_clip - cang    # Angle error
        ang_vel_err = angvel_d - cangvel    # Angular velocity error
        # Desired angular acceleration - (PD controller)
        des_angacc = Kp_ang * ang_err + Kd_ang * ang_vel_err
        # rospy.loginfo(f"AA: {des_angacc}")    # FIXME:
        # Get motor speeds for the desired thrust and ang. acc.
        motor_speeds = self._thr_angaccs_to_motor_speeds(des_thrust, 
            des_angacc)
        self.lock_odom_ms.release()
        return motor_speeds
    
    # Set target (currently only 'pos_d' works)
    def set_target(self, pos_d, ang_d, vel_d, angvel_d):
        self.pos_d = np.array(pos_d)    # [x, y, z]
        self.ang_d = np.array([0., 0., 0.])
        self.vel_d = np.array([0., 0., 0.])
        self.angvel_d = np.array([0., 0., 0.])
    
    # Publish motor speeds
    def publish_ms(self, ms: list):
        ms_msg = Actuators()
        ms_msg.header.stamp = rospy.Time.now()  # Current time
        ms_msg.angular_velocities = ms
        # Publish message
        self.ms_pub_obj.publish(ms_msg)

    # Run once and publish the motor speeds
    def run_and_publish(self):
        # Get motor speeds
        ms = self.run_once()
        # Publish motor speeds
        self.publish_ms(ms.tolist())
        

# Main function
def main():
    # Initialize the node
    rospy.init_node("custom_mav_controller", argv=sys.argv)
    time.sleep(0.1) # Setup delay (threads!)
    # Create MAV Controller
    control_obj = MAVController()
    # Wait for Gazebo
    rospy.loginfo("Waiting for Gazebo...")
    time.sleep(30)
    rospy.loginfo("Wait for Gazebo complete!")
    ctrl_rate_hdlr = rospy.Rate(60)
    # Desired position
    des_pos = [1., 0., 0.5]
    control_obj.set_target(des_pos, None, None, None)
    rospy.loginfo("Starting control loop")
    # Control loop
    while not rospy.is_shutdown():
        # Calculate the motor speeds
        ms = control_obj.run_once()
        # rospy.loginfo(f"MS: {ms.tolist()}")   # FIXME: Remove this
        # Publish the speeds
        control_obj.publish_ms(ms.tolist())
        # Wait
        ctrl_rate_hdlr.sleep()


# Entrypoint
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException as exc:
        rospy.logfatal(f"Error: {exc}")
        traceback.print_exc()
    except Exception as exc:
        rospy.logfatal(f"Non-ROS error: {exc}")
        traceback.print_exc()
