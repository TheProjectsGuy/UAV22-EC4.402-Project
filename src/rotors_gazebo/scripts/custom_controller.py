#!/usr/bin/env python
import rospy
import numpy as np
import math
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from rotors_comm.msg import WindSpeed
import random

#/pelican/command/motor_speed

def c(x):
    ''' cosine of angle in radians'''
    return np.cos(x)


def s(x):
    ''' sine of angle in radians'''
    return np.sin(x)


def t(x):
    ''' tangent of angle in radians'''
    return np.tan(x)

def rotation_matrix(phi, theta, psi):
    return np.array([[1, 0, -np.sin(theta)],
                    [0, np.cos(phi), np.sin(phi)*np.cos(theta)],
                    [0, -np.sin(phi), np.cos(phi)*np.cos(theta)]
                    ])

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z


def body2inertial_rotation(phi, theta, psi):
    ''' 
    Euler rotations from body-frame to global inertial frame
    angle 0 = roll (x-axis, phi)
    angle 1 = pitch (y-axis, theta)
    angle 2 = yaw (z-axis, psi)
    '''

    c1 = c(phi) 
    s1 = s(phi)
    c2 = c(theta)
    s2 = s(theta)
    c3 = c(psi)
    s3 = s(psi)

    R = np.array([[c2*c3, c3*s1*s2 - c1*s3, s1*s3 + c1*s2*c3],
        [c2*s3, c1*c3 + s1*s2*s3, c1*s3*s2 - c3*s1],
        [-s2, c2*s1, c1*c2]])

    return R

goal = np.array([0.0, 0.0,2.0])
# Starting position for UAV
position = np.array([0.0,0.0,0.0])
# Velocity vector
vel = np.array([0.0,0.0,0.0])
# Acceleration vector
acc = np.array([0.0,0.0,0.0])
# Euler angles phi, theta, psi
orientation = np.array([0.0,0.0,0.0])
# omega
omega = np.array([0.0,0.0,0.0])

# Acceleration due to gravity
g = 9.81
# Mass
mass = 1.0
dt = 0.01

kt = 9.9865e-06
k_tau = 1.6e-2
L = 0.3

maxT = 78.0

I = np.array([[0.01, 0, 0],[0, 0.01,0],[0, 0, 0.02]])

# The acceleration control eqn is given as :
# ax = k1*(xd - x) - k2*(vx), where xd is the goal an k, k2 are the gains. 
# Similar equations for ay and az
Kpos = np.array([[1.0, 1.0, 0.0],    #k1ax, k2ax
                 [1.0, 1.0, 0.0],   #k1ay, k2ay
                 [1.0, 1.0, 0.0]    #k1az, k2az
                ])

# The angular acceleration has 2 gains, k_phi and kp, where k_phi is for phi,theta and kp is for p,q,r. pd = k_phi*(phi_d-phi) and pdot = tx*kp*(pd-p). The torque is kp*(pd-p)
#Kang = np.array([[0.0001, 0.00025],    #kphi, kp
#                [0.0001, 0.00025],   #ktheta, kq
#                [0.0001, 0.00025]    #kpsi, kr
#                ])

Kang = np.array([[0.1, 0.0025, 500.0],    #kphi, kp
                 [0.1, 0.0025, 500.0],   #ktheta, kq
                 [0.1, 0.0025, 500.0]    #kpsi, kr
                ])



xp = []
yp = []
zp = []
td = []
pd = []
ps = []
pho = []
tho = []
pso = []
pho.append(0)
tho.append(0)
pso.append(0)
xp.append(0)
yp.append(0)
zp.append(0)
td.append(0)
pd.append(0)
ps.append(0)
thd = []
thd.append(0)
ax = []
ay = []
az = []
ax.append(0)
ay.append(0)
az.append(0)
x_err_sum = 0.0
y_err_sum = 0.0
z_err_sum = 0.0
phi_err_sum = 0.0
theta_err_sum = 0.0
psi_err_sum = 0.0
# For calculating motor speeds (4x4 matrix of 1s with respective signs)
#b = np.array([[1, -1, -1, 1], [1, 1, -1, -1], [1, -1, 1, -1], [1, 1, 1, 1]])
b = np.array([[-1, 1, 1, -1], [1, 1, -1, -1], [-1, 1, -1, 1], [1, 1, 1, 1]])


def callback(data):
    global x_err_sum
    global y_err_sum
    global z_err_sum
    global phi_err_sum
    global theta_err_sum
    global psi_err_sum
    """ws = WindSpeed()
    v = Vector3()
    v.x = (random.random()-0.5)*2.0
    v.y = (random.random()-0.5)*2.0
    v.z = (random.random()-0.5)*2.0
    pub = rospy.Publisher('/pelican/wind_speed', WindSpeed, queue_size=1)
    ws.velocity = v
    pub.publish(ws)
    return"""

    #print("####################")
    #print(data.pose.pose.position)
    #print("######################")
    pub = rospy.Publisher('/pelican/command/motor_speed', Actuators, queue_size=1)
    pub_pos = rospy.Publisher('/position', Point, queue_size=1)
    pub_acc = rospy.Publisher('/acceleration', Point, queue_size=1)
    pub_od = rospy.Publisher('/orientation', Point, queue_size=1)
    pub_rpy = rospy.Publisher('/rpy', Point, queue_size=1)
    pub_om = rospy.Publisher('/vel', Point, queue_size=1)
    position[0] = data.pose.pose.position.x
    position[1] = data.pose.pose.position.y
    position[2] = data.pose.pose.position.z
    vel[0] = data.twist.twist.linear.x
    vel[1] = data.twist.twist.linear.y
    vel[2] = data.twist.twist.linear.z
    #print("vel body = ", vel)
    r,p,y =euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    RWI = np.linalg.inv(body2inertial_rotation(r, p, y))
    #vel = RWI@vel
    v = RWI@np.array([vel[0], vel[1], vel[2]]).T
    vel[0] = v[0]
    vel[1] = v[1]
    vel[2] = v[2]
    #print(vel, v)
    #return

    orientation[0] = r
    orientation[1] = p
    orientation[2] = y
    omega[0] = data.twist.twist.angular.x
    omega[1] = data.twist.twist.angular.y
    omega[2] = data.twist.twist.angular.z
    pos_error = goal - position
    vel_error = -vel
    des_acc = Kpos[:,0]*pos_error + Kpos[:,1]*vel_error# + Kpos[:,2]*np.array([x_err_sum, y_err_sum, z_err_sum])
    x_err_sum+=pos_error[0]
    y_err_sum+=pos_error[1]
    z_err_sum+=pos_error[2]
    #des_acc = np.clip(des_acc, -2, 2)
    
    des_acc[2] = (g+des_acc[2])/(np.cos(orientation[0])*np.cos(orientation[1]))
    Td = mass*des_acc[2]
    #Td = mass*(g-des_acc[2])
    mag_acc = np.linalg.norm(des_acc)
    #des_acc = des_acc/mag_acc
    b_d = np.array([[-np.cos(orientation[2]), -np.sin(orientation[2])],[-np.sin(orientation[2]), np.cos(orientation[2])]])
    A = np.array([mass*des_acc[0]/Td, mass*des_acc[1]/Td]).T
    vars = np.linalg.inv(b_d)@A
    
    #phi_d = np.arcsin(des_acc[1]*mass/Td)
    #theta_d = np.arcsin(-des_acc[0]*mass/(Td*np.cos(phi_d)))
    phi_d = np.arcsin(np.clip(vars[1], -1.0, 1.0))
    theta_d = np.arcsin(np.clip(vars[0]/np.cos(phi_d), -1.0, 1.0))
    """Td = mass*(g+des_acc[2])
    phi_d = (des_acc[1]*np.cos(orientation[2]) - des_acc[0]*np.sin(orientation[2]))/(g-des_acc[2])
    theta_d = -(des_acc[0]*np.cos(orientation[2]) + des_acc[1]*np.sin(orientation[2]))/(g-des_acc[2])"""
    psi_d = 0.0#np.pi/4
    ori_error = np.array([phi_d, theta_d, psi_d]) - orientation
    omega_d = Kang[:,0]*ori_error
    omega_error = omega_d - omega
    #omega_error = -omega
    tau_d = Kang[:,1]*omega_error
    #tau_d = Kang[:,0]*ori_error + Kang[:,1]*omega_error# + Kang[:,2]*np.array([phi_err_sum, theta_err_sum, psi_err_sum])
    phi_err_sum+=ori_error[0]
    theta_err_sum+=ori_error[1]
    psi_err_sum+=ori_error[2]
    #tau_d[0] = tau_d[0]/I[0,0]
    #tau_d[1] = tau_d[1]/I[1,1]
    #tau_d[2] = tau_d[2]/I[2,2]
    
    A = np.array([tau_d[0]/(0.5*L*kt), tau_d[1]/(0.5*L*kt), tau_d[2]/(k_tau), Td/(kt)]).T
    ni = np.linalg.inv(b)@A
    #print("orientation = ", orientation*180/np.pi)
    #print("desired acc = ", des_acc)
    #print("desired orientation = ", phi_d*180/np.pi, theta_d*180/np.pi, Td)
    #print("orientation error = ", ori_error*180/np.pi)
    #print("omega_error = ", omega_error)
    print("position = ", position)
    #print("vel inertial = ", vel)
    #print("tau = ", tau_d)
    #return
    #if position[2]>1.0:
    #    print("EQUAL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    #    ni = np.ones((4,1))*1.0*0.25*g/(kt)
    ni = np.clip(ni, 0, 1000**2)
    ni = np.sqrt(ni)
    #print(ni)
    #quit()
    motor_commands = Actuators()
    motor_commands.angular_velocities.append(ni[0])
    motor_commands.angular_velocities.append(ni[1])
    motor_commands.angular_velocities.append(ni[2])
    motor_commands.angular_velocities.append(ni[3])
    #print(ni, tau_d)
    pub.publish(motor_commands)
    point = Point()
    point.x = position[0]
    point.y = position[1]
    point.z = position[2]
    pub_pos.publish(point)
    point = Point()
    point.x = des_acc[0]
    point.y = des_acc[1]
    point.z = des_acc[2]
    pub_acc.publish(point)
    point = Point()
    point.x = phi_d
    point.y = theta_d
    point.z = psi_d
    pub_od.publish(point)
    point = Point()
    point.x = tau_d[0]
    point.y = tau_d[1]
    point.z = tau_d[2]
    pub_om.publish(point)
    point = Point()
    point.x = r
    point.y = p
    point.z = y
    pub_rpy.publish(point)

    #xp.append(position[0])
    #yp.append(position[1])
    #zp.append(position[2])
    #pho.append(orientation[0])
    #tho.append(orientation[1])
    #pso.append(orientation[2])
    #pd.append(phi_d)
    #td.append(theta_d)
    #a = [xp,yp,zp]
    #np.savetxt("pos.txt", np.array(a).T)
    #a = [pho, tho, pso]
    #np.savetxt("orientation.txt", np.array(a).T)
    #a = [pd, td]
    #np.savetxt("des.txt", np.array(a).T)

"""def callback(data):
    print("######################")
    pub = rospy.Publisher('/pelican/command/motor_speed', Actuators, queue_size=1)
    position[0] = data.pose.pose.position.x
    position[1] = data.pose.pose.position.y
    position[2] = data.pose.pose.position.z
    vel[0] = data.twist.twist.linear.x
    vel[1] = data.twist.twist.linear.y
    vel[2] = data.twist.twist.linear.z
    r,p,y =euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    orientation[0] = r
    orientation[1] = p
    orientation[2] = y
    omega[0] = data.twist.twist.angular.x
    omega[1] = data.twist.twist.angular.y
    omega[2] = data.twist.twist.angular.z
    pos_error = goal - position
    vel_error = -vel
    des_acc = Kpos[:,0]*pos_error + Kpos[:,1]*vel_error
    #des_acc = np.clip(des_acc, -2, 2)
    
    des_acc[2] = (g-des_acc[2])/(np.cos(orientation[0])*np.cos(orientation[1]))
    Td = mass*des_acc[2]
    #Td = mass*(g-des_acc[2])
    mag_acc = np.linalg.norm(des_acc)
    des_acc = des_acc/mag_acc
    phi_d = np.arcsin(des_acc[1]*mass/Td)
    theta_d = np.arcsin(-des_acc[0]*mass/(Td*np.cos(phi_d)))
    psi_d = np.pi/4
    ori_error = np.array([phi_d, theta_d, psi_d]) - orientation
    omega_d = Kang[:,0]*ori_error
    omega_error = omega_d - omega
    tau_d = Kang[:,0]*ori_error - Kang[:,1]*omega
    
    e1 = tau_d[0] * I[0,0]
    e2 = tau_d[1] * I[1,1]
    e3 = tau_d[2] * I[2,2]

    #less typing
    n = 4

    # Thrust desired converted into motor speeds
    weight_speed = Td / (n*kt)

    # Thrust differene in each motor to achieve needed torque on body
    motor_speeds = []
    motor_speeds.append(weight_speed - (e2/((n/2)*kt*L)) - (e3/(n*k_tau)))
    motor_speeds.append(weight_speed - (e1/((n/2)*kt*L)) + (e3/(n*k_tau)))
    motor_speeds.append(weight_speed + (e2/((n/2)*kt*L)) - (e3/(n*k_tau)))
    motor_speeds.append(weight_speed + (e1/((n/2)*kt*L)) + (e3/(n*k_tau)))

    # Ensure that desired thrust is within overall min and max of all motors
    thrust_all = np.array(motor_speeds) * (kt)
    
    over_max = np.argwhere(thrust_all > 66.0)
    under_min = np.argwhere(thrust_all < 0.5)

    if over_max.size != 0:
        for i in range(over_max.size):
            motor_speeds[over_max[i][0]] = 66.0 / (kt)
    if under_min.size != 0:
        for i in range(under_min.size):
            motor_speeds[under_min[i][0]] = 0.5 / (kt)
    ni = [0,0,0,0]
    ni[0] = motor_speeds[1]**0.5
    ni[1] = motor_speeds[0]**0.5
    ni[2] = motor_speeds[3]**0.5
    ni[3] = motor_speeds[2]**0.5
    ni = np.clip(ni, 0, 1000)
    print(ni)
    #quit()
    motor_commands = Actuators()
    motor_commands.angular_velocities.append(ni[0])
    motor_commands.angular_velocities.append(ni[1])
    motor_commands.angular_velocities.append(ni[2])
    motor_commands.angular_velocities.append(ni[3])
    #print(ni)
    pub.publish(motor_commands)
    xp.append(position[0])
    yp.append(position[1])
    zp.append(position[2])
    pho.append(orientation[0])
    tho.append(orientation[1])
    pso.append(orientation[2])
    pd.append(phi_d)
    td.append(theta_d)
    a = [xp,yp,zp]
    np.savetxt("pos.txt", np.array(a).T)
    a = [pho, tho, pso]
    np.savetxt("orientation.txt", np.array(a).T)
    a = [pd, td]
    np.savetxt("des.txt", np.array(a).T)"""

    
def listener():
    rospy.init_node('controller_node', anonymous=True)

    rospy.Subscriber("/pelican/odometry_sensor1/odometry", Odometry, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()