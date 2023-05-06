#! /usr/bin/python3
import json

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Int64, String, Bool
from box import Box
import numpy as np

from sphero_mini.simple_moves import forward, initSphero
from sphero_mini.core import SpheroMini

# heading = 0
# distance = 0
# global heading, move, distance
move = 0
auto = 0

last_sent_yaw = None
last_heading = 0
last_vel = 0

def connect():
    mac_address = rospy.get_param('/sphero/mac_address', default='E0:D2:31:6B:74:3C')
    # conf_file_path = rospy.get_param("/conf_file_path")
    # with open(conf_file_path, 'r') as f:
        # cfg = Box(json.load(f))

    # Connect:
    # sphero = SpheroMini(cfg.MAC_ADDRESS, verbosity = 4)
    sphero = SpheroMini(mac_address, verbosity = 4)
    # battery voltage
    sphero.getBatteryVoltage()
    print(f"Battery voltage: {sphero.v_batt}v")

    # firmware version number
    sphero.returnMainApplicationVersion()
    print(f"Firmware version: {'.'.join(str(x) for x in sphero.firmware_version)}")
    return sphero


def disconnect(sphero):
    sphero.sleep()
    sphero.disconnect()


def create_quaternion(roll, pitch, yaw):
    q = Quaternion()
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)

    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy

    return q


def create_angular_veolocity_vector3(groll, gpitch, gyaw):
    v = Vector3()
    v.x = groll
    v.y = gpitch
    v.z = gyaw

    return v


def create_linear_acc_vector3(xacc, yacc, zacc):
    v = Vector3()
    v.x = xacc
    v.y = yacc
    v.z = zacc

    return v


def get_sensors_data(sphero):
    return {
        "roll": sphero.IMU_roll,
        "pitch": sphero.IMU_pitch,
        "yaw": sphero.IMU_yaw,
        "groll": sphero.IMU_gyro_x,
        "gpitch": sphero.IMU_gyro_y,
        "xacc": sphero.IMU_acc_x,
        "yacc": sphero.IMU_acc_y,
        "zacc": sphero.IMU_acc_z
    }
    

def publish_imu(pub, sensors_values):
    i = Imu()

    i.header.stamp = rospy.Time.now()
    i.orientation = create_quaternion(
        roll=sensors_values["roll"],
        pitch=sensors_values["pitch"],
        yaw=sensors_values["yaw"]
        )
    i.angular_velocity = create_angular_veolocity_vector3(
        groll=sensors_values["groll"],
        gpitch=sensors_values["gpitch"],
        gyaw=0  # We don't have the IMU_gyro_z
    )
    i.linear_acceleration = create_linear_acc_vector3(
        xacc=sensors_values["xacc"],
        yacc=sensors_values["yacc"],
        zacc=sensors_values["zacc"]
    )
    pub.publish(i)

def yaw_callback(yaw):
    # rospy.loginfo(yaw)
    global heading
    global last_sent_yaw
    global move
    global velocity
    global auto
    heading = yaw.data

    rospy.loginfo(auto)
    if auto == True:
        if heading < 0:
            heading = 360 + heading

        if move == 1:  
            # if heading != last_sent_yaw:
            sphero.roll(velocity, heading)
            # sphero.wait(0.2)
            # rospy.loginfo(heading)
            last_sent_yaw = heading
        elif move == 0:
            # sphero.roll(0,heading)
            # sphero.wait(1)
            rospy.loginfo("Not Moving")
    

def dist_callback(dist):
    # rospy.loginfo(dist)
    global distance
    distance = dist.data
    # print(distance)
    # print(distance)
    # rospy.loginfo(distance)

def move_flag_callback(flag):
    global move
    global heading
    move = flag.data
    # if move == 0:
    #     sphero.roll(0,0)
        # sphero.wait(1)

def velocity_callback(vel):
    global velocity
    velocity = vel.data
    # rospy.loginfo(velocity)

def dpad_callback(dir):
    global command
    global last_heading
    global auto
    command = dir.data
    if auto == False:
        if command == "Up":
            sphero.roll(100, 0)
            sphero.wait(0.2)
            sphero.roll(0, 0)
            last_heading = 0
            # rospy.loginfo("Up")
        elif command == "Down":
            sphero.roll(100, 180)
            sphero.wait(0.2)
            sphero.roll(0, 180)
            last_heading = 180
            # rospy.loginfo("Down")
        elif command == "Left":
            sphero.roll(100, 270)
            sphero.wait(0.2)
            sphero.roll(0, 270)
            last_heading = 270
            # rospy.loginfo("Left")
        elif command == "Right":
            sphero.roll(100, 90)
            sphero.wait(0.2)
            sphero.roll(0, 90)
            last_heading = 90
            # rospy.loginfo("Right")
        elif command == "None":
            sphero.roll(0, last_heading)
            sphero.wait(0.2)
            # rospy.loginfo("stopped")
    
def joystick_callback(dat):
    global velocity
    global heading
    global last_vel
    global auto
    temp = dat
    velocity = int(temp.x)
    heading = int(temp.y)

    if auto == False:
        if heading < 0:
            heading = 360 + heading
        
        if velocity >= last_vel + 8 or velocity <= last_vel - 8:
            sphero.roll(velocity, heading)
        last_vel = velocity
        # sphero.wait(0.2)
    # sphero.roll(0, heading)
    # rospy.loginfo("joystick: ", velocity, ", ", heading)

def slider_callback(rgb):
    r = int(rgb.x)
    g = int(rgb.y)
    b = int(rgb.z)
    sphero.setLEDColor(red = r, green = g, blue = b)

def autonomy_callback(bool):
    global auto 
    auto = bool.data

def main(sphero):
    global heading
    global move
    rospy.init_node('sphero', anonymous=True, log_level=rospy.DEBUG)   
    rate = rospy.Rate(10)  # 10 Hz

    # # pub = rospy.Publisher("/imu", Imu, queue_size=5)
    # dist_sub = rospy.Subscriber("/sphero/desired_distance", Int64, dist_callback)
    # yaw_sub = rospy.Subscriber("/sphero/desired_yaw", Int64, yaw_callback)

    # Don't need to hold onto a subscriber object
    rospy.Subscriber("/sphero/move_flag", Int64, move_flag_callback, queue_size=None)
    rospy.Subscriber("/sphero/desired_distance", Int64, dist_callback, queue_size=None)
    rospy.Subscriber("/sphero/desired_yaw", Int64, yaw_callback, queue_size=None)
    rospy.Subscriber("/sphero/desired_velocity", Int64, velocity_callback, queue_size=None)
    rospy.Subscriber('/GUI/dpad', String, dpad_callback, queue_size=None)
    rospy.Subscriber('/GUI/joystick', Vector3, joystick_callback, queue_size=None)
    rospy.Subscriber('/GUI/slider', Vector3, slider_callback, queue_size=None)
    rospy.Subscriber('/GUI/autonomy', Bool, autonomy_callback, queue_size=None)

    # if move == 1:
    #     # if heading != last_sent_yaw:
    #     if heading < 0:
    #         heading = 360 + heading
    #     sphero.roll(0, heading)
    #     # sphero.wait(0.5)
    #     last_sent_yaw = heading
    # elif move == 0:
    #     sphero.roll(0,heading)
    #     # sphero.wait(1)
    #     rospy.loginfo("Not Moving")

    # rospy.wait_for_message("/sphero/desired_yaw", Int64, timeout=10.0)
    rospy.spin()

    # forward(sphero)    
    # print(str(distance) + " " + str(heading))
    # rospy.loginfo(heading)
    # rospy.spin()
    # if heading != 0:
    # approach(sphero, 80, heading)

    # rospy.spin()

    # sphero.configureSensorMask(
    #     IMU_yaw=True,
    #     IMU_pitch=True,
    #     IMU_roll=True,
    #     IMU_acc_y=True,
    #     IMU_acc_z=True,
    #     IMU_acc_x=True,
    #     IMU_gyro_x=True,
    #     IMU_gyro_y=True,
    #     #IMU_gyro_z=True 
    #     )
    # sphero.configureSensordef move_flag_callback(flag):
#     move = flag.dataStream()

    # while not rospy.is_shutdown():
    #     # sensors_values = get_sensors_data(sphero)
    #     #rospy.logdebug(sensors_values)
    #     # publish_imu(pub, sensors_values)
    #     # sphero.setLEDColor(red = 0, green = 255, blue = 0) # Turn LEDs green
    #     rospy.loginfo(heading)
    #     # rospy.loginfo(distance)
    #     rate.sleep()

# def main(sphero):
#     # rospy.loginfo(heading)
#     approach(sphero, 80, heading)

# distance = 100
# heading = 0

if __name__ == "__main__": # test movements without sensor data first
    global sphero
    global heading
    sphero = connect()     # especially the interactive motion
    try:
        initSphero(sphero)
        auto = False
        move = 1
        while(1):
            while move == 1 or auto == False:
                main(sphero)

        # sphero.roll(50, 0)
        # sphero.wait(3)
        # sphero.roll(50, 0)
        # sphero.wait(3)

    except Exception as e: # rospy.ROSInterruptException
        disconnect(sphero)
        raise e
    