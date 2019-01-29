import setup_path
import airsim
import airsim.airsim_types as at
import math
import time
import numpy as np
import pandas as pd

# from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
def toQuat(yaw, pitch, roll):
    pi = 3.1415926535
    yaw *= pi / 180
    pitch *= pi / 180
    roll *= pi / 180
    
    cy = math.cos(yaw * 0.5);
    sy = math.sin(yaw * 0.5);
    cp = math.cos(pitch * 0.5);
    sp = math.sin(pitch * 0.5);
    cr = math.cos(roll * 0.5);
    sr = math.sin(roll * 0.5);

    w = cy * cp * cr + sy * sp * sr;
    x = cy * cp * sr - sy * sp * cr;
    y = sy * cp * sr + cy * sp * cr;
    z = sy * cp * cr - cy * sp * sr;

    q = at.Quaternionr(x_val = -x, y_val = -y, z_val = z, w_val = w)
    
    return q

def setPose(x, y, z, yaw, pitch, roll, client):
    translation = at.Vector3r(x_val = x, y_val = y, z_val = z)
    rotation = toQuat(yaw, pitch, roll)
    camera_name = 'Follow'

    camera_obj = at.CameraPose(camera_name, translation, rotation)

    client.simSetCameraPose(camera_obj)

def main():
    client = airsim.UrdfBotClient()

    print('Testing X')
    setPose(2, 0, 0, 0, 0, 0, client)
    time.sleep(2)
    setPose(-2, 0, 0, 0, 0, 0, client)
    time.sleep(2)

    print('Testing Y')
    setPose(0, 2, 0, 0, 0, 0, client)
    time.sleep(2)
    setPose(0, -2, 0, 0, 0, 0, client)
    time.sleep(2)

    print('Testing Z')
    setPose(0, 0, 2, 0, 0, 0, client)
    time.sleep(2)
    setPose(0, 0, -2, 0, 0, 0, client)
    time.sleep(2)

    print('Testing Yaw')
    setPose(0, 0, 2, 45, 0, 0, client)
    time.sleep(2)
    setPose(0, 0, 2, -45, 0, 0, client)
    time.sleep(2)

    print('Testing Pitch')
    setPose(0, 0, 2, 0, 45, 0, client)
    time.sleep(2)
    setPose(0, 0, 2, 0, -45, 0, client)
    time.sleep(2)

    print('Testing Roll')
    setPose(0, 0, 2, 0, 0, 45, client)
    time.sleep(2)
    setPose(0, 0, 2, 0, 0, -45, client)
    time.sleep(2)

    print('Graceful Termination')

main()
