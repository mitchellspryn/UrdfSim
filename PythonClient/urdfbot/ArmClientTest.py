import os
import sys
import keyboard
import shutil
import time
import numpy as np
import pandas as pd
import json
import jsonpickle
import ArmClient

import setup_path
import airsim
import ClientUtil

def main():
    client = ArmClient.ArmClient()
    client_utils = ClientUtil.Utils()
    output_folder_name = 'arm_out'
    camera_names = ['Follow', 'Arm']
    lidar_names = []
    client_utils.query_bot_state(client, camera_names, output_folder_name, lidar_names)
    
    print('Setting a random pose.')
    client.set_arm_pose(0.6, 0.7, 0.2, 0.3, 0.7)
    time.sleep(5)
    client_utils.query_bot_state(client, camera_names, output_folder_name, lidar_names)

    print('Setting another random pose.')
    client.set_arm_pose(0.3, 0.2, 0.6, 0.8, 0.4)
    time.sleep(5)
    client_utils.query_bot_state(client, camera_names, output_folder_name, lidar_names)

    print('Closing claw')
    client.set_manipulator(0)
    time.sleep(5)
    client_utils.query_bot_state(client, camera_names, output_folder_name, lidar_names)
    print('Opening claw')
    client.set_manipulator(1)
    time.sleep(5)
    client_utils.query_bot_state(client, camera_names, output_folder_name, lidar_names)

    return

main()
