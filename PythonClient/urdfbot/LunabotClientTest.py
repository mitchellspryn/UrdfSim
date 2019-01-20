import os
import sys
import keyboard
import shutil
import time
import numpy as np
import pandas as pd
import json
import jsonpickle
import LunabotClient

import setup_path
import airsim
import ClientUtil

def main():

    client = LunabotClient.LunabotClient()
    client_utils = ClientUtil.Utils()
    camera_names = ['Follow', 'TopBar']
    lidar_names = ['BaseLidar']
    #output_folder_name = 'lunabot_out'
    #client_utils.query_bot_state(client, camera_names, output_folder_name, lidar_names)
    #print('Setting wheels to 45')
    #client.rotate_wheels(0.5)
    #time.sleep(5)
    #client_utils.query_bot_state(client, camera_names, output_folder_name, lidar_names)
    #print('Setting wheels to 90')
    #client.rotate_wheels(1)
    #time.sleep(5)
    #print('Setting wheels to 0')
    #client.rotate_wheels(0)
    #time.sleep(5)

    #print('Lowering bucket')
    #client.rotate_bucket(0)
    #time.sleep(5)
    #print('Raising bucket')
    #client.rotate_bucket(1)
    #time.sleep(5)

    print('Lowering arm')
    client.rotate_arm(0)
    time.sleep(5)
    #client_utils.query_bot_state(client, camera_names, output_folder_name, lidar_names)
    #print('Raising arm')
    #client.rotate_arm(1)
    #time.sleep(5)

    #print('Driving forward')
    #client.drive(1.0, 0)
    #for i in range (0, 5, 1):
    #    client_utils.query_bot_state(client, camera_names, output_folder_name, lidar_names)
    #    time.sleep(1)
    
    #print('Stopping')
    #client.drive(0, 0)
    #time.sleep(5)
    #client_utils.query_bot_state(client, camera_names, output_folder_name, lidar_names)

    #print('Setting wheels to 45')
    #client.rotate_wheels(0.5)
    #time.sleep(5)
    #print('Driving in a circle')
    #client.drive(1, client.pi / 2)
    #time.sleep(5)
    #print('Stopping')
    #client.drive(0, 0)
    
    print('Graceful termination.')

main()