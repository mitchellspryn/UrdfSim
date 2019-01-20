import os
import sys
import keyboard
import shutil
import time
import numpy as np
import pandas as pd
import json
import jsonpickle
import BoxAndCylinderClient

import setup_path
import airsim


def query_sensors(client, unique_path_str):
    folder_name = 'sensors'
    if not (os.path.isdir(folder_name)):
        os.makedirs(folder_name);

    # Query three cameras
    images = client.airsim_client.simGetImages([
        airsim.ImageRequest("Front", airsim.ImageType.Scene),
        airsim.ImageRequest("Follow", airsim.ImageType.Scene),
        airsim.ImageRequest("Wheel", airsim.ImageType.Scene)])

    # Query both accelerometers
    sensor_values = client.airsim_client.readSensors()

    # Get the current bot state
    bot_state = client.airsim_client.getUrdfBotState()

    airsim.write_file(os.path.join(folder_name, '{0}_front.png'.format(unique_path_str)), images[0].image_data_uint8)
    airsim.write_file(os.path.join(folder_name, '{0}_follow.png'.format(unique_path_str)), images[1].image_data_uint8)
    airsim.write_file(os.path.join(folder_name, '{0}_wheel.png'.format(unique_path_str)), images[2].image_data_uint8)

    with open(os.path.join(folder_name, '{0}_state.json'.format(unique_path_str),), 'w') as f:
        f.write(jsonpickle.encode(bot_state))

    for sensor_name in [n for n in sensor_values['readings']]:
        data_file = os.path.join(folder_name, '{0}_data.csv'.format(sensor_name))
        data_dict = sensor_values['readings'][sensor_name]
        keys = sorted([k for k in data_dict])
        if not (os.path.isfile(data_file)):
            with open(data_file, 'w') as f:
                f.write('{0}\n'.format(','.join(keys)))

        with open(data_file, 'a') as f:
            f.write('{0}\n'.format(','.join([str(data_dict[k]) for k in keys])))


def main():
    capture_idx = 0
    print('Initializing...')
    client = BoxAndCylinderClient.BoxAndCylinderClient()
    query_sensors(client, str(capture_idx))
    capture_idx += 1
    time.sleep(2)
    os.system('cls')

    print('Actuating actuator...')
    print('Setting to full...')
    client.set_linear_actuator(1)
    time.sleep(5)
    print('Setting to none...')
    client.set_linear_actuator(0)
    time.sleep(5)
    print('Setting to mid')
    client.set_linear_actuator(0.5)
    time.sleep(5)

    print('Actuating servo...')
    print('Setting to left')
    client.set_servo(1)
    time.sleep(5)
    print('Setting to right')
    client.set_servo(0)
    time.sleep(5)
    print('Setting to mid')
    client.set_servo(0.5)
    time.sleep(5)
    
    print('Driving forward...')
    client.drive_forward()
    for i in range(0, 10, 1):
        time.sleep(0.3)
        query_sensors(client, str(capture_idx))
        capture_idx += 1

    print('Driving backward...')
    client.drive_backward()
    for i in range(0, 10, 1):
        time.sleep(0.3)
        query_sensors(client, str(capture_idx))
        capture_idx += 1

    print('Spinning wheels...')
    client.spin_wheels_in_opposite_directions()
    for i in range(0, 10, 1):
        time.sleep(0.3)
        query_sensors(client, str(capture_idx))
        capture_idx += 1

    print('Applying force...')
    client.stop_driving()
    client.push_car_sideways()
    for i in range(0, 10, 1):
        time.sleep(0.3)
        query_sensors(client, str(capture_idx))
        capture_idx += 1

    print('Stopping push...')
    client.stop_pushing_car_sideways()
    for i in range(0, 10, 1):
        time.sleep(0.3)
        query_sensors(client, str(capture_idx))
        capture_idx += 1

    print('Graceful termination.')

main()