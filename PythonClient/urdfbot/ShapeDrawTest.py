import setup_path
import airsim
import airsim.airsim_types as at
import math
import time
import numpy as np
import pandas as pd

def main():
    client = airsim.UrdfBotClient()

    print('Building shape request...')

    dsr = at.DrawableShapeRequest(shapes = {}, persist_unmentioned = True)

    reference_frame_link = ''
    
    dsr = client.addDrawableShapePoint(dsr, 'point', reference_frame_link, 3, 0, 0, 1, 255, 0, 0, 0)
    dsr = client.addDrawableShapeSphere(dsr, 'sphere', reference_frame_link, 5, 0, 0, 1, 3, 64, 0, 255, 0, 0)
    dsr = client.addDrawableShapeCircle(dsr, 'circle', reference_frame_link, 0, 0, 5, 0, 1, 0, 1, 3, 64, 0, 0, 255, 0)
    dsr = client.addDrawableShapeBox(dsr, 'box', reference_frame_link, 0, -3, 0, 1, 2, 1, 10, 255, 255, 0, 0)
    dsr = client.addDrawableShapeLine(dsr, 'line', reference_frame_link, 1, 1, 1, 5, 5, 5, 1, 0, 255, 255, 0)

    print('Sending request')
    client.simSetDrawableShapes(dsr)

    print('Waiting...')
    time.sleep(2)

    print('Changing color of the sphere')

    dsr2 = at.DrawableShapeRequest(shapes = {}, persist_unmentioned = True)
    dsr2 = client.addDrawableShapeSphere(dsr2, 'sphere', reference_frame_link, 5, 0, 0, 1, 3, 64, 255, 0, 255, 0)

    print('Sending request')
    client.simSetDrawableShapes(dsr2)

    print('Waiting...')
    time.sleep(2)

    print('Dropping all but the sphere')

    dsr3 = at.DrawableShapeRequest(shapes = {}, persist_unmentioned = False)
    dsr3 = client.addDrawableShapeSphere(dsr3, 'sphere', reference_frame_link, 5, 0, 0, 1, 3, 64, 255, 0, 0, 0)

    print('Sending request')
    client.simSetDrawableShapes(dsr3)

    print('Waiting...')
    time.sleep(2)

    print('Dropping all')
    dsr4 = at.DrawableShapeRequest()
    
    print('Sending request')
    client.simSetDrawableShapes(dsr4)

    print('Graceful termination')

main()

