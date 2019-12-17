import setup_path
import airsim
import airsim.airsim_types as at
import math
import time
import numpy as np
import pandas as pd
import random
import time

def main():
    client = airsim.UrdfBotClient()

    random.seed(42)


    cone_class_name = "/Game/StarterContent/Props/SM_Couch.SM_Couch"

    xs = [0, 0.1]
    ys = [0, 0.1]
    time_stamp = 0

    for i in range(0, 3, 1):
        xs.append(random.randint(0, 10))
        ys.append(random.randint(0, 10))

    for i in range(0, len(xs), 1):
        x = xs[i]
        y = ys[i]
        random_position = at.Vector3r(x_val=x, y_val=y, z_val=5)
        random_orientation = at.Quaternionr(x_val=0, y_val=0, z_val=0, w_val=1)
        random_pose = at.Pose(position_val=random_position, orientation_val=random_orientation)

        print('Spawning cone at ({0}, {1})...'.format(x, y))
        client.simSpawnStaticMeshObject(cone_class_name, 'cone_{0}'.format(i), random_pose)

        time.sleep(2)
        ci = client.simGetCollisionInfo()
        if (ci.has_collided and ci.time_stamp > time_stamp):
            print('\tBot has collided with {0}.'.format(ci.object_name))
            time_stamp = ci.time_stamp
        else:
            print('\tBot has not collided.')

    time.sleep(3)

    for i in range(0, len(xs), 1):
        cone_name = 'cone_{0}'.format(i)
        cone_pose = client.simGetObjectPose(cone_name)
        print('Cone "{0}" is at position <{1}, {2}, {3}>'.format(cone_name, cone_pose.position.x_val, cone_pose.position.y_val, cone_pose.position.z_val))

    time.sleep(3)
    
    for i in range(0, len(xs), 1):
        cone_name = 'cone_{0}'.format(i)
        print('Deleting cone {0}...'.format(cone_name))
        client.simDeleteObject(cone_name)

    print('Graceful termination')

main()


