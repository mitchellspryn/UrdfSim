import setup_path
import airsim
import airsim.airsim_types as at
import math
import time
import numpy as np
import pandas as pd

def main():
    client = airsim.UrdfBotClient()

    print('Building request...')
    
    position = at.Vector3r(x_val = 0, y_val = 0, z_val = 100)
    direction = at.Vector3r(x_val = 0, y_val = 0, z_val = -100)
    through_blocking = True
    persist_seconds = 100
    reference_frame_link = ''

    request = at.RayCastRequest(position, direction, reference_frame_link, through_blocking, persist_seconds)
    
    print('Casting ray')
    response = client.simRayCast(request)

    print('Response:')
    print('{0} hits.'.format(len(response['hits'])))

    for i in range(0, len(response['hits']), 1):
        hit = response['hits'][i]
        print('=================')
        print('hit {0}'.format(i))
        print('collided actor: {0}'.format(hit['collided_actor_name']))
        print('hit_point: <{0}, {1}, {2}>'.format(hit['hit_point']['x_val'], hit['hit_point']['y_val'], hit['hit_point']['z_val']))
        print('hit normal: <{0}, {1}, {2}>'.format(hit['hit_normal']['x_val'], hit['hit_normal']['y_val'], hit['hit_normal']['z_val']))
        print('=================')
        print('')

    print('Graceful termination.')

main()
