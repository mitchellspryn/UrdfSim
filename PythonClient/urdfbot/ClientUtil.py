import os
import jsonpickle
import setup_path
import airsim

class Utils():
    def __init__(self):
        self.__query_count = -1

    def query_bot_state(self, client, camera_names, folder_name, lidar_names = []):
        self.__query_count += 1
        if not (os.path.isdir(folder_name)):
            os.makedirs(folder_name);

        # Query three cameras
        images = client.airsim_client.simGetImages([airsim.ImageRequest(cam_name, airsim.ImageType.Segmentation) for cam_name in camera_names])

        # Query both accelerometers
        sensor_values = client.airsim_client.readSensors()

        # Get the current bot state
        bot_state = client.airsim_client.getUrdfBotState()

        for i in range(0, len(camera_names), 1):
            airsim.write_file(os.path.join(folder_name, '{0}_{1}.png'.format(self.__query_count, camera_names[i])), images[i].image_data_uint8)

        with open(os.path.join(folder_name, '{0}_state.json'.format(self.__query_count),), 'w') as f:
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

        for lidar_name in lidar_names:
            with open(os.path.join(folder_name, '{0}_{1}_lidar.xyz'.format(lidar_name, self.__query_count)), 'w') as f:
                data = client.airsim_client.getLidarData(lidar_name)
                point_cloud = data.point_cloud

                f.write('X,Y,Z\n')
                for i in range(0, len(point_cloud), 3):
                    f.write('{0},{1},{2}\n'.format(point_cloud[i], point_cloud[i+1], point_cloud[i+2]))




