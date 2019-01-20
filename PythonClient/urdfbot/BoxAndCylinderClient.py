import setup_path 
import airsim
import airsim.airsim_types as at

class BoxAndCylinderClient():
    def __init__(self):
        self.airsim_client = airsim.UrdfBotClient()
        self.airsim_client.confirmConnection()
        self.airsim_client.enableApiControl(True)

        self.__front_wheel_force_name = 'front_wheel_force'
        self.__back_wheel_force_name = 'back_wheel_force'
        self.__sideways_force_name = 'sideways_force'
        self.__actuator_name = 'base_to_linear_actuated'
        self.__servo_name = 'linear_actuated_to_servo'
        self.__wheel_speed = 50.0
        self.__sideways_force_mag = 10000.0

        #Add forces
        front_wheel_angular_force = at.AddAngularForce(force_name_val=self.__front_wheel_force_name, link_name_val='front_cyl', axis_val=at.Vector3r(x_val=0, y_val=1, z_val=0))
        back_wheel_angular_force = at.AddAngularForce(force_name_val=self.__back_wheel_force_name, link_name_val='back_cyl', axis_val=at.Vector3r(x_val=0, y_val=1, z_val=0))
        sideways_force_name = at.AddLinearForce(force_name_val=self.__sideways_force_name, link_name_val='base_link', application_point_val=at.Vector3r(x_val=0, y_val=0, z_val=0.5), axis_val=at.Vector3r(x_val=0, y_val=1, z_val=0))

        self.airsim_client.addAngularForce(front_wheel_angular_force)
        self.airsim_client.addAngularForce(back_wheel_angular_force)
        self.airsim_client.addLinearForce(sideways_force_name)

    def drive_forward(self):
        self.__drive(self.__wheel_speed, self.__wheel_speed)

    def drive_backward(self):
        self.__drive(-1.0 * self.__wheel_speed, -1.0 * self.__wheel_speed)

    def stop_driving(self):
        self.__drive(0.0, 0.0)

    def spin_wheels_in_opposite_directions(self):
        self.__drive(1.0 * self.__wheel_speed, -1.0 * self.__wheel_speed)

    def push_car_sideways(self):
        self.__sideways_push(self.__sideways_force_mag)

    def stop_pushing_car_sideways(self):
        self.__sideways_push(0.0)

    def set_linear_actuator(self, control_signal_value):
        if (control_signal_value < 0 or control_signal_value > 1):
            raise ValueError('control_signal_value = {0}, which is outside the range [0, 1]'.format(control_signal_value))
        
        update_value = at.UrdfBotControlledMotionComponentControlSignal(component_name=self.__actuator_name, control_signal_values={'Value': control_signal_value})

        self.airsim_client.updateControlledMotionComponentControlSignal(update_value, vehicle_name='')

    def set_servo(self, control_signal_value):
        if (control_signal_value < 0 or control_signal_value > 1):
            raise ValueError('control_signal_value = {0}, which is outside the range [0, 1]'.format(control_signal_valuew))

        update_value = at.UrdfBotControlledMotionComponentControlSignal(component_name=self.__servo_name, control_signal_values={'Value': control_signal_value})

        self.airsim_client.updateControlledMotionComponentControlSignal(update_value, vehicle_name='')

    def __drive(self, front_speed, back_speed):
        forward_update_obj = at.UpdateForceMagnitude(force_name_val=self.__front_wheel_force_name, magnitude_val=front_speed)
        backward_update_obj = at.UpdateForceMagnitude(force_name_val=self.__back_wheel_force_name, magnitude_val=back_speed)

        self.airsim_client.updateForceMagnitude(forward_update_obj)
        self.airsim_client.updateForceMagnitude(backward_update_obj)

    def __sideways_push(self, force_mag):
        sideways_update_obj = at.UpdateForceMagnitude(force_name_val=self.__sideways_force_name, magnitude_val=force_mag)
        self.airsim_client.updateForceMagnitude(sideways_update_obj)