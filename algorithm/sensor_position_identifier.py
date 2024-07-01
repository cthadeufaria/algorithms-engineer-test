from supportlib.sensor_position_finding import SensorPositionFinder, SensorPositionRequester
from algorithm.kinematics_model import KinematicModel


class SensorPositionIdentifier(SensorPositionFinder):
    def __init__(self, 
                 position_requester: SensorPositionRequester):
        super().__init__(position_requester)

    def on_new_sensor_sample(self, 
                             data_dict):
        for sensor, sensor_data in data_dict.items():
            # TODO: identify the sensor position based on the sensor data
            sensor_position = None

            # TODO: call on_sensor_position_found when an unknown sensor position is identified
            if sensor_position not in self.sensor_positions:
                self.position_requester.on_sensor_position_found(sensor, sensor_position)

            # TODO: call on_finish when all sensor positions are identified
            self.position_requester.on_finish()