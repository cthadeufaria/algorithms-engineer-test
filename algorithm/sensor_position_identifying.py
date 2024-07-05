import numpy as np
import matplotlib.pyplot as plt
from supportlib.data_types import Sensor, SensorData
from supportlib.sensor_position_finding import SensorPositionFinder, SensorPositionRequester
from algorithm.kinematics_model import Body
from algorithm.kalman_filter import KalmanFilter
from algorithm.classification import Classifier


class SensorPositionIdentifier(SensorPositionFinder):
    def __init__(self, 
                 position_requester: SensorPositionRequester):
        super().__init__(position_requester)
        self.body = Body()
        self.kalman_filter = dict()
        self.classifier = Classifier()

    def on_new_sensor_sample(self,
                             data_dict: dict[Sensor, SensorData]):

        if len(self.kalman_filter) == 0:
            self.on_first_observation(data_dict)

        for sensor, sensor_data in data_dict.items():
            self.iterate_kalman_filter(sensor, 
                                       sensor_data)

        self.classifier.classify(self.kalman_filter,
                                 data_dict)

        # TODO: identify the sensor position based on the sensor data
        # self.sensor_positions = None
        
        # TODO: call on_sensor_position_found when an unknown sensor position is identified
        # if self.classifier.positions not in self.sensor_positions and isinstance(sensor_position, SensorPosition):
        #     self.position_requester.on_sensor_position_found(sensor, sensor_position)

        # TODO: call on_finish when all sensor positions are identified
        # self.position_requester.on_finish()

    def on_first_observation(self, 
                     data_dict: dict[Sensor, SensorData]):
        for sensor in data_dict.keys():
            self.kalman_filter[sensor] = (KalmanFilter(), KalmanFilter(), KalmanFilter())

    def iterate_kalman_filter(self, 
                              sensor: Sensor, 
                              sensor_data: SensorData):
        self.kalman_filter[sensor][0].update_state_space_system(sensor_data[0][0], 
                                                sensor_data[1])
        self.kalman_filter[sensor][1].update_state_space_system(sensor_data[0][1], 
                                                sensor_data[1])
        self.kalman_filter[sensor][2].update_state_space_system(-sensor_data[0][2], 
                                                sensor_data[1])
        
        for kalman_filter in self.kalman_filter[sensor]:
            kalman_filter.predict()
            kalman_filter.update()
            kalman_filter.append_coordinates()