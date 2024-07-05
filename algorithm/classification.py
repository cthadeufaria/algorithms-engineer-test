import numpy as np
from supportlib.data_types import SensorPosition


class Classifier:
    def __init__(self):
        self.estimated_xyz = dict()
        self.angles = dict()
        self.sensor_positions = dict()
        self.distances = dict()

    def classify(self, kalman_filter, sensor_data):
        for sensor, data in kalman_filter.items():
            self.estimated_xyz[sensor] = np.array([
                data[0].coordinates[-1], 
                data[1].coordinates[-1], 
                data[2].coordinates[-1]
            ])

        for sensor, data in sensor_data.items():
            self.angles[sensor] = np.array([
                np.arccos(data[0][0]),
                (np.pi/2) - np.arcsin(data[0][1]), 
                (np.pi) - np.arccos(-data[0][2])
            ]) * (180 / np.pi)

        self.distance_from_origin()
        return self.decision_tree()

    def distance_from_origin(self):
        for sensor, xyz in self.estimated_xyz.items():
            self.distances[sensor] = np.linalg.norm(xyz - np. array([0, 0, 0]))

    def decision_tree(self):
        for sensor, angle in self.angles.items():
            if angle[0] < 10:
                if SensorPosition.RIGHT_THIGH not in self.sensor_positions.values():
                    self.sensor_positions[sensor] = SensorPosition.RIGHT_THIGH
                    return sensor, SensorPosition.RIGHT_THIGH

                elif SensorPosition.RIGHT_SHANK not in self.sensor_positions.values():
                    max_key = max(self.distances, key=self.distances.get)
                    self.sensor_positions[max_key] = SensorPosition.RIGHT_SHANK
                    return max_key, SensorPosition.RIGHT_SHANK

                elif SensorPosition.LEFT_THIGH not in self.sensor_positions.values() and sensor not in self.sensor_positions.keys():
                    self.sensor_positions[sensor] = SensorPosition.LEFT_THIGH
                    return sensor, SensorPosition.LEFT_THIGH

                elif len(self.sensor_positions.values()) == 3:
                    max_key = max(self.distances, key=self.distances.get)
                    self.sensor_positions[max_key] = SensorPosition.LEFT_SHANK
                    return max_key, SensorPosition.LEFT_SHANK
                            
            if len(self.sensor_positions) == 4 and sensor not in self.sensor_positions.keys():
                self.sensor_positions[sensor] = SensorPosition.CHEST
                return sensor, SensorPosition.CHEST

        return None, None