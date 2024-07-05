import numpy as np
from supportlib.data_types import SensorPosition, SensorData
from algorithm.utils import *


class Limb:
    """A class representing a limb of the patient."""
    def __init__(self):
        self.movement = list()
        self.position = None
        self.lengths = None

class Body:
    """A class representing the body of the patient."""
    def __init__(self):
        self.limbs = [Limb() for _ in range(5)]
        self.assign_positions()
        self.assign_lengths()
        self.perform_leg_movements()
        self.compute_centroids()

    def assign_positions(self):
        """Assign positions to the limbs."""
        self.limbs[0].position = SensorPosition.RIGHT_THIGH
        self.limbs[1].position = SensorPosition.RIGHT_SHANK
        self.limbs[2].position = SensorPosition.LEFT_THIGH
        self.limbs[3].position = SensorPosition.LEFT_SHANK
        self.limbs[4].position = SensorPosition.CHEST

    def assign_lengths(self):
        """Assign lengths to the limbs."""
        self.limbs[0].lengths = range(40, 50)
        self.limbs[1].lengths = range(33, 43)
        self.limbs[2].lengths = range(40, 50)
        self.limbs[3].lengths = range(33, 43)
        self.limbs[4].lengths = range(65, 75)

    def perform_leg_movements(self):
        """Create model dataset from performing repetition of calibration leg movement."""
        time_ranges = [
            np.linspace(0, 2.5, 125), 
            np.linspace(2.5, 5, 125), 
            np.linspace(5, 7.5, 125),
            np.linspace(7.5, 10, 125)
        ]

        for limb in self.limbs:
            if limb.position == SensorPosition.RIGHT_THIGH:
                for length in limb.lengths:
                    for time, theta in zip(time_ranges[0], np.linspace(0., np.pi/2, 125)):
                        limb.movement.append(np.array([time, length*np.sin(theta), 0., length*(1 - np.cos(theta))]))
                    for time, theta in zip(time_ranges[1], np.linspace(np.pi/2, 0., 125)):
                        limb.movement.append(np.array([time, length*np.sin(theta), 0., length*(1 - np.cos(theta))]))
                    for time in time_ranges[2]:
                        limb.movement.append(np.array([time, 0., 0., 0.]))
                    for time in time_ranges[3]:
                        limb.movement.append(np.array([time, 0., 0., 0.]))

            elif limb.position == SensorPosition.LEFT_THIGH:
                for length in limb.lengths:
                    for time in time_ranges[0]:
                        limb.movement.append(np.array([time, 0., 0., 0.]))
                    for time in time_ranges[1]:
                        limb.movement.append(np.array([time, 0., 0., 0.]))
                    for time, theta in zip(time_ranges[2], np.linspace(0., np.pi/2, 125)):
                        limb.movement.append(np.array([time, length*np.sin(theta), 0., length*(1 - np.cos(theta))]))
                    for time, theta in zip(time_ranges[3], np.linspace(np.pi/2, 0., 125)):
                        limb.movement.append(np.array([time, length*np.sin(theta), 0., length*(1 - np.cos(theta))]))

            elif limb.position == SensorPosition.RIGHT_SHANK:
                for length in limb.lengths:
                    for time, theta in zip(time_ranges[0], np.linspace(0., np.pi/2, 125)):
                        limb.movement.append(np.array([time, 0., 0., length*(1 - np.cos(theta))]))
                    for time, theta in zip(time_ranges[1], np.linspace(np.pi/2, 0., 125)):
                        limb.movement.append(np.array([time, 0., 0., length*(1 - np.cos(theta))]))
                    for time in time_ranges[2]:
                        limb.movement.append(np.array([time, 0., 0., 0.]))
                    for time in time_ranges[3]:
                        limb.movement.append(np.array([time, 0., 0., 0.]))

            elif limb.position == SensorPosition.LEFT_SHANK:
                for length in limb.lengths:
                    for time in time_ranges[0]:
                        limb.movement.append(np.array([time, 0., 0., 0.]))
                    for time in time_ranges[1]:
                        limb.movement.append(np.array([time, 0., 0., 0.]))
                    for time, theta in zip(time_ranges[2], np.linspace(0., np.pi/2, 125)):
                        limb.movement.append(np.array([time, 0., 0., length*(1 - np.cos(theta))]))
                    for time, theta in zip(time_ranges[3], np.linspace(np.pi/2, 0., 125)):
                        limb.movement.append(np.array([time, 0., 0., length*(1 - np.cos(theta))]))

            elif limb.position == SensorPosition.CHEST:
                for length in limb.lengths:
                    for time in time_ranges[0]:
                        limb.movement.append(np.array([time, 0., 0., 0.]))
                    for time in time_ranges[1]:
                        limb.movement.append(np.array([time, 0., 0., 0.]))
                    for time in time_ranges[2]:
                        limb.movement.append(np.array([time, 0., 0., 0.]))
                    for time in time_ranges[3]:
                        limb.movement.append(np.array([time, 0., 0., 0.]))

    def compute_centroids(self):
        for limb in self.limbs:
            limb.centroid = compute_centroids(limb.movement)