import numpy as np
from supportlib.data_types import SensorPosition, SensorData
from algorithm.utils import add_noise, save_csv


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
        for limb in self.limbs:
            if limb.position == SensorPosition.RIGHT_THIGH or limb.position == SensorPosition.LEFT_THIGH:
                for length in limb.lengths:
                    for theta in np.linspace(0, np.pi/2, 50):
                        limb.movement.append(np.array([length*np.sin(theta), length*np.sqrt(2*(1-np.cos(theta)))]))
                        #TODO: add noise to the movement

            elif limb.position == SensorPosition.RIGHT_SHANK or limb.position == SensorPosition.LEFT_SHANK:
                for length in limb.lengths:
                    for theta in np.linspace(0, np.pi/2, 50): # TODO: check if this function is correct by plotting the sensor filtered data
                        limb.movement.append(np.array([0., length*np.sin(theta)]))
                        #TODO: add noise to the movement

            elif limb.position == SensorPosition.CHEST:
                for length in limb.lengths:
                    for _ in np.linspace(0, np.pi/2, 50):
                        limb.movement.append((0.))
                        #TODO: add noise to the movement