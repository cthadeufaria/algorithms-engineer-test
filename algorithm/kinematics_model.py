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
        self.limbs = [Limb(np.array((0., 0, -1.)), 0.) for _ in range(5)]
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
                    for theta in range(0, np.pi/2):
                        limb.movement.append(length*np.sqrt(2*(1-np.cos(theta))))
                        #TODO: add noise to the movement

            elif limb.position == SensorPosition.RIGHT_SHANK or limb.position == SensorPosition.LEFT_SHANK:
                for length in limb.lengths:
                    for theta in range(0, np.pi/2):
                        limb.movement.append(length*np.sin(theta))
                        limb.movement = add_noise(limb.movement)
                        #TODO: add noise to the movement

            elif limb.position == SensorPosition.CHEST:
                for length in limb.lengths:
                    for theta in range(0, np.pi/2):
                        limb.movement.append(0.)
                        limb.movement = add_noise(limb.movement)
                        #TODO: add noise to the movement