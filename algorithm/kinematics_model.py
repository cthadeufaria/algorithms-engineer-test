import numpy as np


class Limb:
    def __init__(self, name, dh_params):
        self.name = name
        self.dh_params = dh_params

    def dh_transform(self, theta, d, a, alpha):
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, joint_angles):
        T = np.eye(4)
        for i, (theta, d, a, alpha) in enumerate(self.dh_params):
            theta += joint_angles[i]
            T = np.dot(T, self.dh_transform(theta, d, a, alpha))
        return T[:3, 3], T[:3, :3]


class Body:
    def __init__(self):
        # Define DH parameters for each limb
        self.right_shank = Limb('right_shank', [
            [0, 0, 0, np.pi/2],
            [0, 0, 1, 0],
            [0, 0, 1, 0]
        ])
        self.left_shank = Limb('left_shank', [
            [0, 0, 0, np.pi/2],
            [0, 0, 1, 0],
            [0, 0, 1, 0]
        ])
        # Define other limbs similarly...

    def simulate_movement(self, timesteps):
        right_shank_positions = []
        left_shank_positions = []

        for t in timesteps:
            # Simulate joint angles over time
            if t < len(timesteps) // 2:
                theta_rh = 30 * np.pi / 180 * (t / (len(timesteps) // 2))
                theta_rk = 90 * np.pi / 180 * (t / (len(timesteps) // 2))
                theta_lh = 0
                theta_lk = 0
            else:
                theta_rh = 0
                theta_rk = 0
                theta_lh = 30 * np.pi / 180 * ((t - len(timesteps) // 2) / (len(timesteps) // 2))
                theta_lk = 90 * np.pi / 180 * ((t - len(timesteps) // 2) / (len(timesteps) // 2))

            right_shank_position, _ = self.right_shank.forward_kinematics([theta_rh, theta_rk, 0])
            left_shank_position, _ = self.left_shank.forward_kinematics([theta_lh, theta_lk, 0])

            right_shank_positions.append(right_shank_position)
            left_shank_positions.append(left_shank_position)

        return right_shank_positions, left_shank_positions


# Example usage
body = Body()
timesteps = np.linspace(0, 1, 50)  # 50 timesteps for the movement
right_shank_positions, left_shank_positions = body.simulate_movement(timesteps)

print("Right Shank Positions:", right_shank_positions)
print("Left Shank Positions:", left_shank_positions)
