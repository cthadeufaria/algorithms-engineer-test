import matplotlib.pyplot as plt
fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot(projection='3d')
for limb in self.limbs:
    x = [a[0] for a in limb.movement]
    y = [a[1] for a in limb.movement]
    z = [a[2] for a in limb.movement]
    ax.scatter(x, y, z, label=str(limb.position))
    ax.legend()
plt.savefig("imgs/kinematic_model_xyz.png")

import matplotlib.pyplot as plt
fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot(projection='3d')
for sensor in CORRECT_POSITIONS.values():
    x = self.position_finder.kalman_filter[sensor][0].coordinates
    y = self.position_finder.kalman_filter[sensor][1].coordinates
    z = self.position_finder.kalman_filter[sensor][2].coordinates
    ax.scatter(x, y, z, label=str(sensor))
    ax.legend()
plt.savefig("imgs/sensors_xyz.png")
