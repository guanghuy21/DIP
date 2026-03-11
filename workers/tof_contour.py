import numpy as np
import matplotlib.pyplot as plt

# tof raw data
# raw_A = [20, 18, 16, 11, 12, 13, 15, 15, 13, 10, 15, 15, 
#          15, 15, 13, 15, 15, 15, 15, 15, 13, 15, 15, 15, 
#          15, 15, 13, 15, 15, 15, 15, 15, 15, 15, 15, 15]
# Starting at 3 o'clock (Right) -> Moving Counter-Clockwise
raw_A = [ 20, 19, 18, 18, 17, 16, 16, 18, 18, 18, 16, 16, 17, 20, 22, 21, 21, 21, 18, 15, 13, 12 , 13 , 18, 19 ,21, 21, 21, 20, 18, 17, 16, 16, 20, 21]
A =raw_A  

R_trajectory = 40     # = tree_radius + distance
num_points = len(A)   # no. scanning points
arc_span = 360        # full circle trajectory

# 1. Assign positions (angles) for a full 360 rotation
angles_deg = np.linspace(0, arc_span, num_points, endpoint=False)  #exclusive points / 36 intervals
angles_rad = np.radians(angles_deg)

# 2. Map Coordinates
x_trunk, y_trunk = [], []
x_sensor, y_sensor = [], []

for i in range(num_points):
    # tof trajectory (circle)
    xs = R_trajectory * np.cos(angles_rad[i])
    ys = R_trajectory * np.sin(angles_rad[i])
    x_sensor.append(xs)
    y_sensor.append(ys)
    
    # trunk contour mapping
    r_trunk = R_trajectory - A[i]
    x_trunk.append(r_trunk * np.cos(angles_rad[i]))
    y_trunk.append(r_trunk * np.sin(angles_rad[i]))

# 3. Plot
plt.figure(figsize=(12, 12))

# origin (0,0)
plt.plot(0, 0, 'kx', markersize=12, markeredgewidth=2, label='Orbit Center (0,0)')

# tof sensor trajectory
plt.plot(x_sensor, y_sensor, 'r--', alpha=0.6, label='Circular ToF Path (R=50)')

# path connotation
plt.plot(x_sensor, y_sensor, 'r--', alpha=0.3, label='Full Sensor Path')

# starting point (blue triangle)
plt.plot(x_sensor[0], y_sensor[0], 'b^', markersize=12, label='START Position')

# end point (red square)
plt.plot(x_sensor[17], y_sensor[17], 'rs', markersize=8, fillstyle='none', label='END Position')

# direction arrow (at 45 degrees to show rotation direction)
plt.annotate('', xy=(x_sensor[5], y_sensor[5]), xytext=(x_sensor[0], y_sensor[0]),
             arrowprops=dict(arrowstyle='->', color='blue', lw=2, connectionstyle="arc3,rad=.2"))

# contour mapping: first point to the end to close the line plot
plt.plot(np.append(x_trunk, x_trunk[0]), 
         np.append(y_trunk, y_trunk[0]), 
         'go-', markersize=4, label='Mapped Bark Contour')
plt.fill(x_trunk, y_trunk, 'g', alpha=0.15)

# show all of tof beams 
for i in range(len(A)): 
    plt.annotate('', xy=(x_trunk[i], y_trunk[i]), xytext=(x_sensor[i], y_sensor[i]),
                 arrowprops=dict(arrowstyle='<->', color='blue', lw=1, alpha=0.5))

# formatting
plt.axis('equal')
plt.title("Full 360° B-Scan: Bark Contour from Circular ToF Orbit")
plt.xlabel("X position (cm)")
plt.ylabel("Y position (cm)")
plt.grid(True, linestyle=':', alpha=0.5)
plt.legend(loc='upper right')

plt.show()