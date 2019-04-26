import matplotlib.pyplot as plt
import numpy as np

print("Begin Analysis")

# Make Data Structure
class DataLog:
    speed = []
    throttle = []
    steer_value = []
    angle = []
    cte = []


# populate structure
dl1 = DataLog()
with open("output_files/datalog_1.txt", "r") as f:
    for line in f:
        row = line.split(" ")
        # populate datalog dl1
        dl1.speed.append(float(row[0]))
        dl1.throttle.append(float(row[1]))
        dl1.steer_value.append(float(row[2]))
        dl1.angle.append(float(row[3]))
        dl1.cte.append(float(row[4]))


# PLOTTING

figX = 16
figY = 8

y_cte = np.array(dl1.cte)
y_speed = np.array(dl1.speed)
y_throttle = np.array(dl1.throttle)
y_steer = np.array(dl1.steer_value)
y_angle = np.array(dl1.angle)
t = np.linspace(0, len(y_cte)-1, len(y_cte))


# Plot CTE
f0 = plt.figure(1)
plt.plot(y_cte)
plt.ylabel('CTE')
f0.show()

# PLOT CTE, Steer_Value, Angle
f1, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(figX, figY))
f1.tight_layout()

ax1.plot(y_cte)
ax1.set_title('CTE')

ax2.plot(y_steer)
ax2.set_title('Steering Value')

ax3.plot(y_angle)
ax3.set_title('Angle')

f1.show()

# Plote CTE, Throttle, Speed

f2, (ax4, ax5, ax6) = plt.subplots(3, 1, figsize=(figX, figY))
f2.tight_layout()

ax4.plot(y_cte)
ax4.set_title('CTE')
#ax4.show()

ax5.plot(y_throttle)
ax5.set_title('Throttle')
#ax5.show()

ax6.plot(y_speed)
ax6.set_title('Speed')
#ax6.show()

f2.show()

# must call this line to keep figs alive
input()