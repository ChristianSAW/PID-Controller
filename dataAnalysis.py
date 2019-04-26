import matplotlib.pyplot as plt
import numpy as np

#text_file = open("output_files/datalog_1.txt", "r")
#lines = text_file.read().split(' ')

print("Testing things")

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


y_cte = np.array(dl1.cte)
#t = np.linspace(0, len(y_cte)-1, len(y_cte))

#for i in range(len(y_cte)):
#    print(y_cte[i])


#plt.plot(t, dl1.cte)
plt.plot(y_cte)
plt.ylabel('CTE')
plt.show()

