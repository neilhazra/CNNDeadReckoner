from sklearn.ensemble import RandomForestRegressor
from sklearn.datasets import make_regression
import numpy as np
#X, y = make_regression(n_features=4, n_informative=2, random_state=0, shuffle=False)



#TODO
regrX = RandomForestRegressor(max_depth=20, random_state=0)
regrY = RandomForestRegressor(max_depth=20, random_state=0)


Xinput = []
Xoutput = []

Yinput = []
Youtput = []

data = open('D:/RoboShopRobot/TrainingData.txt')
data.readline()
i = 0
for line in data:
    temp = line.rstrip().split(',')
    Xinput.append([])
    Yinput.append([])
    Xoutput.append([])
    Youtput.append([])
    Xinput[i] = [float(temp[1]), float(temp[2]), float(temp[3]), float(temp[6]), float(temp[8])]
    Xoutput[i] = [float(temp[4])]
    Yinput[i] = [float(temp[1]), float(temp[2]), float(temp[3]), float(temp[7]), float(temp[9])]
    Youtput[i] = [float(temp[5])]
    i = i+1

Xoutput = np.ravel(Xoutput)
Youtput = np.ravel(Youtput)
regrX.fit(Xinput, Xoutput)
regrY.fit(Yinput, Youtput)


print(regrX.feature_importances_)
print(regrY.feature_importances_)

print(regrX.predict([[2, 1, 5.72, -1.1,-1.4]]))
print(regrY.predict([[2, 1, 5.72, 31,32]]))
