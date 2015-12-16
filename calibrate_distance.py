import math
import numpy as np
from matplotlib import pyplot as plt

with open('5m-distances-away.txt', 'r') as f:
    distances = f.read().split('\n')[:-1]
    # last one is always empty, so leave it off

blob_size = [float(x) for x in (distances)]

logblob = []
for e in blob_size:
    if e > 2:
        logblob.append(1/math.sqrt(e))
#logblob = [math.log(d) for d in blob_size]

dist = np.linspace(0, 192, len(logblob))

# minblob = min(logblob) - 1
#
# logblob = [d - minblob for d in logblob]

deg = 1

fit = np.polyfit(logblob, dist, deg)

x = np.linspace(0, 200, len(logblob))
y = []

for i, term in enumerate(fit):
    y.append(term * x**(deg-i))

base = np.zeros(len(y[0]))
for arr in y:
    base += arr

y = base

#y = [d + (minblob-1) for d in y]

#y = np.exp(y)

print(fit)

plt.plot(x, y, logblob, dist)
plt.ylim([0, 1000])
plt.show()
