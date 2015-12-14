import numpy as np
from matplotlib import pyplot as plt

with open('distances.txt', 'r') as f:
    distances = f.read().split('\n')[:-1]
    # last one is always empty, so leave it off

distances = [float(x) for x in distances]

actual = np.linspace(30, 5, len(distances))

start = 350
distances = distances[:-start]
actual = actual[:-start]

deg = 4

fit = np.polyfit(actual, distances, deg)

x = np.linspace(50, 0, len(distances))
y = []

for i, term in enumerate(fit):
    y.append(term * x**(deg-i))

base = np.zeros(len(y[0]))
for arr in y:
    base += arr

y = base

print(fit)

plt.plot(x,y, actual, distances)
plt.show()
