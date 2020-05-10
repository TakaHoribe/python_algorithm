
#!/usr/bin/env python


import math
import time
import matplotlib
matplotlib.use('tkagg')
import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as linalg
from matplotlib import gridspec


x_orig = np.arange(-200.0, 1.0, 0.10)
x = [10**p for p in x_orig]
y1 = [-math.log(p) for p in x]
y2 = [-0.5 * math.log(p) for p in x]
y3 = [-0.1 * math.log(p) for p in x]
y4 = [-0.01 * math.log(p) for p in x]

plt.figure(1)
plt.plot(x, y1, label="nu = 1")
plt.plot(x, y2, label="nu = 0.5")
plt.plot(x, y3, label="nu = 0.1")
plt.plot(x, y4, label="nu = 0.01")
plt.xlim((-0.5, 8))
plt.ylim((-2, 3))
plt.title("-nu * log(x)")
plt.grid()
plt.legend()
plt.show()

# print(y1)
# print(x)