import numpy as np
import matplotlib.pyplot as plt

# Added: Calculate p from student IDs
y = [330, 942]
x = sum(y)
xs = [x]
for i in range(2):
    x = (112 * x) % 111 + 2
    xs.append(x)
print("x0, x1, x2 =", xs)
p = 0.31 + xs[2] / 1000
print("p =", p)

# This is a source code in Sage math
n = 10000 # sample size
N = 10 # number of trial in

# Added: Set seed 
np.random.seed(42)

# Random sequence of Binomial RV.
X = np.random.binomial(N, p, n)
minX = X.min()
maxX = X.max()
e = np.arange(minX - 0.5, maxX + 1)
np.histogram(X, bins=e)
plt.hist(X, bins=e, fill=False)
plt.title("Histogram with bins = e")
plt.show()

Mn = np.mean(X)
pn = Mn / N

# Added: Display simulation results
print("counts =", np.histogram(X, bins=e)[0])
print("min, max =", minX, maxX)
print("Mn =", Mn, " pn =", pn)

# PMF of Binomial Distributions
from scipy import stats
from scipy.stats import binom
k = np.arange(0, N + 1)
PMF = binom.pmf(k, N, pn)
fig, ax = plt.subplots(1, 1)
ax.plot(k, PMF, 'ro', ms=8, mec='r')
ax.vlines(k, 0, PMF, colors='r', lw=3)
plt.show()



