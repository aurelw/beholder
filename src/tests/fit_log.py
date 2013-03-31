import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def func_exp2(x, a, b, c, d):
    return a * np.exp(-b * x**c) + d

def func_exp(x, a, b, c):
    return a * np.exp(-b * x) + c

def func_div(x, a, b, c):
    return a - b/(1+x)

def func_log(x, a, b, c):
    return a * np.log2(b * x) + c

def func_pow(x, a, b, c):
    return a - b / np.power(1+x,c)

x = np.array([0.2,0.21,0.23,0.25,0.3,0.32,0.34,0.36,0.38,0.4,0.42,0.44,0.46,0.48,0.5,0.52,0.54,0.56,0.58,0.596626,0.598633,0.612632,0.648629,0.729639,0.781638,0.793631,0.839644,0.909655,1.00163,1.16964,1.27866,1.42767,1.68268,1.84168,2.0447,2.24571,2.37672,2.57875,2.67374,2.69374,2.75473,3.00675,3.31077,3.50476,3.84386,4.1578,4.20691,4.41483,4.41483])
y = np.array([0,0.054902,0.152941,0.235294,0.364706,0.407843,0.454902,0.486275,0.513726,0.545098,0.564706,0.596078,0.607843,0.627451,0.643137,0.658824,0.666667,0.678431,0.690196,0.698039,0.705882,0.717647,0.729412,0.756863,0.764706,0.776471,0.784314,0.8,0.819608,0.843137,0.85098,0.87451,0.890196,0.894118,0.905882,0.913725,0.921569,0.921569,0.92549,0.921569,0.933333,0.933333,0.937255,0.937255,0.945098,0.941176,0.94902,0.945098,0.945098])

#y = func(x, 2.5, 1.3, 0.5)
#yn = y + 0.2*np.random.normal(size=len(x))

func = func_exp2

popt, pcov = curve_fit(func, x, y, maxfev=10000)


plt.figure()
plt.plot(x, y, 'ko', label="Focus Samples")
x_c = np.linspace(0.2,5,500)
plt.plot(x_c, func(x_c, *popt), 'r-', label="Fitted Curve")
plt.legend(loc='lower right')
plt.show()
