from optparse import OptionParser

import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt


def samplesFromCSV(filename):
    upSamples = []
    downSamples = []

    with open(filename) as f:
        for line in f:
            fields = line.split(',')
            if len(fields) == 3:
                sample = (float(fields[0]), float(fields[1]), int(fields[2]))
                if sample[2] == 0:
                    upSamples.append(sample)
                else:
                    downSamples.append(sample)

    return (upSamples, downSamples)


def expPowFunc(x, a, b, c, d):
    return a * np.exp(-b * x**c) + d


def decomposeSamples(samples):
    x = []
    y = []
    for sample in samples:
        x.append(sample[0])
        y.append(sample[1])
    return (x, y)


def fitFunction(samples, fitFunc):
    #prepare numpy array
    (x, y) = decomposeSamples(samples)
    xarr = np.array(x)
    yarr = np.array(y)

    # do fitting
    return curve_fit(fitFunc, xarr, yarr, maxfev=10000)


def plotSingle(samples, popt, func, minX=0.2, maxX=8.0):
    (x, y) = decomposeSamples(samples)
    plt.figure()
    plt.plot(x, y, 'rx', label="Focus Samples")
    x_c = np.linspace(minX, maxX, 1200)
    plt.plot(x_c, func(x_c, *popt), 'r-', label="Fitted Curve")
    plt.legend(loc='lower right')
    plt.show()


def plotDouble(samples_0, popt_0, samples_1, popt_1, func, minX=0.2, maxX=8.0):
    (x0, y0) = decomposeSamples(samples_0)
    (x1, y1) = decomposeSamples(samples_1)
    plt.figure()
    plt.plot(x0, y0, 'rx', label="Focus Samples Up")
    plt.plot(x1, y1, 'bx', label="Focus Samples Down")
    x_c = np.linspace(minX, maxX, 1200)
    plt.plot(x_c, func(x_c, *popt_0), 'r-', label="Fitted Curve Up")
    plt.plot(x_c, func(x_c, *popt_1), 'b-', label="Fitted Curve Down")
    plt.legend(loc='lower right')
    plt.show()


def main():

    ### parse command line arguments ###
    usage = "usage: %prog [options] <filename>"
    parser = OptionParser(usage)
    parser.add_option("-p", action="store_true", dest="plot")
    (options, args) = parser.parse_args()

    if len(args) == 0:
        parser.print_help()
        return

    filename = args[0]

    #specify function to fit
    func = expPowFunc
    #load data
    (upSamples, downSamples) = samplesFromCSV(filename)
    #fit to data
    (poptUp, pcovUp) = fitFunction(upSamples, func)
    (poptDown, pcovDown) = fitFunction(downSamples, func)

    if (options.plot):
        plotDouble(upSamples, poptUp, downSamples, poptDown, func) 

    print(" ".join(map(str, poptUp)))
    print(" ".join(map(str, poptDown)))


main()
    
