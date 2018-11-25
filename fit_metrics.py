import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import pandas as pd 


def func(x, a, b, c):
    return 1 - (a * np.exp(-b * x) + c)


xdata = np.arange(1,21)
#ydata = pd.read_csv('./map_metric_data/AverageEntropyIg/occupied_voxel_volume.csv', sep=',',header=None)

y_RearSideVoxelIg = np.genfromtxt('./map_metric_data/RearSideVoxelIg/occupied_voxel_volume.csv', delimiter=',')
y_RearSideVoxelIg = y_RearSideVoxelIg[~np.isnan(y_RearSideVoxelIg)]

y_VGIg = np.genfromtxt('./map_metric_data/VasquezGomezAreaFactorIg/occupied_voxel_volume.csv', delimiter=',')
y_VGIg = y_VGIg[~np.isnan(y_VGIg)]

y_OcclusionAwareIg = np.genfromtxt('./map_metric_data/OcclusionAwareIg/occupied_voxel_volume.csv', delimiter=',')
y_OcclusionAwareIg = y_OcclusionAwareIg[~np.isnan(y_OcclusionAwareIg)]

y_UnobservedVoxelIg = np.genfromtxt('./map_metric_data/UnobservedVoxelIg/occupied_voxel_volume.csv', delimiter=',')
y_UnobservedVoxelIg = y_UnobservedVoxelIg[~np.isnan(y_UnobservedVoxelIg)]

y_RearSideEntropyIg = np.genfromtxt('./map_metric_data/RearSideEntropyIg/occupied_voxel_volume.csv', delimiter=',')
y_RearSideEntropyIg = y_RearSideEntropyIg[~np.isnan(y_RearSideEntropyIg)]

y_ProximityCountIg = np.genfromtxt('./map_metric_data/ProximityCountIg/occupied_voxel_volume.csv', delimiter=',')
y_ProximityCountIg = y_ProximityCountIg[~np.isnan(y_ProximityCountIg)]



plt.plot(xdata, y_RearSideVoxelIg, label='RearSideVoxel')
plt.plot(xdata, y_VGIg, label='VasquezGomez')
plt.plot(xdata, y_OcclusionAwareIg, label='OcclusionAware')
plt.plot(xdata, y_UnobservedVoxelIg, label='UnobservedVoxel')
plt.plot(xdata, y_ProximityCountIg, label='ProximityCount')
plt.plot(xdata, y_RearSideEntropyIg, label='RearSideEntropy')
plt.xticks(np.arange(1, 20, step=1))
plt.xlabel('Iteration')
plt.ylabel('SurfaceArea')
plt.legend(loc='best')
plt.show()



popt, pcov = curve_fit(func, xdata, y_RearSideVoxelIg)
plt.plot(xdata, func(xdata, *popt), label='RearSideVoxelIg fit: a=%5.3f, b=%5.3f, c=%5.3f' % tuple(popt))

popt, pcov = curve_fit(func, xdata, y_VGIg)
plt.plot(xdata, func(xdata, *popt), label='VasquezGomez fit: a=%5.3f, b=%5.3f, c=%5.3f' % tuple(popt))

popt, pcov = curve_fit(func, xdata, y_OcclusionAwareIg)
plt.plot(xdata, func(xdata, *popt), label='OcclusionAwareIg fit: a=%5.3f, b=%5.3f, c=%5.3f' % tuple(popt))

popt, pcov = curve_fit(func, xdata, y_UnobservedVoxelIg)
plt.plot(xdata, func(xdata, *popt), label='UnobservedVoxelIg fit: a=%5.3f, b=%5.3f, c=%5.3f' % tuple(popt))

popt, pcov = curve_fit(func, xdata, y_ProximityCountIg)
plt.plot(xdata, func(xdata, *popt), label='ProximityCountIg fit: a=%5.3f, b=%5.3f, c=%5.3f' % tuple(popt))

popt, pcov = curve_fit(func, xdata, y_RearSideEntropyIg)
plt.plot(xdata, func(xdata, *popt), label='RearSideEntropy fit: a=%5.3f, b=%5.3f, c=%5.3f' % tuple(popt))

plt.xticks(np.arange(1, 20, step=1))
plt.xlabel('Iteration')
plt.ylabel('SurfaceArea')
plt.legend(loc='best')
plt.show()