import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import pandas as pd 


def func(x, a, b, c):
    return 1 - (a * np.exp(-b * x) + c)


xdata = np.arange(1,41)
#ydata = pd.read_csv('./map_metric_data/AverageEntropyIg/occupied_voxel_volume.csv', sep=',',header=None)

#y_metric1 = np.genfromtxt('./map_metric_data/40iters/metric1/occupied_voxel_area.csv', delimiter=',')
#y_metric1 = y_metric1[~np.isnan(y_metric1)]

y_RearSideEntropyIg = np.genfromtxt('./map_metric_data/40iters/RearSideEntropyIg/occupied_voxel_area.csv', delimiter=',')
y_RearSideEntropyIg = y_RearSideEntropyIg[~np.isnan(y_RearSideEntropyIg)]


y_AverageEntropyIg = np.genfromtxt('./map_metric_data/40iters/AverageEntropyIg/occupied_voxel_area.csv', delimiter=',')
y_AverageEntropyIg = y_AverageEntropyIg[~np.isnan(y_AverageEntropyIg)]



y_OcclusionAwareIg = np.genfromtxt('./map_metric_data/40iters/OcclusionAwareIg/occupied_voxel_area.csv', delimiter=',')
y_OcclusionAwareIg = y_OcclusionAwareIg[~np.isnan(y_OcclusionAwareIg)]


y_ProximityCountIg = np.genfromtxt('./map_metric_data/40iters/ProximityCountIg/occupied_voxel_area.csv', delimiter=',')
y_ProximityCountIg = y_ProximityCountIg[~np.isnan(y_ProximityCountIg)]


y_RearSideVoxelIg = np.genfromtxt('./map_metric_data/40iters/RearSideVoxelIg/occupied_voxel_area.csv', delimiter=',')
y_RearSideVoxelIg = y_RearSideVoxelIg[~np.isnan(y_RearSideVoxelIg)]


y_UnobservedVoxelIg = np.genfromtxt('./map_metric_data/40iters/UnobservedVoxelIg/occupied_voxel_area.csv', delimiter=',')
y_UnobservedVoxelIg = y_UnobservedVoxelIg[~np.isnan(y_UnobservedVoxelIg)]


y_VasquezGomezAreaFactorIg = np.genfromtxt('./map_metric_data/40iters/VasquezGomezAreaFactorIg/occupied_voxel_area.csv', delimiter=',')
y_VasquezGomezAreaFactorIg = y_VasquezGomezAreaFactorIg[~np.isnan(y_VasquezGomezAreaFactorIg)]


y_UnknownVoxelCountIg = np.genfromtxt('./map_metric_data/40iters/UnknownVoxelCountIg/occupied_voxel_area.csv', delimiter=',')
y_UnknownVoxelCountIg = y_UnknownVoxelCountIg[~np.isnan(y_UnknownVoxelCountIg)]

#plt.plot(xdata, y_metric1, label='UnknownVoxelCount')
plt.plot(xdata, y_RearSideEntropyIg, label='RearSideEntropy')
plt.plot(xdata, y_AverageEntropyIg, label='AverageEntropyIg')
plt.plot(xdata, y_OcclusionAwareIg, label='OcclusionAwareIg')
plt.plot(xdata, y_ProximityCountIg, label='ProximityCountIg')
plt.plot(xdata, y_RearSideVoxelIg, label='RearSideVoxelIg')
plt.plot(xdata, y_UnobservedVoxelIg, label='UnobservedVoxelIg')
plt.plot(xdata, y_VasquezGomezAreaFactorIg, label='VasquezGomezAreaFactorIg')
plt.plot(xdata, y_UnknownVoxelCountIg, '-y', label='UnknownVoxelCount')

plt.xticks(np.arange(1, 40, step=2))
plt.xlabel('Iteration')
plt.ylabel('SurfaceArea')
plt.legend(loc='best')
plt.show()