import numpy as np
import matplotlib.pyplot as plt

data = np.genfromtxt('rabbit_weighted_gain.csv', delimiter=',')

x = np.arange(1,22)
print(data)

plt.plot(x, data, label='csv')

plt.xlabel('Reconstruction Step')
plt.ylabel('Weighted Information Gain')
plt.title('Weighted Information Gain acroseach iteration')
plt.legend()
plt.show()
