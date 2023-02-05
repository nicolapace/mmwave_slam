import numpy as np
from sklearn.linear_model import RANSACRegressor
import matplotlib.pyplot as plt 
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression

# Generate some synthetic data
y = np.array([0.81,0.81,26,26,0.81,0.81,0.81,26,26,26,26,4,4,36,4,4,4,4,4,4,4,4,4,26,26,26,26,0.81,26,26,0.81,26,26,0.81,0.84,0.83,26,0.87,26,26,26,26,26,0.81])
x = np.sort(np.random.rand(len(y)))
# Create the RANSAC regressor and fit it to the data
ransac = RANSACRegressor()
ransac.fit(x.reshape(-1,1), y.reshape(-1,1))

# Use the fitted RANSAC regressor to predict the y-values at a set of new x-values
x_interp = np.linspace(0, 1, len(y))
y_interp = ransac.predict(x_interp.reshape(-1,1))

print(y_interp)

plt.plot(x_interp,y,x_interp,y_interp)
plt.show()
