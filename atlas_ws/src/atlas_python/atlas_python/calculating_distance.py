import pandas as pd
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt
import numpy as np

# Load the data from the CSV file
df = pd.read_csv('tennis_ball_data.csv')

# Check if the dataframe has the expected columns
if 'Distance' in df.columns and 'Tennis_ball_height' in df.columns:
    # Reshape the data for linear regression
    X = df['Tennis_ball_height'].values.reshape(-1, 1)  # Feature
    y = df['Distance'].values  # Target

    # Create and fit the linear regression model
    model = LinearRegression()
    model.fit(X, y)

    # Get the regression parameters
    slope = model.coef_[0]
    intercept = model.intercept_

    print(f"Linear Regression Model Parameters:")
    print(f"Slope (Coefficient): {slope}")
    print(f"Intercept: {intercept}")

    # Predict the target values using the model
    y_pred = model.predict(X)

    # Plot the results
    plt.scatter(X, y, color='blue', label='Actual Data')
    plt.plot(X, y_pred, color='red', label='Fitted Line')
    plt.xlabel('Tennis Ball Height (pixels)')
    plt.ylabel('Distance from Center (pixels)')
    plt.title('Linear Regression on Tennis Ball Data')
    plt.legend()
    plt.show()

else:
    print("Error: The CSV file does not contain the required columns 'Distance' and 'Tennis_ball_height'.")