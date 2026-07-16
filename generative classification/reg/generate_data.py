import numpy as np
# Function to generate training data. 
# Please only update the Student ID part.

############################CHANGE YOUR STUDENT ID###############################
student1_number = 1010390989  # Replace with the actual student number
student2_number = 123465789  # Replace with the actual student number
np.random.seed((student1_number+student2_number)//1000)

# Number of samples
num_samples = 100

# Generate random x values in the range (0, 1)
x_values = np.random.uniform(0, 1, size=num_samples)*2-1

# Generate random values for a1 and a0
a_1 = np.round(np.random.uniform(-1, 1), 1)
a_0 = np.round(np.random.uniform(-1, 1), 1)
while a_1 == 0 or a_0 == 0:
    a_1 = np.round(np.random.uniform(-1, 1), 1)
    a_0 = np.round(np.random.uniform(-1, 1), 1)
sigma = np.sqrt(0.1)  # Standard deviation of the Gaussian noise

# Generate Gaussian noise
w = np.random.normal(0, sigma, size=num_samples)

# Generate z values based on the linear model z = a1x + a0 + w
z_values = a_1 * x_values + a_0 + w

x_values_formatted = np.round(x_values, 4)
z_values_formatted = np.round(z_values, 4)
new_data = np.vstack((x_values_formatted, z_values_formatted)).T

# Path to save the new training data
file_path = 'training.txt'

# Custom format for saving data with exactly four digits after the decimal
with open(file_path, 'w') as f:
    for x, z in new_data:
        f.write(f"{x:0.4f} {z:0.4f}\n")

# print(f'New training data saved to: {file_path}')
print("the customed a1 and a0 are: ",a_1,a_0)