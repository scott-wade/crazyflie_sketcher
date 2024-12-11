import pandas as pd
import numpy as np

# Function to calculate average absolute errors from a given CSV file
def calculate_average_errors(file_path):
    df = pd.read_csv(file_path)

    # Calculate absolute values of error columns
    df['x_Error_abs'] = np.abs(df['x_error'])
    df['y_Error_abs'] = np.abs(df['y_error'])
    df['z_Error_abs'] = np.abs(df['z_error'])

    # Calculate averages of absolute error values
    avg_x_error = df['x_Error_abs'].mean()
    avg_y_error = df['y_Error_abs'].mean()
    avg_z_error = df['z_Error_abs'].mean()

    return avg_x_error, avg_y_error, avg_z_error

# List of CSV files to process
csv_files = ['../HL_Commander_Tests/circle_results.csv', '../Full_State_Commander_Tests/results/circle_no_magnet.csv', '../Full_State_Commander_Tests/results/circle_magnet.csv']

# Initialize a list to store results
results = []

# Process each file and gather results
for file in csv_files:
    avg_x, avg_y, avg_z = calculate_average_errors(file)
    results.append({
        'File': file,
        'Average x_Error': avg_x,
        'Average y_Error': avg_y,
        'Average z_Error': avg_z
    })

# Create a DataFrame from the results
results_df = pd.DataFrame(results)

# Write the results to a new CSV file
results_df.to_csv('circle_average_errors_summary.csv', index=False)

print("Average errors have been calculated and saved to 'average_errors_summary.csv'.")
