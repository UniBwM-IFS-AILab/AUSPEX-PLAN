import csv
#import matplotlib.pyplot as plt
import numpy as np

# File path to your CSV file
csv_file = 'output/results_for_table.csv'

n_iterations = 10.0


# Function to compute time_action_selection / n_executed_actions
def compute_action_selection_ratio(csv_file):
    time_list = np.zeros(4)
    models = []
    with open(csv_file, newline='') as file:
        reader = csv.DictReader(file)
        
        # Iterate over each row in the CSV
        i = 0
        model_index = 0
        for row in reader:
            model = row['model']
            if i == 0 and model_index == 0:  
                models.append(model)
            total_time_action_selection = float(row['total_time_action_selection'])
            n_executed_actions = float(row['n_executed_actions'])
            

            if i >= n_iterations:
                models.append(model)
                model_index += 1
                i = 0
            i += 1
            time_list[model_index] = time_list[model_index] + total_time_action_selection / n_executed_actions
            
    time_list = time_list/n_iterations
    print_list(time_list, models)

def compute_precision(csv_file):
    time_list = np.zeros(4)
    models = []
    with open(csv_file, newline='') as file:
        reader = csv.DictReader(file)
        
        # Iterate over each row in the CSV
        i = 0
        model_index = 0
        for row in reader:
            model = row['model']
            if i == 0 and model_index == 0:  
                models.append(model)
            prec_tp = float(row['prec_tp'])
            prec_fp = float(row['prec_fp'])
            

            if i >= n_iterations:
                models.append(model)
                model_index += 1
                i = 0
            i += 1
            time_list[model_index] = time_list[model_index] + (prec_tp / (prec_tp * prec_fp))
    
    time_list = time_list/n_iterations
    print_list(time_list, models)


def compute_success(csv_file):
    time_list = np.zeros(4)
    models = []
    with open(csv_file, newline='') as file:
        reader = csv.DictReader(file)
        
        # Iterate over each row in the CSV
        i = 0
        model_index = 0
        for row in reader:
            model = row['model']
            if i == 0 and model_index == 0:  
                models.append(model)
            success = float(row['success'])
            

            if i >= n_iterations:
                models.append(model)
                model_index += 1
                i = 0
            i += 1
            time_list[model_index] = time_list[model_index] + success
    
    time_list = time_list/n_iterations
    print_list(time_list, models)


def print_list(vals, models):
    for index, val in enumerate(vals):
        print(f"model: {models[index]} value: {val}")


# Call the function with the path to your CSV file
compute_action_selection_ratio(csv_file)
print("---")
compute_precision(csv_file)
print("---")
compute_success(csv_file)
