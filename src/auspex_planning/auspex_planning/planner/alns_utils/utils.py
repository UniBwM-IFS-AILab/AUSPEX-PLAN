import math
import logging
import pandas as pd
import json
import os
from geopy.distance import geodesic
import matplotlib.pyplot as plt
import csv
import copy

def get_coordinates(location, melted_df):
    if location in melted_df['Locations'].values:
        coords = melted_df.loc[melted_df['Locations'] == location].iloc[0]
        return (coords['Long'], coords['Lat'], coords['Alt'])
    else:
        logging.warning(f"Coordinates for location {location} not found.")
        return None

def calculate_distance_3d(point1, point2):
    lat1, lon1, alt1 = point1
    lat2, lon2, alt2 = point2
    two_d_distance = geodesic((lat1, lon1), (lat2, lon2)).meters
    delta_altitude = alt2 - alt1
    distance_3d = math.sqrt(two_d_distance**2 + delta_altitude**2)
    return distance_3d

def precompute_average_distances(melted_df):
    location_types = melted_df['Locations'].apply(lambda x: ''.join(filter(str.isalpha, x.lower()))).unique()
    avg_distances = {}

    for type1 in location_types:
        for type2 in location_types:
            distances = []
            loc1_list = melted_df[melted_df['Locations'].str.lower().str.startswith(type1)]['Locations']
            loc2_list = melted_df[melted_df['Locations'].str.lower().str.startswith(type2)]['Locations']

            if len(loc1_list) > 1 and type1 == type2:
                for loc1 in loc1_list:
                    for loc2 in loc1_list:
                        if loc1 != loc2:
                            coords1 = get_coordinates(loc1, melted_df)
                            coords2 = get_coordinates(loc2, melted_df)
                            if coords1 and coords2:
                                distance = calculate_distance_3d(coords1, coords2)
                                distances.append(distance)
            else:
                for loc1 in loc1_list:
                    for loc2 in loc2_list:
                        if loc1 != loc2:
                            coords1 = get_coordinates(loc1, melted_df)
                            coords2 = get_coordinates(loc2, melted_df)
                            if coords1 and coords2:
                                distance = calculate_distance_3d(coords1, coords2)
                                distances.append(distance)

            avg_distances[f"{type1}-{type2}"] = sum(distances) / len(distances) if distances else 0
            if not distances:
                logging.warning(f"No distances calculated for pair {type1}-{type2}")

    return avg_distances

'''
Reads a json file and parses it to a panda dataframe
'''
def parseJsonFile(file):
    with open(file) as json_file:
        try:
            config_list = json.load(json_file)
        except json.JSONDecodeError as exc:
            #logging.debug(exc)
            return None

    home = None  # to store the 'home' entry
    data = {
        "Locations": [],
        "Lat": [],
        "Long": [],
        "Alt": []
    }

    # Process each dictionary in the list
    for entry in config_list:
        # Check if the entry is for 'home'
        if entry.get("name") == "home":
            home = entry
        else:
            data["Locations"].append(entry.get("name"))
            centre = entry.get("centre", [])
            if len(centre) >= 3:
                data["Lat"].append(centre[0])
                data["Long"].append(centre[1])
                data["Alt"].append(centre[2])

    melted_df = pd.DataFrame(data)
    melted_df = melted_df.reset_index(drop=True)

    return home, melted_df

def calculate_route_length_check(home_coordinates, route, melted_df):
    total_length=0
        # if isinstance(route,pd.Series):
        #     route= route.tolist()

    logging.debug(f"Here is the route : {route}")
    first_node = route[0]
            ##logging.debug(f"Calculation check first node in {route} is : {first_node}")

    if first_node !='home':

        first_node_coordinates= get_coordinates(first_node,melted_df)

                ##logging.debug(f"Calculation check to coordinates of the first node : {first_node_coordinates}")

        distance_to_first_node = calculate_distance_3d(home_coordinates,first_node_coordinates)

                ##logging.debug(f"Calculation check to see if distance is calculated properly with home : {distance_to_first_node}")


        total_length += distance_to_first_node

    for i in range(len(route)-1):

        u,v = route[i], route[i+1]

        u_coordinates = get_coordinates(u,melted_df)
        v_coordinates = get_coordinates(v,melted_df)

            ##logging.debug(f"Calculation check after getting coordinates  : u {u_coordinates} and v : {v_coordinates}")

        if u_coordinates and v_coordinates:
            distance = calculate_distance_3d(u_coordinates,v_coordinates)
            total_length += distance

    return total_length



def plot_parameter(csv_path):

    df= pd.read_csv(csv_path)

    parameters = df['Parameters']
    best_cost = df['Best Cost']
    best_iteration = df['Best Solution Iteration']    

    plt.figure(figsize=(10,6))
    scatter=plt.scatter(best_iteration,best_cost)

    for i, (param, cost, iteration) in enumerate(zip(parameters, best_cost, best_iteration)):
        plt.annotate(f"{param}", (iteration, cost), textcoords="offset points", xytext=(0,10), ha='center')

    plt.xlabel('Best Iteration')
    plt.ylabel('Cost')
    plt.title('Cost vs. Best Iteration for Different Parameter Sets')
    plt.legend(*scatter.legend_elements(), title='Parameter Sets', bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.show()


def write_to_csv(data, file_name='results.csv'):
    """
    Writes data to a CSV file. If the file does not exist, it creates it and adds a header.
    :param data: A dictionary containing the data to write. Keys are column names.
    :param file_name: The name of the CSV file.
    """
    # Define the CSV header as a list of column names.

    header = ['Parameters', 'Iterations', 
              'Best Solution Iteration', 'Duration', 'Best Cost', 
              'Number of Destroy', 'Number of Repair']

    # Check if the file exists
    file_exists = os.path.isfile(file_name)

    # Open the file in append mode ('a') so we can write without overwriting existing data.

    with open(file_name, 'a', newline='') as csv_file:

        writer = csv.DictWriter(csv_file, fieldnames=header)

        # If the file does not exist, write the header first.
        if not file_exists:
            writer.writeheader()

        # Write the data.
        writer.writerow(data)


'''
List to string function
'''
def list2string(list_):
    if isinstance(list_, list):
        new_list = ','.join(list_)
        return new_list



'''
Reads the average distances json file
'''
def load_avg_distances(file_path):
    with open(file_path, 'r') as f:
        avg_distances = json.load(f)
    avg_distances = {tuple(key.split('-')): value for key, value in avg_distances.items()}
    return avg_distances


def copy_dataframe(df):
    dfcopy = None
    data = {'Vehicle': [], 'Route': []}

    dfcopy = pd.DataFrame.from_dict(data)
    dfcopy['Vehicle'] = pd.Series([], dtype=object)

    i = int(0)
    for entry in df['Route']:
        new_line = {
            'Vehicle': int(i),
            'Route': entry.copy()
        }
        dfcopy = dfcopy._append(new_line, ignore_index=True)
        i+=1
    return dfcopy

'''
For Writing to a JSON File
'''

def write_to_json(vehicle_routes):
    vehicles_json = []
    for vehicle, route in vehicle_routes.iterrows():

        if isinstance(route['Route'],str):
            route_list = route['Route'].split(',')
        else:
            route_list = list(route['Route'])
        
        vehicle_dict = {
            "name": vehicle,
            "route": route_list
        }
        vehicles_json.append(vehicle_dict)

    with open('vehicle_routes.json','w') as json_file:
        json.dump(vehicles_json,json_file,indent=4)

    logging.info("JSON data has been written to vehicle_routes.json")
    
    
    
    
'''
visualization below works perfectly for the first routes of the vehicles
'''
def visualize_routes_and_locations(G, vehicle_routes, home_coordinates, locations_df, path_prefix=""):
    '''
    Expected format
        Vehicle                                              Route
        0       0  [openareas8, openareas10, openareas18, openare...
        1       1  [woods2, woods4, woods3, woods8, woods9, woods...
        2       2                                 [waters1, waters0]
        3       3                                      [urbanareas0]
        4       4               [woods14, woods10, woods11, woods13]
        5       5          [woods5, woods0, woods15, woods6, woods1]
        6       6  [openareas11, openareas17, openareas4, openare...
        7       7  [openareas16, openareas12, openareas19, openar...
        8       8  openareas21,openareas5,openareas7,openareas6,o...
    '''

    vehicle_routes_vis = copy_dataframe(vehicle_routes)
    G_copy = copy.deepcopy(G)
    
    for index, row in vehicle_routes_vis.iterrows():
        existing_route= row['Route']
        existing_route.insert(0, 'home')
        vehicle_routes_vis.at[index,'Route']= existing_route

    G_copy.add_node('home',pos=home_coordinates)

    plt.figure(figsize=(10, 8))

    #Plotted the location points
    home_size= 100
    for idx, row in locations_df.iterrows():
        if row['Locations']=='home':
            plt.scatter(row['Long'], row['Lat'], color='red', s=home_size) #making home point bigger than others
        else:
            plt.scatter(row['Long'], row['Lat'], color='blue', s=100) #plotting locations as blue points
        plt.text(row['Long'], row['Lat'], row['Locations'], fontsize=12, ha='left', va='bottom') #Adding location names


    #Plotted the routes
    for _, path in enumerate(vehicle_routes_vis['Route']):
        route_coordinates = [G_copy.nodes[node]['pos'] for node in path]
        route_coordinates = list(zip(*route_coordinates))
        plt.plot(route_coordinates[1], route_coordinates[0], linestyle='-', marker='o', markersize=6) #0 contains the latitude, 1 contains the longitude

    plt.xlabel('Longitude', fontsize=17)
    plt.ylabel('Latitude', fontsize=17)
    plt.title('VRP Routes and Locations', fontsize=17)
    #plt.legend(loc='upper right')
    plt.grid(True)

    if path_prefix != "":
        file_path = os.path.dirname(__file__)
        plt.savefig(file_path + "alns_graph_"+path_prefix+".png")


    plt.show()