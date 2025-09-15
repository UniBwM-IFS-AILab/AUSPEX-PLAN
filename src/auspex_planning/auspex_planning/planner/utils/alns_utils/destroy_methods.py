import logging,sys
logging.basicConfig(stream=sys.stderr, level=logging.INFO)#DEBUG MEANS PRINTS, INFO MEANS NO PRINT
import random 
from .utils import *

def destroy_dummy(current, rnd_state):
    current_copy = current.copy()
    #remove node from current_copy and return current_copy
    return current_copy

'''
Destroy Function with the most Nodes
'''

def destroy_most_nodes(current, rnd_state):
    current_copy = current.copy()
    vehicle_routes = current_copy.vehicle_routes

    logging.debug("Destroy most node")

    max_nodes = 0
    vehicle_with_most_nodes= None
    for index,row in vehicle_routes.iterrows():
        
        logging.debug(f"Here is the vehicle {index} and route {row['Route']}")
        route_length = len(row['Route'])
        if route_length > max_nodes:
            max_nodes = route_length
            vehicle_with_most_nodes= index

    if vehicle_with_most_nodes is not None:

        logging.debug(f"Here is the maximum length vehicle {vehicle_with_most_nodes}")

        most_nodes_route = vehicle_routes.at[vehicle_with_most_nodes,'Route']

        destroyed_nodes = str(random.choice(most_nodes_route))

        #remaining_nodes = [node for node in most_nodes_route if node not in destroyed_nodes]

        current_copy.destroyed_node_position = most_nodes_route.index(destroyed_nodes)

        most_nodes_route.remove(destroyed_nodes)

        logging.debug(f"Check if node is destroyed correctly : destroyed {destroyed_nodes} and the remaining nodes {most_nodes_route}")

        remaining_nodes=most_nodes_route

        vehicle_routes.at[vehicle_with_most_nodes, 'Route'] = remaining_nodes

        logging.debug(f"Destroyed nodes from vehicle {vehicle_with_most_nodes}: {destroyed_nodes}")
        logging.debug(f"Remaining route for vehicle {vehicle_with_most_nodes}: {remaining_nodes}")

        current_copy.vehicle_routes.at[vehicle_with_most_nodes,'Route'] = remaining_nodes
        current_copy.destroyed_nodes=[destroyed_nodes]
        current_copy.remaining_nodes=remaining_nodes
        current_copy.selected_vehicle=vehicle_with_most_nodes

        for index, row in current_copy.vehicle_routes.iterrows():
            logging.debug(f"Here is the resulting vehicle {index} and its route {row['Route']}")
        

        return current_copy


    else:

        logging.info("There is something wrong with destroy modes nodes!")


        

#New destroy function that detects the vehicle with the highest costed route and destroys the location that has the biggest change
def destroy_nodes_distance(current, rnd_state):
    current_copy = current.copy()
    vehicle_routes = current_copy.vehicle_routes
    graph = current_copy.G
    logging.debug(f"Destroy nodes distance")

    #current_copy.number_of_locations(vehicle_routes)

    for index, row in vehicle_routes.iterrows():
        route= row['Route']
        v_id = row['Vehicle']
        logging.debug(f"Here is the original route of the vehicle {v_id}  : {route}")

    selected_vehicle = random.choice(vehicle_routes['Vehicle'])

    selected_route = vehicle_routes.at[selected_vehicle,'Route']
    
    while len(selected_route)<=1:

        selected_vehicle = random.choice(vehicle_routes.index)
        selected_route = vehicle_routes.at[selected_vehicle, 'Route']
    # if len(selected_route) == 0:
    #     logging.ERROR("Error in destroy. Empty route given.")
    #     return current_copy
    
    logging.debug(f"Here is the randomly selected vehicle no {selected_vehicle} and its route {selected_route}")
    if not isinstance(selected_route,list):
        selected_route=[selected_route]

    num_nodes_to_destroy = 1#num_nodes_to_destroy = max(1, int(len(vehicle_route) * destroy_fraction))  # Ensure at least 1 node is destroyed



    starting_node= selected_route[0]
    try: #Adding this part bc of the unscriptable error here
        sorted_vehicle_route = sorted(selected_route, key=lambda node: graph[starting_node].get(node, {'weight': -float('inf')})['weight'], reverse=True)
        logging.debug(f"Here is the sorted vehicle route {sorted_vehicle_route}")

    except Exception as e:

        sorted_vehicle_route=[]


    destroyed_nodes= sorted_vehicle_route[:num_nodes_to_destroy]
    
    destroyed_nodes_positions = [selected_route.index(node) for node in destroyed_nodes]
    current_copy.destroyed_node_position= destroyed_nodes_positions[0]
    remaining_nodes = [node for node in selected_route if node not in destroyed_nodes]

    logging.debug(f"Here are the destroyed node: {destroyed_nodes} with positions: {current_copy.destroyed_node_position} and the remaining ones: {selected_route} and remainings {remaining_nodes}")
    current_copy.destroyed_nodes = destroyed_nodes
    current_copy.remaining_nodes = remaining_nodes
    current_copy.selected_vehicle = selected_vehicle
    current_copy.vehicle_routes.at[selected_vehicle,'Route']=remaining_nodes
    logging.debug(f"ITERATION NUMBER {current_copy.iteration_number}")
    logging.debug(f"Destroy nodes distance after")

    for index, row in vehicle_routes.iterrows():
        route= row['Route']
        v_id = row['Vehicle']
        logging.debug(f"Here is the new route of the vehicle {v_id}  : {route}")
    
        
    return current_copy
    



def destroy_nodes_change(current,rnd_state):
    logging.debug("Destroy Change")
    current_copy = current.copy()
    melted_df=current_copy.melted_df
    vehicle_costs ={}
    vehicle_routes=current_copy.vehicle_routes
    
    destroyed_selected = random.randint(0, len(vehicle_routes))
    
    logging.debug(f"selected vehicle : {destroyed_selected}")

    for index, row in vehicle_routes.iterrows():
        vehicle_id = row['Vehicle']
        route = row['Route']
        route_cost = calculate_route_length_check(current_copy.home_coordinates, route,melted_df)
        logging.debug(f"Checking for the vehicle id {vehicle_id} cost is {route_cost}")
        vehicle_costs[vehicle_id]= route_cost

    highest_cost_vehicle = max(vehicle_costs, key=vehicle_costs.get)


    highest_vehicle_route = vehicle_routes.loc[vehicle_routes['Vehicle'] == highest_cost_vehicle, 'Route'].iloc[0]
    logging.debug(f"Checking for the highest cost vehicle : {highest_cost_vehicle} and its route {highest_vehicle_route}") #works until here

    # if len(highest_vehicle_route) == 0:
    #     logging.ERROR("Error in destroy. Empty route given.")
    #     return current_copy
        
    highest_node = find_node_with_highest(highest_vehicle_route,melted_df)


    logging.debug(f"Highest node is {highest_node}")
    logging.debug(f"Checking after calculation highest vehicle route without changes {highest_vehicle_route}")

    
    if len(highest_vehicle_route)<=1 or destroyed_selected==highest_cost_vehicle:

        if len(highest_vehicle_route)>1:

            logging.debug(f"Highest node before {highest_node}")
            highest_cost_vehicle = random.choice(vehicle_routes.index)
            highest_vehicle_route = vehicle_routes.loc[vehicle_routes['Vehicle'] == highest_cost_vehicle, 'Route'].iloc[0]
            highest_node = find_node_with_highest(highest_vehicle_route,melted_df)


        while len(highest_vehicle_route)<=1:


        #highest_second = find_node_with_highest

            highest_cost_vehicle = random.choice(vehicle_routes.index)
            highest_vehicle_route = vehicle_routes.loc[vehicle_routes['Vehicle'] == highest_cost_vehicle, 'Route'].iloc[0]
            highest_node = find_node_with_highest(highest_vehicle_route,melted_df)
            #current_copy.destroyed_node_position = highest_vehicle_route.index(highest_node)


            logging.debug(f"Problem here highest node is {highest_node} and highest vehicle route {highest_vehicle_route}")
        
        current_copy.destroyed_node_position = highest_vehicle_route.index(highest_node)
        highest_vehicle_route.remove(highest_node)

        current_copy.vehicle_routes.at[highest_cost_vehicle,'Route']=highest_vehicle_route

        current_copy.destroyed_nodes = [highest_node]
        current_copy.remaining_nodes=highest_vehicle_route
        current_copy.selected_vehicle = highest_cost_vehicle
        destroyed_selected=highest_cost_vehicle

        logging.debug(f"If the same vehicle is selected again here")
        logging.debug(f"selected vehicle 2 : {destroyed_selected} and vehicle {highest_cost_vehicle} and the destroyed node{highest_node} and remaining {highest_vehicle_route}")
        for index, row in current_copy.vehicle_routes.iterrows():
            route = row['Route']
            vehicle_id = row['Vehicle']
        

        return current_copy
    
    if highest_node in highest_vehicle_route and highest_cost_vehicle!=destroyed_selected:

        current_copy.destroyed_node_position = highest_vehicle_route.index(highest_node)

        highest_vehicle_route.remove(highest_node)
        #updated_route_str=','.join(highest_vehicle_route)
        logging.debug(f"Checked if the same vehicle is selected or not -->its not")

        #vehicle_routes.loc[vehicle_routes['Vehicle'] == highest_cost_vehicle, 'Route'] = updated_route_str
        
        current_copy.vehicle_routes.at[highest_cost_vehicle,'Route']=highest_vehicle_route

        current_copy.destroyed_nodes = [highest_node]
        current_copy.remaining_nodes=highest_vehicle_route
        current_copy.selected_vehicle = highest_cost_vehicle
        destroyed_selected=highest_cost_vehicle
        logging.debug(f"selected vehicle 1 : {destroyed_selected} and vehicle {highest_cost_vehicle} and the destroyed node{highest_node} and remaining {highest_vehicle_route}")
        for index, row in current_copy.vehicle_routes.iterrows():
            route = row['Route']
            vehicle_id = row['Vehicle']
            logging.debug(f"Making sure {vehicle_id} --> {route}")
        
        

        return current_copy
    


    logging.debug(f"Checking the destroyed nodes {current_copy.destroyed_nodes}, and the remaining: {current_copy.remaining_nodes}")
    logging.debug(f"Shouldn't be here!!!")
    #logging.debug(f"Here is the updated version: {current_copy.printVehicleRoutesExtern(current_copy.vehicle_routes)}")
    
    #logging.debug(f"Here is the no : {current.number_of_locations(vehicle_routes)}")



def find_node_with_highest(route,melted_df):
    highest_distance = 0
    node_before_highest_gap = None

    for i in range(0,len(route)-1):

        current_node = route[i]
        next_node = route[i+1]

        current_coords = get_coordinates(current_node,melted_df)
        next_coords= get_coordinates(next_node, melted_df)

        distance=calculate_distance_3d(current_coords,next_coords)
        logging.debug(f"Distance between current node and next node for i {i} is {distance}")

        if distance >highest_distance:
            highest_distance=distance
            node_before_highest_gap=current_node
            logging.debug(f"Highest distance is for the node {node_before_highest_gap} and distance is {highest_distance}")

    return node_before_highest_gap

def destroy_random(current,rnd_state):
    current_copy = current.copy() #This line in every destroy function
    vehicle_routes_store = current_copy.vehicle_routes

    selected_vehicle = random.choice(vehicle_routes_store.index)
    selected_vehicle_route = vehicle_routes_store.at[selected_vehicle, 'Route']

    while len(selected_vehicle_route)<=1: #burda loop a girmemesi için 2. condition ekle 1 vehicle varsa

        selected_vehicle = random.choice(vehicle_routes_store.index)
        selected_vehicle_route = vehicle_routes_store.at[selected_vehicle, 'Route']

    if len(selected_vehicle_route) == 0:
        logging.ERROR("Error in destroy. Empty route given.")
        return current_copy

    destroyed_nodes= random.sample(selected_vehicle_route, 1)
    destroyed_nodes_positions = [selected_vehicle_route.index(node) for node in destroyed_nodes]
    current_copy.destroyed_node_position= destroyed_nodes_positions[0]
    remaining_nodes=[node for node in selected_vehicle_route if node not in destroyed_nodes]

    #The actual destroy
    current_copy.vehicle_routes.at[selected_vehicle,'Route']=remaining_nodes

    current_copy.destroyed_nodes=destroyed_nodes
    current_copy.remaining_nodes=remaining_nodes
    current_copy.selected_vehicle=selected_vehicle
    logging.debug("Destroy random")
    
    
    return current_copy