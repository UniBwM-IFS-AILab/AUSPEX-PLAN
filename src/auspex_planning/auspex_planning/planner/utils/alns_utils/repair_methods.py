import logging,sys
logging.basicConfig(stream=sys.stderr, level=logging.INFO)#DEBUG MEANS PRINTS, INFO MEANS NO PRINT

from .utils import * 
import re
import random
import copy

def repair_dummy(current_after_destroy, rnd_state):
    #add node to current_after_destroy
    return current_after_destroy    

"""
Repair based on the type of node
"""
#Is this a good solution since it has a loop for all the vehicles?
def repair_solution_type(current,rnd_state):

    destroyed_nodes = current.destroyed_nodes[0]
    vehicle_routes = current.vehicle_routes
    melted_df = current.melted_df
    G = current.G

    logging.debug("Repair Type")

    node_type_match = re.match(r"([a-zA-Z]+)", destroyed_nodes)

    if not node_type_match:
        
        logging.error("Failed to determine the node type.")
        return current
    
    node_type = node_type_match.group(1)

    logging.debug(f"Inside repair node type is: {node_type} and the node is {destroyed_nodes}")
    max_count = 0
    vehicle_with_most_similar_type = None

    for index, row in vehicle_routes.iterrows():
        count = sum(1 for node in row['Route'] if node.startswith(node_type))
        logging.debug(f"Inside loop count: {count}")
        if count > max_count:
            max_count = count
            vehicle_with_most_similar_type = index
    
    logging.debug(f"Outside of loop vehicle with most similar type : {vehicle_with_most_similar_type}")

    if vehicle_with_most_similar_type is None:
        logging.error("No suitable vehicle found to repair.")
        return current

    target_route = vehicle_routes.at[vehicle_with_most_similar_type,'Route']

    best_config, best_cost = find_best_insertion(current.home_coordinates, target_route,destroyed_nodes, G , melted_df)

    vehicle_routes.at[vehicle_with_most_similar_type,'Route']= best_config

    current.vehicle_routes= vehicle_routes
    
    current.reset()
    

    for index, row in current.vehicle_routes.iterrows():
        logging.debug(f"Result is for vehicle {index} route is {row['Route']}")

    return current



"""
Repair to the closest vehicle node
"""
def repair_solution_closest(current,rnd_state):

    destroyed_nodes = current.destroyed_nodes[0]
    vehicle_routes = current.vehicle_routes
    melted_df = current.melted_df
    G = current.G

    logging.debug("Repair closest")

    min_distance= float('inf')
    closest_node = 0
    vehicle_with_closest = None
    destroyed_coord = get_coordinates(destroyed_nodes,melted_df)

    for index, row in current.vehicle_routes.iterrows():

        logging.debug(f"Closest repair before vehicle {index}, with route {row['Route']}")  


    for index, row in vehicle_routes.iterrows():

        for node in row['Route']:

            if node != destroyed_nodes:

                node_coord = get_coordinates(node,melted_df)
                distance= calculate_distance_3d(destroyed_coord,node_coord)

                if distance <min_distance:

                    min_distance = distance
                    closest_node = node
                    vehicle_with_closest = index

    if closest_node is None:

        logging.debug("There's an error closest node does not exist")

    logging.debug(f"Here is the destroyed_node : {destroyed_nodes} and the closest node : {closest_node} and its vehicle is : {vehicle_with_closest}")

    target_route = vehicle_routes.at[vehicle_with_closest,'Route']

    best_config, best_cost = find_best_insertion(current.home_coordinates, target_route, destroyed_nodes,G, melted_df,)

    vehicle_routes.at[vehicle_with_closest,'Route'] = best_config

    current.vehicle_routes = vehicle_routes

    for index, row in current.vehicle_routes.iterrows():

        logging.debug(f"Closest repair final vehicle {index}, with route {row['Route']}")  


    current.reset()
    

    return current


"""
Repair at lowest nodes
"""
def repair_solution_lowest(current,rnd_state):
    destroyed_nodes = current.destroyed_nodes[0]
    vehicle_routes = current.vehicle_routes
    melted_df = current.melted_df
    G = current.G

    logging.debug("Repair lowest")

    min_nodes=float('inf')
    vehicle_with_lowest= None

    for index,row in vehicle_routes.iterrows():
        logging.debug(f"Repair check here vehicle {index} and route {row['Route']}")

        route_length= len(row['Route'])
        if route_length < min_nodes:
            min_nodes=route_length
            vehicle_with_lowest=index
    
    least_nodes_route = vehicle_routes.at[vehicle_with_lowest,'Route']

    logging.debug(f"Repair check least vehicle  is {vehicle_with_lowest} and its route is {least_nodes_route}")

    best_config,best_cost = find_best_insertion(current.home_coordinates, least_nodes_route,destroyed_nodes,G, melted_df )
    
    vehicle_routes.at[vehicle_with_lowest,'Route'] = best_config


    current.vehicle_routes = vehicle_routes


    

    current.reset()
    return current

        

        


"""Repair at random position"""
def repair_solution_random(current_after_destroy, rnd_state):
    destroyed_nodes = current_after_destroy.destroyed_nodes[0]
    vehicle_routes_des = current_after_destroy.vehicle_routes
    

    logging.debug("Repair random")

    #Randomly choosing the vehicle
    repair_selected_vehicle = random.choice(current_after_destroy.vehicle_routes.index)
    selected_vehicle_route = vehicle_routes_des.at[repair_selected_vehicle, 'Route']

    # Determine the positions where destroyed_nodes should be inserted
    insert_positions = random.randint(0, len(selected_vehicle_route))
    logging.debug(f"destroyed_nodes check inside repair_random: {destroyed_nodes}")
    selected_vehicle_route.insert(insert_positions,destroyed_nodes)
    vehicle_routes_des.at[repair_selected_vehicle,'Route']=selected_vehicle_route

    current_after_destroy.vehicle_routes=vehicle_routes_des
    

    current_after_destroy.reset()
    return current_after_destroy

'''Repair Best Inside Same Vehicle'''
def repair_solution_same(current_after_destroy,rnd_state):
    destroyed_nodes = current_after_destroy.destroyed_nodes[0]
    updated_routes = current_after_destroy.vehicle_routes
    G=current_after_destroy.G
    melted_df=current_after_destroy.melted_df
    selected_vehicle = current_after_destroy.selected_vehicle


    logging.debug("Repair same")

    selected_vehicle_route = updated_routes.at[selected_vehicle,'Route']

    best_config, best_cost = find_best_insertion(current_after_destroy.home_coordinates, selected_vehicle_route,destroyed_nodes,G,melted_df)

    updated_routes.at[selected_vehicle,'Route']=best_config

    current_after_destroy.vehicle_routes=updated_routes

    for index, row in current_after_destroy.vehicle_routes.iterrows():
        route = row['Route']
        vehicle_id = row['Vehicle']

    
    current_after_destroy.reset()

    return current_after_destroy

'''Repair Best In Same Position Among All Vehicles '''
#Destroyed position in the route is going to be used for insertion

def repair_solution_position(current_after_destroy,rnd_state):
    destroyed_nodes = current_after_destroy.destroyed_nodes[0]
    updated_routes = current_after_destroy.vehicle_routes
    G=current_after_destroy.G
    melted_df=current_after_destroy.melted_df
    
    logging.debug("Repair Same Insert")

    lowest_change = float('inf')
    lowest_change_vehicle= -1
    best_config_all = []

    for selected_vehicle in updated_routes['Vehicle']:

            original_route = updated_routes.at[selected_vehicle, 'Route']
            route_copy = copy.deepcopy(original_route)
            if current_after_destroy.destroyed_node_position < len(route_copy):

                route_copy.insert(current_after_destroy.destroyed_node_position, destroyed_nodes)
            else:

                route_copy.insert(len(route_copy),destroyed_nodes)

            new_cost = calculate_route_length_repair(current_after_destroy.home_coordinates, route_copy, G, melted_df)
            original_cost = calculate_route_length_repair(current_after_destroy.home_coordinates,original_route, G, melted_df)
            cost_change = new_cost - original_cost
            if cost_change < lowest_change:

                lowest_change = cost_change
                lowest_change_vehicle = selected_vehicle
                best_config_all = route_copy

    if lowest_change_vehicle != -1:
            
        updated_routes.at[lowest_change_vehicle, 'Route'] = best_config_all
    current_after_destroy.vehicle_routes = copy_dataframe(updated_routes)
    current_after_destroy.reset()

    return current_after_destroy   

'''Repair Best Approximation'''

def repair_solution_best_approx(current_after_destroy, rnd_state):
    destroyed_nodes = current_after_destroy.destroyed_nodes[0]
    updated_routes = current_after_destroy.vehicle_routes
    G=current_after_destroy.G
    melted_df=current_after_destroy.melted_df
    avg_distances=current_after_destroy.avg_distances 

    logging.debug(f"Friendly check for the repair function : destroyed_nodes {destroyed_nodes}")


    best_config_all = []
    lowest_change_vehicle=-1
    lowest_change= float('inf')

    for selected_vehicle in updated_routes['Vehicle']:
        original_route = updated_routes.at[selected_vehicle,'Route']

        best_config_iter,best_cost_iter= find_best_insertion_approx(current_after_destroy.home_coordinates, original_route, destroyed_nodes,G, melted_df,avg_distances)

        logging.debug(f"Friendly check inside first loop best config and cost {best_config_iter} and {best_cost_iter}")

        if best_cost_iter<lowest_change:
            lowest_change=best_cost_iter
            lowest_change_vehicle=selected_vehicle
            best_config_all = best_config_iter

    logging.debug(f"Friendly check after loop best config all {best_config_all}")
    updated_routes.at[lowest_change_vehicle, 'Route'] = best_config_all

    logging.debug(f"Here is the lowest_change vehicle inside repair approx: {best_config_all}")
    current_after_destroy.vehicle_routes = copy_dataframe(updated_routes)

    current_after_destroy.reset()

    # for index, row in current_after_destroy.vehicle_routes.iterrows():
    #     route= row['Route']
    #     logging.debug(f"Here is the vehicle {index} and its route {route}")
    #current_after_destroy.number_of_locations(updated_routes)
    
    return current_after_destroy


'''Repair Best Position Greedy'''
def repair_solution_best_position(current_after_destroy, rnd_state):
    destroyed_nodes = current_after_destroy.destroyed_nodes[0]
    updated_routes = current_after_destroy.vehicle_routes
    G=current_after_destroy.G
    melted_df=current_after_destroy.melted_df
    #current_after_destroy.number_of_locations(updated_routes)

    logging.debug("Repair best")

    best_config_all = []
    lowest_change_vehicle=-1
    lowest_change= float('inf')

    for selected_vehicle in updated_routes['Vehicle']:
        original_route = updated_routes.at[selected_vehicle,'Route']

        best_config_iter,best_cost_iter= find_best_insertion(current_after_destroy.home_coordinates, original_route, destroyed_nodes,G, melted_df)

        if best_cost_iter<lowest_change:
            lowest_change=best_cost_iter
            lowest_change_vehicle=selected_vehicle
            best_config_all = best_config_iter

    updated_routes.at[lowest_change_vehicle, 'Route'] = best_config_all

    current_after_destroy.vehicle_routes = copy_dataframe(updated_routes)

    current_after_destroy.reset()
    #current_after_destroy.number_of_locations(updated_routes)

    return current_after_destroy

def repair_greedy(current,rnd_state):

    rand= random.random()
    threshold = current.threshold
    decay = current.decay

    if(rand>threshold):
        repair_solution_best_position(current,rnd_state)
    else:
        repair_solution_random(current,rnd_state)
        current.threshold = threshold*decay

    return current
########################################################################################################################################################
#######################################################           Below are Helper Functions      ######################################################
########################################################################################################################################################
########################################################################################################################################################
'''Function for the Repair Best Position'''
def find_best_insertion(home_coordinates, current_route,destroyed_nodes,graph, melted_df):
    best_config= current_route.copy()
    best_cost= float('inf')

    for i in range(len(current_route)+1):
        route_copy= current_route.copy()

        if len(destroyed_nodes) != 0:
            route_copy.insert(i,destroyed_nodes)

        route_length= calculate_route_length_repair(home_coordinates, route_copy, graph, melted_df)

        if route_length < best_cost:
            best_cost=route_length
            best_config= route_copy

    return best_config,best_cost


'''Function for the Repair Best Position'''
def calculate_route_length_repair(home_coordinates, route, graph, melted_df):
    total_length=0
    if len(route) != 0:
        first_node = route[0]
        if first_node in graph:
            first_node_coordinates= get_coordinates(first_node,melted_df)
            distance_to_first_node = calculate_distance_3d(home_coordinates,first_node_coordinates)
            total_length += distance_to_first_node

    for i in range(len(route)-1):
        u,v = route[i], route[i+1]
        u_coordinates = get_coordinates(u,melted_df)
        v_coordinates = get_coordinates(v,melted_df)

        if u_coordinates and v_coordinates:
            distance = calculate_distance_3d(u_coordinates,v_coordinates)
            total_length += distance
    return total_length


'''Function for the Repair Best Approximation'''
def find_best_insertion_approx(home_coordinates, current_route,destroyed_nodes,graph, melted_df, avg_distances):
    best_config= current_route.copy()
    best_cost= float('inf')

    for i in range(len(current_route)+1):
        route_copy= current_route.copy()

        if len(destroyed_nodes) != 0:
            route_copy.insert(i,destroyed_nodes)

        route_length= calculate_route_length_approx(home_coordinates, route_copy, graph, melted_df, avg_distances)

        if route_length < best_cost:
            best_cost=route_length
            best_config= route_copy

    logging.debug(f"Friendly check for find best insertion : {best_config} and {best_cost}")
    return best_config,best_cost

'''Function for the Repair Best Approximation'''
def calculate_route_length_approx(home_coordinates, route, graph, melted_df, avg_distances):
    total_cost = 0
    if len(route) != 0:
        first_node = route[0]
        if first_node in graph:
            first_node_coordinates= get_coordinates(first_node,melted_df)
            distance_to_first_node = calculate_distance_3d(home_coordinates,first_node_coordinates)
            total_cost += distance_to_first_node

    for i in range(len(route) - 1):
        u, v = route[i], route[i + 1]
        u_type = ''.join(filter(str.isalpha, u.lower()))
        v_type = ''.join(filter(str.isalpha, v.lower()))
        total_cost += avg_distances.get((u_type, v_type), 0)

    logging.debug(f"Friendly check inside calculate route length approx total cost is {total_cost}")

    return total_cost


def calculate_total_cost(home_coordinates, routes,graph, melted_df):
    total_cost=0
    for route in routes:
        total_cost+=calculate_route_length_repair(home_coordinates, route,graph, melted_df)
    return total_cost