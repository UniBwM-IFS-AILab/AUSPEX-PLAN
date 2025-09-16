from auspex_planning.planner.task_planners.planner_base import PlannerBase
from ament_index_python.packages import get_package_share_directory

from auspex_msgs.msg import ActionInstance, Plan, PlatformClass
from auspex_planning.planner.utils.alns_utils.utils import *
from auspex_planning.planner.utils.alns_utils.destroy_methods import *
from auspex_planning.planner.utils.alns_utils.repair_methods import *
from auspex_planning.planner.utils.alns_utils.alns import ALNS
from auspex_planning.planner.utils.alns_utils.alpha_beta_UCB import AlphaBetaUCB
from auspex_planning.planner.utils.alns_utils.vrpc import VRPState
from auspex_planning.planner.utils.converter import AUSPEXConverter

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from copy import deepcopy

# from alns import ALNS
from alns.accept import RecordToRecordTravel
from alns.select import RouletteWheel
from alns.stop import MaxRuntime
from alns.stop import MaxIterations
from alns.select import AlphaUCB

from sklearn.cluster import KMeans
import networkx as nx
import logging,sys
import time
import os
import threading

logging.basicConfig(stream=sys.stderr, level=logging.INFO)#DEBUG MEANS PRINTS, INFO MEANS NO PRINT

class MVPR_ALNS_Planner(PlannerBase):
    planner_key = 'mvrp_alns_planner'
    def __init__(self, kb_client):
        '''
        Knowledge Base
        '''
        self._kb_client = kb_client
        self._converter = AUSPEXConverter()

        '''
        For Parsing the Json File
        '''
        self._home_coordinates = None
        self._melted_df = None

        self._lock = threading.Lock()

        '''
        Start of ALNS lib
        '''
        #op_coupling = np.array([[True, True], [True, True], [True, True]])
        SEED= 123

        rnd_state = np.random.default_rng(seed=SEED)

        self._alns= ALNS(rnd_state)
        self._alns.add_destroy_operator(destroy_random, "Destroy Random")
        self._alns.add_destroy_operator(destroy_nodes_change,'Destroy with Change')
        self._alns.add_destroy_operator(destroy_nodes_distance,"Destroy with Distance")
        self._alns.add_destroy_operator(destroy_most_nodes,"Destroy with Most Node")


        self._alns.add_repair_operator(repair_solution_best_position, "Repair Solution Best")
        self._alns.add_repair_operator(repair_solution_random,'Repair Random')
        self._alns.add_repair_operator(repair_solution_same,'Repair Solution Same Vehicle')
        self._alns.add_repair_operator(repair_solution_position,'Repair Solution Same Insert')
        self._alns.add_repair_operator(repair_solution_lowest,'Repair Solution Lowest None')
        self._alns.add_repair_operator(repair_solution_closest,"Repair Solution Closest Node")
        self._alns.add_repair_operator(repair_solution_type, 'Repair Solution Type Based')
        self._alns.add_repair_operator(repair_solution_best_approx, 'Repair Solution Best Approximation')

        weights=[20,10,1,0]
        alpha = 0.8
        beta = 0.0 # [0,1] -> 1 will linearly weight execution time over optimality

        self._selector = AlphaBetaUCB(scores=weights,alpha=alpha,beta=beta,num_destroy=4,num_repair=8)
        # alpha=0.5
        # self._selector = AlphaUCB(scores=weights,alpha=alpha,num_destroy=len(alns.destroy_operators),num_repair=len(alns.repair_operators))


        # self._stop_criteria = MaxRuntime(200)
        max_iterations= 1000
        self._stop_criteria = MaxIterations(max_iterations)


    def get_operation_area(self):
        areas_db = self._kb_client.query(collection='area', key='', value='')
        if not areas_db:
            return None, None

        home = None  # to store the 'home' entry
        data = {
            "Locations": [],
            "Lat": [],
            "Long": [],
            "Alt": []
        }

        for area in areas_db:
            name = area.get('name')
            points = area.get('points', [])

            if points and len(points[0]) >= 3:
                centre = [float(x) for x in points[0]]
            else:
                centre = []

            if 'home' in name.lower():
                home = area['points'][0] if 'points' in area and len(area['points']) > 0 else None
                continue

            if name and len(centre) >= 3:
                data["Locations"].append(name)
                data["Lat"].append(centre[0])
                data["Long"].append(centre[1])
                data["Alt"].append(centre[2])

        df = pd.DataFrame(data).reset_index(drop=True)
        return home, df

    def plan_mission(self, team_id):
        print(f"[INFO]: ALNS Planner Selected for team : {team_id}")
        self._home_coordinates = None
        self._melted_df = None

        self._home_coordinates, melted_df = self.get_operation_area()
        if self._home_coordinates is None or melted_df is None:
            print("[ERROR]: No home coordinates or melted_df found. Returning...")
            return []

        """
        Get number of vehicles
        """
        vhcl_dict = self._kb_client.query('platform', 'platform_id', 'team_id', team_id)
        num_vehicles = len(vhcl_dict)

        if num_vehicles == 0:
            print("[Warning]: No vehicle specified. Defaulting to offline planning...")
            num_vehicles = 2

        """
        Query objective
        """
        objective_function = self._kb_client.query('config', 'params', 'team_id', team_id)
        if objective_function == []:
            objective_function = "shortest_distance"
            print(f'No objective function given. Defaulting to {objective_function}.')
        else:
            objective_function = objective_function[0]

        '''
        Initializes the graph with ndoes and edges -> also the vehicles routes -> adds cluster ids to the melted_df
        '''
        G, vehicle_routes_list, melted_df = self.initializeRoutes(num_vehicles, melted_df)

        '''
        Insert home position
        '''
        new_line = {
            'Locations': 'home',
            'Lat': self._home_coordinates[0],
            'Long': self._home_coordinates[1],
            'Alt': self._home_coordinates[2],
            'Cluster': -1
        }

        melted_df = melted_df._append(new_line, ignore_index=True)

        init_vehicle_routes = pd.DataFrame(columns=['Vehicle', 'Route'])
        for vehicle, path in enumerate(vehicle_routes_list):
            init_vehicle_routes = pd.concat([init_vehicle_routes, pd.DataFrame({'Vehicle': [vehicle], 'Route': [path]})], ignore_index=True)

        '''
        Create a Vehicle Routing Problem State
        '''

        package_share_directory = get_package_share_directory('auspex_planning')
        workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(package_share_directory))))

        avg_distances_file_path = os.path.join(workspace_dir, "src", 'auspex_planning', "auspex_planning", "planner", "alns_utils", 'average_distances.json')
        avg_distances = load_avg_distances(avg_distances_file_path)
        initial_state = VRPState(objective_function=objective_function, home_coordinates=self._home_coordinates, vehicle_routes=init_vehicle_routes, melted_df=melted_df, G=G, avg_distances= avg_distances)

        self._alns.on_best(initial_state.my_on_best)
        self._alns.on_better(initial_state.my_on_better)
        self._alns.on_reject(initial_state.my_reject)
        self._alns.on_accept(initial_state.my_accept)

        accept = RecordToRecordTravel.autofit(initial_state.objective(),0.025,0.0001,15000)

        show_vis = True
        fig_suffix = "figure_description"
        if show_vis:
            visualize_routes_and_locations(G, init_vehicle_routes, self._home_coordinates, melted_df, path_prefix=fig_suffix+"_start")

        try:
            '''
            Start ALNS
            '''
            start_time=time.time()
            result= self._alns.iterate(initial_state, self._selector, accept, self._stop_criteria)
            end_time = time.time()
            duration= end_time - start_time

            '''
            Get Results
            '''
            best_solution_vehicles= result.best_state.vehicle_routes

            """
            Printing Results
            """
            initial_state.printVehicleRoutesExtern(best_solution_vehicles)

            if show_vis:
                fig2 = plt.figure(2)
                ax = fig2.add_subplot()
                result.plot_objectives(ax=ax)
                result.plot_operator_counts()

            if show_vis:
                visualize_routes_and_locations(G, best_solution_vehicles, self._home_coordinates,  melted_df, path_prefix=fig_suffix +"_solution")
            print(best_solution_vehicles)

            actions_list = []
            for index, row in best_solution_vehicles.iterrows():
                plan_msg = Plan()
                plan_msg.team_id = team_id

                if len(vhcl_dict) == 0:
                    platform_id = "vhcl"+ str(row['Vehicle'])
                else:
                    platform_id = vhcl_dict[row['Vehicle']]['platform_id']

                plan_msg.platform_id = platform_id
                plan_msg.priority = 0
                plan_msg.tasks = self._converter.convert_plan_mvrp2auspex(platform_id, row['Route'])
                plan_msg.actions = self._converter.convert_plan_mvrp2auspex(platform_id, row['Route'])
                actions_list.append(plan_msg)

            return actions_list
        except Exception as e:
            print(e)
            return []

    def feedback(self, team_id, feedback_msg):
        pass

    def result(self, team_id, result_msg):
        pass

    '''
    Computes clusters with kmeans and assigns each vehicle one cluster
    '''
    def initializeRoutes(self, num_of_vehicles, melted_df):
        G = nx.Graph()

        for idx, row in melted_df.iterrows():
            G.add_node(row['Locations'], pos=(row['Lat'], row['Long'], row['Alt']))
        for u, u_data in G.nodes(data=True):
            for v, v_data in G.nodes(data=True):
                if u != v:
                    distance = calculate_distance_3d(u_data['pos'], v_data['pos'])
                    G.add_edge(u, v, weight=distance)

        shortest_distance = float('inf')

        for node in G.nodes:
            tour, total_distance = self.nearest_neighbor(G, node)
            if total_distance < shortest_distance:
                shortest_distance = total_distance

        kmeans = KMeans(n_clusters=num_of_vehicles, random_state=0)

        clusters = kmeans.fit_predict(melted_df[['Lat','Long', 'Alt']])

        melted_df['Cluster']  = clusters

        for vehicle in range(num_of_vehicles):

            if vehicle not in melted_df['Cluster']:

                nearest_cluster = kmeans.transform(melted_df[['Lat', 'Long','Alt']]).mean(axis=0).argmin()
                idx = melted_df[melted_df['Cluster'] == nearest_cluster].index[0]
                melted_df.at[idx, 'Cluster'] = vehicle

        vehicle_routes = [[] for _ in range(num_of_vehicles)]

        for vehicle in range(num_of_vehicles):
            cluster_locations= melted_df[melted_df['Cluster']==vehicle]

            if len(cluster_locations)>0:

                subgraph = G.subgraph(cluster_locations['Locations'])

                if subgraph.number_of_edges() >0:
                    vehicle_routes[vehicle]= nx.approximation.traveling_salesman_problem(subgraph,cycle=False)

                elif subgraph.number_of_nodes() == 1:
                    vehicle_routes[vehicle] = list(subgraph.nodes)

        return G, vehicle_routes, melted_df

    '''
    Computes the distance in 3D
    '''
    def nearest_neighbor(self, graph, start_node):
        unvisited_nodes = set(graph.nodes)
        current_node = start_node
        unvisited_nodes.remove(current_node)
        tour = [current_node]
        total_distance = 0

        while unvisited_nodes:
            nearest_node = min(unvisited_nodes, key=lambda node: graph[current_node][node]['weight'])
            tour.append(nearest_node)
            total_distance += graph[current_node][nearest_node]['weight']
            current_node = nearest_node
            unvisited_nodes.remove(current_node)

        tour.append(start_node)
        total_distance += graph[current_node][start_node]['weight']

        return tour, total_distance



    ###################################################################################################################################################################
    ###################################################################################################################################################################
    ############################################################################     MULTI Threading Solution             #############################################
    ###################################################################################################################################################################
    ###################################################################################################################################################################

    '''
    For multithreading
    '''
    def run_alns(self, weights, alpha, max_iterations, initial_solution, results, index):
        #logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
        alns = ALNS()
        logging.info("Starting to ALNS")
        initial_solution.iteration_best=-1
        initial_solution.iteration_number=0
        select = AlphaUCB(scores=weights, alpha=alpha,
                        num_destroy=len(alns.destroy_operators),
                        num_repair=len(alns.repair_operators))
        accept = RecordToRecordTravel.autofit(initial_solution.objective(), 0.025, 0.0001, 15000)
        stop = MaxIterations(max_iterations)

        result = alns.iterate(initial_solution, select, accept, stop)

        with self._lock:

            results[index] = (result.best_state.objective(), result.best_state.vehicle_routes)
            logging.info("Finishing to ALNS")

            logging.info(f"Here is the result {results[index][0]} and {results[index][1]}")

    '''
    For Creating Threads
    '''
    def loop(self, num_cycles,initial_solution):

        thread_configs = [
            ([20, 10, 1, 0], 0.8, 3, initial_solution),
            ([20, 5, 1, 0], 0.4, 3, initial_solution)
        ]


        global_initial_solution = deepcopy(initial_solution)

        for i in range(num_cycles):
            results= {}
            threads= []
            numThreads=2
            for index in range(numThreads):

                weights, alpha, max_iterations, ini_sol = thread_configs[index%2]
                thread_initial_solution = deepcopy(global_initial_solution)
                t = threading.Thread(target=self.run_alns, args=(weights, alpha, max_iterations, thread_initial_solution, results, index))
                threads.append(t)
                t.start()

            for t in threads:
                t.join()


            current_best_index = min(results, key=lambda x: results[x][0])
            current_best_routes = results[current_best_index][1]
            global_initial_solution.vehicle_routes = deepcopy(current_best_routes)
