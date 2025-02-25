import copy
from geopy.distance import geodesic
import math
import logging
import pandas as pd

#This class represents the state of the vehicle routing problem, stores vehicle routes and copies and calculates the objective(total cost)
class VRPState:
    def __init__(self, objective_function, home_coordinates, vehicle_routes, melted_df = None, G = None, avg_distances = None):
        self.objective_function = objective_function
        self.home_coordinates = home_coordinates
        self.vehicle_routes = self.copy_dataframe(vehicle_routes)
        self.destroyed_nodes = []
        self.remaining_nodes = []
        self.selected_vehicle = 0
        self.melted_df = melted_df
        self.G = G
        self.avg_distances = avg_distances
        self.decay=0.8
        self.threshold =0.5
        self.best_vehicle_routes=self.copy_dataframe(self.vehicle_routes)
        self.best_objective=self.objective()
        self.iteration_number=0
        self.repair_iteration=0
        self.iteration_best = -1
        self.destroyed_node_position = -1




    def copy(self):
        """
        Helper method to ensure each solution state is immutable.
        """
        help = VRPState(self.objective_function, self.home_coordinates, self.vehicle_routes, self.melted_df, self.G,self.avg_distances)
        help.destroyed_nodes = self.destroyed_nodes.copy()
        help.remaining_nodes = self.remaining_nodes.copy()
        help.threshold = copy.deepcopy(self.threshold)
        return help

    def reset(self):

        self.remaining_nodes=[]
        self.destroyed_nodes=[]
        self.selected_vehicle=-1

    def my_on_best(self,cand_state,rnd_state):
        self.iteration_best = self.iteration_number
        # print("Best")
        self._step(cand_state)

    def my_on_better(self,cand_state,rnd_state):
        # print("Better")
        self._step(cand_state)

    def my_accept(self,cand_state,rnd_state):
        # print("Accept")
        self._step(cand_state)

    def my_reject(self,cand_state,rnd_state):
        # print("Reject")
        self._step(cand_state)

    def _step(self, cand_state)->None:
        self.iteration_number+=1
        logging.info(f"Step: {self.iteration_number}")

    def objective(self) -> float:
        total_objective = 0
        df_copy = self.copy_dataframe(self.vehicle_routes)
        if self.objective_function == "shortest_distance":
            for route in df_copy['Route']:
                route.insert(0, 'home')
                route_objective = self.calculate_route_objective(route)
                total_objective += route_objective
        elif self.objective_function == "fewest_vehicles":
            pass
        elif self.objective_function == "shortest_time":
            pass
        else:
            for route in df_copy['Route']:
                route.insert(0, 'home')
                route_objective = self.calculate_route_objective(route)
                total_objective += route_objective

        return total_objective

    def calculate_route_objective(self, route):
        route_distance = 0
        for i in range(len(route)-1):
            point1= self.get_coordinate(route[i])
            point2= self.get_coordinate(route[i+1])

            route_distance += self.calculate_distance_3d(point1, point2)
        return route_distance

    def get_coordinate(self,location):
        if location in self.melted_df['Locations'].values:
            coords = self.melted_df.loc[self.melted_df['Locations'] == location].iloc[0]
            return (coords['Long'],coords['Lat'],coords['Alt'])
        else:
            logging.debug(f"Coordinates for the location {location} is not found")
            return None

    def calculate_distance_3d(self,point1,point2):

        lat1,lon1,alt1 = point1
        lat2,lon2,alt2 = point2
        two_d_distance = geodesic((lat1,lon1),(lat2,lon2)).meters

        delta_altitude = alt2-alt1

        distance_3d= math.sqrt(two_d_distance**2 + delta_altitude**2)

        return distance_3d

    def copy_dataframe(self, df):
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

    def list2string(self, list_):
        if isinstance(list_, list):
            new_list = ','.join(list_)
            return new_list

    def printVehicleRoutes(self):
        i = 0
        for route in self.vehicle_routes['Route']:
            logging.info(f'Vehicle {i} with Route: {self.list2string(route)}')
            i+=1
    def printVehicleRoutesExtern(self, vehicle_routes_cp):
        i = 0
        for route in vehicle_routes_cp['Route']:
            logging.info(f'Vehicle {i} with Route: {self.list2string(route)}')
            i+=1

    def printBestVehicleRoutes(self):
        i = 0
        for route in self.best_vehicle_routes['Route']:
            logging.info(f'Vehicle {i} with Route: {self.list2string(route)}')
            i+=1

    def countVehicleRoutes(self):
        totel_length = 0
        for route in self.vehicle_routes['Route']:
            totel_length += len(route)
        return totel_length

    def number_of_locations(self, updated_routes):
        total_location=0
        total_location = updated_routes['Route'].apply(len).sum()
        logging.info(f"Number of vehicles are {total_location}")