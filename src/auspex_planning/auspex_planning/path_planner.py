from .planner.path_planners.pattern_path_planner import PatternPathPlanner
from auspex_msgs.msg import Plan, ActionInstance
from fractions import Fraction
from upf_msgs.msg import (
    Atom,
    Real
)
import copy

class Path_Planner():
    def __init__(self, kb_client):
        self._kb_client = kb_client
        self._pattern_path_planner = PatternPathPlanner(kb_client)

    def plan_path(self, team_id, task_plans):
        """
        Plans a path based on the provided tasks.

        BEWARE: from here plans are already assigned to platforms
        """
        print("[INFO]: Path Planner Selected")

        areas = {}
        areas_db = self._kb_client.query(collection='area', key='', value='')
        if len(areas_db) > 0:
            for area in areas_db:
                if not area:
                    continue
                areas[area['name']] = area

        for plan in task_plans:
            if plan.platform_id == "":
                continue
                
            actions = []

            for task in plan.tasks:

                if task.action_name == 'search_poi':
                    continue
                elif task.action_name == 'search_area':

                    search_area, flight_height = self.getSearchArea(areas, task.parameters[1].symbol_atom[0])
                    if flight_height is None:
                        continue
                    new_actions = self._pattern_path_planner.plan_search_area(plan.platform_id, search_area, flight_height)
                    for new_action in new_actions:
                        new_action.task_id = task.id
                        actions.append(new_action)
                else:
                    # Replace every symbol area with GPS Coords
                    new_action = copy.deepcopy(task)
                    parameters = []
                    continue_outer = False
                    for parameter in task.parameters:
                        new_parameter = [copy.deepcopy(parameter)]
                        if len(parameter.symbol_atom) == 0:
                            parameters.extend(new_parameter)
                            continue
                        s_atom = parameter.symbol_atom[0] # From Message Definition, only one symbol_atom is allowed

                        if s_atom == "home": #for not assigned home areas
                            s_atom = s_atom + "_" + plan.platform_id

                        if s_atom in areas:
                            if task.action_name == 'search_area_uav':
                                new_parameter = self.createCoordinateAtoms(self.getPoseAtom4Area(position=1, areas=areas, area=s_atom))
                                new_parameter.extend(self.createCoordinateAtoms(self.getPoseAtom4Area(position=3, areas=areas, area=s_atom)))
                            else:
                                new_parameter = self.createCoordinateAtoms(self.getPoseAtom4Area(position=0, areas=areas, area=s_atom))


                        parameters.extend(new_parameter)

                    if parameters == []:
                        continue

                    new_action.parameters = parameters
                    new_action.task_id = task.id
                    actions.append(new_action)

            for idx, act in enumerate(actions):
                act.id = idx
            plan.actions = actions
        return task_plans

    def getSearchArea(self, areas, area):
        search_area = []
        flight_height = None

        if area == "last_known_position":

            latlonalt = self.getPoseAtom4Area(position=0, areas=areas, area=area)
            if latlonalt == []:
                return [], None

            lat, lon, alt = latlonalt
            square_size_m = 10
            dlat = (square_size_m / 2.0) / 111320.0
            dlon = (square_size_m / 2.0) / (111320.0 * abs(math.cos(math.radians(lat))) + 1e-8)

            search_area.append((lat - dlat, lon - dlon))
            search_area.append((lat + dlat, lon - dlon))
            search_area.append((lat + dlat, lon + dlon))
            search_area.append((lat - dlat, lon + dlon))

            flight_height = alt

        else:

            if area not in areas:
                print("[ERROR] Area not found in database: ", area)
            else:

                mean_alt = 0.0
                for point in areas[area]['points'][1:]:
                    search_area.append((float(point[0]), float(point[1])))
                    mean_alt += float(point[2])

                flight_height = (mean_alt)/len(search_area)

        return search_area, flight_height

    def getPoseAtom4Area(self, position, areas, area): # 0 == centre; 1 == lower bounds; 3 == upper_bounds
        lat = 0.0
        lon = 0.0
        alt = 0.0


        if (area == "last_known_position" or "home" in area) and position != 0:
            print("[ERROR] Manual Search Area can not be searched via search_area_uav")
            return []

        if area not in areas:
            print("[ERROR] Area not found in database: ", area)
            return []

        try:
            lat = float(areas[area]['points'][position][0])
            lon = float(areas[area]['points'][position][1])
            alt = float(areas[area]['points'][position][2])
        except (KeyError, IndexError, ValueError) as e:
            print(f"[ERROR] Failed to extract coordinates for area '{area}', position '{position}': {e}")
            return []

        if "home_" in area:
            if lat == 0.0 and lon == 0.0 and alt == 0.0:
                print("[ERROR] Home position not set, returning ...")
                return []
            alt = 10.0

        return [lat, lon, alt]

    def createCoordinateAtoms(self, lat_lon_alt_list):
        cords = []
        if lat_lon_alt_list == []:
            return cords

        lat = lat_lon_alt_list[0]
        lon = lat_lon_alt_list[1]
        alt = lat_lon_alt_list[2]

        lat_real = Real()
        lat_fraction = Fraction.from_float(lat)
        lat_real.numerator = lat_fraction.numerator
        lat_real.denominator = lat_fraction.denominator
        lat_atom = Atom()
        lat_atom.real_atom = [lat_real]


        lon_real = Real()
        lon_fraction = Fraction.from_float(lon)
        lon_real.numerator = lon_fraction.numerator
        lon_real.denominator = lon_fraction.denominator
        lon_atom = Atom()
        lon_atom.real_atom = [lon_real]

        alt_real = Real()
        alt_fraction = Fraction.from_float(alt)
        alt_real.numerator = alt_fraction.numerator
        alt_real.denominator = alt_fraction.denominator
        alt_atom = Atom()
        alt_atom.real_atom = [alt_real]

        cords.append(lat_atom)
        cords.append(lon_atom)
        cords.append(alt_atom)

        return cords