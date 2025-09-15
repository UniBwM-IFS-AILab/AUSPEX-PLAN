#!/usr/bin/env python3
import math
import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString, Polygon


class SearchPatternGenerator:

    def metersToDeltaLatLon(self, meters, direction):
        meters = meters/ 1000.0
        if direction == 'lat':
            dlat = meters/111.320
            return dlat
        elif direction == 'lon':
            dlon = meters/(111.320*math.cos(math.pi/4.0))
            return dlon
        else:
            return 0.0

    def deltaLatLonToMeters(self, lat1, lon1, lat2, lon2):
        avg_lat = (math.pi/180 * (lat1 + lat2)) / 2
        dlat = 111.320 * (lat1 - lat2)
        dlon = 111.320 * math.cos(avg_lat) * (lon1 - lon2)
        distance = math.sqrt(dlat * dlat + dlon * dlon)
        return distance * 1000.0

    def mow_the_lawn(self, search_area, starting_point, fov, flight_height, turning_radius=0.0, sweeping_direction='', overlap=0.0):
        if fov >= math.pi or fov <= 0:
            print("error: fov between ]0, pi[")
            return []

        starting_point = Point(starting_point)
        polygon = Polygon(search_area)
        x_min = polygon.bounds[0]
        y_min = polygon.bounds[1]
        x_max = polygon.bounds[2]
        y_max = polygon.bounds[3]
        lon_max = y_max - y_min
        lat_max = x_max - x_min

        if sweeping_direction != 'lat' and sweeping_direction != 'lon':
            if lon_max > lat_max:
                sweeping_direction = 'lat'
            else:
                sweeping_direction = 'lon'

        sweeping_dist = 0.0

        footprint = 2*flight_height*math.tan(0.5*fov)

        sweeping_dist = self.metersToDeltaLatLon(footprint, sweeping_direction)

        x_start = x_min - 0.5*sweeping_dist
        y_start = y_min - 0.5*sweeping_dist

        sweeping_lines = []

        if sweeping_direction == 'lon':
            num_of_lines = math.floor((x_max - x_min)/sweeping_dist) + 1
            for i in range(1, num_of_lines, 1):
                x_val = x_start + i*sweeping_dist
                line = LineString([(x_val, y_min), (x_val, y_max)])
                sweeping_lines.append(line)
        else: # lat
            num_of_lines = math.floor((y_max - y_min)/sweeping_dist) + 1
            for i in range(1, num_of_lines, 1):
                y_val = y_start + i*sweeping_dist
                line = LineString([(x_min, y_val), (x_max, y_val)])
                sweeping_lines.append(line)

        intersections = []
        for line in sweeping_lines:
            intersection = polygon.intersection(line)
            intersections.append(intersection)

        waypoints = []

        turning_dist = self.metersToDeltaLatLon(turning_radius, sweeping_direction)

        counter = 1
        for intersection in intersections:
            lx, ly = intersection.xy
            if len(lx) > 0.0 and len(ly) > 0.0:
                if sweeping_direction == 'lon':
                    ly1 = ly[0]
                    ly2 = ly[1]
                    if ly1 > ly2:
                        ly1 -= turning_dist
                        ly2 += turning_dist
                    else:
                        ly1 += turning_dist
                        ly2 -= turning_dist
                    p1 = Point(lx[0], ly1, flight_height)
                    p2 = Point(lx[1], ly2, flight_height)
                else: # lat
                    lx1 = lx[0]
                    lx2 = lx[1]
                    if lx1 > lx2:
                        lx1 -= turning_dist
                        lx2 += turning_dist
                    else:
                        lx1 += turning_dist
                        lx2 -= turning_dist
                    p1 = Point(lx1, ly[0], flight_height)
                    p2 = Point(lx2, ly[1], flight_height)
                if counter % 2 == 0:
                    waypoints.append(p1)
                    waypoints.append(p2)
                else:
                    waypoints.append(p2)
                    waypoints.append(p1)
                counter += 1

        if len(waypoints) == 0:
            print("No waypoints found for given search area and parameters.")
            return []

        self.orderPoints(waypoints, starting_point)

        # comment in for debugging
        # self._displayPolygon(polygon, sweeping_lines, waypoints, starting_point)

        return waypoints

    def orderPoints(self, waypoints, starting_point):
        min_distance = float('inf')
        min_index = 0
        for i in range(-2,1,1):
            distance_to_start = starting_point.distance(waypoints[i])
            if distance_to_start < min_distance:
                min_index = i
                min_distance = distance_to_start

        match min_index:
            #case 0:
            case 1:
                self.swap(waypoints)
            case -1:
                self.reverse(waypoints)
            case -2:
                self.swap(waypoints)
                self.reverse(waypoints)

    def swap(self, waypoints):
        for i in range(0,len(waypoints)-1,2):
            waypoints[i], waypoints[i+1] = waypoints[i+1], waypoints[i]

    def reverse(self, waypoints):
        waypoints.reverse()

    def _displayPolygon(self, polygon, lines, points, starting_point):
        x, y = polygon.exterior.xy
        plt.fill(y, x, alpha=0.7, fc='orange', ec='black')
        plt.plot(y, x, color='black')

        for line in lines:
            line_x, line_y = line.xy
            plt.plot(line_y, line_x, color='blue', linewidth=2, alpha=0.7)

        plt.plot(starting_point.y, starting_point.x, color='crimson', marker='x')
        plt.text(starting_point.y, starting_point.x, 'start', fontsize=12)

        counter = 0
        for point in points:
            plt.plot(point.y, point.x, color='deeppink', marker='o')
            plt.text(point.y, point.x, str(counter), fontsize=12)
            counter+=1

        plt.title('Lawnmower Flight Pattern')
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.grid()
        plt.show()
        return
