import math

class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = 0
        self.verbose = verbose

    def reward_function(self, params):

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                        y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                x2=second_closest_coords[0],
                                y1=closest_coords[1],
                                y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                x2=closest_coords[0],
                                y1=car_coords[1],
                                y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                x2=second_closest_coords[0],
                                y1=car_coords[1],
                                y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                            (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                            car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                            x2=second_closest_coords[0],
                                                            y1=new_car_coords[1],
                                                            y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[-1.56094, -1.45028, 3.1562, 0.06778],
                        [-1.35473, -1.52276, 3.44774, 0.0634],
                        [-1.13742, -1.58155, 3.79382, 0.05934],
                        [-0.90887, -1.62818, 4.0, 0.05831],
                        [-0.66862, -1.66394, 4.0, 0.06072],
                        [-0.41647, -1.69032, 4.0, 0.06338],
                        [-0.15221, -1.7087, 3.71298, 0.07134],
                        [0.12441, -1.72025, 3.07932, 0.08991],
                        [0.41436, -1.72577, 2.65729, 0.10914],
                        [0.7216, -1.72582, 2.36211, 0.13007],
                        [1.02622, -1.72113, 2.12721, 0.14322],
                        [1.32102, -1.70186, 1.88207, 0.15697],
                        [1.59848, -1.66053, 1.68101, 0.16687],
                        [1.85089, -1.59316, 1.49027, 0.1753],
                        [2.07185, -1.49942, 1.3, 0.18463],
                        [2.25743, -1.38197, 1.3, 0.16894],
                        [2.40528, -1.24465, 1.3, 0.15522],
                        [2.51159, -1.0909, 1.3, 0.14379],
                        [2.57214, -0.92509, 1.3, 0.13578],
                        [2.57873, -0.75262, 1.3, 0.13277],
                        [2.51244, -0.58525, 1.68257, 0.10699],
                        [2.39749, -0.43028, 1.96073, 0.09841],
                        [2.24046, -0.29111, 2.31251, 0.09073],
                        [2.04759, -0.16934, 2.92399, 0.07801],
                        [1.82885, -0.06256, 4.0, 0.06085],
                        [1.59614, 0.03523, 4.0, 0.06311],
                        [1.33521, 0.15095, 4.0, 0.07136],
                        [1.07629, 0.27114, 4.0, 0.07136],
                        [0.81906, 0.39506, 4.0, 0.07138],
                        [0.56327, 0.52217, 4.0, 0.07141],
                        [0.30871, 0.65199, 3.76466, 0.0759],
                        [0.05518, 0.7841, 3.31983, 0.08611],
                        [-0.19737, 0.91837, 2.97559, 0.09612],
                        [-0.44348, 1.04352, 2.69788, 0.10234],
                        [-0.69012, 1.158, 2.38599, 0.11396],
                        [-0.93695, 1.25662, 2.11954, 0.1254],
                        [-1.18304, 1.33464, 1.90047, 0.13585],
                        [-1.42685, 1.38812, 1.65923, 0.15043],
                        [-1.66614, 1.41369, 1.65923, 0.14504],
                        [-1.89808, 1.40859, 1.65923, 0.13982],
                        [-2.11834, 1.36842, 1.65923, 0.13494],
                        [-2.3208, 1.28875, 1.65923, 0.13113],
                        [-2.4968, 1.16519, 1.65923, 0.1296],
                        [-2.62878, 0.98994, 1.99752, 0.10983],
                        [-2.72235, 0.78249, 2.17722, 0.10453],
                        [-2.77637, 0.55124, 2.35874, 0.10068],
                        [-2.79055, 0.30645, 2.56201, 0.09571],
                        [-2.7681, 0.05963, 2.68382, 0.09234],
                        [-2.71299, -0.1798, 2.44078, 0.10066],
                        [-2.63019, -0.40542, 2.44078, 0.09847],
                        [-2.52494, -0.61413, 2.44078, 0.09577],
                        [-2.40146, -0.80493, 2.44078, 0.09311],
                        [-2.26276, -0.97765, 2.44078, 0.09076],
                        [-2.10925, -1.12998, 2.44078, 0.0886],
                        [-1.9391, -1.25649, 2.69294, 0.07873],
                        [-1.75594, -1.3623, 2.89987, 0.07294]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 5
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 8
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1 
        STANDARD_TIME = 9.5
        FASTEST_TIME = 6.5
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                        (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3
            
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        STANDARD_TIME = 9.5  # seconds (time that is easily done by model)
        FASTEST_TIME = 6.5  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                    (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################
        
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        
        # Always return a float value
        return float(reward)


reward_object = Reward(verbose=True) # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
