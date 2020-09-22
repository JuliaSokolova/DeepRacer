import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        import math

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
        racing_track = [[0.63069, 2.80612, 1.39051, 0.08247],
                        [0.63367, 2.6908, 1.42543, 0.08093],
                        [0.64672, 2.57569, 1.43201, 0.08089],
                        [0.66972, 2.46184, 1.43201, 0.08111],
                        [0.70252, 2.35023, 1.43201, 0.08124],
                        [0.74488, 2.24178, 1.43201, 0.08131],
                        [0.79653, 2.13733, 1.43236, 0.08135],
                        [0.85714, 2.03762, 1.43797, 0.08115],
                        [0.92634, 1.94325, 1.45216, 0.08058],
                        [1.00366, 1.85469, 1.47829, 0.07953],
                        [1.08862, 1.77223, 1.51975, 0.0779],
                        [1.18065, 1.69601, 1.58004, 0.07563],
                        [1.27917, 1.62597, 1.66287, 0.07269],
                        [1.38351, 1.56189, 1.7726, 0.06908],
                        [1.49304, 1.50337, 1.91501, 0.06484],
                        [1.60709, 1.44988, 2.09957, 0.06],
                        [1.72504, 1.40077, 2.34494, 0.05449],
                        [1.8463, 1.3553, 2.69405, 0.04807],
                        [1.97026, 1.31267, 3.26832, 0.04011],
                        [2.09618, 1.27199, 4.0, 0.03308],
                        [2.22315, 1.23232, 4.0, 0.03326],
                        [2.35577, 1.19068, 4.0, 0.03475],
                        [2.48814, 1.14836, 4.0, 0.03474],
                        [2.6201, 1.10491, 4.0, 0.03473],
                        [2.75156, 1.06007, 4.0, 0.03472],
                        [2.88246, 1.01371, 4.0, 0.03472],
                        [3.01284, 0.96594, 4.0, 0.03471],
                        [3.14278, 0.91698, 4.0, 0.03472],
                        [3.2724, 0.86711, 3.31872, 0.04185],
                        [3.40179, 0.81664, 2.77135, 0.05011],
                        [3.52534, 0.76799, 2.46303, 0.05391],
                        [3.64883, 0.72099, 2.25365, 0.05863],
                        [3.77225, 0.67658, 2.09894, 0.06249],
                        [3.89566, 0.63577, 1.97999, 0.06565],
                        [4.01913, 0.59935, 1.88801, 0.06818],
                        [4.14273, 0.56803, 1.81882, 0.07011],
                        [4.26652, 0.5424, 1.77038, 0.07141],
                        [4.39051, 0.52303, 1.74145, 0.07206],
                        [4.51466, 0.51041, 1.73098, 0.07209],
                        [4.6389, 0.50498, 1.73098, 0.07184],
                        [4.76312, 0.50709, 1.73098, 0.07178],
                        [4.8872, 0.517, 1.73098, 0.07191],
                        [5.01096, 0.53487, 1.73779, 0.07196],
                        [5.13423, 0.56074, 1.76051, 0.07154],
                        [5.2568, 0.59459, 1.79758, 0.07074],
                        [5.37847, 0.63625, 1.84733, 0.06962],
                        [5.49902, 0.68552, 1.90803, 0.06825],
                        [5.61825, 0.74209, 1.978, 0.06672],
                        [5.73595, 0.80562, 2.05561, 0.06507],
                        [5.85194, 0.87573, 2.13932, 0.06335],
                        [5.96605, 0.95201, 2.22768, 0.06162],
                        [6.07813, 1.03404, 2.31933, 0.05988],
                        [6.18805, 1.1214, 2.41297, 0.05819],
                        [6.2957, 1.21366, 2.5073, 0.05655],
                        [6.40099, 1.31043, 2.60104, 0.05498],
                        [6.50386, 1.41132, 2.69282, 0.05351],
                        [6.60426, 1.51598, 2.78117, 0.05214],
                        [6.70214, 1.62407, 2.86453, 0.0509],
                        [6.79747, 1.73528, 2.94121, 0.0498],
                        [6.89022, 1.84935, 3.0094, 0.04885],
                        [6.98037, 1.96604, 3.06724, 0.04807],
                        [7.06789, 2.08511, 3.11288, 0.04747],
                        [7.15274, 2.20639, 3.14255, 0.0471],
                        [7.23487, 2.32968, 3.10729, 0.04768],
                        [7.31419, 2.45484, 3.05507, 0.0485],
                        [7.39063, 2.58173, 2.987, 0.04959],
                        [7.46407, 2.7102, 2.90476, 0.05094],
                        [7.53435, 2.84012, 2.81045, 0.05256],
                        [7.6013, 2.97136, 2.70642, 0.05444],
                        [7.66469, 3.10378, 2.59514, 0.05657],
                        [7.72428, 3.23722, 2.47907, 0.05895],
                        [7.77975, 3.37151, 2.36051, 0.06155],
                        [7.83077, 3.50645, 2.24159, 0.06436],
                        [7.87696, 3.6418, 2.12422, 0.06733],
                        [7.91791, 3.77731, 2.01014, 0.07042],
                        [7.95317, 3.91265, 1.90091, 0.07358],
                        [7.98227, 4.04748, 1.79797, 0.07672],
                        [8.00473, 4.18141, 1.7027, 0.07975],
                        [8.02008, 4.31398, 1.61638, 0.08257],
                        [8.02783, 4.44473, 1.54025, 0.08503],
                        [8.02756, 4.57311, 1.47543, 0.08702],
                        [8.01887, 4.69859, 1.42291, 0.08839],
                        [8.00143, 4.82057, 1.38346, 0.08907],
                        [7.97497, 4.93849, 1.35758, 0.08901],
                        [7.93933, 5.05174, 1.3454, 0.08825],
                        [7.89443, 5.15975, 1.3454, 0.08694],
                        [7.84031, 5.26197, 1.3454, 0.08597],
                        [7.77711, 5.35788, 1.3454, 0.08537],
                        [7.70509, 5.44702, 1.34661, 0.0851],
                        [7.6246, 5.52897, 1.36047, 0.08443],
                        [7.53611, 5.60338, 1.38574, 0.08343],
                        [7.44017, 5.66995, 1.42077, 0.08219],
                        [7.33739, 5.72846, 1.46351, 0.08081],
                        [7.22847, 5.77874, 1.51165, 0.07936],
                        [7.11414, 5.82067, 1.56268, 0.07793],
                        [6.99516, 5.8542, 1.6141, 0.07658],
                        [6.87233, 5.8793, 1.66359, 0.07536],
                        [6.74642, 5.89599, 1.70924, 0.07431],
                        [6.61821, 5.90433, 1.74974, 0.07343],
                        [6.48845, 5.90439, 1.78458, 0.07271],
                        [6.35786, 5.89627, 1.81411, 0.07212],
                        [6.22712, 5.88008, 1.8396, 0.07161],
                        [6.09684, 5.85599, 1.8631, 0.07111],
                        [5.96758, 5.82417, 1.88736, 0.07053],
                        [5.83982, 5.78483, 1.91564, 0.06978],
                        [5.71395, 5.73825, 1.95169, 0.06876],
                        [5.5903, 5.68472, 1.99973, 0.06738],
                        [5.46907, 5.62464, 2.06472, 0.06553],
                        [5.35038, 5.55844, 2.15287, 0.06313],
                        [5.23424, 5.48663, 2.27267, 0.06008],
                        [5.12056, 5.40981, 2.43627, 0.05632],
                        [5.00914, 5.32863, 2.65735, 0.05188],
                        [4.89973, 5.24377, 2.94323, 0.04704],
                        [4.79201, 5.15593, 3.20821, 0.04332],
                        [4.68576, 5.06561, 2.85954, 0.04877],
                        [4.58079, 4.97316, 2.57278, 0.05437],
                        [4.48082, 4.88243, 2.36017, 0.0572],
                        [4.3796, 4.7942, 2.18647, 0.06141],
                        [4.27684, 4.70907, 2.05854, 0.06482],
                        [4.17225, 4.6277, 1.96998, 0.06727],
                        [4.06554, 4.55072, 1.91633, 0.06866],
                        [3.95644, 4.47878, 1.89512, 0.06896],
                        [3.84475, 4.41244, 1.89512, 0.06855],
                        [3.73031, 4.35216, 1.89512, 0.06825],
                        [3.61306, 4.29823, 1.89512, 0.0681],
                        [3.493, 4.25083, 1.90553, 0.06774],
                        [3.37022, 4.20992, 1.94869, 0.06641],
                        [3.24488, 4.17531, 2.02843, 0.06411],
                        [3.11717, 4.14663, 2.15328, 0.06078],
                        [2.98738, 4.12334, 2.35447, 0.056],
                        [2.85585, 4.10467, 2.53881, 0.05233],
                        [2.72282, 4.09006, 2.76056, 0.04848],
                        [2.58852, 4.07896, 2.7395, 0.04919],
                        [2.45315, 4.07089, 2.44718, 0.05541],
                        [2.31676, 4.0658, 2.22553, 0.06133],
                        [2.18425, 4.05787, 2.0075, 0.06612],
                        [2.05357, 4.04652, 1.88164, 0.06971],
                        [1.92516, 4.03106, 1.74079, 0.07429],
                        [1.79951, 4.01084, 1.62282, 0.07842],
                        [1.67722, 3.98511, 1.53782, 0.08126],
                        [1.55874, 3.95349, 1.4423, 0.08503],
                        [1.44467, 3.91541, 1.37925, 0.08719],
                        [1.33565, 3.87043, 1.33673, 0.08823],
                        [1.23223, 3.8183, 1.31105, 0.08834],
                        [1.13519, 3.75862, 1.3, 0.08763],
                        [1.0452, 3.69128, 1.3, 0.08646],
                        [0.96286, 3.61637, 1.3, 0.08563],
                        [0.88871, 3.53415, 1.3, 0.08517],
                        [0.82324, 3.44505, 1.30126, 0.08497],
                        [0.76687, 3.34963, 1.31223, 0.08445],
                        [0.71994, 3.24861, 1.33005, 0.08375],
                        [0.68272, 3.14281, 1.35176, 0.08297],
                        [0.65542, 3.03313, 1.37444, 0.08224],
                        [0.63816, 2.92055, 1.39051, 0.08191]]
            
            
            

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
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
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
        STANDARD_TIME = 37
        FASTEST_TIME = 27
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
        STANDARD_TIME = 37  # seconds (time that is easily done by model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
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


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
