def reward_function(params):
    # Read input parameters
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    # Calculate edge of track
    edge = 0.5 * track_width
    
    # Give higher reward if the car is closer to center line and vice versa
    distance_reward = max(1e-3, 1 - ((distance_from_center/edge)**0.4))
    
    # Incentivize going fast on straight ways and slower on curves
    steering_angle = params['steering_angle']
    speed = params['speed']
    speed_diff = abs(2.75-speed)
    max_speed_diff = 0.75 #set it carefully in range [0.01,0.3]

    if -10 < steering_angle < 10:
        speed_reward = max(1e-3, 1-((speed_diff/max_speed_diff)**0.4))
    elif steering_angle < -10 or steering_angle > 10:
        if speed < 1.2:
            speed_reward = max(1e-3, 1-((abs(0.8-speed)/0.8)**0.4))
        if speed < 1.8:
            speed_reward = 1.0
        else:
            speed_reward = max(1e-3, 1-((abs(1.5-speed)/0.5)**0.4))
    
    # Off track penalty
    off_track_penalty = 0
    all_wheels_on_track = params['all_wheels_on_track']
    if not all_wheels_on_track:
        off_track_penalty = 0.25
    
    # Reward based on progress
    # TBD
    
    reward = distance_reward*3 + speed_reward*8 - off_track_penalty
    
    return float(reward)
