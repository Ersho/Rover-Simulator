import numpy as np
import math

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.picking_up:
        return Rover
    """
    if Rover.rover_been[0][2] == 2:
        distance = math.sqrt(sum([(a - b) ** 2 for a, b in zip(Rover.pos, Rover.rover_been[0][:2])]))
        if distance < 50:
            Rover.mode = 'turn-around'
    if Rover.rover_been[1][2] == 2:
        distance = math.sqrt(sum([(a - b) ** 2 for a, b in zip(Rover.pos, Rover.rover_been[1][:2])]))
        if distance < 50:
            Rover.mode = 'turn-around'
    if Rover.rover_been[0][2] == 1:
        distance = math.sqrt(sum([(a - b) ** 2 for a, b in zip(Rover.pos, Rover.rover_been[0][:2])]))
        if distance > 60:
            Rover.rover_been[0][2] = 2
    if Rover.rover_been[1][2] == 1:
        distance = math.sqrt(sum([(a - b) ** 2 for a, b in zip(Rover.pos, Rover.rover_been[1][:2])]))
        if distance > 60:
            Rover.rover_been[1][2] = 2
    """
    for i in range(2):
        if Rover.rover_been[i][2] == 1:
            distance = math.sqrt(sum([(a - b) ** 2 for a, b in zip(Rover.pos, Rover.rover_been[i][:2])]))
            if distance > 60:
                Rover.rover_been[i][2] = 2
        if Rover.rover_been[i][2] == 2:
            distance = math.sqrt(sum([(a - b) ** 2 for a, b in zip(Rover.pos, Rover.rover_been[i][:2])]))
            if distance < 60 and Rover.steering == 0:
                Rover.mode = 'turn-around'
    
          
    if len(Rover.rock_angles) > 1:
        if Rover.near_sample:
            Rover.brake = Rover.brake_set * 5
            Rover.send_pickup = True
        elif Rover.vel >=1.0:
            Rover.brake = Rover.brake_set * 5
        elif Rover.vel < 0.8 and Rover.vel>=0.5:
            Rover.brake = 0
            Rover.throttle = 0
        elif Rover.vel < 0.5:
            Rover.brake = 0
            Rover.throttle = Rover.throttle_set
        for i in range(100):
            Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/ np.pi), -15, 15)
            Rover.throttle = 0.5
            if Rover.near_sample:
                Rover.throttle = 0
                Rover.brake = 1
                Rover.send_pickup = True
                break
            return Rover
        if Rover.samples_to_find == 0:
            Rover.mode = 'end'
    elif Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle
                for i in range(2):
                    distance = math.sqrt(sum([(a - b) ** 2 for a, b in zip(Rover.pos, Rover.rover_been[i][:2])]))
                    if distance < 10:
                        Rover.rover_been[i][2] = 1
                                          
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set * 3
                    
                else: # Else coast
                    Rover.throttle = Rover.throttle_set
                    Rover.steering = 0
                #Rover.throttle = Rover.throttle_set
                #Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)

                if Rover.vel > 0.05 and Rover.mode == 'forward':
                    Rover.stuck_time = 0
                elif Rover.total_time > 1 and Rover.vel <= 0.05:
                    if Rover.stuck_time == 0:
                        Rover.stuck_time = Rover.total_time
                    else:
                        Rover.mode = 'steer'
        
            
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
        elif Rover.mode == 'steer':
            if Rover.vel > 0.5:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            else:
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = -15 if np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15) <= 3 else 15
                if len(Rover.nav_angles) >= Rover.stop_forward:
                    Rover.mode = 'forward'
                    
        elif Rover.mode == 'turn-around':
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            else:
                Rover.brake = 0
                if (Rover.yaw < 90 or Rover.yaw > 300) and Rover.steering !=0 :
                    Rover.throttle = Rover.throttle_set * 5
                    Rover.mode = 'forward'
                if Rover.yaw > 260 and Rover.yaw < 310 and Rover.steering == 0:
                    Rover.steering = 1
                elif Rover.yaw < 120 and Rover.yaw > 80 and Rover.steering == 0:
                    Rover.steering = 2

                if Rover.steering == 1:
                    Rover.steer = -15
                elif Rover.steering == 2:
                    Rover.steer = 15

        elif Rover.mode == 'end':
            if Rover.vel > 0.5:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            else:
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = 0
                
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

