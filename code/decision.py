import numpy as np
import time


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    
    if Rover.samples_found == 6:
        print('RETURNING HOME')
        if abs(Rover.pos[0] - Rover.start_pos[0]) < 20 and abs(Rover.pos[1] - Rover.start_pos[1]) < 20:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
            print('RETURNING HOME!!! BEAM ME UP')
            return Rover
    
    # check if the rover is in a doughnut
    if Rover.steer == 15 or Rover.steer == -15:
        # if the wheel are full over more then 10 seconds, in a doughnut
        if time.time() - Rover.wheel_lock < Rover.max_wheel_lock:
            # initiate doughnut mode
            print("STEERING LOCK DETECTED")
            Rover.mode ='doughnut'
    # Rover is caugth in a doughnut so try to escape it
    if Rover.mode == 'doughnut':
        print("DOUGHNUT EVASION INITIATED")
        if time.time() - Rover.wheel_lock > (Rover.max_wheel_lock+5):
            Rover.mode = 'forward'
            Rover.wheel_lock = time.time()
        else:
            # perform evasion to get out doughnut
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            if Rover.steer > 0:
                Rover.steer = -15
            else:
                Rover.steer = 15
            # if in doghnut mode, exit function after evasion performed
        return Rover
    if Rover.mode == 'stuck':
        print('STUCK!! EVASION STARTED')
        if time.time() - Rover.stuck_time > (Rover.max_stuck + 1):
            Rover.mode = 'forward'
            Rover.stuck_time = time.time()
        else:
            # Perform evasion to get unstuck
            Rover.throttle = 0
            Rover.barke = 0
            Rover.steer = -15
        return Rover
         
           
        
    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if Rover.vel < 0.2 and Rover.throttle != 0:
                # if the velocity is still 0 after throttle, it's stuck
                if time.time() - Rover.stuck_time > Rover.max_stuck:
                #initiate stuck mose after 5 seconds of moving
                    Rover.mode = 'stuck'
                    return Rover
                else:
                    # Reset stuck time
                    Rover.stuck_time = time.time()
               
            if Rover.sample_seen:
                if Rover.picking_up !=0:
                    print ('SUCESSFULLY PICKRD UP SAMPLE')
                    # Reset sample_seen flag
                    Rover.sample_seen = False
                    Rover.sample_timer = time.time()
                    return Rover
               
                if time.time() - Rover.sample_timer > Rover.sample_max_search:
                    print('UNABLE TO FIND SAMPLE IN THE UNIT')
                    Rover.sample_seen = False
                    Rover.sampler_time = time.time()
                    return Rover
                avg_rock_angle = np.mean(Rover.rock_angle * 180/np.pi)
                if -15 < avg_rock_angle < 15:
                    # Only drive staright for sample if it's within 13 deg
                    print('APPROACH SAMPLE HEAD ON')
                    if max(Rover.rock_dist) < 20:
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                        Rover.steer = avg_rock_angle
                    else:
                        # Set throttle at half normal speed during approach
                        Rover.throttle = Rover.throttle_set
                        Rover.steer = avg_rock_angle
                elif -50 < avg_rock_angle < 50:
                    print('ROTATING TO SAMPLE: ', avg_rock_angle)
                    if Rover.vel > 0 and max(Rover.rock_dist) < 50:
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                        Rover.steer = 0
                    else:
                        Rover.throttle = 0
                        Rover.brake = 0
                        Rover.steer = avg_rock_angle/6
                else:
                    # keep logic simple and ignore samples +/- degrees
                    print('LOST SIGHT OF THE SAMPLE')
                    Rover.sample_seen = False
            elif len(Rover.nav_angles) > 50:
                # if mode is forward, navigable looks good
                # and velocity is below max, then throttle
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else:
                    Rover.throttle = 0
                Rover.brake = 0
                # set steering to avearge angle to the range +/-15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            else:
                # if there is lack of navigable terrain then to stop mode
               
                # set mode to stop and hit the brake
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'
             
        # if we're already in stop mode then make diffrent decision
        elif Rover.mode == 'stop':
            # if we're in stop mode but still moving keep brakin
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
              
            # if we're not moving (vel<0.2) then do something else
            elif Rover.vel < 0.2:
                # Rover is stopped with vision data; see if there's a path forward
                if len(Rover.nav_angles) < 100:
                    Rover.throttle = 0
                    Rover.brake = 0
                    # Turn range +/- 15 deg, when stooped the next line will induce 4-whell turning
                    Rover.steer = -15
                else:
                    # set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15,15)
                    Rover.mode = 'forward'
               
         
        # Just to make the rover do something 
        # even if no modifications have been made to the code      
        else:
            Rover.throttle = rover.throttle_set
            Rover.steer = 0
            Rover.brake = 0
            
               
               
          
        # If in a state where want to pickup a rock send pickup command
        if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
            Rover.send_pickup = True
            Rover.sample_seen = False
    
        return Rover

