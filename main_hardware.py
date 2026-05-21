# imports
from pyniryo import *
import time

# ----------------------------------------------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------------------------------------------

if __name__=="__main__": 

    robot_ip_address = '10.10.10.10'
    workspace_name = "FCRPMM FINAL"  # Robot's Workspace Name
    robot = NiryoRobot(robot_ip_address) # Connect to robot

    # Clear collision if detected during a previous movement
    if robot.collision_detected:
        robot.clear_collision_detected()

    robot.calibrate_auto() # Calibrate robot if the robot needs calibration
    robot.update_tool()    # Updating tool
    robot.open_gripper()
    print(robot.get_sounds())

    sensor_pin_id = 'DI5'  # digital input pin the IR sensor is wired to
    conveyor_id = robot.set_conveyor() # Activating connexion with the Conveyor Belt
    ir_sensor = robot.get_digital_io_state()

    print(ir_sensor) # printing the IO state for debugging

    # while (not(robot.sensor))

    
    # Saved joint positions for each stage of the pick-and-place routine
    initial_joints = JointsPosition(0, 0, 0, 0, 0, 0)                          # home/rest position
    observation_joints = JointsPosition(0.0297,0.4585,-0.9309,0.0507,-1.241,-0.0505)  # looking at the conveyor with the camera
    approach_joints = JointsPosition(-0.025,-0.1959,-0.6416,-0.0213,-0.7854,-0.0934)   # close to the block before picking
    ready_to_place = JointsPosition(1.4587,0.2191,-0.3704,-0.0551,-1.4282,-0.1226)     # intermediate pose to avoid collisions
    red_joints = JointsPosition(1.1102,-0.4747,-0.6143,-0.3113,-0.5262,-0.1456)        # drop position for red blocks
    green_joints = JointsPosition(1.8971,-0.4277,-0.7506,-0.3143,-0.3713,-0.1456)      # drop position for green blocks
    blue_joints = JointsPosition(1.4314,-0.3398,-0.734,-0.052,-0.5262,-0.1456)         # drop position for blue blocks
    going_back = JointsPosition(0.8576,0.1661,-0.9006,0.0154,-0.8453,0.0016)           # safe pose for the return path

    initial_pose = PoseObject(0, 0, 0, 0, 0, 0)

    # Doing the initial process: start the conveyor, set the speed and move to the home position
    robot.run_conveyor(conveyor_id)
    robot.set_arm_max_velocity(100) # capping the arm speed at 100%
    robot.led_ring_solid([255, 255, 255])
    print("Moving")
    robot.move(initial_joints)

    objects = 6 # amount of objects to sort

    try: # try/except clause to make debugging easier
        while objects!=0: # While loop for the amount of objects

            robot.move(observation_joints) # move to the camera observation pose
            robot.open_gripper()

            if robot.digital_read(sensor_pin_id) == PinState.LOW: # IR sensor reads LOW when a part blocks the beam
                robot.stop_conveyor(conveyor_id) # stop the conveyor once the part is detected
                print(f"Connected: {robot.play_sound('connected.wav')}") # play a sound for feedback

                # ask the camera for the block's pose and split the result for debugging
                target_pose = robot.get_target_pose_from_cam(workspace_name, 0, ObjectShape.ANY, ObjectColor.ANY)
                target_pose_coord = (target_pose[1]) # the actual pose to move to
                print(f"Target pose: {target_pose}")
                print(f"0: {target_pose[0]}")
                print(f"1: {target_pose[1]}")
                print(f"2: {target_pose[2]}")
                print(f"3: {target_pose[3]}")

                robot.move(target_pose_coord) # moving to the block

                robot.shift_pose(RobotAxis.Z, -0.01, True) # shift down so the gripper reaches the block

                robot.close_gripper() # gripper closes once it is at the block position

                color = target_pose[3] # the colour the camera detected

                # checking for the colour: RED
                if (color == ObjectColor.RED):
                    print(f"Colour: {color}")
                    robot.led_ring_solid([255, 0, 0]) # light up the LED ring to show the detected colour
                    time.sleep(1)
                    robot.led_ring_solid([255, 255, 255])
                    robot.move(ready_to_place) # intermediate step to avoid collisions
                    robot.move(red_joints) # moving towards the red place section
                    robot.open_gripper() # release the block


                # checking for the colour: GREEN
                elif (color == ObjectColor.GREEN):
                    print(f"Colour: {color}")
                    robot.led_ring_solid([0, 255, 0]) # light up the LED ring to show the detected colour
                    time.sleep(1)
                    robot.led_ring_solid([255, 255, 255])
                    robot.move(ready_to_place) # intermediate step to avoid collisions
                    robot.move(green_joints) # moving towards the green place section
                    robot.open_gripper() # release the block


                # checking for the colour: BLUE
                elif (color == ObjectColor.BLUE):
                    print(f"Colour: {color}")
                    robot.led_ring_solid([0, 0, 255]) # light up the LED ring to show the detected colour
                    time.sleep(1)
                    robot.led_ring_solid([255, 255, 255])
                    robot.move(ready_to_place) # intermediate step to avoid collisions
                    robot.move(blue_joints) # moving towards the blue place section
                    robot.open_gripper() # release the block

                '''
                After each block is placed on the section:
                - Move back through a safe pose to the observation pose
                - Start the conveyor again
                - Take away one from the object count
                - Print the objects left for debugging
                '''
                robot.move(going_back) # safe pose to avoid collision on the way back
                robot.move(observation_joints)
                robot.run_conveyor(conveyor_id)

                objects -= 1

                print(f"Objects left: {objects}")

        robot.move(initial_joints) # moving back to the initial joints once there are no more blocks left
        robot.stop_conveyor(conveyor_id) # stopping the conveyor
        robot.led_ring_rainbow(5, 2, True) # making the ring light up after the robot has finished placing all the 6 pieces

    except Exception as e:
        robot.unset_conveyor(conveyor_id) # Deactivating connexion with the Conveyor Belt
        robot.close_connection() # closing connection with the robot
        print(f"Error: {e}")


    
