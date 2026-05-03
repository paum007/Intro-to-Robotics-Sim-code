# imports
from helpers_sim import *
import time


# ----------------------------------------------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------------------------------------------

if __name__=="__main__": 

    robot = NiryoRobot()
    robot.open_gripper()

    initial_pose = robot.get_target_pose('Initial Pose')               # [380.221, 0.0, 429.030, 0.0, 0.003, 0.0]
    observation_pose = robot.get_target_pose('Observation Pose')       # [235.0, 160.0, 200.0, 0.0, 90.0, 35.0]
    approach_pose = robot.get_target_pose('Approach Pick Pose')        # [10.0, 325.0, 120.0, 0.0, 90.0, 90.0]
    approach_place_pose = robot.get_target_pose('Approach Place Pose') # [87.99, -17.63, -24.21, 0.0, -48.15, 2.01]
    red_approach_pose = robot.get_target_pose('Red Approach Pose')     # [76.32, -29.35, -8.39, 0.0, -52.26, 13.11]
    red_place_pose = robot.get_target_pose('Red Place Pose')           # [80.0, 325.0, 27.5, 0.0, 90.0, 90.0]
    green_approach_pose = robot.get_target_pose('Green Approach Pose') # [88.75, -27.41, -11.42, 0.0, -51.16, 0.67]
    green_place_pose = robot.get_target_pose('Green Place Pose')       # [10.0, 325.0, 27.5, 0.0, 90.0, 90.0]
    blue_approach_pose = robot.get_target_pose('Blue Approach Pose')   # [100.32, -28.74, -9.35, 0.0, -51.91, -10.89]
    blue_place_pose = robot.get_target_pose('Blue Place Pose')         # [-60.0, 325.0, 27.5, 0.0, 90.0, 90.0]

    # You have pose and you want to get the joints, so you use inverse kinematics
    initial_joints = robot.inverse_kinematics(initial_pose)
    #print("----------------")
    #print(f"Initial joints: {initial_joints}") # joint angles that correcpond to that pose. 

    # To go back, you use the forward kinematics. Use the joint angles to get to the pose
    initial_pose = robot.forward_kinematics(initial_joints)
    #print("----------------")
    #print(f"Initial Pose: {initial_pose}")

    observation_joints = robot.inverse_kinematics(observation_pose)
    #print("----------------")
    #print(f"Inverse kinematics of the joint: {observation_joints}")

    ''' 
    Doing the initial process: 
    - Putting object on conveyor 
    - Starting conveyor 
    - Moving to the initial joints
    '''
    robot.put_on_conveyor()
    robot.move_joints(initial_joints)
    robot.run_conveyor()
    approach_pose_joints = robot.inverse_kinematics(approach_pose)

    # Setting the positions of the robot 
    approach_place_joints = robot.inverse_kinematics(approach_place_pose)

    objects = 6
    
    try: # try/except clause to make debugging easier
        while (objects != 0): # While loop for the amount of object
            
            # Calcualting the inverse kinematics of the colours
            red_app = robot.inverse_kinematics(red_approach_pose)
            red_pos = robot.inverse_kinematics(red_place_pose)
            green_app = robot.inverse_kinematics(green_approach_pose)
            green_pos = robot.inverse_kinematics(green_place_pose)
            blue_app = robot.inverse_kinematics(blue_approach_pose)
            blue_pos = robot.inverse_kinematics(blue_place_pose)
            
            robot.move_joints(observation_joints)

            if (robot.detect_part()): # checking if there's a part in front of the IR sensor
                
                # prints for debugging
                print("----------------")
                print("Part detected: true")
                print("----------------")
                print("Stopping conveyor")
                robot.stop_conveyor() # stop the conveyor once the part is detected

                # getting the colour and pose of the block and splitting the tuple into its own variables
                color_pose = robot.get_color_and_pose()
                block_color = color_pose[0] # type:ignore
                block_pos = color_pose[1]  # type:ignore
                
                print(f"Position of the block: {block_pos}")
                
                # splitting the position tuple into x and y
                block_x = block_pos[1] 
                block_z = block_pos[2]

                print(f"X-position of the block: {block_x}")
                print(f"Z-position of the block: {block_z}")

                print(f"Approach pose joints: {approach_pose_joints}")
                print(f"Approach pose: {approach_pose}")
                print(f"Approach pose [1]: {approach_pose[1]}")
                print(f"Approach pose [2]: {approach_pose[2]}")

                # setting the x and z coordinates of the block to update the approach pose matrix so that the block is picked up at the right place
                approach_pose[1] = block_x 
                approach_pose[2] = block_z

                approach_pose_new = robot.inverse_kinematics(approach_pose) # updatng the inerse kinematics of the approach pose with those new heights

                # ---- MAIN LOGIC ---- # 

    # TODO:The blocks still go through each other. Make it so that the approach is further in and test it again with the time import to see if it changes.
                # checking for the colour: RED
                if (block_color == 'RED'):
                    print("----------------")
                    print(f"Moving to the pick pose: {approach_pose_new}")
                    robot.move_joints(approach_pose_new) # moving to the approach joints

                    print("----------------")
                    # gripper closes once the gripper is at the block positiion
                    print("Closing gripper")
                    robot.close_gripper()
                    robot.move_joints(approach_place_joints)
                    # moving towards the red place section
                    robot.move_joints(red_app) # intermediate step for the approach to avoid collisions when clacing other blocks
                    robot.move_joints(red_pos)
                    robot.open_gripper()
                    robot.move_joints(red_app) # going back to the intermediate position to avoid collision with the gripper
                    
                    # getting the new heights of the block to account for the block that has already been placed. Result: blocks placed on top of each other 
                    red_approach_pose[2] += (block_z - 10)
                    red_place_pose[2] += (block_z - 10)

                    print(f"New position of the red place pose {red_place_pose}")

                # checking for the colour: GREEN
                elif (block_color == 'GREEN'):

                    print("----------------")
                    print(f"Moving to the pick pose: {approach_pose_new}")
                    robot.move_joints(approach_pose_new) # moving to the approach joints

                    print("----------------")
                    # gripper closes once the gripper is at the block positiion
                    print("Closing gripper")
                    robot.close_gripper()
                    robot.move_joints(approach_place_joints)
                    
                    # moving towards the green place section
                    robot.move_joints(green_app) # intermediate step for the approach to avoid collisions when clacing other blocks
                    robot.move_joints(green_pos)
                    robot.open_gripper()
                    robot.move_joints(green_app) # going back to the intermediate position to avoid collision with the gripper

                    # getting the new heights of the block to account for the block that has already been placed. Result: blocks placed on top of each other 
                    green_approach_pose[2] += (block_z - 10)
                    green_place_pose[2] += (block_z - 10)

                    print(f"New position of the red place pose {green_place_pose}")

                # checking for the colour: BLUE
                elif (block_color == 'BLUE'):

                    print("----------------")
                    print(f"Moving to the pick pose: {approach_pose_new}")
                    robot.move_joints(approach_pose_new) # moving to the approach joints

                    print("----------------")
                    # gripper closes once the gripper is at the block positiion
                    print("Closing gripper")
                    robot.close_gripper()
                    robot.move_joints(approach_place_joints)
                    
                    # moving towards the green place section
                    robot.move_joints(blue_app) # intermediate step for the approach to avoid collisions when clacing other blocks
                    robot.move_joints(blue_pos)
                    robot.open_gripper()
                    robot.move_joints(blue_app) # going back to the intermediate position to avoid collision with the gripper

                    # getting the new heights of the block to account for the block that has already been placed. Result: blocks placed on top of each other 
                    blue_approach_pose[2] += (block_z - 10)
                    blue_place_pose[2] += (block_z - 10)

                    print(f"New position of the red place pose {blue_place_pose}")
                
                ''' 
                After each block is placed on the section: 
                - Place new block on conveyor
                - Take away one from the object count
                - Print the objects left for debugging
                - Start the conveyor again
                '''
                robot.put_on_conveyor()
                objects -=1
                print(f"Objects left: {objects}")
                robot.run_conveyor()

        robot.move_joints(initial_joints) # moving back to the initial joints once there's no more blocks left
        print("Done!!") # printing done for confirmation

    # Exceptions allowing for more readable debugging
    except KeyboardInterrupt:
        print("Program stopped")

    except Exception as e:
        print(f"Error: {e}")

    robot.close_connection() # closing connection with the robot
