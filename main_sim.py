from helpers_sim import *


# ----------------------------------------------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------------------------------------------

if __name__=="__main__": 

    robot = NiryoRobot()
    robot.open_gripper()

    initial_pose = robot.get_target_pose('Initial Pose')         # [380.221, 0.0, 429.030, 0.0, 0.003, 0.0]
    observation_pose = robot.get_target_pose('Observation Pose') # [235.0, 160.0, 200.0, 0.0, 90.0, 35.0]
    approach_pose = robot.get_target_pose('Approach Pick Pose')  # [10.0, 325.0, 120.0, 0.0, 90.0, 90.0]
    red_place_pose = robot.get_target_pose('Red Place Pose')     # [80.0, 325.0, 27.5, 0.0, 90.0, 90.0]
    green_place_pose = robot.get_target_pose('Green Place Pose') # [10.0, 325.0, 27.5, 0.0, 90.0, 90.0]
    blue_place_pose = robot.get_target_pose('Blue Place Pose')   # [-60.0, 325.0, 27.5, 0.0, 90.0, 90.0]

    # You have pose and you want to get the joints, so you use inverse kinematics
    initial_joints = robot.inverse_kinematics(initial_pose)
    print("----------------")
    print(f"Initial joints: {initial_joints}") # joint angles that correcpond to that pose. 

    # To go back, you use the forward kinematics. Use the joint angles to get to the pose
    initial_pose = robot.forward_kinematics(initial_joints)
    print("----------------")
    print(f"Initial Pose: {initial_pose}")

    observation_joints = robot.inverse_kinematics(observation_pose)
    print("----------------")
    print(f"Inverse kinematics of the joint: {observation_joints}")



    robot.put_on_conveyor()
    robot.move_joints(initial_joints)
    robot.run_conveyor()
    approach_pose_1 = robot.inverse_kinematics(approach_pose)

    objects = 6
    
    try: 
        while (objects != 0):

            # find an intermediate pose to pick up the object so that it can slow down before it picks up the block.
            # find a better way to place the block.
            # the blocks should stack on top of each other.

            # Track trajectory: L = [p1 (initial pose), p2 (observation pose), p3 (approach pose), p4 (Object pose), p5 (Approach again), p6 (Approach place), p7 (coloured area), p8 (approach place), p9 (initial pose)]

            red_pos = robot.inverse_kinematics(red_place_pose)
            green_pos = robot.inverse_kinematics(green_place_pose)
            blue_pos = robot.inverse_kinematics(blue_place_pose)
            
            robot.move_joints(observation_joints)

            if (robot.detect_part()):

                print("----------------")
                print(f"Part detected: {robot.detect_part()}")
                print("----------------")
                print("Stopping conveyor")
                robot.stop_conveyor()



                color_pose = robot.get_color_and_pose()
                block_color = color_pose[0]
                block_pos = color_pose[1]
                
                print(f"Position of the block: {block_pos}")

                block_x = block_pos[1]

                print(f"X-position of the block: {block_x}")

                block_z = block_pos[2]

                print(f"Z-position of the block: {block_z}")

                print(f"Approach_pose_1: {approach_pose_1}")
                print(f"Approach pose: {approach_pose}")
                print(f"Approach pose [1]: {approach_pose[1]}")
                print(f"Approach pose [2]: {approach_pose[2]}")

                approach_pose[1] = block_x
                approach_pose[2] = block_z

                approach_pose_joints = robot.inverse_kinematics(approach_pose)

                if (block_color == 'RED'):
                    print("----------------")
                    print(f"Moving to the pick pose: {approach_pose_joints}")
                    robot.move_joints(approach_pose_joints)

                    print("----------------")
                    print("Closing gripper")

                    robot.close_gripper()

                    robot.move_joints(approach_pose_1)

                    robot.move_joints(red_pos)
                    robot.open_gripper()
                    red_place_pose[2] += (block_z - 10)

                    print(f"New position of the red place pose {red_place_pose}")


                elif (block_color == 'GREEN'):

                    print("----------------")
                    print(f"Moving to the pick pose: {approach_pose_joints}")
                    robot.move_joints(approach_pose_joints)

                    print("----------------")
                    print("Closing gripper")
                    robot.close_gripper()

                    robot.move_joints(approach_pose_1)

                    robot.move_joints(green_pos)
                    robot.open_gripper()

                    green_place_pose[2] += (block_z - 10)

                    print(f"New position of the red place pose {green_place_pose}")

                elif (block_color == 'BLUE'):

                    print("----------------")
                    print(f"Moving to the pick pose: {approach_pose_joints}")
                    robot.move_joints(approach_pose_joints)

                    print("----------------")
                    print("Closing gripper")
                    robot.close_gripper()
                    
                    robot.move_joints(approach_pose_1)

                    robot.move_joints(blue_pos)
                    robot.open_gripper()

                    blue_place_pose[2] += (block_z - 10)

                    print(f"New position of the red place pose {blue_place_pose}")
                
                robot.put_on_conveyor()
                objects -=1
                print(f"Objects left: {objects}")
                robot.run_conveyor()

        robot.move_joints(initial_joints)
        print("Done!!")

    except KeyboardInterrupt:
        print("Program stopped")

    except Exception as e:
        print(f"Error: {e}")

    robot.close_connection()
