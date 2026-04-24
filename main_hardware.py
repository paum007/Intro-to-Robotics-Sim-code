from pyniryo import *
import time

# ----------------------------------------------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------------------------------------------

if __name__=="__main__": 

    robot_ip_address = '10.10.10.10'
    workspace_name = "PauFraser2"  # Robot's Workspace Name
    robot = NiryoRobot(robot_ip_address) # Connect to robot

    # Clear collision if detected during a previous movement
    if robot.collision_detected:
        robot.clear_collision_detected()

    robot.calibrate_auto() # Calibrate robot if the robot needs calibration
    robot.update_tool()    # Updating tool
    robot.open_gripper()
    print(robot.get_sounds())
    # print(f"Reboot: {robot.play_sound('reboot.wav')}")
    # print(f"Calibration: {robot.play_sound('calibration.wav')}")
    # print(f"Error: {robot.play_sound('error.wav')}")
    # print(f"Ready: {robot.play_sound('ready.wav')}")
    # print(f"Stop: {robot.play_sound('stop.wav')}")
    # print(f"Connected: {robot.play_sound('connected.wav')}")
    # print(f"Disconnected: {robot.play_sound('disconnected.wav')}")
    # print(f"Warn: {robot.play_sound('warn.wav')}")
    # print(f"Learning: {robot.play_sound('learning_trajectory.wav')}")

    sensor_pin_id = 'DI5'  # Setting variables
    conveyor_id = robot.set_conveyor() # Activating connexion with the Conveyor Belt
    ir_sensor = robot.get_digital_io_state()

    print(ir_sensor)

    # while (not(robot.sensor))

    
    initial_joints = JointsPosition(0, 0, 0, 0, 0, 0)
    observation_joints = JointsPosition(-0.089,0.1267,-0.6885,-0.0858,-1.2901,0.0338)
    approach_joints = JointsPosition(-0.025,-0.1959,-0.6416,-0.0213,-0.7854,-0.0934)
    ready_to_place = JointsPosition(1.4587,0.2191,-0.3704,-0.0551,-1.4282,-0.1226)
    red_joints = JointsPosition(1.1102,-0.4747,-0.6143,-0.3113,-0.5262,-0.1456)
    green_joints = JointsPosition(1.8971,-0.4277,-0.7506,-0.3143,-0.3713,-0.1456)
    blue_joints = JointsPosition(1.4314,-0.3398,-0.734,-0.052,-0.5262,-0.1456)
    going_back = JointsPosition(0.8576,0.1661,-0.9006,0.0154,-0.8453,0.0016)

    initial_pose = PoseObject(0, 0, 0, 0, 0, 0)
    

    robot.run_conveyor(conveyor_id)
    robot.set_arm_max_velocity(100)
    print("Moving")
    robot.move(initial_joints)

    objects = 6

    try:
        while objects!=0:
            
            robot.move(observation_joints)
            robot.open_gripper()

            if robot.digital_read(sensor_pin_id) == PinState.LOW:
                robot.stop_conveyor(conveyor_id)
                print(f"Connected: {robot.play_sound('connected.wav')}")

                target_pose = robot.get_target_pose_from_cam(workspace_name, 0, ObjectShape.ANY, ObjectColor.ANY)
                target_pose_coord = (target_pose[1])
                print(f"Target pose: {target_pose}")
                print(f"0: {target_pose[0]}")
                print(f"1: {target_pose[1]}")
                print(f"2: {target_pose[2]}")
                print(f"3: {target_pose[3]}")

                robot.move(target_pose_coord)
                # target_pos = robot.move_to_object(workspace_name, 0, ObjectShape.ANY, ObjectColor.ANY)
                
                # robot.shift_pose(RobotAxis.X, 0.007, True) # shift forward
                # robot.shift_pose(RobotAxis.Y, 0.018, True) # shift left
                robot.shift_pose(RobotAxis.Z, -0.01, True) # shift down
                
                robot.close_gripper()
                
                color = target_pose[3]

                if (color == ObjectColor.RED):
                    print(f"Colour: {color}")
                    robot.move(ready_to_place)
                    robot.move(red_joints)
                    robot.open_gripper()


                elif (color == ObjectColor.GREEN):
                    print(f"Colour: {color}")
                    robot.move(ready_to_place)
                    robot.move(green_joints)
                    robot.open_gripper()


                elif (color == ObjectColor.BLUE):
                    print(f"Colour: {color}")
                    robot.move(ready_to_place)
                    robot.move(blue_joints)
                    robot.open_gripper()
                
                robot.move(going_back)
                robot.move(observation_joints)
                robot.run_conveyor(conveyor_id)
                
                objects -= 1
                
                print(f"Objects left: {objects}")

        robot.move(initial_joints)
        robot.stop_conveyor(conveyor_id)
        robot.play_sound('booting.wav')

    # except KeyboardInterrupt:
    #     robot.unset_conveyor(conveyor_id) # Deactivating connexion with the Conveyor Belt
    #     robot.close_connection()

    except Exception as e:
        robot.unset_conveyor(conveyor_id) # Deactivating connexion with the Conveyor Belt
        robot.close_connection()
        print(f"Error: {e}")

    # robot.move(initial_joints)
    # robot.move(observation_joints)
    # robot.move(approach_joints)
    # robot.move(ready_to_place)
    # robot.move(red_joints)
    # robot.move(green_joints)
    # robot.move(blue_joints)
    # robot.move(initial_joints)


    