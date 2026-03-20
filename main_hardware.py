from pyniryo import *

# ----------------------------------------------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------------------------------------------

if __name__=="__main__": 

    robot_ip_address = '10.10.10.10'
    workspace_name = "PauFraser3"  # Robot's Workspace Name
    robot = NiryoRobot(robot_ip_address) # Connect to robot

    # Clear collision if detected during a previous movement
    if robot.collision_detected:
        robot.clear_collision_detected()

    robot.calibrate_auto() # Calibrate robot if the robot needs calibration
    robot.update_tool()    # Updating tool
    robot.open_gripper()

    sensor_pin_id = 'DI5'  # Setting variables
    conveyor_id = robot.set_conveyor() # Activating connexion with the Conveyor Belt
    ir_sensor = robot.get_digital_io_state()

    print(ir_sensor)

    # while (not(robot.sensor))

    
    initial_joints = JointsPosition(0, 0, 0, 0, 0, 0)
    observation_joints = JointsPosition(-0.0692,0.4039,-0.9067,-0.0152,-1.0247,-0.0443)
    approach_joints = JointsPosition(-0.025,-0.1959,-0.6416,-0.0213,-0.7854,-0.0934)
    ready_to_place = JointsPosition(1.4587,0.2191,-0.3704,-0.0551,-1.4282,-0.1226)
    red_joints = JointsPosition(1.1102,-0.4747,-0.6143,-0.3113,-0.5262,-0.1456)
    green_joints = JointsPosition(1.8971,-0.4277,-0.7506,-0.3143,-0.3713,-0.1456)
    blue_joints = JointsPosition(1.4314,-0.3398,-0.734,-0.052,-0.5262,-0.1456)

    robot.run_conveyor(conveyor_id)

    robot.move_joints(initial_joints)

    objects = 6

    try:
        while objects!=0:
            
            robot.move_joints(observation_joints)
            robot.open_gripper()

            if robot.digital_read(sensor_pin_id) == PinState.LOW:

                robot.stop_conveyor(conveyor_id)

                target_pos = robot.move_to_object(workspace_name, 0, ObjectShape.ANY, ObjectColor.ANY)
                
                print(f"0: {target_pos[0]}")
                print(f"1: {target_pos[1]}")
                print(f"2: {target_pos[2]}")
                robot.shift_pose(RobotAxis.X, 0.010, True) # forward
                robot.shift_pose(RobotAxis.Y, 0.010, True) # right/left
                robot.shift_pose(RobotAxis.Z, -0.01, True) # down

                robot.close_gripper()
                
                color = target_pos[2]

                if (color == ObjectColor.RED):
                    print(f"Colour: {color}")
                    robot.move_joints(ready_to_place)
                    robot.move_joints(red_joints)
                    robot.open_gripper()


                elif (color == ObjectColor.GREEN):
                    print(f"Colour: {color}")
                    robot.move_joints(ready_to_place)
                    robot.move_joints(green_joints)
                    robot.open_gripper()


                elif (color == ObjectColor.BLUE):
                    print(f"Colour: {color}")
                    robot.move_joints(ready_to_place)
                    robot.move_joints(blue_joints)
                    robot.open_gripper()
                
                robot.move_joints(observation_joints)
                robot.run_conveyor(conveyor_id)
                
                objects -= 1
                
                print(f"Objects left: {objects}")

        robot.move_joints(initial_joints)
        robot.stop_conveyor(conveyor_id)

    except KeyboardInterrupt:
        robot.stop_conveyor(conveyor_id)
        robot.move_joints(initial_joints)
        robot.unset_conveyor(conveyor_id) # Deactivating connexion with the Conveyor Belt
        robot.close_connection()

    except Exception as e:
        robot.unset_conveyor(conveyor_id) # Deactivating connexion with the Conveyor Belt
        robot.close_connection()
        print(f"Error: {e}")

    # robot.move_joints(initial_joints)
    # robot.move_joints(observation_joints)
    # robot.move_joints(approach_joints)
    # robot.move_joints(ready_to_place)
    # robot.move_joints(red_joints)
    # robot.move_joints(green_joints)
    # robot.move_joints(blue_joints)
    # robot.move_joints(initial_joints)


    