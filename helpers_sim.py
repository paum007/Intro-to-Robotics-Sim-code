from robodk import *   # Basic matrix operations
from robodk.robolink import * # API to communicate with RoboDK
import numpy as np
np.set_printoptions(suppress=True, precision=4)
import cv2
import time

class NiryoRobot:
    """
    A class to control the Niryo Ned2 robot in RoboDK, including camera handling,
    gripper operations, and conveyor belt management.
    """
    def __init__(self):
        """Initialize the robot, tools, frames, camera, and color ranges."""
        # Connect to RoboDK
        self.RDK = Robolink()
        self.RDK.setSimulationSpeed(1)  # Real-time simulation
        self.replace_objects()

        # Load robot, tool, and base frame
        self.robot = self.RDK.Item('Niryo Ned2', ITEM_TYPE_ROBOT)
        self.tool = self.RDK.Item('Gripper (Open)', ITEM_TYPE_TOOL)
        self.frame = self.RDK.Item('Niryo Ned2 Base', ITEM_TYPE_FRAME)
        self.robot.setPoseTool(self.tool)
        self.robot.setPoseFrame(self.frame)
        self.tool_pose = self.robot.PoseTool()  # Tool pose relative to the flange

        # Camera and pose attributes
        self.cam_item = self.RDK.Item('My Camera', ITEM_TYPE_CAMERA)
        self.cam_frame = self.RDK.Item('My Camera Frame', ITEM_TYPE_FRAME)
        self.cam_item.setParam('Open', 1)
        self.RDK.ShowRoboDK()

        # Transformation matrices
        self.TCP = self.RDK.Item('Observation Pose', ITEM_TYPE_TARGET).Pose()
        self.T_flange_camera = self.cam_frame.Pose()
        self.T_flange_camera_rot = self.cam_frame.Pose()
        self.T_flange_camera_rot[:3,3] = [0,0,0]
        self.T_robot_flange = self.TCP * invH(self.tool_pose)
        self.T_robot_camera = self.T_robot_flange * self.T_flange_camera

        # Color ranges for detection (in HSV)
        self.COLOR_RANGES = {
            "RED": [(0, 100, 100), (10, 255, 255)],       # Red lower range
            "GREEN": [(40, 50, 50), (80, 255, 255)],      # Green range
            "BLUE": [(100, 50, 50), (140, 255, 255)]      # Blue range
        }


    def replace_objects(self):
        """
        Replace objects on the workspace.
        """
        self.RDK.RunCode('ReplaceObjects', True)


    def close_connection(self):
        """
        Close the camera and release associated resources.
        This function closes all OpenCV windows and stops the RoboDK camera feed.
        """
        cv2.destroyAllWindows()
        self.RDK.Cam2D_Close(self.cam_item)


    def wait(self, duration):
        """
        Pause the program for a specified duration.

        Args:
            duration (float): Duration in seconds to pause the execution.
        """
        time.sleep(duration)


    def open_gripper(self):
        """
        Open the robot's gripper.
        """
        robot_tools = self.robot.Childs()
        for tool in robot_tools:
            if 'Open' in tool.Name():
                opened = tool
            elif 'Close' in tool.Name():
                closed = tool
        opened.setVisible(True)
        closed.setVisible(False)
        opened.DetachAll()


    def close_gripper(self):
        """
        Close the robot's gripper.
        """
        robot_tools = self.robot.Childs()
        for tool in robot_tools:
            if 'Open' in tool.Name():
                opened = tool
            elif 'Close' in tool.Name():
                closed = tool
        opened.setVisible(False)
        closed.setVisible(True)
        opened.AttachClosest()


    def run_conveyor(self):
        """
        Start the conveyor.
        """
        self.RDK.RunCode('RunConveyor', True)


    def stop_conveyor(self):
        """
        Stop the conveyor.
        """
        self.RDK.Item('RunConveyor', ITEM_TYPE_PROGRAM_PYTHON).Stop()


    def put_on_conveyor(self):
        """
        Place objects on the conveyor.
        """
        self.RDK.RunCode('PutOnConveyor', True)


    def detect_part(self):
        """
        Detect parts using an IR sensor.
        
        Returns:
            bool: True if a part is detected, False otherwise.
        """
        TARGET_NAME = 'Sensor'
        LASER_PLANE = [0, -1, 0]  # Normal vector of the detection plane - Laser's orientation
        TOLERANCE_CHECK_MM = 10   # Distance threshold for detection
        PICKABLE_OBJECTS_KEYWORD = 'Part' # Keyword to filter parts
        RECHECK_PERIOD = 0.001    # Re-check frequency in seconds

        # Retrieve the sensor target
        target = self.RDK.Item(TARGET_NAME)
        if not target.Valid():
            raise Exception(f'Target "{TARGET_NAME}" not found.')

        # Get target pose and define detection plane
        target_pose = target.PoseAbs()
        detect_plane_point = target_pose.Pos()
        detect_plane_vector = LASER_PLANE

        # Get all objects containing the keyword
        all_objects = self.RDK.ItemList(ITEM_TYPE_OBJECT, True)
        check_objects = [self.RDK.Item(name) for name in all_objects if PICKABLE_OBJECTS_KEYWORD in name]

        if not check_objects:
            raise Exception(f'No parts found with keyword: {PICKABLE_OBJECTS_KEYWORD}.')

        # Check objects continuously
        # while True:
        for item in check_objects:
            pos = item.PoseAbs().Pos()
            """Check proximity of a point to the detection plane."""
            pos_proj = proj_pt_2_plane(pos, detect_plane_point, detect_plane_vector)
            distance_to_plane = norm(subs3(pos, pos_proj))
            if distance_to_plane < TOLERANCE_CHECK_MM:
                print('Object detected by IR sensor')
                return True
            
            # # Wait before rechecking
            # pause(RECHECK_PERIOD)


    def get_joints(self):
        """
        Get the current joint angles of the robot.

        Returns:
            List[float]: A list of joint angles in degrees.
        """
        joints = self.robot.Joints().list()
        return joints


    def get_pose(self):
        """
        Get the current pose of the robot's end-effector.

        Returns:
            Pose: The pose of the end-effector as [x, y, z, roll, pitch, yaw].
                - x, y, z are expressed in mm.
                - roll, pitch, yaw are expressed in degrees.
        """
        pose = self.robot.Pose()
        return pose


    def get_target_joints(self, target_name):
        """
        Get the joint configuration required to move the robot to a specified target.

        Args:
            target_name (str): The name of the target.

        Returns:
            List[float]: A list of joint angles to reach the target in degrees.

        Raises:
            Exception: If the target with the specified name is not found.
        """
        target = self.RDK.Item(target_name, ITEM_TYPE_TARGET)
        if not target.Valid():
            raise Exception(f'Target "{target_name}" not found.')
        
        joints = target.Joints().list()
        return joints


    def get_target_pose(self, target_name):
        """
        Get the Cartesian pose of a specified target.

        Args:
            target_name (str): The name of the target.

        Returns:
            List[float]: The pose of the target as [x, y, z, roll, pitch, yaw].
                        - x, y, z are expressed in mm.
                        - roll, pitch, yaw are expressed in degrees.
        
        Raises:
            Exception: If the target with the specified name is not found.
        """
        target = self.RDK.Item(target_name, ITEM_TYPE_TARGET)
        if not target.Valid():
            raise Exception(f'Target "{target_name}" not found.')
        
        pose = pose_2_xyzrpw(target.Pose())
        return pose


    def move_joints(self, q):
        """
        Moves the robot to a specified joint configuration along a non-linear path.

        Args:
            q (List[float]): The target joint angles in degrees.
        """
        self.robot.MoveJ(q)


    def move_pose(self, pose):
        """
        Moves the robot end-effector in a straight line to a specified Cartesian pose.

        Args:
            pose (List[float]): The target pose as [x, y, z, roll, pitch, yaw].
                                - x, y, z are expressed in mm.
                                - roll, pitch, yaw are expressed in degrees.
        """
        pose = xyzrpw_2_pose(pose)
        self.robot.MoveL(pose)


    def move_to_home_pose(self):
        """
        Move to a position where the forearm lays on shoulder.
        """
        joints = list(np.rad2deg([0.0, 0.3, -1.3, 0.0, 0.0, 0.0]))
        self.move_joints(joints)
    

    def forward_kinematics(self, joints):
        """
        Compute the forward kinematics to determine the end-effector pose from given joint angles.

        Args:
            joints (List[float]): The joint angles in radians or degrees (depending on the robot configuration).

        Returns:
            Pose: The pose of the end-effector as a 4x4 transformation matrix.
        """
        flange_pose = self.robot.SolveFK(joints)  # Calculate the flange pose
        TCP_pose = flange_pose * self.tool_pose  # Adjust for the tool's pose relative to the flange
        return TCP_pose


    def inverse_kinematics(self, pose):
        """
        Compute the inverse kinematics to determine the joint angles required for a given end-effector pose.

        Args:
            pose (List[float]): The desired pose of the end-effector as [x, y, z, roll, pitch, yaw].
                                - x, y, z are expressed in mm.
                                - roll, pitch, yaw are expressed in degrees.

        Returns:
            List[float]: The joint angles in radians or degrees (depending on the robot configuration).
        """
        TCP_pose = xyzrpw_2_pose(pose)  # Convert pose to 4x4 transformation matrix
        flange_pose = TCP_pose * invH(self.tool_pose)  # Adjust for the flange's pose relative to the tool
        joints = self.robot.SolveIK(flange_pose).list()  # Solve for joint angles
        return joints


    def get_camera_settings_and_intrinsics(self):
        """
        Retrieve camera settings and intrinsic parameters.

        Returns:
            np.array: Intrinsic camera matrix.
        """
        settings = self.cam_item.setParam('Settings')
        settings_list = [setting.split('=') for setting in settings.strip().split(' ')]
        for setting in settings_list:
            key = setting[0].upper()
            val = setting[-1]
            if key == 'FOCAL_LENGTH':
                focal_length = float(val) # in mm
            elif key == 'PIXELSIZE':
                pixel_size = float(val) * 1e-3 # convert µm to mm
            elif key == 'SIZE':
                w, h = map(int, val.split('x'))

        # Calculate intrinsic parameters
        cx = w / 2  # image center x
        cy = h / 2  # image center y
        fx = fy = focal_length / pixel_size  # focal length in pixels

        # Intrinsic matrix K
        K = np.array([[fx, 0, cx],
                      [0, fy, cy],
                      [0,  0,  1]])
        return(K)


    def image_to_camera_coordinates(self, u, v, K, Z_c):
        """
        Convert image coordinates to camera coordinates.

        Args:
            u, v (int): Image coordinates.
            K (np.array): Intrinsic matrix.
            Z_c (float): Depth in camera frame.

        Returns:
            np.array: 3D coordinates in the camera frame.
        """
        # Inverse of the intrinsic matrix K
        K_inv = np.linalg.inv(K)
        
        # Image point in homogeneous coordinates
        image_point = np.array([u, v, 1])
        
        # Back-projection to 3D camera coordinates
        camera_coordinates = K_inv.dot(image_point) * Z_c  # Multiply by Z_c to get the scale
        return camera_coordinates


    def camera_to_robot_coordinates(self, X_c, Y_c, Z_c):
        """
        Convert camera coordinates to robot coordinates.

        Args:
            X_c, Y_c, Z_c (float): Camera frame coordinates.

        Returns:
            np.array: 3D coordinates in the robot frame.
        """
        camera_coordinates = transl(X_c, Y_c, Z_c)
        robot_coordinates = self.T_robot_camera * camera_coordinates
        return robot_coordinates
    

    def detect_colored_circles(self, image):
        """
        Detect colored circles in an image.

        Args:
            image (np.array): Input image.

        Returns:
            list: Detected objects with color, center, and radius.
        """
        # Convert to HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        detected_objects = []
        for color, (lower, upper) in self.COLOR_RANGES.items():
            # Create a binary mask for the color
            mask = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
            # Reduce noise
            mask = cv2.GaussianBlur(mask, (9, 9), 2, 2)
            # Detect circles using Hough Circle Transform
            circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1.2, 30, param1=50, param2=30, minRadius=10, maxRadius=100)
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for circle in circles[0, :]:
                    x, y, radius = circle
                    detected_objects.append((color, (x, y), radius))
        return detected_objects
    

    def get_color_and_pose(self):
        """
        Get the color and pose of a detected object.

        Returns:
            tuple: Detected object's color and pose.
        """
        while self.cam_item.setParam('isOpen') == '1':
            bytes_img = self.RDK.Cam2D_Snapshot('', self.cam_item)
            if isinstance(bytes_img, bytes) and bytes_img != b'':
                nparr = np.frombuffer(bytes_img, np.uint8)
                img_socket = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                detected_objects = self.detect_colored_circles(img_socket)
                if detected_objects:
                    color, center, radius = detected_objects[0]
                    print(f"Color detected: {color}")

                    # Draw circle and center
                    # img = img_socket.copy()
                    # cv2.circle(img, center, radius, (255, 255, 255), 2)
                    # cv2.circle(img, center, 2, (255, 255, 255), 3)
                    # cv2.imshow("Detected Circles", img)
                    # cv2.waitKey(500)
        
                    #----------------------------------
                    # Convert image point to camera coordinates
                    u, v = center # Image point (u, v)
                    Z_c = 251 # Assumed or known depth in the camera frame (Z_c) in mm
                    K = self.get_camera_settings_and_intrinsics()
                    X_c, Y_c, Z_c = self.image_to_camera_coordinates(u, v, K, Z_c)
                    #----------------------------------
                    # Convert camera coordinates to robot base coordinates
                    robot_pose = self.camera_to_robot_coordinates(X_c, Y_c, Z_c)
                    #----------------------------------
                    # Convert camera frame orientation to the TCP (gripper) orientation 
                    robot_pose = robot_pose * invH(self.T_flange_camera_rot)
                    return color, pose_2_xyzrpw(robot_pose)
                else:
                    print('No objects detected')


    def jacobian_matrix(self, q):
        T = np.array(self.robot.SolveFK(q)).T # T = FK(q)
        R = T[0:3, 0:3]
        epsilon=1e-6

        J = np.zeros((6, 6))
        q = np.deg2rad(q)

        for i in range(6):
            q_plus_epsilon = q.copy()
            q_plus_epsilon[i] += epsilon

            q_plus_epsilon = np.rad2deg(q_plus_epsilon).tolist()
            Ti =  np.array(self.robot.SolveFK(q_plus_epsilon)).T # Ti = FK(q_plus_epsilon)

            dT_dq = (Ti - T) / epsilon
            dR_dq = dT_dq[0:3, 0:3]
            S = dR_dq @ np.transpose(R)
            wx, wy, wz = S[2, 1], S[0, 2], S[1, 0]

            J[0:3, i] = dT_dq[0:3, 3]
            J[3, i] = wx
            J[4, i] = wy
            J[5, i] = wz

        return J


    def numerical_inverse_kinematics(self, target_pose):
        q = self.get_joints()
        target_pose = xyzrpw_2_pose(target_pose)
        target_flange_pose = target_pose * invH(self.tool_pose)
        target_R = (np.array(target_flange_pose).T)[:3,:3]

        while True:
            pose = self.robot.SolveFK(q) 
            delta_x = target_flange_pose.Pos()[0] - pose.Pos()[0]
            delta_y = target_flange_pose.Pos()[1] - pose.Pos()[1]
            delta_z = target_flange_pose.Pos()[2] - pose.Pos()[2]
            
            R = (np.array(pose).T)[:3,:3]

            # Find the Rotation Matrix Difference
            dR = target_R @ R.T
            
            # Compute the Skew-Symmetric Matrix
            S = (dR - dR.T)/2 
            omega = np.array([S[2, 1], S[0, 2], S[1, 0]])
            wx, wy, wz = omega[0], omega[1], omega[2]

            # Using quaternion: https://gamedev.stackexchange.com/questions/189950/calculate-angular-velocity-from-rotation-matrix-difference 
            # r = ROT.from_matrix(dR).as_quat()
            # axis = r[:3]/ np.linalg.norm(r[:3])
            # angle = 2 * np.arccos(r[3])
            # omega = axis * angle
            # wx, wy, wz = omega[0], omega[1], omega[2]
            
            delta = np.vstack([delta_x, delta_y, delta_z, wx, wy, wz])
            delta = delta/2

            J = self.jacobian_matrix(q)
            if np.linalg.matrix_rank(J) == 6: 
                dq = np.linalg.inv(J) @ delta
                dq = dq.reshape(-1).tolist()

                q = [x + y for x, y in zip(q, np.rad2deg(dq))]
                self.move_joints(q)
            else: 
                print("Robot singularity occurred!!")
            
            if np.linalg.norm(delta) < 1e-4:
                break


    def move_with_jacobian(self, dist_to_move=100, V=np.array([50,0,0,0,0,0])):
        delta_t = 0.1
        moved_dist = 0
        q = self.get_joints()
        
        while moved_dist < dist_to_move:
            J = self.jacobian_matrix(q)
            q_dot = np.linalg.inv(J).dot(V)
            q_new = [x + y*delta_t for x, y in zip(q, np.rad2deg(q_dot))]
            self.move_joints(q_new)

            moved_dist += V[0] * delta_t
            q = q_new
    