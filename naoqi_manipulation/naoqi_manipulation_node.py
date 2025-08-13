import rclpy
from rclpy.node import Node
import qi
import argparse
import sys

from std_srvs.srv import SetBool, Trigger
from naoqi_utilities_msgs.srv import (
    GoToPosture, SetStiffnesses, SetOpenCloseHand, SetBreathing,
    SetSecurityDistance, SetMoveArmsEnabled, PlayAnimation
)

class NaoqiManipulationNode(Node):
    """
    ROS2 Node to manage manipulation functionalities of a NAO robot,
    specifically ALMotion, ALRobotPosture, and ALBehaviorManager.
    """
    def __init__(self, ip, port):
        """
        Initializes the node, NAOqi service clients, and ROS2 services.
        """
        super().__init__('naoqi_manipulation_node')
        self.get_logger().info("Initializing NaoqiManipulationNode...")
        self.posture = "stand" # Assume robot starts in Stand posture
        self.tangential_security_state = True
        self.orthogonal_security_state = True

        # --- NAOqi Session Management ---
        self.session = self._create_qi_session(ip, port)
        if self.session is None:
            # Error is already logged in the helper function
            sys.exit(1)

        # --- NAOqi Service Clients ---
        try:
            self.al_motion = self.session.service("ALMotion")
            self.al_robot_posture = self.session.service("ALRobotPosture")
            self.al_behavior_manager = self.session.service("ALBehaviorManager")
            self.get_logger().info("NAOqi service clients obtained successfully.")
        except Exception as e:
            self.get_logger().error(f"Could not connect to NAOqi services: {e}")
            self.destroy_node()
            sys.exit(1)

        # --- ROS2 Services for ALMotion ---
        self.set_stiffnesses_service = self.create_service(
            SetStiffnesses,
            '~/set_stiffnesses',
            self.set_stiffnesses_callback
        )
        self.toggle_smart_stiffness_service = self.create_service(
            SetBool,
            '~/toggle_smart_stiffness',
            self.toggle_smart_stiffness_callback
        )
        self.set_open_close_hand_service = self.create_service(
            SetOpenCloseHand,
            '~/set_open_close_hand',
            self.set_open_close_hand_callback
        )
        self.toggle_breathing_service = self.create_service(
            SetBreathing,
            '~/toggle_breathing',
            self.toggle_breathing_callback
        )
        self.set_move_arms_enabled_service = self.create_service(
            SetMoveArmsEnabled,
            '~/set_move_arms_enabled',
            self.set_move_arms_enabled_callback
        )

        # --- Security Services ---
        self.set_tangential_security_distance_service = self.create_service(
            SetSecurityDistance,
            '~/set_tangential_security_distance',
            self.set_tangential_security_distance_callback
        )
        self.set_orthogonal_security_distance_service = self.create_service(
            SetSecurityDistance,
            '~/set_orthogonal_security_distance',
            self.set_orthogonal_security_distance_callback
        )
        self.toggle_arms_collision_protection_service = self.create_service(
            SetBool,
            '~/toggle_arms_collision_protection',
            self.toggle_arms_collision_protection_callback
        )
        self.enable_default_security_service = self.create_service(
            Trigger,
            '~/enable_default_security',
            self.enable_default_security_callback
        )

        # --- ROS2 Services for ALRobotPosture ---
        self.go_to_posture_service = self.create_service(
            GoToPosture,
            '~/go_to_posture',
            self.go_to_posture_callback
        )

        # --- ROS2 Services for ALBehaviorManager ---
        self.play_animation_service = self.create_service(
            PlayAnimation,
            '~/play_animation',
            self.play_animation_callback
        )

        self.get_logger().info("Manipulation functionalities node is ready.")

    def set_stiffnesses_callback(self, request, response):
        """
        Callback to set the stiffness for a joint or a group of joints.
        """
        try:
            # Convert ROS2 message array types to standard Python lists for qi
            joint_names_list = list(request.joint_names)
            stiffnesses_list = list(request.stiffnesses)

            self.get_logger().info(f"Request to set stiffness for {joint_names_list} to {stiffnesses_list}.")
            
            try:
                if len(stiffnesses_list) == 1:
                    self.get_logger().info("Setting uniform stiffness for all joints.")
                    self.al_motion.setStiffnesses(joint_names_list, stiffnesses_list[0])
                else:
                    self.get_logger().info("Setting individual stiffness for each joint.")
                    self.al_motion.setStiffnesses(joint_names_list, stiffnesses_list)
                
                response.success = True
                response.message = "Stiffness set successfully."
            except RuntimeError as e:
                # This specific error is sometimes thrown by NAOqi even if the command succeeds.
                # We can ignore it and assume success.
                self.get_logger().warning(f"Ignored NAOqi error during setStiffnesses: {e}")
                response.success = True
                response.message = "Stiffness set, but a non-critical NAOqi error was ignored."
            except Exception as e:
                response.success = False
                response.message = f"Error setting stiffness: {e}"
                self.get_logger().error(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Error processing stiffness request: {e}"
            self.get_logger().error(response.message)
        return response

    def toggle_smart_stiffness_callback(self, request, response):
        """
        Callback to enable or disable smart stiffness.
        """
        try:
            state = 'enabling' if request.data else 'disabling'
            self.get_logger().info(f"Request to {state} smart stiffness.")
            self.al_motion.setSmartStiffnessEnabled(request.data)
            response.success = True
            response.message = f"Smart stiffness {state}d."
        except Exception as e:
            response.success = False
            response.message = f"Error toggling smart stiffness: {e}"
            self.get_logger().error(response.message)
        return response

    def set_open_close_hand_callback(self, request, response):
        """
        Callback to open or close a specific hand.
        """
        try:
            state_str = "open" if request.state else "close"
            self.get_logger().info(f"Request to {state_str} hand: {request.hand}.")
            value = 1.0 if request.state else 0.0
            self.al_motion.setAngles(request.hand, value, 0.2)
            response.success = True
            response.message = f"Hand {request.hand} is now {state_str}."
        except Exception as e:
            response.success = False
            response.message = f"Error setting hand state: {e}"
            self.get_logger().error(response.message)
        return response

    def toggle_breathing_callback(self, request, response):
        """
        Callback to enable or disable breathing movements for a joint group.
        """
        try:
            state = 'enable' if request.enabled else 'disable'
            self.get_logger().info(f"Request to {state} breathing for {request.joint_group}.")
            
            if request.enabled:
                current_posture = self.posture.lower()
                joint_group = request.joint_group
                
                if current_posture == "stand":
                    self.al_motion.setBreathEnabled(joint_group, True)
                elif current_posture in ["sit", "sit-relax", "lying-back", "lying-front"]:
                    # If the robot is sitting, let him move his upper body, but not his legs
                    if joint_group.lower() not in ["body", "legs", "all"]:
                        self.al_motion.setBreathEnabled(joint_group, True)
                    else:
                        self.get_logger().info(f"Breathing for {joint_group} not enabled in {current_posture} posture.")
                        response.success = False
                        response.message = f"Breathing for {joint_group} not enabled in {current_posture} posture."
                        return response
                # If posture is "rest", do nothing as per the original logic
            else: # disable
                self.al_motion.setBreathEnabled(request.joint_group, False)

            response.success = True
            response.message = f"Breathing for {request.joint_group} set to {request.enabled}."
        except Exception as e:
            response.success = False
            response.message = f"Error toggling breathing: {e}"
            self.get_logger().error(response.message)
        return response

    def set_move_arms_enabled_callback(self, request, response):
        """
        Callback to enable or disable arm movements during motion.
        """
        try:
            self.get_logger().info(f"Request to set arm movement: Left={request.left_arm_enabled}, Right={request.right_arm_enabled}.")
            self.al_motion.setMoveArmsEnabled(request.left_arm_enabled, request.right_arm_enabled)
            response.success = True
            response.message = "Arm movement settings updated."
        except Exception as e:
            response.success = False
            response.message = f"Error setting arm movement: {e}"
            self.get_logger().error(response.message)
        return response

    def set_tangential_security_distance_callback(self, request, response):
        """
        Callback to set the tangential security distance.
        """
        try:
            self.get_logger().info(f"Request to set tangential security distance to {request.distance}m.")
            self.al_motion.setTangentialSecurityDistance(request.distance)
            if request.distance < 0.1:
                self.tangential_security_state = False
            else:
                self.tangential_security_state = True
            
            # If both security distances are set to low values, disable external collision protection
            if not (self.tangential_security_state and self.orthogonal_security_state):
                self.al_motion.setExternalCollisionProtectionEnabled("All", False)
                self.get_logger().info("External collision protection disabled due to security distance settings.")
            response.success = True
            response.message = "Tangential security distance updated."
        except Exception as e:
            response.success = False
            response.message = f"Error setting tangential security distance: {e}"
            self.get_logger().error(response.message)
        return response

    def set_orthogonal_security_distance_callback(self, request, response):
        """
        Callback to set the orthogonal security distance.
        """
        try:
            self.get_logger().info(f"Request to set orthogonal security distance to {request.distance}m.")
            self.al_motion.setOrthogonalSecurityDistance(request.distance)
            if request.distance < 0.4:
                self.orthogonal_security_state = False
            else:
                self.orthogonal_security_state = True

            # If both security distances are set to low values, disable external collision protection
            if not (self.tangential_security_state and self.orthogonal_security_state):
                self.al_motion.setExternalCollisionProtectionEnabled("All", False)
                self.get_logger().info("External collision protection disabled due to security distance settings.")
            response.success = True
            response.message = "Orthogonal security distance updated."
        except Exception as e:
            response.success = False
            response.message = f"Error setting orthogonal security distance: {e}"
            self.get_logger().error(response.message)
        return response

    def toggle_arms_collision_protection_callback(self, request, response):
        """
        Callback to enable or disable collision protection for the arms.
        """
        try:
            state = 'enable' if request.data else 'disable'
            self.get_logger().info(f"Request to {state} arms collision protection.")
            self.al_motion.setCollisionProtectionEnabled("Arms", request.data)
            self.al_motion.setExternalCollisionProtectionEnabled("Arms", request.data)
            response.success = True
            response.message = f"Arms collision protection {state}d."
        except Exception as e:
            response.success = False
            response.message = f"Error toggling arms collision protection: {e}"
            self.get_logger().error(response.message)
        return response

    def enable_default_security_callback(self, request, response):
        """
        Callback to enable default security distances and arm collision protection.
        """
        try:
            self.get_logger().info("Request to enable default security settings.")
            self.al_motion.setOrthogonalSecurityDistance(0.4)
            self.al_motion.setTangentialSecurityDistance(0.1)
            self.al_motion.setCollisionProtectionEnabled("Arms", True)
            self.al_motion.setExternalCollisionProtectionEnabled("All", True)
            response.success = True
            response.message = "Default security settings enabled."
        except Exception as e:
            response.success = False
            response.message = f"Error enabling default security: {e}"
            self.get_logger().error(response.message)
        return response

    def go_to_posture_callback(self, request, response):
        """
        Callback to make the robot go to a predefined posture.
        """
        try:
            posture_name = request.posture_name.lower()
            self.get_logger().info(f"Request to go to posture '{posture_name}'.")

            # Common action: disable breathing before changing posture
            self.al_motion.setBreathEnabled("All", False)

            if posture_name == "stand":
                self.al_robot_posture.goToPosture("Stand", 0.5)
                self.al_motion.wakeUp()
            elif posture_name == "rest":
                self.al_robot_posture.goToPosture("Crouch", 0.5)
                self.al_motion.rest()
            elif posture_name == "sit":
                self.al_robot_posture.goToPosture("Sit", 0.5)
            elif posture_name == "sit-relax":
                self.al_robot_posture.goToPosture("SitRelax", 0.5)
            elif posture_name == "lying-back":
                self.al_robot_posture.goToPosture("LyingBack", 0.5)
            elif posture_name == "lying-front":
                self.al_robot_posture.goToPosture("LyingBelly", 0.5)
            else:
                response.success = False
                response.message = f"Unknown posture '{posture_name}'."
                self.get_logger().error(response.message)
                return response

            self.posture = posture_name  # Update current posture
            response.success = True
            response.message = f"Posture '{request.posture_name}' reached."
            self.get_logger().info(f"Robot is in {posture_name} position!")

        except Exception as e:
            response.success = False
            response.message = f"Error going to posture: {e}"
            self.get_logger().error(response.message)
        return response

    def play_animation_callback(self, request, response):
        """
        Callback to run a predefined animation (behavior) from the 'animations' family.
        """
        try:
            animation = request.animation_name
            self.get_logger().info(f"Request to play animation '{animation}'.")

            full_behavior_name = f"animations/{animation}"

            installed_behaviors = self.al_behavior_manager.getInstalledBehaviors()

            if full_behavior_name in installed_behaviors:
                # startBehavior is non-blocking
                self.al_behavior_manager.startBehavior(full_behavior_name)
                response.success = True
                response.message = f"Animation '{full_behavior_name}' started successfully."
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = f"Animation '{full_behavior_name}' does not exist on the robot."
                self.get_logger().error(response.message)

        except Exception as e:
            response.success = False
            response.message = f"Error playing animation: {e}"
            self.get_logger().error(response.message)
        
        return response

    def _create_qi_session(self, ip, port):
        """Helper function to create and connect a qi session."""
        session = qi.Session()
        try:
            self.get_logger().info(f"Attempting to connect to NAOqi at tcp://{ip}:{port}")
            session.connect(f"tcp://{ip}:{port}")
            self.get_logger().info("Successfully connected to NAOqi.")
            return session
        except RuntimeError as e:
            self.get_logger().error(f"Can't connect to Naoqi at ip '{ip}' on port {port}.\n"
                                    f"Error: {e}\nPlease check your script arguments.")
            return None

    def destroy_node(self):
        """Custom destroy function to clean up session."""
        self.get_logger().info("Closing NAOqi session.")
        if self.session and self.session.isConnected():
            self.session.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot IP address. On Robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")
    parsed_args, _ = parser.parse_known_args(args=sys.argv[1:])

    naoqi_manipulation_node = NaoqiManipulationNode(ip=parsed_args.ip, port=parsed_args.port)

    try:
        rclpy.spin(naoqi_manipulation_node)
    except KeyboardInterrupt:
        print("Closing the manipulation functionalities node.")
    finally:
        # The destroy_node method will be called automatically on shutdown
        rclpy.shutdown()

if __name__ == '__main__':
    main()