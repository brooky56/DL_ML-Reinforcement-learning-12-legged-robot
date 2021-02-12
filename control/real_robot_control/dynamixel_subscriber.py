import numpy as np
import time

"""Ros packages"""
import actionlib
import roslib
import rospy

"""Ros Dynamixel packages"""
from dynamixel_workbench_msgs.srv import JointCommand, JointCommandRequest

"""Messages packages"""
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Header
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, \
    FollowJointTrajectoryGoal


class JointSystem:
    def __init__(self, joint_states_name):

        """
        Init stage: initialize params, services, subcribers and publishers

        :param joint_states_name: string name of listening topic for the sytem joints
        :return None
        """

        rospy.loginfo('Initialization of joint system of strirus multi-legged robot')
        self.joint_states_topic_name = joint_states_name

        # Check system workaround and Subcribe for the joint states
        self._check_joint_system(timeout=1.0)
        sub = rospy.Subscriber(self.joint_states_topic_name, JointState, self.joint_states_callback)

        # Init servo motors configurations
        self._init_servo_configurations(650, 180, 0.35)

        # Start the Publisher: publish positions of the joints
        self.dynamixel_servo_pose_publisher = rospy.Publisher('/dynamixel_servo_position', JointState, queue_size=1)

        # Init action clietn and wait service client /joint_command to be running
        self.joint_trajectory_action = actionlib.SimpleActionClient(
            self.joint_states_topic_name + '_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('Searching for Joint Trajectory Action...')
        joint_command_service_name = "/strirus_joint_command"
        self.joint_trajectory_action.wait_for_server(joint_command_service_name)
        rospy.loginfo('Joint Trajectory Action is found...')

        # Connect to the service: create connection
        self.joint_command_service = rospy.ServiceProxy(joint_command_service_name, JointCommand)

    def _check_joint_system(self, timeout):

        """
        Helper function: cheking joint system ready stage, check joints topic listening opportunity

        :param timeout: time period of reactivating waiting for listenning messages in the topic
        :return None
        """

        self.joint_states_msg = None
        rospy.loginfo("Searching for " + self.joint_states_topic_name + " to be listen...")
        while self.joint_states_msg is None and not rospy.is_shutdown():
            try:
                self.joint_states_msg = rospy.wait_for_message(self.joint_states_topic_name, JointState, timeout)
                rospy.loginfo("Current " + self.joint_states_topic_name + " is ready: ")
            except:
                rospy.logerr(
                    "Current " + self.joint_states_topic_name + " not ready, start researching listening topic, timeout = {0}".format(
                        timeout))

    def _init_servo_configurations(self, max, min, k):

        """
        Initializing servo motors start configurations: set max and min angle value, and specifying joint range coefficient 
        
        :param max, min: int k:double - init params for servo motor
        :return None
        """

        self.max_range = max
        self.min_range = min
        self.midlle_pose = (max + min) / 2.0
        self.half_range = (max - min) / 2.0
        self.joint_range_coeff = k
        self.joint_zero_conf = np.array([0.0, -1.0, 0.0, -1.0, 0.0, -1.0, 0.0, -1.0, 0.0, -1.0, 0.0, -1.0])
        self.joint_mirror_conf = np.array([1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0])

    def get_joint_names(self):

        """
        Helper function: return names of joint states that is active at time

        :param self
        :return: string[] of joint names 
        
        """

        return self.joint_states_msg.name

    def joint_states_callback(self, message):

        """
        Callback function for publishing the resulting position values to the joint states via rosmsg sensor_msg/JointState:
            std_msgs/Header header
              uint32 seq
              time stamp
              string frame_id
            string[] name
            float64[] position
            float64[] velocity
            float64[] effort

        :param message
        :return position 
        """

        if len(message.points) > 0:
            rospy.loginfo('Incomming trajectory points {0}'.format(message.points))
            pose = np.array(message.points[0].positions)
            tmp_pose = self.joint_range_coeff * (pose + self.joint_zero_conf) * self.joint_mirror_conf
            servo_pose = (self.half_range * tmp_pose + self.midlle_pose).astype(int)
        else:
            rospy.loginfo('No incomming message...')

    def move_particular_joint(self, joint_name, position, unit='rad'):

        """
        Command to move a particular joint to goal position

        rossrv dynamixel_workbench_msgs/JointCommand
            string unit
            uint8 id
            float32 goal_position
            bool result

        :param joint_name:
        :param position: position value
        :param units: radians
        :return: None
        """

        # Crearting request command to move joint
        joint_command_request = JointCommandRequest()
        joint_command_request.unit = unit
        joint_command_request.id = joint_name
        joint_command_request.goal_position = position

        rospy.loginfo("START: joint movement")
        # Send through the connection the name of the object
        result = self.joint_command_service(joint_command_request)
        rospy.loginfo("Joint movement status:" + str(result))
        rospy.loginfo("JOINT" + str(joint_name) + " movement FINISHED")

    def move_all_joints(self, position_arr, unit='rad'):

        """
        Command to move joint to goal positions 

        rossrv dynamixel_workbench_msgs/JointCommand
            string unit
            uint8 id
            float32 goal_position
            bool result

        :param position_arr: goal positions array for all joints
        :param units: radians
        :return: None
        """

        rospy.loginfo("START: move all joints")

        # Check that all joints has goal position value 
        number_of_joints = len(self.joint_states_msg.name)

        if len(position_arr) == number_of_joints:

            # Init rostopic publish JointState elements
            goal_joint_pose = JointState()
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = self.joint_states_msg.header.frame_id

            goal_joint_pose.header = header
            goal_joint_pose.name = self.joint_states_msg.name
            goal_joint_pose.position = position_arr

            # Pass values from message
            goal_joint_pose.velocity = self.joint_states_msg.velocity
            goal_joint_pose.effort = self.joint_states_msg.effort

            rospy.loginfo("START: joints movement publish")
            self.dynamixel_servo_pose_publisher.publish(goal_joint_pose)
            rospy.logwarn("FINISHED: joint goal positions publishing finished")
        else:
            rospy.logerr(
                "The position array sended via topic has not correct amount of position values (Numnber of sended elements) =" + str(
                    number_of_joints))

        rospy.loginfo("JOINTS movement FINISHED")


if __name__ == "__main__":
    rospy.init_node('dynamixel_servo_subscriber')
    rospy.spin()
