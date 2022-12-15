import numpy as np

# ROS stuff
import rospy
from schunk_sdh_ros.msg import ContactInfoArray
from schunk_sdh.msg import TactileSensor
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray, Float64, String
from controller_manager_msgs.srv import SwitchController


class SDHInterfaceSim(object):
    def __init__(self):
        self.tactile_data = None
        self.contact_info = None
        self.info_joint_state = None
        self.joint_state = None
        self.joint_names = None
        self.mode = "position"

        rospy.Subscriber("/dsa_controller/tactile_data", TactileSensor, self.tactileSensorCB)
        rospy.Subscriber("/dsa_controller/contact_info", ContactInfoArray, self.contactInfoCB)
        rospy.Subscriber("/joint_states", JointState, self.jointStateCB)
        rospy.Subscriber("sdhi_sim/stop", String, self.stopCB)

        self.cmd_joint_state_pub = rospy.Publisher(
            "/pos_based_pos_traj_controller/follow_joint_trajectory/goal",
            FollowJointTrajectoryActionGoal,
            queue_size=1,
            latch=True,
        )
        self.cmd_joint_vel_pub = rospy.Publisher(
            "/joint_group_vel_controller/command", Float64MultiArray, queue_size=1, latch=True
        )

        rospy.wait_for_service("/controller_manager/switch_controller")
        self.switch_controller = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)

        self.joint_limit_lower = -0.20
        self.joint_limit_upper = 0.8

        while self.joint_names is None:
            rospy.sleep(0.1)

    def tactileSensorCB(self, msg):
        self.tactile_data = msg

    def contactInfoCB(self, msg):
        self.contact_info = msg

    def jointStateCB(self, msg):
        if not self.joint_names and len(msg.name) > 1:
            self.joint_names = msg.name
            rospy.loginfo(self.joint_names)
        elif len(msg.name) > 1:
            self.joint_state = msg

    def stopCB(self, msg):
        if msg.data == "stop":
            self.cmdZeroJointVel()
        else:
            rospy.logwarn("Invalid stop message received")

    def switch_operation_mode(self, mode):
        if mode == "velocity":
            self.turn_velocity_controller_on()
            self.mode = "velocity"
        elif mode == "position":
            self.turn_position_controller_on()
            self.mode = "position"
        else:
            rospy.logwarn("Invalid mode")

    def turn_velocity_controller_on(self):
        try:
            self.switch_controller(["joint_group_vel_controller"], ["pos_based_pos_traj_controller"], 1, True, 0.5)
            rospy.loginfo("Switching to velocity control")
        except:
            rospy.logwarn("Switching failed, was the controller already in velocity control mode?")

    def turn_position_controller_on(self):
        try:
            self.switch_controller(["pos_based_pos_traj_controller"], ["joint_group_vel_controller"], 1, True, 0.5)
            rospy.loginfo("Switching to position control")
        except:
            rospy.logwarn("Switching failed, was the controller already in position control mode?")

    def all_close(self, goal, tolerance=0.01):
        actual = np.asarray(self.joint_state.position)
        goal = np.asarray(goal)
        dif = np.fabs(actual - goal)
        if any(dif > tolerance):
            return False
        return True

    def reorder_joint_pos(self, joint_pos):
        order = [3, 4, 5, 6, 0, 1, 2]
        joint_pos = joint_pos[order]
        return joint_pos

    def cmdJointVel(self, joint_vel):
        """
        To command a velocity to the hand..
        """
        joint_vel = np.asarray(joint_vel)
        joint_vel = np.insert(joint_vel, 5, joint_vel[0])
        vel_msg = Float64MultiArray()
        vel_msg.data = joint_vel
        self.cmd_joint_vel_pub.publish(vel_msg)

    def cmdJointVelAcc(self, joint_vel_acc):
        """
        To command a velocity to the hand..
        """
        joint_vel_acc = np.asarray(joint_vel_acc[:7])
        joint_vel_acc = np.insert(joint_vel_acc, 5, joint_vel_acc[0])
        vel_acc_msg = Float64MultiArray()
        vel_acc_msg.data = joint_vel_acc
        self.cmd_joint_vel_pub.publish(vel_acc_msg)

    def cmdJointState(self, goal, time=0.5):
        """
        To command a joint position to the hand..
        """
        if self.mode != "position":
            self.switch_operation_mode("position")
        joint_pos = np.asarray(goal)
        joint_pos = self.reorder_joint_pos(joint_pos)
        joint_pos = np.insert(joint_pos, 2, joint_pos[4])
        rospy.sleep(0.1)
        rate = rospy.Rate(30)
        point = JointTrajectoryPoint()
        point.positions = joint_pos.tolist()
        point.time_from_start = rospy.Duration(time)

        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = self.joint_names
        traj.points.append(point)

        action_goal = FollowJointTrajectoryActionGoal()
        action_goal.header.stamp = rospy.Time.now()
        action_goal.goal.trajectory = traj
        action_goal.goal.goal_time_tolerance = rospy.Duration(time)
        while not self.all_close(joint_pos):
            self.cmd_joint_state_pub.publish(action_goal)
            rate.sleep()
        self.switch_operation_mode("velocity")

    def cmdGoToStartPos(self):
        joint_pos = ([0.0,]) * 7
        self.cmdJointState(joint_pos)

    def cmdZeroJointVel(self):
        """
        To command zero velocity to the hand..
        """
        joint_vel = ([0.0,]) * 7
        if self.mode != "velocity":
            self.switch_operation_mode("velocity")
        self.cmdJointVel(joint_vel)

    def cmdOpen(self):
        self.cmdZeroJointVel()
        joint_pos = [np.pi/3, -np.pi/4, np.pi/6, -np.pi/4, np.pi/6, -np.pi/4,  np.pi/6]
        self.cmdJointState(joint_pos, time=1.0)

    def process_joint_state(self):
        joint_state = self.joint_state
        joint_state.name = [
            "sdh_knuckle_joint",
            "sdh_thumb_2_joint",
            "sdh_thumb_3_joint",
            "sdh_finger_12_joint",
            "sdh_finger_13_joint",
            "sdh_finger_22_joint",
            "sdh_finger_23_joint",
        ]
        if len(joint_state.position) == 7:
            return joint_state
        else:
            order = [4, 5, 6, 0, 1, 2, 3]
            joint_state = self.joint_state
            joint_state.position = np.delete(joint_state.position, 2)
            joint_state.velocity = np.delete(joint_state.velocity, 2)
            joint_state.position = joint_state.position[order]
            joint_state.velocity = joint_state.velocity[order]
            return joint_state

    def get_info(self):
        self.info_joint_state = self.process_joint_state()
        return self.tactile_data, self.contact_info, self.info_joint_state
