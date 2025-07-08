import math
import rospy, sys
import copy
import moveit_commander
import moveit_msgs.msg 
import trajectory_msgs.msg 
import geometry_msgs.msg

if __name__ == "__main__":

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_circle_demo', anonymous=True)

    # robot = moveit_commander.RobotCommander()

    arm = moveit_commander.MoveGroupCommander('arm')

    end_effector_link = arm.get_end_effector_link()
    reference_frame = "base_link"
    arm.set_pose_reference_frame(reference_frame)

    arm.allow_replanning(True)

    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.01)

    arm.set_max_acceleration_scaling_factor(0.8)
    arm.set_max_velocity_scaling_factor(0.8)

    arm.set_named_target('stand')
    arm.go()
    rospy.sleep(1)
    
    joint_positions = [0.0, 0.0, -1.57, 0.0, -1.57, 0.0]

    arm.set_joint_value_target(joint_positions)
    arm.set_start_state_to_current_state()
    arm.go()

    # target_pose = geometry_msgs.msg.Pose()
    # target_pose.orientation.x = 0.0
    # target_pose.orientation.y = 0.707
    # target_pose.orientation.z = 0.0
    # target_pose.orientation.w = 0.707

    # target_pose.position.x = 0.2
    # target_pose.position.y = 0.2
    # target_pose.position.z = 0.3
    
    # arm.set_pose_target(target_pose)
    # arm.go()

    # print(arm.get_current_pose())
    # rospy.sleep(1)

    start_pose = arm.get_current_pose(arm.get_end_effector_link()).pose
    wpose = copy.deepcopy(start_pose)
    waypoints = []
    # waypoints.append(copy.deepcopy(target_pose))

    radius = 0.06
    centerA = wpose.position.y
    centerB = wpose.position.x + radius
    th = 3.14
    while th < 6.28 :
        wpose.position.y = centerA + radius * math.sin(th)
        wpose.position.x = centerB + radius * math.cos(th)
        waypoints.append(copy.deepcopy(wpose))
        print(wpose.position)

        th = th + 0.001
    
    jump_threshold = 0.0
    eef_step = 0.01
    fraction = 0.0
    maxtries = 100
    attempts = 0
    plan = None

    while fraction < 1.0 and attempts < maxtries :
        (plan, fraction) = arm.compute_cartesian_path(waypoints, eef_step, jump_threshold)
        attempts += 1
        if attempts % 10 == 0 :
            rospy.loginfo("Still trying after %d attempts...", attempts)
    
    if fraction == 1:
        rospy.loginfo("Path computed successfully. Moving the arm.")

        arm.execute(plan)
        # rospy.sleep(100)

        rospy.sleep(0.1)

    else :
        rospy.INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries)

    arm.set_named_target("stand")
    arm.go()
    # print(arm.get_current_pose())
    rospy.sleep(0.1)

    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)