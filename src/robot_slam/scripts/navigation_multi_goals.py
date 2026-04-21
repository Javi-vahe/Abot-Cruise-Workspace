#!/usr/bin/env python
# _*_ coding:utf-8 _*_

import rospy

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Pose2D
from tf_conversions import transformations
from math import pi
from std_msgs.msg import Empty, Float32, Int16
# from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
# from ar_track_alvar_msgs.msg import AlvarMarker

from robot_slam.srv import Adjust, Block

import sys
from imp import reload
reload(sys)
sys.setdefaultencoding('utf-8')
import os
import ascii_art

# import rospkg
# rospack = rospkg.RosPack()
# robot_slam_path = rospack.get_path("robot_slam")
sbd = []
tcd = []
robot_slam_path = "/home/abot/Abot_Cruise_ws/src/robot_slam/voices/"
sbd.append(robot_slam_path + "number1.mp3") # 识别点1-8
sbd.append(robot_slam_path + "number2.mp3")
sbd.append(robot_slam_path + "number3.mp3")
sbd.append(robot_slam_path + "number4.mp3")
sbd.append(robot_slam_path + "number5.mp3")
sbd.append(robot_slam_path + "number6.mp3")
sbd.append(robot_slam_path + "number7.mp3")
sbd.append(robot_slam_path + "number8.mp3")

tcd.append(robot_slam_path + "point1.mp3")  # 停车点1-8
tcd.append(robot_slam_path + "point2.mp3")
tcd.append(robot_slam_path + "point3.mp3")
tcd.append(robot_slam_path + "point4.mp3")
tcd.append(robot_slam_path + "point5.mp3")
tcd.append(robot_slam_path + "point6.mp3")
tcd.append(robot_slam_path + "point7.mp3")
tcd.append(robot_slam_path + "point8.mp3")
zd = robot_slam_path + "ending.mp3" # 终点
print("sbd1 mp3 path: ", sbd[0])
print("zd mp3 path: ", zd)

############################################ Global variables ############################################
id = 255
ids_to_wait_block1 = [1, 2, 3, 4, 7, 8]
ids_to_wait_block2 = [1, 2, 3, 4, 5, 6]
ids_to_wait_block3 = [3, 4, 5, 6, 7, 8]
ids_to_wait_block4 = [1, 2, 5, 6, 7, 8]
ids_to_wait_empty = [0]
ids_to_wait = ids_to_wait_block1
is_moving = False
break_wait = False
enable_identify = True

############################################ Global functions ############################################
def sleep_for_sec(wait_time):
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time <= wait_time:
        None

############################################ Class definition ############################################
class navigation_demo:
    def __init__(self):
        # Publisher
        self.pub_target_pose = rospy.Publisher("/target_pose", Pose2D, queue_size=1)# pub to cmd_vel_relay
        self.pub_cmd_vel_factor = rospy.Publisher("/cmd_vel_factor", Float32, queue_size=1)
        self.pub_req_api = rospy.Publisher("/req_api", Empty, queue_size=1)
        # Subscriber
        self.sub_formula_id = rospy.Subscriber("/formula_id", Int16, self.formula_cb)
        self.ar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_cb)# ar code
        self.object_sub = rospy.Subscriber('/object_position', Point, self.object_cb) # han zi
        self.end_move_base_sub = rospy.Subscriber("/pid_follow_planner/end_move_base", Empty, self.end_cb)
        self.sub_enable_identify = rospy.Subscriber("/enable_identify", Empty, self.enable_identify_cb)
        # Service cli
        self.adjust_service = rospy.ServiceProxy("/adjust", Adjust)
        self.block_service = rospy.ServiceProxy("/block_judge", Block)
        # Action cli
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        # inittial objects
        self.target_pose = Pose2D()

########################################
    def enable_identify_cb(self, msg):
        global enable_identify
        enable_identify = True

    def end_cb(self, msg):
        self.move_base.cancel_goal()
        
    def formula_cb(self, msg):
        global id, is_moving, break_wait, ids_to_wait, enable_identify
        if enable_identify == False:
            return
        if msg.data in ids_to_wait:
            id = msg.data
        else:
            break_wait = True   # 运算结果不对时, break wait
        rospy.logwarn("Fomula result: %d" % msg.data)
        # self.move_base.cancel_goal()

    def ar_cb(self, data):
        global id, is_moving, ids_to_wait, enable_identify
        if enable_identify == False:
            return
        ar_markers = data
        # print(ar_markers.markers[0])
        if (len(ar_markers.markers) == 1):
            ar_marker = ar_markers.markers[0]
            if ar_marker.id != 255:
                id = ar_marker.id
            if is_moving and len(ids_to_wait)!=1 and (id in ids_to_wait):
                self.move_base.cancel_goal()
                is_moving = False
                rospy.logwarn("Success wait for id, cancel goal!!!")

    def object_cb(self, data):
        global id, is_moving, ids_to_wait, enable_identify
        if enable_identify == False:
            return
        point_msg = data
        #rospy.loginfo('z = %d', point_msg.z)
        if (point_msg.z != 255) :
            if(point_msg.z>=100 and point_msg.z<=199):
                id = 1
            elif(point_msg.z>=200 and point_msg.z<=299):
                id = 2
            elif(point_msg.z>=300 and point_msg.z<=399):
                id = 3
            elif(point_msg.z>=400 and point_msg.z<=499):
                id = 4
            elif(point_msg.z>=500 and point_msg.z<=599):
                id = 5
            elif(point_msg.z>=600 and point_msg.z<=699):
                id = 6
            elif(point_msg.z>=700 and point_msg.z<=799):
                id = 7
            elif(point_msg.z>=800 and point_msg.z<=899):
                id = 8
        if is_moving and len(ids_to_wait)!=1 and (id in ids_to_wait):
            self.move_base.cancel_goal()
            is_moving = False
            rospy.logwarn("Success wait for id, cancel goal!!!")

    def set_ids_to_wait(self, block_num):
        global ids_to_wait, ids_to_wait_block1, ids_to_wait_block2, ids_to_wait_block3, ids_to_wait_block4, ids_to_wait_empty, id
        id = 0
        if block_num == 1:
            ids_to_wait = ids_to_wait_block1
        elif block_num == 2:
            ids_to_wait = ids_to_wait_block2
        elif block_num == 3:
            ids_to_wait = ids_to_wait_block3
        elif block_num == 4:
            ids_to_wait = ids_to_wait_block4
        elif block_num == 0:
            ids_to_wait = ids_to_wait_empty
        else:
            rospy.logerr("Failed to set ids_to_wait !!!")

    def wait_for_ids_for_sec(self, wait_time):
        global id, break_wait, ids_to_wait
        if_send_req = False
        if (True):    # 所有的都需要识别算式
            if_send_req = True
        start_time = rospy.Time.now().to_sec()
        # loop until timeout or id equals id to wait for
        while (id not in ids_to_wait) and rospy.Time.now().to_sec() - start_time <= wait_time:
            if if_send_req and rospy.Time.now().to_sec() - start_time >= 1.0:   # 停车0.5秒后再发送调用api请求
                msg_to_pub = Empty()
                self.pub_req_api.publish(msg_to_pub)
                if_send_req = False
                print("wating for llm ...")
            elif break_wait == True:
                break
        break_wait = False

    def _done_cb(self, status, result):
        global is_moving
        is_moving = False
        self.target_pose.x = 0.0
        self.target_pose.y = 0.0
        self.target_pose.theta = 0.0
        self.pub_target_pose.publish(self.target_pose)
        rospy.loginfo("navigation done! status:%d result:%s"%(status, result))

    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")

    def _feedback_cb(self, feedback):
        self.pub_target_pose.publish(self.target_pose)
        #rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)

    def goto(self, p):
        global is_moving
        self.target_pose.x = p[0]
        self.target_pose.y = p[1]
        self.target_pose.theta = p[2]
        rospy.loginfo("[Navi] goto %s"%p)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2])
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        is_moving = True
        result = self.move_base.wait_for_result(rospy.Duration(60))
        if not result:
            self.move_base.cancel_goal()
            is_moving = False
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!"%p)
        return True

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True
    
    def call_adjust_service(self, goal):
        try:
            rospy.wait_for_service("/adjust", rospy.Duration(3))
        except rospy.ROSException as e:
            rospy.logerr("Service wait failed: %s" % e)
            return False
        try:
            goal_x = goal[0]
            goal_y = goal[1]
            goal_yaw = goal[2]
            response = self.adjust_service(goal_x, goal_y, goal_yaw)
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return False
    
    def call_block_service(self):
        try:
            rospy.wait_for_service("/block_judge", rospy.Duration(3))
        except rospy.ROSException as e:
            rospy.logerr("Service /block_judge wait failed: %s" % e)
            return 0
        try:
            block_req = Block()
            response = self.block_service(block_req)
            return response.next_block
        except rospy.ServiceException as e:
            rospy.logerr("Service /block_judge call failed: %s" % e)
            return 0
# class navigation_demo

############################################ main ############################################
if __name__ == "__main__":
    rospy.init_node('navigation_demo',anonymous=True)
    goalList_X_Block_1 = rospy.get_param('~goalList_X_Block_1', '2.0, 2.0')
    goalList_Y_Block_1 = rospy.get_param('~goalList_Y_Block_1', '2.0, 4.0')
    goalList_Yaw_Block_1 = rospy.get_param('~goalList_Yaw_Block_1', '0, 90.0')

    goalList_X_Block_2 = rospy.get_param('~goalList_X_Block_2', '2.0, 2.0')
    goalList_Y_Block_2 = rospy.get_param('~goalList_Y_Block_2', '2.0, 4.0')
    goalList_Yaw_Block_2 = rospy.get_param('~goalList_Yaw_Block_2', '0, 90.0')

    goalList_X_Block_3 = rospy.get_param('~goalList_X_Block_3', '2.0, 2.0')
    goalList_Y_Block_3 = rospy.get_param('~goalList_Y_Block_3', '2.0, 4.0')
    goalList_Yaw_Block_3 = rospy.get_param('~goalList_Yaw_Block_3', '0, 90.0')

    goalList_X_Block_4 = rospy.get_param('~goalList_X_Block_4', '2.0, 2.0')
    goalList_Y_Block_4 = rospy.get_param('~goalList_Y_Block_4', '2.0, 4.0')
    goalList_Yaw_Block_4 = rospy.get_param('~goalList_Yaw_Block_4', '0, 90.0')

    goals_block1 = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalList_X_Block_1.split(","),goalList_Y_Block_1.split(","),goalList_Yaw_Block_1.split(","))]
    goals_block2 = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalList_X_Block_2.split(","),goalList_Y_Block_2.split(","),goalList_Yaw_Block_2.split(","))]
    goals_block3 = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalList_X_Block_3.split(","),goalList_Y_Block_3.split(","),goalList_Yaw_Block_3.split(","))]
    goals_block4 = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalList_X_Block_4.split(","),goalList_Y_Block_4.split(","),goalList_Yaw_Block_4.split(","))]

    identify_goals = [goals_block1[0], goals_block2[0], goals_block3[0], goals_block4[0]]

    print ('Please 1 to continue: ')
    input = input()
    # r = rospy.Rate(1)
    # r.sleep()
    if (str(input) == '1'):
        navi = navigation_demo()
        def navi_goto_tcd():
            global id, ids_to_wait, enable_identify
            if id in ids_to_wait:
                enable_identify = False
                ascii_art.print_num(id)
                if (id & 1) == 0:    # 2 4 6 8
                    num = 2
                else:   # 1 3 5 7
                    num = 1
                if num == 2:
                    temp = id
                else:
                    temp = id+1
                if temp / 2 == 1:
                    goals_block = goals_block1
                elif temp / 2 == 2:
                    goals_block = goals_block2
                elif temp / 2 == 3:
                    goals_block = goals_block3
                elif temp / 2 == 4:
                    goals_block = goals_block4
                voice_id = id-1
                os.system('mplayer %s' % sbd[voice_id])
                navi.set_ids_to_wait(0)
                navi.goto(goals_block[num])  # Destination goals_num
                navi.call_adjust_service(goals_block[num])
                os.system('mplayer %s' % tcd[voice_id])
        
        def navi_goto_block():
            block_num_ = navi.call_block_service()
            rospy.logwarn("next block: %d" % block_num_)
            navi.set_ids_to_wait(block_num_)
            navi.goto(identify_goals[block_num_-1]) # Identification location
            navi.wait_for_ids_for_sec(13.5) # 等待时间
            navi_goto_tcd()

        # start\/ \/ \/

        # Block 1
        navi_goto_block()
        
        # Block 2
        navi_goto_block()
        
        # Block 3
        navi_goto_block()
        
        # Block 4
        navi_goto_block()

        # go to end
        factor = Float32()
        factor.data = 0.8
        navi.pub_cmd_vel_factor.publish(factor)
        navi.set_ids_to_wait(0)
        navi.goto(goals_block4[3])  # End
        factor.data = 1.0
        navi.pub_cmd_vel_factor.publish(factor)
        navi.call_adjust_service(goals_block4[3])
        os.system('mplayer %s' % zd)

        rospy.loginfo("Arrived end point!!!")
    
############################################ End ############################################
    print("********** Program finishied **********")
