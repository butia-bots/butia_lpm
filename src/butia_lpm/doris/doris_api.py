#!/usr/bin/env python3
import rospy
import os
os.environ["LANGCHAIN_HANDLER"] = "langchain"
import actionlib
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from butia_world_msgs.srv import GetPose, GetKey
import moveit_commander
from geometry_msgs.msg import PoseStamped
import math
import langchain
from langchain.cache import SQLiteCache
langchain.llm_cache = SQLiteCache(database_path=".langchain.db")
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import tf2
from gazebo_msgs.srv import GetModelState
from butia_vision_msgs.msg import Recognitions3D
from std_srvs.srv import Empty
from sensor_msgs.msg import Image

recognitions = Recognitions3D()
img_msg = Image()
rospy.init_node('chatgpt_planner', anonymous=True)

mock_tools = rospy.get_param('~mock_tools')
valid_locations = [k for k,v in rospy.get_param('/butia_world/pose/targets', {}).items()]
valid_objects = [k for k in rospy.get_param('/butia_object_recognition_node/classes', [])]
object_in_hand = None

def get_location_pose(location_name):
    return world_pose_client('target/'+location_name+'/pose').pose

def recognize_objects():
    global recognitions
    #start_recognitions()
    rospy.Rate(5).sleep()
    #stop_recognitions()
    return [description.label for description in recognitions.descriptions]

def recognize_object(object_type):
    global recognitions
    recognitions = Recognitions3D()
    description_object = None
    max_score = 0.0
    start = rospy.Time.now()
    #start_recognitions()
    while description_object == None and (rospy.Time.now() - start < rospy.Duration(5.0)):
        for description in recognitions.descriptions:
            #rospy.loginfo(description)
            label_class = description.label
            description.bbox
            rospy.loginfo(label_class)
            if label_class == object_type and description.score > max_score:
                description_object = description
                max_score = description.score
    #stop_recognitions()
    return description_object

def get_object_pose(object_description):
    ps_object = PoseStamped()
    ps_object.header = object_description.header
    ps_object.pose = object_description.bbox.center
    recognition_to_map = None
    while recognition_to_map == None:
        try:
            recognition_to_map = tf_buffer.lookup_transform("map", ps_object.header.frame_id, rospy.Time())
        except tf2.LookupException or tf2.ExtrapolationException or tf2.ConnectivityException:
            pass
    transformed_ps = tf2_geometry_msgs.do_transform_pose(ps_object, recognition_to_map)
    return transformed_ps

def recognize_object_pose(object_type):
    global recognitions
    recognitions = Recognitions3D()
    ps_object = None
    max_score = 0.0
    start = rospy.Time.now()
    start_recognitions()
    while ps_object == None and (rospy.Time.now() - start < rospy.Duration(5.0)):
        for description in recognitions.descriptions:
            #rospy.loginfo(description)
            label_class = description.label
            bb = description.bbox
            rospy.loginfo(label_class)
            if label_class == object_type and description.score > max_score:
                ps_object = PoseStamped()
                ps_object.header = description.header
                ps_object.pose = bb.center
                max_score = description.score
    stop_recognitions()
    if ps_object is not None:
        recognition_to_map = None
        while recognition_to_map == None:
            try:
                recognition_to_map = tf_buffer.lookup_transform("map", ps_object.header.frame_id, rospy.Time())
            except tf2.LookupException or tf2.ExtrapolationException or tf2.ConnectivityException:
                pass
        transformed_ps = tf2_geometry_msgs.do_transform_pose(ps_object, recognition_to_map)
        return transformed_ps.pose
    else:
        return None

def get_object_pose_gazebo(object_name):
    model_state = model_state_client(object_name, object_name+"::link")
    object_pose = PoseStamped()
    object_pose.header.frame_id = "base_footprint"
    object_pose.pose = model_state.pose
    base_footprint_to_map = tf_buffer.lookup_transform(object_pose.header.frame_id, "map", rospy.Time())
    transformed_pose = tf2_geometry_msgs.do_transform_pose(object_pose, base_footprint_to_map)
    transformed_pose.pose.position.z += 0.75
    transformed_pose.pose.position.x += 0.75
    return model_state.pose

def go_to_location(pose):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose = pose
    move_base_client.send_goal(goal)
    move_base_client.wait_for_result()
    #print(move_base_client.wait_for_result())
    rospy.Rate(1).sleep()

def get_object_manipulation_pose(object_description):
    object_pose = PoseStamped()
    object_pose.header = object_description.header
    object_pose.pose = object_description.bbox.center

    object_to_base = tf_buffer.lookup_transform("base_footprint", object_pose.header.frame_id, rospy.Time())

    transformed_pose = tf2_geometry_msgs.do_transform_pose(object_pose, object_to_base)
    transformed_pose.pose.position.z = 0.0
    transformed_pose.pose.orientation.x = 0.0
    transformed_pose.pose.orientation.y = 0.0
    transformed_pose.pose.orientation.z = 0.0
    transformed_pose.pose.orientation.w = 1.0

    #print(transformed_pose)
    transformed_pose.pose.position.x -= 0.8
    #print(transformed_pose)
    object_to_map = tf_buffer.lookup_transform("map", transformed_pose.header.frame_id, rospy.Time())
    
    transformed_pose = tf2_geometry_msgs.do_transform_pose(transformed_pose, object_to_map)
    return transformed_pose.pose
    

def pick_up_object(object_description, object_pose):

    gripper_group.set_named_target("Open")
    gripper_group.go(wait=True)
    gripper_group.stop()

    arm_group.set_named_target("PrePick")
    arm_group.go(wait=True)
    arm_group.stop()

    
    

    arm_to_map = tf_buffer.lookup_transform(arm_group.get_planning_frame(), object_pose.header.frame_id, rospy.Time())

    transformed_pose = tf2_geometry_msgs.do_transform_pose(object_pose, arm_to_map)
    '''if object_description.bbox.size.z < 0.08:
        transformed_pose.pose.position.z -= object_description.bbox.size.z/2'''
    #print(transformed_pose)
    #print(object_description)

    #planning_scene.add_box('shelf', transformed_pose, size=(0.1, 1.0, 1.0))

    #print('vertical approach')
    current_pose = arm_group.get_current_pose()
    x = current_pose.pose.position.x
    y = current_pose.pose.position.y
    z = transformed_pose.pose.position.z + 0.1
    roll = 0.0
    pitch = 0.0
    yaw = math.atan(y/x)
    arm_group.set_pose_target([x,y,z,roll,pitch,yaw])
    arm_group.go(wait=True)
    arm_group.stop()
    
    horizontal_approach = 0.1
    
    #print('horizontal approach')
    x = transformed_pose.pose.position.x
    y = transformed_pose.pose.position.y
    z = transformed_pose.pose.position.z
    roll = 0.0
    pitch = 0.0
    yaw = math.atan(y/x)
    y -= (horizontal_approach)*math.sin(yaw)
    x -= (horizontal_approach)*math.cos(yaw)
    math.sqrt((x**2)+(y**2))
    #rospy.loginfo(arm_group.get_planning_frame())
    #rospy.loginfo(f"{x} {y} {z} {roll} {pitch} {yaw}")
    arm_group.set_pose_target([x,y,z,roll,pitch,yaw])
    arm_group.go(wait=True)
    arm_group.stop()

    #planning_scene.remove_world_object(name='shelf')

    y += (horizontal_approach)*math.sin(yaw)
    x += (horizontal_approach)*math.cos(yaw)
    arm_group.set_pose_target([x,y,z,roll,pitch,yaw])
    arm_group.go(wait=True)
    arm_group.stop()
    for i in range(10):
        gripper_group.set_named_target("Closed")
        success = gripper_group.go(wait=True)
    #gripper_group.stop()
    lift = 0.1
    z += lift
    arm_group.set_pose_target([x,y,z,roll,pitch,yaw])
    arm_group.go(wait=True)
    arm_group.stop()


    # Move arm back to home position
    arm_group.set_named_target("PrePick")
    arm_group.go(wait=True)
    rospy.Rate(1/5.).sleep()
    arm_group.stop()

    #rospy.Rate(1.0).sleep()

def place_down_object(pose):
    # Set support surface pose in the "map" frame
    support_surface_pose = PoseStamped()
    support_surface_pose.header.frame_id = "map"
    support_surface_pose.pose = pose


    arm_group.set_named_target("PrePick")
    arm_group.go(wait=True)
    arm_group.stop()
    

    arm_to_map = tf_buffer.lookup_transform(arm_group.get_planning_frame(), support_surface_pose.header.frame_id, rospy.Time())

    transformed_pose = tf2_geometry_msgs.do_transform_pose(support_surface_pose, arm_to_map)

    x = transformed_pose.pose.position.x
    y = transformed_pose.pose.position.y
    z = transformed_pose.pose.position.z + 0.1
    roll = 0.0
    pitch = 0.0
    yaw = math.atan(y/x)
    math.sqrt((x**2)+(y**2))
    rospy.loginfo(f"{x} {y} {z}")
    arm_group.set_pose_target([x,y,z,roll,pitch,yaw])
    arm_group.go(wait=True)
    arm_group.stop()
    z -= 0.1
    arm_group.set_pose_target([x,y,z,roll,pitch,yaw])
    arm_group.go(wait=True)
    arm_group.stop()

    # Open gripper to release object
    gripper_group.set_named_target("Open")
    gripper_group.go(wait=True)
    gripper_group.stop()
    z += 0.1
    arm_group.set_pose_target([x,y,z,roll,pitch,yaw])
    arm_group.go(wait=True)
    arm_group.stop()

    # Move arm back to home position
    arm_group.set_named_target("PrePick")
    arm_group.go(wait=True)
    arm_group.stop()

def deliver_object_to_person():
    arm_group.set_named_target("PrePick")
    arm_group.go(wait=True)
    arm_group.stop()

    gripper_group.set_named_target("Open")
    gripper_group.go(wait=True)
    gripper_group.stop()

    arm_group.set_named_target("PrePick")
    arm_group.go(wait=True)
    arm_group.stop()

def on_recognitions(msg):
    global recognitions
    recognitions = msg

def on_camera_img(msg):
    global img_msg
    img_msg = msg

#def describe_camera_img():
#    cv_img = ros_numpy.numpify(img_msg)
#    inputs = blip2_processor(cv_img, "", return_tensors="pt").to("cuda")
#    out = blip2_model.generate(**inputs)
#    return blip2_processor.decode(out[0], skip_special_tokens=True)

def navigate(location):
    global mock_tools
    global valid_locations
    location = location.lower().strip().replace(' ', '_')
    if mock_tools:
        if location not in valid_locations:
            return location + ' is not a predefined location. ' + list_predefined_locations('')
        return "DoRIS is now at " + location
    else:
        if location not in valid_locations:
            return location + ' is not a predefined location. ' + list_predefined_locations('')
        pose = get_location_pose(location_name=location+"_inspect")
        go_to_location(pose)
        return "DoRIS is now at " + location

def pick(object_and_location):
    global mock_tools
    global valid_objects
    global valid_locations
    global object_in_hand
    object, location = object_and_location.split(',')
    object = object.lower().strip().replace(' ', '_')
    location = location.lower().strip().replace(' ', '_')
    if object_in_hand != None:
        return 'DoRIS has a ' + object_in_hand + ' in hand, and must place it somewhere before atempting to pick a ' + object
    if object not in valid_objects:
        return object + ' is not a valid object type. ' + list_object_types('')
    if location not in valid_locations:
        return location + ' is not a predefined location. ' + list_predefined_locations('')
    object_in_hand = object
    if mock_tools:
        return "DoRIS has picked a " + object + " and is now at the " + location
    pose = get_location_pose(location_name=location+'_inspect')
    go_to_location(pose)
    pose = recognize_object_pose(object_type=object)
    if pose is not None:
        go_to_location(get_location_pose(location_name=location+'_manipulation'))
        pick_up_object(pose=pose, object_type=object, support_type=location)
        return 'DoRIS has picked a ' + object + " and is now at the " + location
    else:
        return 'there is no ' + object + ' on the ' + location

def place(object_and_location):
    global mock_tools
    global valid_locations
    global object_in_hand
    object, location = object_and_location.split(',')
    location = location.lower().strip().replace(' ', '_')
    object = object.lower().strip().replace(' ', '_')
    if object != object_in_hand:
        return "DoRIS does not have an " + object + ' in hand, and cannot place the object on the ' + location
    if location not in valid_locations:
        return location + ' is not a predefined location. ' + list_predefined_locations('')
    object_in_hand = None
    if mock_tools:
        return "DoRIS has placed an object on the " + location
    go_to_location(get_location_pose(location_name=location+'_manipulation'))
    place_down_object(get_location_pose(location_name=location+'_support'))
    return "DoRIS has placed an object on the " + location

def inspect(location):
    global mock_tools
    global valid_locations
    location = location.lower().strip().replace(' ', '_')
    if location not in valid_locations:
        return location + ' is not a predefined location. ' + list_predefined_locations('')
    if mock_tools:
        return "there is a bottle at the " + location
    go_to_location(get_location_pose(location_name=location+'_inspect'))
    object_names = recognize_objects()
    return "there are the following objects at the " + location + ": " + ','.join(object_names)

def list_object_types(text):
    global valid_objects
    return "DoRIS can recognize the following object types: " + ','.join(valid_objects)

def list_predefined_locations(text):
    global valid_locations
    return "the following locations are available: " + ','.join(valid_locations)

def get_valid_objects():
    return valid_objects

def get_valid_locations():
    return valid_locations

def python_tool(text):
    global python_agent
    return python_agent.run('control a ros robot in the gazebo simulator to ' + text)

if not mock_tools:
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    arm_group = moveit_commander.MoveGroupCommander('arm', robot_description="doris_arm/robot_description", ns="doris_arm")
    arm_group.set_max_velocity_scaling_factor(0.5)
    rospy.loginfo(arm_group.get_pose_reference_frame())
    gripper_group = moveit_commander.MoveGroupCommander('gripper', robot_description="doris_arm/robot_description", ns="doris_arm")
    gripper_group.set_max_velocity_scaling_factor(0.5)
    planning_scene = moveit_commander.PlanningSceneInterface(ns="doris_arm")

    move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    move_base_client.wait_for_server()
        
    world_pose_client = rospy.ServiceProxy("/butia_world/get_pose", GetPose)
    world_closest_key_client = rospy.ServiceProxy("/butia_world/get_closest_key", GetKey)

    model_state_client = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    recognitions_sub = rospy.Subscriber('/butia_vision/br/object_recognition3d', Recognitions3D, on_recognitions)

    start_recognitions = rospy.ServiceProxy('/butia_vision/br/object_recognition/start', Empty)
    stop_recognitions = rospy.ServiceProxy('/butia_vision/br/object_recognition/stop', Empty)

    image_sub = rospy.Subscriber('/butia_vision/bvb/image_rgb', Image, on_camera_img)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    api_locals = {
        'get_location_pose': get_location_pose,
        'get_object_pose': get_object_pose,
        'get_object_manipulation_pose': get_object_manipulation_pose,
        'go_to_location': go_to_location,
        'recognize_object': recognize_object,
        'recognize_objects': recognize_objects,
        'pick_up_object': pick_up_object,
        'place_down_object': place_down_object,
        'get_valid_objects': get_valid_objects,
        'get_valid_locations': get_valid_locations
    }