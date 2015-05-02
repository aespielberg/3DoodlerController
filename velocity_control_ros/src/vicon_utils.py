## Routines to help with vicon

import roslib
from geometry_msgs.msg import PoseStamped,Pose,PointStamped,Vector3Stamped,Point
from mit_msgs.msg import MocapPositionArray
from euclid import *
import rospy
import math
import sys


all_subjects = {}
watch_list = []
origin = None
want_inv = None
my_name = None
my_frame = None


class SubjectException(Exception):
    pass


def normalize(angle):
    while angle >= math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def get_subject(subject):
    while not rospy.is_shutdown():
        try:
            mp = all_subjects[subject]
            if mp.translational.x != 0.0 or mp.translational.y != 0.0 or mp.translational.z != 0.0:
                return mp
        except KeyError:
            pass
        rospy.sleep(0.01)

    sys.exit(0) # Don't return None; if ROS is finished, just help it along ...


def get_pose(vicon_pose):
    mag = math.sqrt(vicon_pose.axisangle.x * vicon_pose.axisangle.x +
                    vicon_pose.axisangle.y * vicon_pose.axisangle.y +
                    vicon_pose.axisangle.z * vicon_pose.axisangle.z
                   )
    axis = Vector3(vicon_pose.axisangle.x,
                   vicon_pose.axisangle.y,
                   vicon_pose.axisangle.z)
    pose = Pose3().new_pose(Point3(0.001 * vicon_pose.translational.x,
                                   0.001 * vicon_pose.translational.y,
                                   0.001 * vicon_pose.translational.z),
                            Quaternion.new_rotate_axis(mag, axis)
                           )
    return pose


def axisangle_to_quaternion(aa):
    axis = Vector3()
    axis.x = aa.x
    axis.y = aa.y
    axis.z = aa.z
    angle = math.sqrt(aa.x * aa.x + aa.y * aa.y + aa.z * aa.z)
    return Quaternion.new_rotate_axis(angle, axis)


def geometry_pose_to_pose(pose):
    if isinstance(pose,PoseStamped):
        pose = pose.pose
    pt = Point3()
    pt.x = pose.position.x
    pt.y = pose.position.y
    pt.z = pose.position.z
    rot = Quaternion()
    rot.x = pose.orientation.x
    rot.y = pose.orientation.y
    rot.z = pose.orientation.z
    rot.w = pose.orientation.w
    new_pose = Pose3()
    return new_pose.new_pose(pt, rot)


def geometry_point_to_point(point):
    pt = Point3()
    pt.x = point.x
    pt.y = point.y
    pt.z = point.z
    return pt


def geometry_vector_to_vector(vector):
    pt = Vector3()
    pt.x = vector.x
    pt.y = vector.y
    pt.z = vector.z
    return pt


def pose_to_geometry_pose(pose):
    ps = Pose()
    ps.position.x = pose.trans.x
    ps.position.y = pose.trans.y
    ps.position.z = pose.trans.z
    ps.orientation.x = pose.rot.x
    ps.orientation.y = pose.rot.y
    ps.orientation.z = pose.rot.z
    ps.orientation.w = pose.rot.w
    return ps


def point_to_geometry_point(point):
    ps = Point()
    ps.x = point.x
    ps.y = point.y
    ps.z = point.z
    return ps


def vector_to_geometry_vector(vector):
    ps = geometry_msgs.msg.Vector3()
    ps.x = vector.x
    ps.y = vector.y
    ps.z = vector.z
    return ps


def empty_geometry_pose(frame):
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.orientation.w = 1.0
    return ps


def vicon_to_geometry_pose(frame):
    while frame[0] == '/':
        frame = frame[1:]
    part = get_subject(frame)

    pt = Point3()
    pt.x = 0.001 * part.translational.x
    pt.y = 0.001 * part.translational.y
    pt.z = 0.001 * part.translational.z
    rot = axisangle_to_quaternion(part.axisangle)
    ps = Pose()
    ps.position.x = pt.x
    ps.position.y = pt.y
    ps.position.z = pt.z
    ps.orientation.x = rot.x
    ps.orientation.y = rot.y
    ps.orientation.z = rot.z
    ps.orientation.w = rot.w
    return ps


def get_subject_transform(frame):
    while frame[0] == '/':
        frame = frame[1:]
    part = get_subject(frame)
    pt = Point3()
    pt.x = 0.001 * part.translational.x
    pt.y = 0.001 * part.translational.y
    pt.z = 0.001 * part.translational.z
    rot = axisangle_to_quaternion(part.axisangle)
    pose = Pose3()
    return pose.new_pose(pt, rot).get_matrix()


def _subject_name_ok(name):
    # NOTE: this could fail transliently
    return 'map' == name or name in all_subjects


#TODO: should be public so I made a version with the "public" name.
def subject_name_ok(name):
    # NOTE: this could fail transliently
    return 'map' == name or name in all_subjects


def transform_by_subjects(ps, to_frame):
    '''Return a transformed PoseStamped'''
    while '/' == to_frame[0]:
        to_frame = to_frame[1:]
    while '/' == ps.header.frame_id[0]:
        ps.header.frame_id = ps.header.frame_id[1:]
    if (to_frame == ps.header.frame_id):
        return ps

    while (not _subject_name_ok(to_frame) or not _subject_name_ok(ps.header.frame_id)) and not rospy.is_shutdown():
        print "missing subject %s %s" % (to_frame if to_frame not in all_subjects else "", ps.header.frame_id if ps.header.frame_id not in all_subjects else "")
        rospy.sleep(0.01)

    if isinstance(ps, PoseStamped):
        coord = geometry_pose_to_pose(ps.pose)
    elif isinstance(ps, PointStamped):
        coord = geometry_point_to_point(ps.point)  
    elif isinstance(ps, Vector3Stamped):
        coord = geometry_point_to_point(ps.vector)
    else:
        return None

    if to_frame == 'map':
        # Destination frame is in global coordinates
        forward = get_subject_transform(ps.header.frame_id)
        combined = forward * coord
    elif ps.header.frame_id == 'map':
        # Source frame is in global coordinates
        backward = get_subject_transform(to_frame)
        combined = backward.inverse() * coord
    else:
        # General transform between two objects
        backward = get_subject_transform(to_frame)
        forward = get_subject_transform(ps.header.frame_id)
        combined = backward.inverse() * forward * coord

    if isinstance(ps, PoseStamped):
        ps_out = PoseStamped()
        ps_out.pose = pose_to_geometry_pose(combined)
    elif isinstance(ps, PointStamped):
        ps_out = PointStamped()
        ps_out.point = point_to_geometry_point(combined)
    elif isinstance(ps, Vector3Stamped):
        ps_out = Vector3Stamped()
        ps_out.vector = vector_to_geometry_vector(combined)

    ps_out.header.stamp = ps.header.stamp
    ps_out.header.frame_id = to_frame

    return ps_out


def handle_subjects(msg):
    global all_subjects
    for part in msg.mocap_position:
        if part.translational.x == 0.0 and part.translational.y == 0.0 and part.translational.z == 0.0 and part.axisangle.x == 0.0 and part.axisangle.y == 0.0 and part.axisangle.z == 0.0:
            if part.name in all_subjects:
                del all_subjects[part.name]
        else:
            all_subjects[part.name] = part


vicon_sub = rospy.Subscriber('/all_subjects', MocapPositionArray,
                             handle_subjects)
