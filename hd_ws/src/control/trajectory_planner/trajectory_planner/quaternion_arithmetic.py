from math import sqrt, sin, cos, asin, acos, pi
from geometry_msgs.msg import Quaternion, Point, Pose


def norm(x):
    if isinstance(x, (float, int)):
        return abs(x)
    if isinstance(x, (tuple, list)):
        return sqrt(sum(s**2 for s in x))
    if isinstance(x, Point):
        return sqrt(x.x**2+x.y**2+x.z**2)
    if isinstance(x, Quaternion):
        return sqrt(x.w**2+x.x**2+x.y**2+x.z**2)


def normalize(axis):
    if isinstance(axis, (Point, Quaternion)):
        axis = (axis.x, axis.y, axis.z)
    n = sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
    if n == 0:
        return axis
    axis = (axis[0]/n, axis[1]/n, axis[2]/n)
    return axis


def quat_normalize(q):
    """normalize for quaternions"""
    n = sqrt(q.x**2+q.y**2+q.z**2+q.w**2)
    if n == 0:
        return q
    q.x /= n
    q.y /= n
    q.z /= n
    q.w /= n
    return q


def quat(axis, angle):
    """calculate the quaternion associated to a rotation of a certain angle around a certain axis"""
    if isinstance(axis, (Point, Quaternion)):
        axis = (axis.x, axis.y, axis.z)
    orientation = Quaternion()
    axis = normalize(axis)
    orientation.w = cos(angle/2)
    orientation.x = axis[0]*sin(angle/2)
    orientation.y = axis[1]*sin(angle/2)
    orientation.z = axis[2]*sin(angle/2)
    return orientation


def reverse_trig(cost, sint):
    """retrieve angle from its cos and sin"""
    if cost >= 0 and sint >= 0:
        angle = asin(sint)
    elif cost >= 0 and sint <= 0:
        angle = asin(sint)
    elif cost <= 0 and sint >= 0:
        angle = acos(cost)
    else:
        angle = -acos(cost)
    return angle


def reverse_quat(q):
    """retrieve axis and angle of rotation from quaternion"""
    q = make_quat(q)
    cost = q.w
    sint = norm(quat_to_point(q))
    if sint != 0:
        axis = [q.x/sint, q.y/sint, q.z/sint]
    else:
        axis = [0.0, 0.0, 0.0]
    angle = 2*reverse_trig(cost, sint)
    return (axis, angle)


def turn_around(q, axis=(1.0, 0.0, 0.0), angle=pi):
    axis = point_image(axis, q)
    r = quat(axis, angle)
    return mul(r, q)


def nb_to_quat(x):
    x = float(x)
    q = Quaternion()
    q.w = x
    q.x = q.y = q.z = 0.0
    return q


def point_to_quat(p):
    q = Quaternion()
    q.w = 0.0
    q.x = p.x
    q.y = p.y
    q.z = p.z
    return q


def quat_to_point(q):
    # only if q.w is 0
    p = Point()
    p.x = q.x
    p.y = q.y
    p.z = q.z
    return p


def list_to_quat(l):
    q = Quaternion()
    q.x = l[0]
    q.y = l[1]
    q.z = l[2]
    if len(l) < 4:
        q.w = 0.0
    else:
        q.w = l[3]


def list_to_point(l):
    p = Point()
    p.x = l[0]
    p.y = l[1]
    p.z = l[2]
    return p


def make_quat(x):
    """convert x to a Quaternion instance"""
    if isinstance(x, Quaternion):
        return x
    if isinstance(x, (float, int)):
        return nb_to_quat(x)
    elif isinstance(x, Point):
        return point_to_quat(x)
    elif isinstance(x, (tuple, list)):
        return list_to_quat(x)
    #rospy.logerror("Invalid expression for making quaternion : " + str(x))
    raise


def make_point(x):
    """convert x to a Point instance"""
    if isinstance(x, Point):
        return x
    if isinstance(x, Quaternion):
        return quat_to_point(x)
    if isinstance(x, (tuple, list)):
        return list_to_point(x)
    raise


def inv(q):
    """quaternion multiplicative inverse (only works if norm of q is 1"""
    q = make_quat(q)
    q_ = Quaternion()
    q_.w = q.w
    q_.x = -q.x
    q_.y = -q.y
    q_.z = -q.z
    return q_


def add(q1, q2):
    """quaternion addition"""
    q1 = make_quat(q1)
    q2 = make_quat(q2)
    ans = Quaternion()
    ans.w = q1.w + q2.w
    ans.x = q1.x + q2.x
    ans.y = q1.y + q2.y
    ans.z = q1.z + q2.z
    return ans


def point_add(p1, p2):
    """point (vector) addition"""
    p1 = make_point(p1)
    p2 = make_point(p2)
    ans = Point()
    ans.x = p1.x + p2.x
    ans.y = p1.y + p2.y
    ans.z = p1.z + p2.z
    return ans


def mul(q1, q2):
    """quaternion multiplication"""
    q1 = make_quat(q1)
    q2 = make_quat(q2)
    ans = Quaternion()
    ans.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z
    ans.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y
    ans.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x
    ans.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w
    return ans


def point_image(point, q):
    """calculate the image of the point under the rotation described by the quaternion q"""
    point = make_point(point)
    p = point_to_quat(point)
    q_ = inv(q)
    p = mul(mul(q, p), q_)
    return quat_to_point(p)


def reverse_pose(pose):
    """return rev_pose such that compose_poses(pose, rev_pose) is the trivial pose"""
    res = Pose()
    res.orientation = inv(pose.orientation)
    res.position = point_image(pose.position, res.orientation)
    res.position.x = -res.position.x
    res.position.y = -res.position.y
    res.position.z = -res.position.z
    return res


def compose_poses(pose1, pose2):
    """pose1 with respect to origin, pose2 with respect to pose1"""
    res = Pose()
    v = point_image(pose2.position, pose1.orientation)
    res.position = point_add(pose1.position, v)
    axis2, angle2 = reverse_quat(pose2.orientation)
    axis2 = point_image(axis2, pose1.orientation)
    q = quat(axis2, angle2)
    res.orientation = mul(q, pose1.orientation)
    res.orientation = quat_normalize(res.orientation)
    return res


def colinear(v1, v2):
    for j in range(len(v1)):
        if v1[j] != 0:
            break
    else:
        return True
    return all(v1[j]*v2[i] == v1[i]*v2[j] for i in range(len(v1)))


def solve_2d(v1, v2):
    pass

def solve_system(v1, v2):
    s = v1[1]*v2[0] - v2[1]*v1[0]
    t = v1[2]*v2[0] - v2[2]*v1[0]
    r = v1[1]*v2[2] - v2[1]*v1[2]
    if s == t == r == 0:
        return [0,0,0]
    if s == 0:
        if t == 0:
            pass
