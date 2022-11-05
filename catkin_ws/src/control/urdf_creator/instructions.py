from arithmetic import *
from utils import *


class Comment:
    LAYOUT = '<!-- %s -->\n'

    def __init__(self, text):
        self.text = text

    def write(self):
        return self.LAYOUT % self.text


class Separation(Comment):
    def __init__(self):
        super().__init__(SEPARATION)


class Blank:
    def __init__(self, nb):
        self.nb_lines = nb

    def write(self):
        return "\n"*self.nb_lines


class Geometry:
    LAYOUT = '<geometry>\n%s</geometry>\n'

    def write(self):
        return self.LAYOUT % ""


class Meshes(Geometry):
    pass


class Cylinder(Geometry):
    def __init__(self, radius, length):
        self.type = "cylinder"
        self.radius = radius
        self.length = length


class Box(Geometry):
    BOX_LAYOUT = '\t<box size="%s"/>\n'

    def __init__(self, xyz):
        self.xyz = xyz

    def write(self):
        if isinstance(self.xyz, (list, tuple)):
            xyz_s = " ".join(map(str, self.xyz))
        elif isinstance(self.xyz, str):
            xyz_s = self.xyz
        elif isinstance(self.xyz, (Constant, Arithmetic)):
            xyz_s = str(self.xyz)
        else:
            raise
        xyz_s = self.BOX_LAYOUT % xyz_s
        return self.LAYOUT % xyz_s


class Origin:
    LAYOUT = "<origin %s />\n"

    def __init__(self, xyz=UNDEFINED, rpy=UNDEFINED):
        self.xyz = xyz
        self.rpy = rpy

    def write(self):
        s = ""
        if self.xyz != UNDEFINED:
            if isinstance(self.xyz, (list, tuple)):
                xyz_s = " ".join(map(str, self.xyz))
            elif isinstance(self.xyz, str):
                xyz_s = self.xyz
            elif isinstance(self.xyz, (Constant, Arithmetic)):
                xyz_s = str(self.xyz)
            else:
                raise
            s += 'xyz="%s"' % xyz_s
        if self.rpy != UNDEFINED:
            if isinstance(self.rpy, (list, tuple)):
                rpy_s = " ".join(map(str, self.xyz))
            elif isinstance(self.rpy, str):
                rpy_s = self.rpy
            elif isinstance(self.rpy, (Constant, Arithmetic)):
                rpy_s = str(self.rpy)
            else:
                raise
            s += ' rpy="%s"' % rpy_s
        if s:
            return self.LAYOUT % s
        return ""


class Material:
    LAYOUT = '<material name="%s">\n%s</material>\n'
    RGBA_LAYOUT = '\t<color rgba="%s"/>\n'

    def __init__(self, name, rgba=UNDEFINED):
        assert(rgba == UNDEFINED or self.is_rgba(rgba))
        self.name = name
        self.rgba = rgba

    def write(self):
        if self.rgba != UNDEFINED:
            if isinstance(self.rgba, str):
                rgba_s = self.rgba
            else:
                rgba_s = " ".join(map(str, self.rgba))
            rgba_s = self.RGBA_LAYOUT % rgba_s
            return self.LAYOUT % (self.name, rgba_s)
        return self.LAYOUT % (self.name, "")

    @staticmethod
    def is_rgba(seq):
        integer01 = lambda x: isinstance(x, (int, float)) and 0 <= x <= 1
        is_in_list_form = (isinstance(seq, (tuple, list)) and len(seq) == 4
                and all(map(integer01, seq)))
        is_in_str_form = (isinstance(seq, str) and len(seq.split()) == 4 and all(map(integer01, seq.split())))
        return is_in_list_form or is_in_str_form


class Inertia:
    LAYOUT = '<inertia ixx="%s" ixy="%s" ixz="%s" iyy="%s" iyz="%s" izz="%s"/>\n'

    def __init__(self, matrix=None, ixx=0, ixy=0, ixz=0, iyy=0, iyz=0, izz=0):
        assert(matrix is None or self.is_symmetrical(matrix))
        if matrix is None:
            self.ixx, self.ixy, self.ixz, self.iyy, self.iyz, self.izz = ixx, ixy, ixz, iyy, iyz, izz
        else:
            self.set_mat(matrix)

    def set_mat(self, matrix):
        assert(self.is_symmetrical(matrix))
        self.ixx = matrix[0][0]
        self.iyy = matrix[1][1]
        self.izz = matrix[2][2]
        self.ixy = matrix[0][1]
        self.ixz = matrix[0][2]
        self.iyz = matrix[1][2]

    def write(self):
        inertia_s = self.LAYOUT % (self.ixx, self.ixy, self.ixz, self.iyy, self.iyz, self.izz)
        return inertia_s

    @staticmethod
    def is_symmetrical(matrix):
        return (isinstance(matrix, (tuple, list)) and isinstance(matrix[0], (tuple, list))
                and len(matrix) == len(matrix[0]) == 3
                and all(matrix[i][j] == matrix[j][i] for i in range(1, len(matrix)) for j in range(i)))


class LinkVisual:
    LAYOUT = '<visual>\n%s%s%s</visual>\n'

    def __init__(self, geometry=UNDEFINED, origin=UNDEFINED, material=UNDEFINED):
        assert((geometry == UNDEFINED or isinstance(geometry, Geometry))
               and (material == UNDEFINED or isinstance(material, Material))
               and (origin == UNDEFINED or isinstance(origin, Origin)))
        self.geometry = geometry
        self.material = material
        self.origin = origin

    def write(self):
        geometry_s = ""
        material_s = ""
        origin_s = ""
        if self.geometry != UNDEFINED:
            geometry_s = indent(self.geometry.write())
        if self.material != UNDEFINED:
            material_s = indent(self.material.write())
        if self.origin != UNDEFINED:
            origin_s = indent(self.origin.write())
        return self.LAYOUT % (geometry_s, material_s, origin_s)


class LinkCollision:
    LAYOUT = '<collision>\n%s%s</collision>\n'

    def __init__(self, geometry=UNDEFINED, origin=UNDEFINED):
        assert((geometry == UNDEFINED or isinstance(geometry, Geometry))
               and (origin == UNDEFINED or isinstance(origin, Origin)))
        self.geometry = geometry
        self.origin = origin

    def write(self):
        geometry_s = ""
        origin_s = ""
        if self.geometry != UNDEFINED:
            geometry_s = indent(self.geometry.write())
        if self.origin != UNDEFINED:
            origin_s = indent(self.origin.write())
        return self.LAYOUT % (geometry_s, origin_s)

    @classmethod
    def from_visual(cls, visual):
        assert isinstance(visual, LinkVisual)
        return cls(visual.geometry, visual.origin)


class LinkInertial:
    LAYOUT = '<inertial>\n%s%s</inertial>\n'
    MASS_LAYOUT = '\t<mass value="%s"/>\n'

    def __init__(self, mass=UNDEFINED, inertia=UNDEFINED, origin=UNDEFINED):
        assert((inertia == UNDEFINED or isinstance(inertia, Inertia))
               and (origin == UNDEFINED or isinstance(origin, Origin)))
        """if inertia == UNDEFINED or isinstance(inertia, Inertia):
            self.inertia = inertia
        else:
            self.inertia = Inertia(inertia)
        if origin == UNDEFINED or isinstance(origin, Origin):
            self.origin = origin
        else:
            self.origin = Origin(origin)"""
        self.mass = mass
        self.inertia = inertia
        self.origin = origin

    def write(self):
        mass_s = ""
        inertia_s = ""
        origin_s = ""
        if self.mass != UNDEFINED:
            mass_s = self.MASS_LAYOUT % str(self.mass)
        if self.inertia != UNDEFINED:
            inertia_s = indent(self.inertia.write())
        if self.origin != UNDEFINED:
            origin_s = indent(self.origin.write())
        return self.LAYOUT % (mass_s, inertia_s, origin_s)

    @staticmethod
    def is_symmetrical(matrix):
        return (isinstance(matrix, (tuple, list)) and isinstance(matrix[0], (tuple, list))
                and len(matrix) == len(matrix[0]) == 3
                and all(matrix[i][j] == matrix[j][i] for i in range(1, len(matrix)) for j in range(i)))


class Link:
    LAYOUT = '<link name="%s">\n%s%s%s</link>'

    def __init__(self, name, visual=UNDEFINED, collision=UNDEFINED, inertial=UNDEFINED):
        assert(isinstance(name, str)
               and (visual == UNDEFINED or isinstance(visual, LinkVisual))
               and (collision == UNDEFINED or collision == NATURAL or isinstance(collision, LinkCollision))
               and (inertial == UNDEFINED or isinstance(inertial, LinkInertial)))
        self.name = name
        self.visual = visual
        if collision == NATURAL:
            collision = LinkCollision.from_visual(visual)
        self.collision = collision
        self.inertial = inertial

    def write(self):
        visual_s = ""
        collision_s = ""
        inertial_s = ""
        if self.visual != UNDEFINED:
            visual_s = indent(self.visual.write())
        if self.collision != UNDEFINED:
            collision_s = indent(self.collision.write())
        if self.inertial != UNDEFINED:
            inertial_s = indent(self.inertial.write())
        return self.LAYOUT % (self.name, visual_s, collision_s, inertial_s)


class Limit:
    LAYOUT = '<limit %s %s %s %s/>\n'
    EFFORT_LAYOUT = 'effort="%s"'
    VELOCITY_LAYOUT = 'velocity="%s"'
    LOWER_LAYOUT = 'lower="%s"'
    UPPER_LAYOUT = 'upper="%s"'

    def __init__(self, effort, velocity, lower=UNDEFINED, upper=UNDEFINED):
        self.effort = effort
        self.velocity = velocity
        self.lower = lower
        self.upper = upper

    def write(self):
        effort_s = ""
        velocity_s = ""
        lower_s = ""
        upper_s = ""
        if self.effort != UNDEFINED:
            effort_s = self.EFFORT_LAYOUT % str(self.effort)
        if self.velocity != UNDEFINED:
            velocity_s = self.VELOCITY_LAYOUT % str(self.velocity)
        if self.lower != UNDEFINED:
            lower_s = self.LOWER_LAYOUT % str(self.lower)
        if self.upper != UNDEFINED:
            upper_s = self.UPPER_LAYOUT % str(self.upper)
        return self.LAYOUT % (effort_s, velocity_s, lower_s, upper_s)


class Mimic:
    LAYOUT = '<mimic %s %s %s/>\n'
    JOINT_LAYOUT = 'joint="%s"'
    MULTIPLIER_LAYOUT = 'multiplier="%s"'
    OFFSET_LAYOUT = 'offset="%s"'

    def __init__(self, joint, multiplier=UNDEFINED, offset=UNDEFINED):
        self.joint = joint
        self.multiplier = multiplier
        self.offset = offset
    
    def write(self):
        joint_s = ""
        multiplier_s = ""
        offset_s = ""
        if self.joint != UNDEFINED:
            joint_s = self.JOINT_LAYOUT % str(self.joint)
        if self.multiplier != UNDEFINED:
            multiplier_s = self.MULTIPLIER_LAYOUT % str(self.multiplier)
        if self.offset != UNDEFINED:
            offset_s = self.OFFSET_LAYOUT % str(self.offset)
        return self.LAYOUT % (joint_s, multiplier_s, offset_s)


class Joint:
    REVOLUTE = "revolute"
    CONTINUOUS = "continuous"
    PRISMATIC = "prismatic"
    FIXED = "fixed"
    FLOATING = "floating"
    PLANAR = "planar"

    LAYOUT = '<joint name="%s" type="%s">\n' + '%s'*9 + "</joint>"
    PARENT_LAYOUT = '<parent link="%s"/>\n'
    CHILD_LAYOUT = '<child link="%s"/>\n'
    AXIS_LAYOUT = '<axis xyz="%s"/>\n'

    def __init__(self, type, name, parent, child, origin=UNDEFINED, axis=UNDEFINED, limit=UNDEFINED,
                 calibration=UNDEFINED, dynamics=UNDEFINED, mimic=UNDEFINED, safety_controller=UNDEFINED):
        assert(True)

        self.type = type
        self.name = name
        self.parent = parent
        self.child = child
        self.origin = origin
        self.axis = axis
        self.limit = limit
        self.calibration = calibration
        self.dynamics = dynamics
        self.mimic = mimic
        self.safety_controller = safety_controller

    def write(self):
        parent_s = ""
        child_s = ""
        origin_s = ""
        axis_s = ""
        limit_s = ""
        calibration_s = ""
        dynamics_s = ""
        mimic_s = ""
        safety_controller_s = ""
        if self.parent != UNDEFINED:
            parent_s = indent(self.PARENT_LAYOUT % str(self.parent))
        if self.child != UNDEFINED:
            child_s = indent(self.CHILD_LAYOUT % str(self.child))
        if self.origin != UNDEFINED:
            origin_s = indent(self.origin.write())
        if self.axis != UNDEFINED:
            if isinstance(self.axis, (list, tuple)):
                axis_s = indent(self.AXIS_LAYOUT % " ".join(map(str, self.axis)))
            else:
                axis_s = indent(self.AXIS_LAYOUT % str(self.axis))
        if self.limit != UNDEFINED:
            limit_s = indent(self.limit.write())
        if self.mimic != UNDEFINED:
            mimic_s = indent(self.mimic.write())
        # TODO: calibration, dynamics, mimic, safety_controller

        return self.LAYOUT % (self.name, self.type, parent_s, child_s, origin_s, axis_s,
                              limit_s, calibration_s, dynamics_s, mimic_s, safety_controller_s)


class RevoluteJoint(Joint):
    def __init__(self, name, parent, child, origin=UNDEFINED, axis=UNDEFINED, limit=UNDEFINED, calibration=UNDEFINED, dynamics=UNDEFINED, mimic=UNDEFINED, safety_controller=UNDEFINED):
        type = Joint.REVOLUTE
        super().__init__(type, name, parent, child, origin, axis, limit, calibration, dynamics, mimic, safety_controller)


class PrismaticJoint(Joint):
    def __init__(self, name, parent, child, origin=UNDEFINED, axis=UNDEFINED, limit=UNDEFINED, calibration=UNDEFINED, dynamics=UNDEFINED, mimic=UNDEFINED, safety_controller=UNDEFINED):
        type = Joint.PRISMATIC
        super().__init__(type, name, parent, child, origin, axis, limit, calibration, dynamics, mimic, safety_controller)

class FixedJoint(Joint):
    def __init__(self, name, parent, child, origin=UNDEFINED, axis=UNDEFINED, limit=UNDEFINED, calibration=UNDEFINED, dynamics=UNDEFINED, mimic=UNDEFINED, safety_controller=UNDEFINED):
        type = Joint.FIXED
        super().__init__(type, name, parent, child, origin, axis, limit, calibration, dynamics, mimic, safety_controller)

