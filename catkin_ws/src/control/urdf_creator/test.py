from compiler import *
import sys
from utils import *


class Test(RobotURDF):
    def __init__(self):
        super().__init__("astra")
        self.comment("="*10 + "CONSANTS" + "="*10)
        self.comment("base_link is rover")
        self.box_link_dim("base_link", (0.6, 0.5, 0.16125))   # base_linnk is rover
        self.box_link_origin("base_link",
                             btm_shift=(0,0,0),
                             top_shift=(0.525-self.base_link_dim_x/2,0,self.base_link_dim_z/2))

        self.box_link_dim("link1", (0.08, 0.1505, 0.138))
        self.box_link_origin("link1",
                             btm_shift=(0, -(0.094-self.link1_dim_y/2), self.link1_dim_z/2),
                             top_shift=(0, (self.link1_dim_x-0.0565), 0.098-self.link1_dim_z/2))

        self.box_link_dim("link2", (0.096, 0.07, 0.487225))
        self.box_link_origin("link2",
                             btm_shift=(0,0,-(0.048-self.link2_dim_z/2)),
                             top_shift=(0.039225-self.link2_dim_x/2, 0, self.link2_dim_z/2-0.039225))

        self.box_link_dim("motor_j3", (0.09, 0.07, 0.3))
        self.box_link_origin("motor_j3",
                             btm_shift=(-0.084225, 0, 0.004388),
                             top_shift=(0, 0, 0))

        self.box_link_dim("link3", (0.115, 0.112, 0.387))
        self.box_link_origin("link3",
                             btm_shift=((0.02-self.link3_dim_x/2), 0, -(0.087-self.link3_dim_z/2)),
                             top_shift=((0.0415-self.link3_dim_x/2), 0, self.link3_dim_z/2))

        self.box_link_dim("link4", (0.080, 0.175, 0.1208))
        self.box_link_origin("link4",
                             btm_shift=(0, 0, self.link4_dim_z/2),
                             top_shift=(0, 0, self.link4_dim_z/2-0.031))
        
        self.box_link_dim("link5", (0.045, 0.045, 0.09455))
        self.box_link_origin("link5",
                             btm_shift=(0, 0, self.link5_dim_z/2-0.0318),
                             top_shift=(0, 0, self.link5_dim_z/2))
        
        self.box_link_dim("gripper_base", (0.1565, 0.125, 0.103))
        self.box_link_origin("gripper_base",
                             btm_shift=((self.gripper_base_dim_x/2-0.076), -(0.0671-self.gripper_base_dim_y/2), self.gripper_base_dim_z/2),
                             top_shift=(-(self.gripper_base_dim_x/2-0.076), 0.0671-self.gripper_base_dim_y/2, self.gripper_base_dim_z/2))

        self.box_link_dim("gripper_left_finger", (0.0215, 0.016, 0.045))
        self.box_link_origin("gripper_left_finger",
                             btm_shift=(0, self.gripper_left_finger_dim_y/2, self.gripper_left_finger_dim_z/2),
                             top_shift=(0, 0, 0))
        
        self.box_link_dim("gripper_right_finger", (0.0215, 0.016, 0.045))
        self.box_link_origin("gripper_right_finger",
                             btm_shift=(0, -self.gripper_right_finger_dim_y/2, self.gripper_right_finger_dim_z/2),
                             top_shift=(0, 0, 0))
        
        names = ["base_link", "link1", "link2", "motor_j3", "link3", "link4", "link5", "gripper_base", "gripper_left_finger", "gripper_right_finger"]

        self.blank_lines(1)
        self.separation()
        self.blank_lines(4)
        self.comment("="*10 + "LINKS" + "="*10)

        for name in names:
            self.classic_link(name)

        self.blank_lines(1)
        self.separation()
        self.blank_lines(4)
        self.comment("="*10 + "JOINTS" + "="*10)

        self.classic_revolute_joint("joint1", parent="base_link", child="link1", axis="0 0 1", limit=Limit(1, 0.5, lower=-2*PI, upper=2*PI))
        self.classic_revolute_joint("joint2", parent="link1", child="link2", axis="0 1 0", limit=Limit(1, 0.5, lower=-1.34, upper=2.07))
        self.classic_fixed_joint("motor_j3_joint", parent="link2", child="motor_j3")
        self.classic_revolute_joint("joint3", parent="link2", child="link3", axis="0 1 0", limit=Limit(1, 0.5, lower=-1.1, upper=1))
        self.classic_revolute_joint("joint4", parent="link3", child="link4", axis="0 0 1", limit=Limit(1, 0.5, lower=-2*PI, upper=2*PI))
        self.classic_revolute_joint("joint5", parent="link4", child="link5", axis="0 1 0", limit=Limit(1, 0.5, lower=-1.9, upper=1.9))
        self.classic_revolute_joint("joint6", parent="link5", child="gripper_base", axis="0 0 1", limit=Limit(1, 0.5, lower=-PI, upper=PI))
        self.classic_prismatic_joint("gripper_left_finger_joint", parent="gripper_base", child="gripper_left_finger", axis="0 1 0", limit=Limit(1, 0.5, lower=0, upper=0.04))
        mimic = Mimic("gripper_left_finger_joint", "-1", "0")
        self.classic_prismatic_joint("gripper_right_finger_joint", parent="gripper_base", child="gripper_right_finger", axis="0 1 0", mimic=mimic)

    def box_link_dim(self, name, dim):
        self.blank_lines(3)
        self.comment(name + " constants")
        self.def_constant(name+"_dim_x", dim[0])
        self.def_constant(name+"_dim_y", dim[1])
        self.def_constant(name+"_dim_z", dim[2])
        self.def_constant(name+"_dim", Arithmetic.conc(dim))

    def box_link_origin(self, name, btm_shift, top_shift):

        self.def_constant(name+"_btm_shift_x", btm_shift[0])
        self.def_constant(name+"_btm_shift_y", btm_shift[1])
        self.def_constant(name+"_btm_shift_z", btm_shift[2])

        self.def_constant(name+"_top_shift_x", top_shift[0])
        self.def_constant(name+"_top_shift_y", top_shift[1])
        self.def_constant(name+"_top_shift_z", top_shift[2])

        self.def_constant(name+"_origin", Arithmetic.conc(btm_shift))
        self.def_constant(name+"_attach_point", Arithmetic.conc([top_shift[i]+btm_shift[i] for i in range(3)]))

    def classic_link(self, name):
        self.blank_lines(2)
        self.comment(name)
        geometry = Box(getattr(self, name+"_dim"))
        origin = Origin(getattr(self, name+"_origin"))
        visual = LinkVisual(geometry, origin)
        collision = LinkCollision(geometry, origin)
        link = Link(name, visual, collision)
        self.add_link(link)

    def classic_revolute_joint(self, name, parent, child, axis, limit=None):
        self.blank_lines(2)
        self.comment(name)
        parent_s = str(parent)
        if name == "joint3":
            rpy = "0 ${pi/2} 0"
        else:
            rpy = UNDEFINED
        origin = Origin(xyz=self.names[parent_s+"_attach_point"], rpy=rpy)
        if limit is None:
            limit = Limit(1, 0.5, lower=-PI, upper=PI)
        joint = RevoluteJoint(name, parent, child, origin, axis, limit)
        self.add_joint(joint)
    
    def classic_prismatic_joint(self, name, parent, child, axis, limit=None, mimic=UNDEFINED):
        self.blank_lines(2)
        self.comment(name)
        parent_s = str(parent)
        origin = Origin(xyz=self.names[parent_s+"_attach_point"])
        if limit is None:
            limit = Limit(1, 0.5, lower=0, upper=0.1)
        joint = PrismaticJoint(name, parent, child, origin, axis, limit, mimic=mimic)
        self.add_joint(joint)

    def classic_fixed_joint(self, name, parent, child, limit=None):
        self.blank_lines(2)
        self.comment(name)
        parent_s = str(parent)
        origin = Origin(xyz=self.names[parent_s+"_attach_point"])
        if limit is None:
            limit = Limit(1, 0.5, lower=-PI, upper=PI)   # arbitrary
        axis = "1 0 0"  # arbitrary
        joint = FixedJoint(name, parent, child, origin, axis, limit)
        self.add_joint(joint)


if __name__ == "__main__":
    urdf = Test()
    urdf.compile(sys.argv[1])
    #roslaunch urdf_tutorial display.launch model:=test.urdf.xacro
