from arithmetic import *
from instructions import *


class RobotURDF:
    def __init__(self, name):
        self.name = name
        self.header = f'<?xml version="1.0"?>\n<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{self.name}">\n'
        self.links = []
        self.joints = []
        self.constants = []
        self.materials = []

        self.names = {}
        self.components = []

    def newattr(self, elem):
        if hasattr(self, elem.name):
            raise
        setattr(self, elem.name, elem)
        self.names[elem.name] = elem

    def new_link(self, link):
        assert isinstance(link, Link)
        self.links.append(link)
        self.newattr(link)

    def new_joint(self, joint):
        self.joints.append(joint)
        self.newattr(joint)

    def blank_lines(self, nb):
        self.components.append(Blank(nb))

    def comment(self, text):
        c = Comment(text)
        self.components.append(c)

    def separation(self):
        c = Separation()
        self.components.append(c)

    def def_constant(self, name, value):
        c = Constant(name, value)
        self.constants.append(c)
        self.components.append(c)
        self.newattr(c)

    def declare_name(self, name):
        c = Name(name)
        self.newattr(c)

    def add_link(self, link):
        self.links.append(link)
        self.components.append(link)
        self.newattr(link)

    def add_joint(self, joint):
        self.joints.append(joint)
        self.components.append(joint)
        self.newattr(joint)

    def add_material(self, material):
        self.materials.append(material)
        self.components.append(material)
        self.newattr(material)

    def compile(self, filename):
        comment = "<!-- %s -->"
        with open(filename, "w", encoding="utf-8") as f:
            f.write(self.header)
            f.write(comment % SEPARATION)
            f.write("\n"*5)
            code = "".join(c.write() for c in self.components)
            f.write(indent(code))
            f.write("\n"*5)
            f.write("</robot>")



