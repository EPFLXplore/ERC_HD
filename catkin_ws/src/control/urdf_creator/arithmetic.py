class Name:
    def __init__(self, name):
        self.name = name

    def write(self):
        return str(self.name)

    def __str__(self):
        return str(self.name)


class Constant:
    LAYOUT = '<xacro:property name="%s" value="%s" />\n'

    def __init__(self, name, value):
        self.name = name
        self.value = str(value)

    def write(self):
        return self.LAYOUT % (self.name, self.value)

    def __str__(self):
        return "${%s}" % self.name

    def __add__(self, other):
        return Arithmetic(self)+other

    def __radd__(self, other):
        return other+Arithmetic(self)

    def __sub__(self, other):
        return Arithmetic(self)-other

    def __rsub__(self, other):
        return other-Arithmetic(self)

    def __mul__(self, other):
        return Arithmetic(self)*other

    def __rmul__(self, other):
        return other*Arithmetic(self)

    def __truediv__(self, other):
        return Arithmetic(self)/other

    def __rtruediv__(self, other):
        return other/Arithmetic(self)

    def __neg__(self):
        return -Arithmetic(self)


class Arithmetic:
    def __init__(self, expression=None):
        if expression is None:
            expression = ""
        if isinstance(expression, Arithmetic):
            self.expression = expression.expression
        elif isinstance(expression, str):
            self.expression = expression
        elif isinstance(expression, Constant):
            self.expression = expression.name
        else:
            self.expression = str(expression)

    def __add__(self, other):
        ans = Arithmetic()
        if isinstance(other, Arithmetic):
            ans.expression = "(" + self.expression + "+" + other.expression + ")"
        elif isinstance(other, str):
            ans.expression = "(" + self.expression + "+" + other + ")"
        elif isinstance(other, Constant):
            ans.expression = "(" + self.expression + "+" + other.name + ")"
        else:
            ans.expression = "(" + self.expression + "+" + str(other) + ")"
        return ans

    def __radd__(self, other):
        return self + other

    def __sub__(self, other):
        ans = Arithmetic()
        if isinstance(other, Arithmetic):
            ans.expression = "(" + self.expression + "-" + other.expression + ")"
        elif isinstance(other, str):
            ans.expression = "(" + self.expression + "-" + other + ")"
        elif isinstance(other, Constant):
            ans.expression = "(" + self.expression + "-" + other.name + ")"
        else:
            ans.expression = "(" + self.expression + "-" + str(other) + ")"
        return ans

    def __rsub__(self, other):
        ans = Arithmetic()
        if isinstance(other, Arithmetic):
            ans.expression = "(" + other.expression + "-" + self.expression + ")"
        elif isinstance(other, str):
            ans.expression = "(" + other + "-" + self.expression + ")"
        elif isinstance(other, Constant):
            ans.expression = "(" + other.name + "-" + self.expression + ")"
        else:
            ans.expression = "(" + str(other) + "-" + self.expression + ")"
        return ans

    def __mul__(self, other):
        ans = Arithmetic()
        if isinstance(other, Arithmetic):
            ans.expression = "(" + self.expression + "*" + other.expression + ")"
        elif isinstance(other, str):
            ans.expression = "(" + self.expression + "*" + other + ")"
        elif isinstance(other, Constant):
            ans.expression = "(" + self.expression + "*" + other.name + ")"
        else:
            ans.expression = "(" + self.expression + "*" + str(other) + ")"
        return ans

    def __rmul__(self, other):
        return self*other

    def __truediv__(self, other):
        ans = Arithmetic()
        if isinstance(other, Arithmetic):
            ans.expression = "(" + self.expression + "/" + other.expression + ")"
        elif isinstance(other, str):
            ans.expression = "(" + self.expression + "/" + other + ")"
        elif isinstance(other, Constant):
            ans.expression = "(" + self.expression + "/" + other.name + ")"
        else:
            ans.expression = "(" + self.expression + "/" + str(other) + ")"
        return ans

    def __rtruediv__(self, other):
        ans = Arithmetic()
        if isinstance(other, Arithmetic):
            ans.expression = "(" + other.expression + "/" + self.expression + ")"
        elif isinstance(other, str):
            ans.expression = "(" + other + "/" + self.expression + ")"
        elif isinstance(other, Constant):
            ans.expression = "(" + other.name + "/" + self.expression + ")"
        else:
            ans.expression = "(" + str(other) + "/" + self.expression + ")"
        return ans

    def __neg__(self):
        ans = Arithmetic()
        ans.expression = "-(" + self.expression + ")"
        return ans

    def __str__(self):
        return "${%s}" % self.expression

    def __repr__(self):
        return str(self)

    @classmethod
    def conc(cls, *seq):
        if len(seq) == 1 and isinstance(seq[0], (list, tuple, set, dict)):
            s = " ".join(map(str, seq[0]))
        else:
            s = " ".join(map(str, seq))
        return s


PI = Arithmetic("pi")
