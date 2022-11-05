UNDEFINED = "undef"
NATURAL = "natural"
SEPARATION = "="*120


def indent(block, indents=1):
    s = "\t"
    for i, c in enumerate(block):
        s += c
        if c == "\n" and len(block) > i+1 and block[i+1] != "\n":
            s += "\t"*indents
    return s
