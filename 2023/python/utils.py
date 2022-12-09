

def select_tag(key, current):
    OFFSET = 48
    NB_AR_TAGS = 4
    tags_set = set([ord(str(x)) for x in range(NB_AR_TAGS)])

    if key in tags_set:
        return int(key) - OFFSET
    else:
        return current


# def draw(frame, ):
