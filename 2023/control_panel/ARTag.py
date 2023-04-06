import CPO

class ARTag(CPO):

    def __init__(self, position, tag_id, type):
        super.__init__(position)
        self._tag_id = tag_id
        self._type = type


    def get_tag_id(self):
        return self._tag_id

    def get_type(self):
        return self._type  


