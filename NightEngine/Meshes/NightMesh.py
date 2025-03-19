# NightMesh.py

class NightMesh:
    def __init__(self):
        self.attributes = {}
        self.vertex_count = 0
        self.collision_shape = None # pybullet collision shape

    def add_attribute(self, variable_name:str, data_type:str, data:list):
        self.attributes[variable_name] = {"data_type": data_type, "data": data}

    def set_collision_shape(self, collision_shape):
        self.collision_shape = collision_shape
