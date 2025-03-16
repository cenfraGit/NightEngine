# NightMesh.py

from NightEngine.NightUtils import NightUtils

class NightMesh:
    def __init__(self):
        self.attributes = {}
        self.vertex_count = None

    def add_attribute(self, variable_name:str, data_type:str, data:list):
        self.attributes[variable_name] = {"data_type": data_type, "data": data}
