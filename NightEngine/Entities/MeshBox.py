# MeshBox.py

from NightEngine.NightMesh import NightMesh

class MeshBox(NightMesh):
    def __init__(self, width=1):

        super().__init__()

        corner0 = [-width/2, -width/2, -width/2]
        corner1 = [ width/2, -width/2, -width/2]
        corner2 = [-width/2,  width/2, -width/2]
        corner3 = [ width/2,  width/2, -width/2]
        corner4 = [-width/2, -width/2,  width/2]
        corner5 = [ width/2, -width/2,  width/2]
        corner6 = [-width/2,  width/2,  width/2]
        corner7 = [ width/2,  width/2,  width/2]
        
        color_x_positive, color_x_negative = [0.5, 0.0, 0.0], [0.0, 0.5, 0.0]
        color_y_positive, color_y_negative = [0.0, 0.0, 0.5], [0.5, 0.5, 0.0]
        color_z_positive, color_z_negative = [0.5, 0.0, 0.5], [0.0, 0.5, 0.5]

        positions = [
            corner5, corner1, corner3, corner5, corner3, corner7,
            corner0, corner4, corner6, corner0, corner6, corner2,
            corner6, corner7, corner3, corner6, corner3, corner2,
            corner0, corner1, corner5, corner0, corner5, corner4,
            corner4, corner5, corner7, corner4, corner7, corner6,
            corner1, corner0, corner2, corner1, corner2, corner3
        ]

        colors = ([color_x_positive]*6 +
                  [color_x_negative]*6 +
                  [color_y_positive]*6 +
                  [color_y_negative]*6 +
                  [color_z_positive]*6 +
                  [color_z_negative]*6)

        self.add_attribute("vertex_position", "vec3", positions)
        self.add_attribute("vertex_color",    "vec3", colors)
        self.vertex_count = len(positions)
