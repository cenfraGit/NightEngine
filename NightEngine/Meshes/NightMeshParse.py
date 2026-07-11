# NightMeshParse.py
# reads obj assumming triangulation

from NightEngine.Meshes.NightMesh import NightMesh
from scipy.spatial import ConvexHull
import pybullet as p
import numpy as np
import math

class NightMeshParse(NightMesh):
    def __init__(self, filename, color=[1.0, 1.0, 1.0], collision=True, collision_mode="hull"):
        """collision_mode:
        - "hull": convex hull of the model (works for dynamic bodies)
        - "trimesh": exact concave triangle mesh, static only (mass=0)
        """
        positions = []
        vertices = []
        faces = []
        normals = []
        uvs = []
        with open(filename) as file:
            for line in file:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                line_elements = line.split(' ')
                # check if vertex
                if (line_elements[0] == "v"):
                    vertices.append([float(line_elements[1]), float(line_elements[2]), float(line_elements[3])])
                # check if face
                elif (line_elements[0] == "f"):
                    face = []
                    # get data for each vertex in the face
                    for i in range(1, len(line_elements)):
                        # split vertex position/uv/normal data
                        vertex_data = line_elements[i].split('/')
                        face.append([int(v)-1 if v else None for v in vertex_data])
                    faces.append(face)
                # vertex normal
                elif (line_elements[0] == "vn"):
                    normals.append([float(line_elements[1]), float(line_elements[2]), float(line_elements[3])])
                # textures
                elif (line_elements[0] == "vt"):
                    uvs.append([float(line_elements[1]), float(line_elements[2])])

        positions = []
        normals_new = []
        uvs_new = []
        
        for face in faces:
            for vert in face:
                positions.append(vertices[vert[0]])

                if len(vert) > 1 and vert[1] is not None:
                    uvs_new.append(uvs[vert[1]])
                else:
                    uvs_new.append([0.0, 0.0])
                    
                if len(vert) > 2 and vert[2] is not None:
                    normals_new.append(normals[vert[2]])
                else:
                    normals_new.append([0.0, 1.0, 0.0]) 

        colors = [color for _ in range(len(positions))]

        super().__init__()
        self.add_attribute("vertex_position", "vec3", positions)
        self.add_attribute("vertex_color",    "vec3", colors)
        self.add_attribute("vertex_normal",   "vec3", normals_new)
        self.add_attribute("vertex_uv",       "vec2", uvs_new)
        self.vertex_count = len(positions)

        if collision:
            if collision_mode == "trimesh":
                # bullet builds a concave triangle mesh straight from
                # the file. only valid for static (mass=0) objects.
                self.set_collision_shape(p.createCollisionShape(p.GEOM_MESH,
                                                                fileName=filename))
            elif collision_mode == "hull":
                # convex hull of the unique vertices: safe for dynamic
                # bodies, and keeps the vertex count far below
                # bullet's shared-memory upload limit
                points = np.array(vertices)
                hull_points = points[ConvexHull(points).vertices]
                if len(hull_points) > 1024:
                    stride = math.ceil(len(hull_points) / 1024)
                    hull_points = hull_points[::stride]
                self.set_collision_shape(p.createCollisionShape(p.GEOM_MESH,
                                                                vertices=hull_points.tolist()))
            else:
                raise Exception(f"unknown collision_mode: {collision_mode}")
