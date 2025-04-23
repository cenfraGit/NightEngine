# NightMeshParse.py
# reads obj assumming triangulation

from NightEngine.Meshes.NightMesh import NightMesh
import pybullet as p

class NightMeshParse(NightMesh):
    def __init__(self, filename, color=[1.0, 1.0, 1.0], collision=True):
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

        # if collision:
        #     self.set_collision_shape(p.createCollisionShape(p.GEOM_MESH, fileName=filename))
