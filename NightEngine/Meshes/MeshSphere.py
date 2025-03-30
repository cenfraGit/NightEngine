# MeshSphere.py

# ------------------------------------------------------------
# generated with AI!
# ------------------------------------------------------------

from NightEngine.Meshes.NightMesh import NightMesh
import pybullet as p
import math

class MeshSphere(NightMesh):
    def __init__(self, radius=1.0, segments=16, color=[1.0, 1.0, 1.0]):
        super().__init__()

        positions = []   # List to hold vertex positions
        colors = []      # List to hold vertex colors
        normals = []     # List to hold vertex normals
        uvs = []         # List to hold texture coordinates

        # Generate vertices for triangles
        for lat in range(segments):  # Latitude: 0 to segments - 1
            theta0 = math.pi * lat / segments         # Starting latitude angle
            theta1 = math.pi * (lat + 1) / segments     # Ending latitude angle
            sin_theta0, cos_theta0 = math.sin(theta0), math.cos(theta0)
            sin_theta1, cos_theta1 = math.sin(theta1), math.cos(theta1)

            for lon in range(segments):  # Longitude: 0 to segments - 1
                phi0 = 2 * math.pi * lon / segments         # Starting longitude angle
                phi1 = 2 * math.pi * (lon + 1) / segments     # Ending longitude angle
                sin_phi0, cos_phi0 = math.sin(phi0), math.cos(phi0)
                sin_phi1, cos_phi1 = math.sin(phi1), math.cos(phi1)

                # Four corner vertices of the current quad
                v0 = [radius * sin_theta0 * cos_phi0, radius * cos_theta0, radius * sin_theta0 * sin_phi0]
                v1 = [radius * sin_theta1 * cos_phi0, radius * cos_theta1, radius * sin_theta1 * sin_phi0]
                v2 = [radius * sin_theta1 * cos_phi1, radius * cos_theta1, radius * sin_theta1 * sin_phi1]
                v3 = [radius * sin_theta0 * cos_phi1, radius * cos_theta0, radius * sin_theta0 * sin_phi1]

                # Compute normals by normalizing the vertex positions
                n0 = [v0[i] / radius for i in range(3)]
                n1 = [v1[i] / radius for i in range(3)]
                n2 = [v2[i] / radius for i in range(3)]
                n3 = [v3[i] / radius for i in range(3)]

                # Assign colors for the vertices (customize as needed)
                # c0 = [0.5 + 0.2 * cos_phi0, 0.5 + 0.2 * sin_theta0, 0.5 + 0.2 * cos_theta0]
                # c1 = [0.5 + 0.2 * cos_phi0, 0.5 + 0.2 * sin_theta1, 0.5 + 0.2 * cos_theta1]
                # c2 = [0.5 + 0.2 * cos_phi1, 0.5 + 0.2 * sin_theta1, 0.5 + 0.2 * cos_theta1]
                # c3 = [0.5 + 0.2 * cos_phi1, 0.5 + 0.2 * sin_theta0, 0.5 + 0.2 * cos_theta0]

                c0 = color
                c1 = color
                c2 = color
                c3 = color

                # Calculate UV coordinates
                uv0 = [lon / segments, lat / segments]
                uv1 = [lon / segments, (lat + 1) / segments]
                uv2 = [(lon + 1) / segments, (lat + 1) / segments]
                uv3 = [(lon + 1) / segments, lat / segments]

                # First triangle of the quad
                positions.extend([v0, v2, v1])
                normals.extend([n0, n2, n1])
                colors.extend([c0, c2, c1])
                # colors.extend(color)
                uvs.extend([uv0, uv2, uv1])

                # Second triangle of the quad
                positions.extend([v0, v3, v2])
                normals.extend([n0, n3, n2])
                colors.extend([c0, c3, c2])
                # colors.extend(color)
                uvs.extend([uv0, uv3, uv2])

        # Add attributes to the MeshSphere
        self.add_attribute("vertex_position", "vec3", positions)
        self.add_attribute("vertex_color", "vec3", colors)
        self.add_attribute("vertex_normal", "vec3", normals)  # Add normals
        self.add_attribute("vertex_uv", "vec2", uvs)        # Add texture coordinates

        # Store the vertex count
        self.vertex_count = len(positions)

        # add collision shape
        self.set_collision_shape(p.createCollisionShape(p.GEOM_SPHERE,
                                                       radius=radius))
        
