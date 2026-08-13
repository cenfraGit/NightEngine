# MeshHemisphere.py
#
# An upper hemisphere (dome), optionally closed with a flat bottom disc.
# Built for photometric calibration: a glossy dome reflects each light as
# a specular highlight whose position on the surface encodes that light's
# direction, which is what light-source calibration solves for.
#
# Generation mirrors MeshSphere's latitude/longitude loop, with theta
# restricted to 0..pi/2 (MeshSphere's theta already runs 0..pi from +Y,
# so this is the upper half). Normals are the normalised position, which
# is what makes specular localisation exact.

from NightEngine.Meshes.NightMesh import NightMesh
import pybullet as p
import numpy as np
import math


class MeshHemisphere(NightMesh):

    def __init__(self, radius=1.0, segments=32, color=[1.0, 1.0, 1.0],
                 collision=False, cap=True):

        super().__init__()

        positions = []
        colors = []
        normals = []
        uvs = []

        # ------------------------------------------------------------
        # dome: latitude 0 (+Y pole) down to pi/2 (equator)
        # ------------------------------------------------------------

        rings = max(2, segments // 2)

        for lat in range(rings):
            theta0 = (math.pi / 2) * lat / rings
            theta1 = (math.pi / 2) * (lat + 1) / rings
            sin_t0, cos_t0 = math.sin(theta0), math.cos(theta0)
            sin_t1, cos_t1 = math.sin(theta1), math.cos(theta1)

            for lon in range(segments):
                phi0 = 2 * math.pi * lon / segments
                phi1 = 2 * math.pi * (lon + 1) / segments
                sin_p0, cos_p0 = math.sin(phi0), math.cos(phi0)
                sin_p1, cos_p1 = math.sin(phi1), math.cos(phi1)

                v0 = [radius * sin_t0 * cos_p0, radius * cos_t0, radius * sin_t0 * sin_p0]
                v1 = [radius * sin_t1 * cos_p0, radius * cos_t1, radius * sin_t1 * sin_p0]
                v2 = [radius * sin_t1 * cos_p1, radius * cos_t1, radius * sin_t1 * sin_p1]
                v3 = [radius * sin_t0 * cos_p1, radius * cos_t0, radius * sin_t0 * sin_p1]

                n0 = [c / radius for c in v0]
                n1 = [c / radius for c in v1]
                n2 = [c / radius for c in v2]
                n3 = [c / radius for c in v3]

                uv0 = [lon / segments, lat / rings]
                uv1 = [lon / segments, (lat + 1) / rings]
                uv2 = [(lon + 1) / segments, (lat + 1) / rings]
                uv3 = [(lon + 1) / segments, lat / rings]

                # winding matches MeshSphere so face culling behaves the same
                positions.extend([v0, v2, v1])
                normals.extend([n0, n2, n1])
                colors.extend([color, color, color])
                uvs.extend([uv0, uv2, uv1])

                positions.extend([v0, v3, v2])
                normals.extend([n0, n3, n2])
                colors.extend([color, color, color])
                uvs.extend([uv0, uv3, uv2])

        # ------------------------------------------------------------
        # optional flat bottom, so the dome is a closed solid
        # ------------------------------------------------------------

        if cap:
            centre = [0.0, 0.0, 0.0]
            down = [0.0, -1.0, 0.0]
            for lon in range(segments):
                phi0 = 2 * math.pi * lon / segments
                phi1 = 2 * math.pi * (lon + 1) / segments
                e0 = [radius * math.cos(phi0), 0.0, radius * math.sin(phi0)]
                e1 = [radius * math.cos(phi1), 0.0, radius * math.sin(phi1)]
                # reversed relative to the dome so the normal faces -Y
                positions.extend([centre, e0, e1])
                normals.extend([down, down, down])
                colors.extend([color, color, color])
                uvs.extend([[0.5, 0.5],
                            [0.5 + 0.5 * math.cos(phi0), 0.5 + 0.5 * math.sin(phi0)],
                            [0.5 + 0.5 * math.cos(phi1), 0.5 + 0.5 * math.sin(phi1)]])

        self.add_attribute("vertex_position", "vec3", positions)
        self.add_attribute("vertex_color", "vec3", colors)
        self.add_attribute("vertex_normal", "vec3", normals)
        self.add_attribute("vertex_uv", "vec2", uvs)
        self.vertex_count = len(positions)

        # a hemisphere is convex, so a convex-hull collision shape is exact
        # (bullet has no hemisphere primitive)
        if collision:
            unique = np.unique(np.array(positions, dtype=np.float32), axis=0)
            if len(unique) > 1024:
                stride = math.ceil(len(unique) / 1024)
                unique = unique[::stride]
            self.set_collision_shape(
                p.createCollisionShape(p.GEOM_MESH, vertices=unique.tolist()))
