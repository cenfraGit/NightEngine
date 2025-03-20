# example_starplanet.py

from NightEngine.NightBase import NightBase
from NightEngine.NightCamera import NightCamera
from NightEngine.Objects.NightObject import NightObject
from NightEngine.Materials.NightMaterialDefault import NightMaterialDefault
from NightEngine.Meshes.MeshSphere import MeshSphere
import pybullet as p

class Example(NightBase):
    def setup(self):
        self.scene = self.create_scene()
        self.camera = NightCamera()
        self.camera.set_position([0, 20, 40])
        self.set_gravity(0, 0, 0)

        self.light_directional["direction"] = [1.0, 1.0, 0.0]

        self.star = NightObject(MeshSphere(10, 32), NightMaterialDefault(gl_wireframe=False), mass=30)
        self.star.set_position([0, 0, 0])
        self.scene.add(self.star)

        self.planet = NightObject(MeshSphere(3, 32), NightMaterialDefault(gl_wireframe=False), mass=1)
        self.planet.set_position([40, 0, 0])
        self.scene.add(self.planet)

        self.initial_velocity = False

    def update(self):
        
        sun_pos, _ = p.getBasePositionAndOrientation(self.star.physics_id)
        planet_pos, _ = p.getBasePositionAndOrientation(self.planet.physics_id)

        p.changeDynamics(self.planet.physics_id, -1, linearDamping=0, angularDamping=0)

        if not self.initial_velocity:
            p.resetBaseVelocity(self.planet.physics_id, linearVelocity=[0, 0, 5])
            self.initial_velocity = True
            
        G = 30
        dx = sun_pos[0] - planet_pos[0]
        dy = sun_pos[1] - planet_pos[1]
        dz = sun_pos[2] - planet_pos[2]
        dist_sq = dx**2 + dy**2 + dz**2
        dist = dist_sq**0.5
        force = G * self.star.mass * self.planet.mass / dist_sq
        fx = force * dx / dist
        fy = force * dy / dist
        fz = force * dz / dist

        p.applyExternalForce(objectUniqueId=self.planet.physics_id,
                             linkIndex=-1,
                             forceObj=[fx, fy, fz],
                             posObj=planet_pos,
                             flags=p.WORLD_FRAME)

        self.draw_scene(self.camera)

if __name__ == "__main__":
    engine = Example()
    engine.run()
