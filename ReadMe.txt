!!Run only on Release x64!!
Controls:

camera:
WASD + Mouse - move camera camera
EQ - move camera Up/Down


T - switch between spheres
ctrl + WASD - apply velocity in camera looking direction
K - Turn ON/OFF Wind
Enter - switch ON/OFF debug view

Additional controls:
Arrows Up/Down change gravity
Space - lift selected sphere up
G - lift All spheres up


you can cofigure Cloth and spheres in GameEngine/scenes/physics.json

Hook’s Law: SimplePhysics/cSimpleSoftBody.cpp line 237
AABB vs AABB broadphase: SimplePhysics/cSimplePhysicsWorld.cpp line 121
Wind: SimplePhysics/cSimpleSoftBody.cpp line 116


