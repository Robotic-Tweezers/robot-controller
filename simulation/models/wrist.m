servo3 = create_rigid_body('servo1', 0.1, [0, 0, 0], [1, 1, 1, 0, 0, 0]);
servo2 = create_rigid_body('servo2', 0.1, [0, 0, 0], [1, 1, 1, 0, 0, 0]);
servo3 = create_rigid_body('servo3', 0.1, [0, 0, 0], [1, 1, 1, 0, 0, 0]);



function [body] = create_rigid_body(name, mass, center_of_mass, inertia)
body = rigidBody(name);
body.Mass = mass;
body.CenterOfMass = center_of_mass;
body.Inertia = inertia;
end