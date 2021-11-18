function [robot] = create_robot()
%CREATE_ROBOT Create a robot object used in controls analysis
%   Detailed explanation goes here
%   https://www.mathworks.com/help/robotics/ug/build-a-robot-step-by-step.html
%   https://www.mathworks.com/help/robotics/ug/build-basic-rigid-body-tree-models.html

robot = rigidBodyTree("DataFormat", 'column', 'MaxNumBodies', 6);
base = robot.Base;

% Servo1 connected to Link 1 represented as a cylinder with r=2cm and l=5cm
servo1 = rigidBody('servo1');
servo1_collision = collisionCylinder(0.02, 0.05);
addCollision(servo1, servo1_collision);
fixed_joint = rigidBodyJoint('base_joint', 'fixed');
fixed_joint.setFixedTransform(trvec2tform([0, 0, 0.05 / 2]));
servo1.Joint = fixed_joint;
addBody(robot, servo1, base.Name);

link1 = rigidBody('link1');
link1_collision = collisionCylinder(0.01, 0.05);
addCollision(link1, link1_collision);
theta1 = rigidBodyJoint('theta1', 'revolute');
theta1.setFixedTransform(trvec2tform([0, 0, 0.05]));
theta1.JointAxis = [0 0 1];
link1.Joint = theta1;
addBody(robot, link1, servo1.Name);

servo2 = rigidBody('servo2');
servo2_collision = collisionCylinder(0.02, 0.05);
addCollision(servo2, servo2_collision);
servo2_fixed_joint = rigidBodyJoint('servo2_basejoint', 'fixed');
servo2_fixed_joint.setFixedTransform(denavit_hartenberg_transform(0, 0.02, 0, pi / 2));
servo2.Joint = servo2_fixed_joint;
addBody(robot, servo2, link1.Name);

link2_1 = rigidBody('link2_1');
link2_1_collision = collisionCylinder(0.01, 0.03);
addCollision(link2_1, link2_1_collision);
theta2 = rigidBodyJoint('theta2', 'revolute');
theta2.setFixedTransform(trvec2tform([0, 0, 0.03]));
theta2.JointAxis = [0 0 1];
link2_1.Joint = theta2;
addBody(robot, link2_1, servo2.Name);

showdetails(robot);
show(robot,"Collisions","on","Frames","off");

end

