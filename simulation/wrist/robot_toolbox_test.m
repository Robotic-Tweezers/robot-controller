% https://www.mathworks.com/help/robotics/ug/build-a-robot-step-by-step.html
% https://www.mathworks.com/help/robotics/ug/build-basic-rigid-body-tree-models.html
robot = rigidBodyTree("DataFormat","column");
base = robot.Base;

rotatingBase = rigidBody("rotating_base");
arm1 = rigidBody("arm1");
arm2 = rigidBody("arm2");
arm3 = rigidBody("arm3");
gripper = rigidBody("gripper");

collBase = collisionCylinder(0.05,0.04); % cylinder: radius,length
collBase.Pose = trvec2tform([0 0 0.04/2]);
coll1 = collisionBox(0.01,0.02,0.15); % box: length, width, height (x,y,z)
coll1.Pose = trvec2tform([0 0 0.15/2]);
coll2 = collisionBox(0.01,0.02,0.15); % box: length, width, height (x,y,z)
coll2.Pose = trvec2tform([0 0 0.15/2]);
coll3 = collisionBox(0.01,0.02,0.15); % box: length, width, height (x,y,z)
coll3.Pose = trvec2tform([0 0 0.15/2]);
collGripper = collisionSphere(0.025); % sphere: radius
collGripper.Pose = trvec2tform([0 -0.015 0.025/2]);

addCollision(rotatingBase,collBase)
addCollision(arm1,coll1)
addCollision(arm2,coll2)
addCollision(arm3,coll3)
addCollision(gripper,collGripper)

jntBase = rigidBodyJoint("base_joint","revolute");
jnt1 = rigidBodyJoint("jnt1","revolute");
jnt2 = rigidBodyJoint("jnt2","revolute");
jnt3 = rigidBodyJoint("jnt3","revolute");
jntGripper = rigidBodyJoint("gripper_joint","revolute");

jnt1.JointAxis = [1 0 0]; % x-axis
jnt2.JointAxis = [1 0 0];
jnt3.JointAxis = [1 0 0];
jntGripper.JointAxis = [0 1 0] % y-axis

setFixedTransform(jnt1,trvec2tform([0.015 0 0.04]))
setFixedTransform(jnt2,trvec2tform([-0.015 0 0.15]))
setFixedTransform(jnt3,trvec2tform([0.015 0 0.15]))
setFixedTransform(jntGripper,trvec2tform([0 0 0.15]))

bodies = {base,rotatingBase,arm1,arm2,arm3,gripper};
joints = {[],jntBase,jnt1,jnt2,jnt3,jntGripper};

figure("Name","Assemble Robot","Visible","on")
for i = 2:length(bodies) % Skip base. Iterate through adding bodies and joints.
            bodies{i}.Joint = joints{i};
            addBody(robot,bodies{i},bodies{i-1}.Name)
            show(robot,"Collisions","on","Frames","off");
            drawnow;
end

showdetails(robot)

figure("Name","Interactive GUI")
gui = interactiveRigidBodyTree(robot,"MarkerScaleFactor",0.25);

function [inertia_tensor] = cylinder_inertia_approx(mass, length, r_major, r_minor)
inertia_xx = mass * (3 * (r_major + r_minor) + length) / 12;
inertia_yy = inertia_xx;
inertia_zz = mass * (r_major + r_minor) / 2;
inertia_tensor = [inertia_xx inertia_yy inertia_zz 0 0 0];
end