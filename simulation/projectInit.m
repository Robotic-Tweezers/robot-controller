% ====================================================================== %
% Project Setup Script 
% ====================================================================== %

fprintf("Starting project...\n");

project = matlab.project.rootProject;

if isempty(project)
    fprintf("Error: Project file does not appear to be open.\n");
    return
end

fprintf("Project info: \n");
disp(project);

repository_directory = extractBefore(project.RootFolder, 'simulation');

robot = createSphericalWrist();
showdetails(robot);
% Kp = parameters('KP');
% Kv = parameters('KV');
Kp = diag([0.1 0.5 0.2 0.9 0.5 0.9]);
Kv = diag([0.1 0.5 0.1 0.1 0.5 0.1]);
length1 = 10;
length2 = 15;
theta_0 = [0 0 0]';
DOF = 3;

fprintf("Running unit tests...\n");

for i = 1:size(project.Files, 2)
    if extractBetween(project.Files(i).Path, project.RootFolder + '\', '\') == "test"
        script = extractAfter(project.Files(i).Path, project.RootFolder + "\test\");
        disp(script)
        run(script);
    end
end

fprintf("Setup complete, cleaning workspace.\n");

% Cleanup
clear run_config_command script status commandOut params_str param name value i;