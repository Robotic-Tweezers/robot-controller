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

fprintf("Parsing project configurations from python script...\n");
repository_directory = extractBefore(project.RootFolder, 'simulation');
run_config_command = ...
    'python ' + repository_directory + '\config\defines.py 1';

[status, commandOut] = system(run_config_command);

if status ~= 0
    fprintf("Error: Filed to execute project configurations.\n");
    return
end

parameters = containers.Map;
params_str = strsplit(commandOut, '\n')';
params_str(end, :) = [];
for i=1:size(params_str, 1)
    param = split(params_str(i))';
    name = char(param(1));
    value = char(param(2));
    if ~isnan(str2double(value))
        parameters(name) = str2double(value);
    else
        parameters(name) = value;
    end
end

fprintf("Done.\n");

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
clear run_config_command;
clear script;
clear status;
clear commandOut;
clear params_str;
clear param;
clear name;
clear value;
clear i;