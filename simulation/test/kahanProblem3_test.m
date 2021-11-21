% Test 1
s_unit = [1, 1, 1]' ./ sqrt(3);
t_unit = [1, 1, 1]' ./ sqrt(3);
u = [-1, -1, 1]';
v = [-1, 0, 0]';

[theta, phi] = kahanProblem3(s_unit, t_unit, u, v);
assert(isnan(theta(1)) && isnan(phi(1)))

% Test 2
s_unit = [1, 0, 0]';
t_unit = [0, 0, 1]';
u = [1, 0, -1]';
v = [-1, 0, 1]';

[theta, phi] = kahanProblem3(s_unit, t_unit, u, v);
assert(isequal(round(theta, 4), [-3.1416; 3.1416]) && isequal(round(phi, 4), [3.1416; -3.1416]))

% Test 3
s_unit = [1, 0, 0]';
t_unit = [0, 1, 0]';
u = [1, 0, -1]';
v = [-1, 0, 1]';

[theta, phi] = kahanProblem3(s_unit, t_unit, u, v);
assert(isequal(round(theta, 4), [3.1416; 0]) && isequal(round(phi, 4), [1.5708; 3.1416]))

% Test 4
s_unit = [1, 0, 0]';
t_unit = [1, 0, 0]';
u = [1, 0, 1]';
v = [1, 0, 1]';

[theta, phi] = kahanProblem3(s_unit, t_unit, u, v);
assert(isequal(theta, [0;0]) && isequal(phi, [0;0]))

% Test 5
s_unit = [1, 1, 1]' ./ sqrt(3);
t_unit = [-1, 0, 0]';
u = [1, 0, 1]';
v = [1, 0, 1]';

[theta, phi] = kahanProblem3(s_unit, t_unit, u, v);
assert(isequal(round(theta, 4), [0; 2.0944]) && isequal(round(phi, 4), [0; 1.5708]))

clear s_unit;
clear t_unit;
clear u;
clear v;
clear theta;
clear phi;

fprintf("PASS\n");