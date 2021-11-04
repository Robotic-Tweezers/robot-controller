s_unit = [ [1, 1, 1] ./ sqrt(3);
        [1, 0, 0];
        [0, 1, 0];
        [0, 0, 1];
        [-1, -1, -2] ./ sqrt(6)];

u = [   [-1, -1, -1];
        [1, 1, 0];
        [1, 0, 0];
        [0, -1, 0];
        [0, 1, 0]];

v = [   [-1, 0, 0];
        [1, -1, 0];
        [0, 0, 1];
        [1, 0, 0];
        [0, 1, 0]];

expected = [0.0, round(pi, 3), round(-pi/2, 3), round(pi/2, 3), 0]; % expected(1) not used
result = kahan_problem2(s_unit(1,1:3)', u(1,1:3)', v(1,1:3)');

if (result == "NaN")
    fprintf("TC %d: PASS\n", 1);
    display_vector("s_u", s_unit(1, 1:3));
    display_vector("u", u(1, 1:3));
    display_vector("v", v(1, 1:3));
    fprintf("Expected: %f\nRecieved: %f\n\n", expected(1), result);
else
    fprintf("TC %d: FAIL\nExpected: %f\nRecieved: %f\n", 1, expected(1), result);
end

for i = 2:5
    result = kahanP2(s_unit(i,1:3)', u(i,1:3)', v(i,1:3)');

    if (result == expected(i))
        fprintf("TC %d: PASS\n", i);
        display_vector("s_u", s_unit(i, 1:3));
        display_vector("u", u(i, 1:3));
        display_vector("v", v(i, 1:3));
        fprintf("Expected: %f\nRecieved: %f\n\n", expected(i), result);
    else
        fprintf("TC %d: FAIL\nExpected: %f\nRecieved: %f\n", i, expected(i), result);
    end
end