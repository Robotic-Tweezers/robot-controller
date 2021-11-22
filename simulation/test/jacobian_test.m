% TODO: test
jacobian_matrix = jacobian([0 0 0]);
expected = [
    [0, -15, 0];
    [0, 0, 0];
    [0, 0, 0];
    [0, 0, 0];
    [0, 1, 0];
    [-1, 0, -1];
];
assert(isequal(round(jacobian_matrix, 4), round(expected, 4)));

jacobian_matrix = jacobian([0 pi / 2 0]);
expected = [
    [0, 0, 0];
    [15, 0, 0];
    [0, 15, 0];
    [0, 0, -1];
    [0, 1, 0];
    [-1, 0, 0];
];
assert(isequal(round(jacobian_matrix, 4), round(expected, 4)));


clear jacobian_matrix;
clear expected;

fprintf("PASS\n");