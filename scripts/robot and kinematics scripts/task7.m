%% Task 7
robot = load_robot;

% T_nut matrix
T_nut = [
    -1  0  0  2.70;
     0  1  0  0.25;
     0  0 -1  1.02;
     0  0  0  1
];

% T_top_left matrix
T_top_left = [
     0  0 -1  1.48;
    -1  0  0  0.85;
     0  1  0  2.40;
     0  0  0  1
];

q_nut = inversekinematic(robot, T_nut);
q_topleft = inversekinematic(robot, T_top_left);

J_nut = manipulator_jacobian(robot, q_nut);
J_topleft = manipulator_jacobian(robot, q_topleft);

disp(J_nut);
disp(J_topleft);
