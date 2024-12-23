%% Inverse Kinematics
%input T (the arm matrix into the file). Find T using the direct kinematics
%So find T_final with desired q-vals
%note that tool locations in cartesian space are already in T

function q = inversekinematic(robot, T, teach_arg)
if nargin < 2
        teach_arg = 2; % Set a default value for b
end
disp("performing inverse kinematics")

Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);
a1 = T(1,3);
a3 = T(3,3);

disp(Px);
disp(Py);
disp(Pz);
disp(a1);
disp(a3);


%solving for primatic joints
q1 = Px - 1.6;
q2 = Py - 0.25;
q3 = 2.4 - Pz;

%solving for tool orientation
q4 = atan(a1 / a3);
if round(cos(q4), 2) == 0
    q5 = pi*log(-a1 / sin(q4));
else
    q5 = pi*log(-a3 / cos(q4));
end

q = [q1;
    q2;
    q3;
    q4;
    q5];






