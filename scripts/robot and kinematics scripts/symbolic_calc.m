%%Symbolic Computations for paramaters
syms  q_1 q_2 q_3 q_4 q_5
link_6_length = 0;
link_1_length = 2.6;



theta = [pi/2 pi/2 pi/2 pi q_4 q_5];
d=     [2.6 q_1+1.6 q_2+0.25 q_3+0.2 0 0];
a=     [0 0 0 0 0 0];
alpha= [pi/2 pi/2 -pi/2 -pi/2 pi/2 0];


T_01 = transformation_matrix(theta(1), alpha(1), d(1), a(1));
T_12 = transformation_matrix(theta(2), alpha(2), d(2), a(2));
T_23 = transformation_matrix(theta(3), alpha(3), d(3), a(3));
T_34 = transformation_matrix(theta(4), alpha(4), d(4), a(4));
T_45 = transformation_matrix(theta(5), alpha(5), d(5), a(5));
T_56 = transformation_matrix(theta(6), alpha(6), d(6), a(6));

T_06=T_01*T_12*T_23*T_34*T_45*T_56; %Arm Matrix
T_06_rounded = vpa(T_06, 4);
disp("======================ARM MATRIX===================");
disp(T_06_rounded);

%% FINDING THE JACOBIAN MATRIX
tool_config_vector = [T_06_rounded(1:3, 4); T_06_rounded(1, 3)*exp(q_5 / pi); 
    T_06_rounded(2, 3)*exp(q_5 / pi); T_06_rounded(3, 3)*exp(q_5 / pi)];
disp("======================TOOL CONFIG VECTOR===================");
disp(tool_config_vector);

j1 = diff(tool_config_vector,q_1);
j2 = diff(tool_config_vector,q_2);
j3 = diff(tool_config_vector,q_3);
j4 = diff(tool_config_vector,q_4);
j5 = diff(tool_config_vector,q_5);

jacobian=[j1 j2 j3 j4 j5];

disp("======================JACOBIAN MATRIX===================");
disp(jacobian)

function T = transformation_matrix(theta, alpha, d, a)
    % Calculate the cosines and sines
    Ctheta = cos(theta);
    Stheta = sin(theta);
    Calpha = cos(alpha);
    Salpha = sin(alpha);

    % Construct the transformation matrix
    T = [Ctheta, -Calpha*Stheta, Salpha*Stheta, a*Ctheta;
         Stheta, Calpha*Ctheta, -Salpha*Ctheta, a*Stheta;
         0, Salpha, Calpha, d;
         0, 0, 0, 1]; 
end



