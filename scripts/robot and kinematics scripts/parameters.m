%% ROBOT PARAMATERS FILE
function robot = parameters()

robot.name='my_robot';

Link1_length = 5;
link_6_length = 5;
q = [0 0 0 0 0];

robot.DH.theta = '[pi/2 pi/2 pi/2 pi q(4) q(5)]';
robot.DH.d=     '[2.6 q(1)+1.6 q(2)+0.25 q(3)+0.2 0 0]';
robot.DH.a=     '[0 0 0 0 0 0]';
robot.DH.alpha= '[pi/2 pi/2 -pi/2 -pi/2 pi/2 0]';
robot.J=[];

%inverse kinematics for robot

robot.inversekinematic_fn = 'inversekinematic(robot, T)';

%R: rotational, T: translational
robot.kind=['T' 'T' 'T' 'R' 'R'];

% Initialize the base transformation matrix T0
robot.T0 = eye(4); % Identity matrix as the base frame

%number of degrees of freedom
robot.DOF = 5;

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [1
                1
                1
                1
                1];%not available

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-0.75) deg2rad((2.75 - 1.6)); %Axis 1, minimum, maximum
                deg2rad(0) deg2rad((3.4 - 0.25)); %Axis 2, minimum, maximum
                deg2rad(0) deg2rad((1.56 - 0.2)); %Axis 3, min max
                deg2rad(0) deg2rad(90);%Axis 4, min max
                deg2rad(-360) deg2rad(360)]; %Axis 4: Unlimited (400� default)
            
robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time
% end effectors maximum velocity
robot.linear_velmax = 0.5; %chosen somewhat randomly

%% INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;

%% GRAPHICS
robot.graphical.has_graphics=1;
robot.graphical.color = [25 20 40];
%for transparency
robot.graphical.draw_transparent=1;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-10 10 -10 10 0 10];
robot = read_graphics(robot);

%% Robot dynamics
robot.has_dynamics=0;

%consider friction in the computations
robot.dynamics.friction=0;

robot.motors=load_motors([5 5 5 4 4 4]);
%Speed reductor at each joint
robot.motors.G=[300 300 300 300 300 300];