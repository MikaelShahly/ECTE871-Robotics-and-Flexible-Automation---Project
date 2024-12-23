%%Project Robot Paramaters

function robot = paramaters_draft()

robot.name = "draft_rob";

Link1_length = 5;
link_6_length = 5;
q = [0 0 0 0 0];

%% Original params
%robot.DH.theta = '[pi/2 pi/2 pi/2 pi q(4) q(5)]';
%robot.DH.d=     '[10 q(1)+10 q(2)+10 q(3)+10 0 10]';
%robot.DH.a=     '[0 0 0 0 2 0]';
%robot.DH.alpha= '[pi/2 pi/2 -pi/2 -pi/2 -pi/2 0]';
%robot.J=[];
%robot.name='robot'

%% Updated params
robot.DH.theta = '[pi/2 pi/2 pi/2 pi q(4) q(5)]';
robot.DH.d=     '[2.6 q(1)+1 q(2)+0 q(3)+0 0 0.2]';
robot.DH.a=     '[0 0 0 0 0 0]';
robot.DH.alpha= '[pi/2 pi/2 -pi/2 -pi/2 pi/2 0]';
robot.J=[];

%R: rotational, T: translational
robot.kind=['T' 'T' 'T' 'R' 'R'];

% Initialize the base transformation matrix T0
robot.T0 = eye(4); % Identity matrix as the base frame

%number of degrees of freedom
robot.DOF = 5;

%Evaluate the kinematic parameters
Theta=eval(robot.DH.theta);
d=eval(robot.DH.d);
a=eval(robot.DH.a);
alpha=eval(robot.DH.alpha);

%ARM MATRIX
T01=dh(Theta(1),	d(1),	a(1),	alpha(1));
T12=dh(Theta(2),	d(2),	a(2),	alpha(2));
T23=dh(Theta(3),	d(3),	a(3),	alpha(3));
T34=dh(Theta(4),	d(4),	a(4),	alpha(4));
T45=dh(Theta(5),	d(5),	a(5),	alpha(5));
T56=dh(Theta(6),	d(6),	a(6),	alpha(6));

T06=T01*T12*T23*T34*T45*T56; %Arm Matrix
disp(T06)


%draw the robot
%read graphics files
robot.graphical.has_graphics=0;
robot.graphical.color = [25 20 40];
%for transparency
robot.graphical.draw_transparent=1;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
axis([-10 10 -10 10 0 10]);
robot = read_graphics(robot);


drawrobot3d(robot,q);
fk_result = robot.forward_kinematics();
disp(fk_result);