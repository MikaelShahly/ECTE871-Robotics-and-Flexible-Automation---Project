%% Trajectory calculations for task B10
robot = load_robot;
%%calc arm matrix at start and end point
Tinit = directkinematic(robot, [-0.75 0 1.2 0, 0]); 
Tfinal = directkinematic(robot, [-0.2 2 0.1 pi/2 0]);
%%calc start and end point in base coord
delta = 0.001;  
init_point = [Tinit(1,4), Tinit(2,4), Tinit(3,4)];
final_point = [Tfinal(1,4), Tfinal(2,4), Tfinal(3,4)];

%generate the trajectory
distance = sqrt((final_point - init_point) * (final_point - init_point)');
V = (final_point - init_point);
d = delta * V / distance; %steps along each axis
num_points = distance / delta;
location = init_point;
for i=1:num_points
   location = [location; i*d + init_point];
end
location=[location; final_point];
figure(2)
x=location(:,1); y=location(:,2); z=location(:,3);
plot3(x,y,z), title('Trajectory'), xlabel('X (m)'), ylabel('Y (m)'), zlabel('Z(m)')
grid
T=Tinit;
qs=[];
V = [V [0 0 0]];
vqs=[];
for i=1:length(location),
    T(1,4)=location(i,1);
    T(2,4)=location(i,2);
    T(3,4)=location(i,3);
    q = inversekinematic(robot, T);
    vq = compute_joint_velocity(robot, q(:,1), V/norm(V)');
    %vq = compute_joint_velocity(robot, q(:,2), V/norm(V)');
    qs=[qs q(:,1)];
    vqs = [vqs vq];
end
figure(3), hold, plot(qs(1,:), 'r'),plot(qs(2,:), 'g'), plot(qs(3,:), 'b'), plot(qs(4,:), 'c'), plot(qs(5,:), "black")
legend('q_1 (rad)','q_2 (rad)','q_3 (rad)', 'q_4 (rad)', "q_5 (rad)"), title('Joint trajectories'), xlabel('Stepnumber')
grid
figure(4), hold, plot(vqs(1,:), 'r'),plot(vqs(2,:), 'g'), plot(vqs(3,:), 'b'), plot(vqs(4,:), 'c'), plot(vqs(5,:), 'black')
legend('vq_1 (rad/s)','vq_2 (rad/s)','vq_3 (rad/s)', 'vq_4 (rad/s)', 'vq_5 (rad/s)'), title('Joint speeds (rad/s)')
grid


