%% Calculting the tool-tip in 4 different joint vectors
robot = load_robot;
robot.equipment{1} = load_robot;
robot.tool = load_robot;
robot.graphical.draw_axes=0;

joints_1 = [0, 0, 0, pi/2, 0];
joints_2 = [0.2, 1.2, 1.2, 0 0];
joints_3 = [0.2, 2, 0, pi/2 0];
joints_4 = [0.2, 2, 1, pi/2 0];

endpoint_pos_1 = directkinematic(robot, joints_1);
endpoint_pos_2 = directkinematic(robot, joints_2);
endpoint_pos_3 = directkinematic(robot, joints_3);
endpoint_pos_4 = directkinematic(robot, joints_4);

drawrobot3d(robot, joints_1);
saveas(gcf, 'robot-pos-1.fig');
drawrobot3d(robot, joints_2 );
saveas(gcf, 'robot-pos-2.fig');
drawrobot3d(robot, joints_3);
saveas(gcf, 'robot-pos-3.fig');
drawrobot3d(robot, joints_4);
saveas(gcf, 'robot-pos-4.fig');
