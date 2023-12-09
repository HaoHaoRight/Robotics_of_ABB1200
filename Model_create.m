clear all
clc %

% % 定义机器人模型加入-pi/2是偏置作用 可视化
% ML2 = Link([0  0.3991     0      -pi/2       0   ]);
% ML3 = Link([0   0       0.448     0          0  -pi/2 ]);
% ML4 = Link([0   0      0.042     -pi/2       0   ]);
% ML5 = Link([0   0.451     0       pi/2     0   ]);
% ML6 = Link([0   0         0      -pi/2       0   ]);
% ML7 = Link([0   0.082     0       0         0   ]);

% 定义机器人模型 不加-pi/2是计算用的模型
ML2 = Link([0  0.3991     0      -pi/2       0   ]);
ML3 = Link([0   0       0.448     0          0   ]);
ML4 = Link([0   0      0.042     -pi/2       0   ]);
ML5 = Link([0   0.451     0       pi/2       0   ]);
ML6 = Link([0   0         0      -pi/2       0   ]);
ML7 = Link([0   0.082     0       0          0   ]);
% 定义关节角度限制（弧度）
joint1_angle_max = 170/180*pi;
joint1_angle_min = -170/180*pi;
joint2_angle_max = 130/180*pi + pi/2;
joint2_angle_min = -100/180*pi + pi/2;
joint3_angle_max = 70/180*pi;
joint3_angle_min = -200/180*pi;
joint4_angle_max = 270/180*pi;
joint4_angle_min = -270/180*pi;
joint5_angle_max = 130/180*pi;
joint5_angle_min = -130/180*pi;
joint6_angle_max = 400/180*pi;
joint6_angle_min = -400/180*pi;

% 设置关节角度限制
ML2.qlim = [joint1_angle_min, joint1_angle_max];
ML3.qlim = [joint2_angle_min, joint2_angle_max];
ML4.qlim = [joint3_angle_min, joint3_angle_max];
ML5.qlim = [joint4_angle_min, joint4_angle_max];
ML6.qlim = [joint5_angle_min, joint5_angle_max];
ML7.qlim = [joint6_angle_min, joint6_angle_max];
save('robot_ABB1200.mat', 'ML2', 'ML3', 'ML4', 'ML5', 'ML6', 'ML7');
robot = SerialLink([ML2 ML3 ML4 ML5 ML6 ML7], 'name', 'ABB1200');
%% 加入teach指令，则可调整各个关节角度
% robot = SerialLink([ML2 ML3 ML4 ML5 ML6 ML7],'name','sixsix');
% robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'ABB1200');
f = 1
figure(f)
% robot.plot(theta);
robot.teach
title('六轴机械臂模型可调节');