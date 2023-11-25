clear all
clc

% 定义机器人模型
ML2 = Link([pi/2  327     0     pi/2      0   ]);
ML3 = Link([pi/2   0     225     0        0   ]);
ML4 = Link([0      0      0     pi/2      0   ]);
ML5 = Link([0     100     0    -pi/2      0   ]);
ML6 = Link([0      0      0     pi/2      0   ]);
ML7 = Link([0     050      0       0       0   ]);

% 定义关节角度限制（弧度）
joint1_angle_max = 230/180*pi;
joint1_angle_min = -230/180*pi;
joint2_angle_max = 113/180*pi + pi/2;
joint2_angle_min = -115/180*pi + pi/2;
joint3_angle_max = 55/180*pi;
joint3_angle_min = -205/180*pi;
joint4_angle_max = 230/180*pi;
joint4_angle_min = -230/180*pi;
joint5_angle_max = 120/180*pi;
joint5_angle_min = -125/180*pi;
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
