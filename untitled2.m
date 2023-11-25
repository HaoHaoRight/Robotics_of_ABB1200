%给定末端执行器的初始位置
p1 =[
0.617222144    0.465154659    -0.634561241    -0.254420286
-0.727874557   0.031367208	  -0.684992502	  -1.182407321
-0.298723039   0.884673523	  0.357934776	  -0.488241553
0	           0	          0	              1
];

%给定末端执行器的终止位置
p2 = [
-0.504697849	-0.863267623	-0.007006569	0.664185871
-0.599843651	0.356504321	    -0.716304589	-0.35718173
0.620860432	    -0.357314539	-0.697752567	2.106929688
0	            0	            0	            1    
];


%利用运动学反解ikine求解各关节转角
% Inverse kinematics by optimization without joint limits
% q = R.ikine(T) are the joint coordinates (1 N) corresponding to the robot end-effector
% pose T which is an SE3 object or homogenenous transform matrix (4 4), and N is the
% number of robot joints.

init_ang = robot.ikine(p1);%使用运动学迭代反解的算法计算得到初始的关节角度
targ_ang = robot.ikine(p2);%使用运动学迭代反解的算法计算得到目标关节角度
