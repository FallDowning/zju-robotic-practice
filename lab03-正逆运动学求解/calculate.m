clear;
clc;

L(1) = Link('revolute', 'd', 0.23, 'a', 0, 'alpha', 0, 'offset', 0 ,'modified');
L(2) = Link('revolute', 'd', -0.054, 'a', 0, 'alpha', -pi/2, 'offset', -pi/2 ,'modified');
L(3) = Link('revolute', 'd', 0, 'a', 0.185, 'alpha', 0, 'offset', 0 ,'modified');
L(4) = Link('revolute', 'd', 0.077, 'a', 0.17, 'alpha', 0, 'offset', pi/2 ,'modified');
L(5) = Link('revolute', 'd', 0.077, 'a', 0, 'alpha', pi/2, 'offset', pi/2 ,'modified');
L(6) = Link('revolute', 'd', 0.0855, 'a', 0, 'alpha', pi/2, 'offset', 0 ,'modified');
L(1).qlim = [-10*pi/9, 10*pi/9];
L(2).qlim = [-pi/2, pi/2];
L(3).qlim = [-2*pi/3, 2*pi/3];
L(4).qlim = [-5*pi/6, 5*pi/6];
L(5).qlim = [-5*pi/6, 5*pi/6];
L(6).qlim = [-pi, pi];
robot = SerialLink(L, 'name', 'zju-1');

q1 = [pi/6, 0, pi/6, 0, pi/3, 0];
q2 = [pi/6, pi/6,pi/3,0,pi/3,pi/6];
q3 = [pi/2,0,pi/2,-pi/3,pi/3,pi/6];
q4 = [-pi/6,-pi/6,-pi/3,0,pi/12,pi/2];
q5 = [pi/12,pi/12,pi/12,pi/12,pi/12,pi/12];

q = [-85.78, 75.41, 50.2, -34.1, 0, 15] / 180 * pi;





% 计算正运动学
P_1 = robot.fkine(q);
disp(P_1);

disp(size(P_1));

P_1 = P_1.T;
disp(size(P_1));


R_1 = P_1(1:3, 1:3);
eul_1 = rotm2eul(R_1,'XYZ');


eul_1 = eul_1 * 180 / pi; 


% 显示结果
disp('末端执行器的位置：');
disp(P_1(1:3,4));
disp('末端执行器的xyz欧拉角：');
disp(eul_1);


% 计算逆运动学
% 定义欧拉角 (单位：弧度)
eul = [ -180  0  -90 ]/180 * pi;

% 定义平移向量
translation = [     0.348004121976367
  -0.022031675538176
   0.09];

% 通过欧拉角计算旋转矩阵
T = eul2tform(eul, 'XYZ');

% 将平移信息加入齐次变换矩阵
T(1:3, 4) = translation;

q = robot.ikunc(T);              % 计算逆运动学，求解关节角                % 将弧度转换为角度
disp(q);
disp('关节角：');
disp(q');

J = robot.jacob0(q);            % 计算雅克比矩阵

disp('雅克比矩阵：');
disp(J);
disp('雅克比矩阵的行列式：');
disp(det(J));

