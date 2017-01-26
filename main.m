clear all;
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Declare symbolic variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
syms a_2 a_3 a_4 a_5;
sym_a_vec = [a_2, a_3, a_4, a_5];
val_a_vec = [0, 15, 0, 5];
syms alpha_2 alpha_3 alpha_4 alpha_5;
sym_alpha_vec = [alpha_2, alpha_3, alpha_4, alpha_5];
val_alpha_vec = [pi/2, pi/2, pi/2, pi/2];
syms d_3 d_4 d_5 d_6;
sym_d_vec = [d_3, d_4, d_5, d_6];
val_d_vec = [0, 0, 0, 0];
syms theta_3 theta_4 theta_5 theta_6;
sym_theta_vec = [theta_3, theta_4, theta_5, theta_6];
val_theta_vec = [0, 0, 0, 0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Declare transformation matrices for joints
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sym_T2_3 = ...
[              cos(theta_3),             -sin(theta_3),             0,               a_2; ...
  cos(alpha_2)*sin(theta_3), cos(alpha_2)*cos(theta_3), -sin(alpha_2), -d_3*sin(alpha_2); ...
  sin(alpha_2)*sin(theta_3), sin(alpha_2)*cos(theta_3),  cos(alpha_2),  d_3*cos(alpha_2); ...
                          0,                         0,             0,                 1];
sym_T3_4 = ...
[              cos(theta_4),             -sin(theta_4),             0,               a_3; ...
  cos(alpha_3)*sin(theta_4), cos(alpha_3)*cos(theta_4), -sin(alpha_3), -d_4*sin(alpha_3); ...
  sin(alpha_3)*sin(theta_4), sin(alpha_3)*cos(theta_4),  cos(alpha_3),  d_4*cos(alpha_3); ...
                          0,                         0,             0,                 1];
sym_T4_5 = ...
[              cos(theta_5),             -sin(theta_5),             0,               a_4; ...
  cos(alpha_4)*sin(theta_5), cos(alpha_4)*cos(theta_5), -sin(alpha_4), -d_5*sin(alpha_4); ...
  sin(alpha_4)*sin(theta_5), sin(alpha_4)*cos(theta_5),  cos(alpha_4),  d_5*cos(alpha_4); ...
                          0,                         0,             0,                 1];
sym_T5_6 = ...
[              cos(theta_6),             -sin(theta_6),             0,               a_5; ...
  cos(alpha_5)*sin(theta_6), cos(alpha_5)*cos(theta_6), -sin(alpha_5), -d_6*sin(alpha_5); ...
  sin(alpha_5)*sin(theta_6), sin(alpha_5)*cos(theta_3),  cos(alpha_5),  d_6*cos(alpha_5); ...
                          0,                         0,             0,                 1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Declare combined transformation matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                      
sym_T0_7 = sym_T2_3 * sym_T3_4 * sym_T4_5 * sym_T5_6; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obtain operational space 6x1 matrix (symbolically)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sym_Op_S = vertcat(find_lin_mat(sym_T0_7), find_rot_ang(sym_T0_7));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obtain Jacobian (symbolically)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sym_Jacobian = jacobian(sym_Op_S, [theta_3, theta_4, theta_5, theta_6]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obtain Jacobian (numerically)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
val_Jacobian = eval(subs(sym_Jacobian, ...
    horzcat(sym_a_vec, sym_alpha_vec, sym_d_vec, sym_theta_vec), ...
    horzcat(val_a_vec, val_alpha_vec, val_d_vec, val_theta_vec)));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate Start Point
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
start_point = eval(subs(sym_Op_S, ...
    horzcat(sym_a_vec, sym_alpha_vec, sym_d_vec, sym_theta_vec), ...
    horzcat(val_a_vec, val_alpha_vec, val_d_vec, val_theta_vec))); 

val_theta_vec = [3 * pi/2, 3 * pi/2, 3 * pi/2, 3 * pi/2];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate End Point
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end_point = eval(subs(sym_Op_S, ...
    horzcat(sym_a_vec, sym_alpha_vec, sym_d_vec, sym_theta_vec), ...
    horzcat(val_a_vec, val_alpha_vec, val_d_vec, val_theta_vec))); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Test Inverse Kinematics Method: 
% while loop conditions are based on preset tolerances 
% new current position is generated via the pseudo-inverted Jacobian
% ss is the dynamic step size (generated each iteration of the loop 
% depending on the difference between the current position and the 
% desired position)
% val_theta_vec_temp is the current values for theta_3, theta_4, etc.
% dq is the calculated change in q
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ss = 0.25 * (end_point - start_point);
val_theta_vec_temp = [0, 0, 0, 0];
current_point = eval(subs(sym_Op_S, ...
    horzcat(sym_a_vec, sym_alpha_vec, sym_d_vec, sym_theta_vec), ...
    horzcat(val_a_vec, val_alpha_vec, val_d_vec, val_theta_vec_temp))); 
x1(1, 1) = current_point(1, 1);
y1(1, 1) = current_point(2, 1);
z1(1, 1) = current_point(3, 1);
i = 2;
while (~isWithin(current_point(1, 1), end_point(1, 1), 0.1) || ...
       ~isWithin(current_point(2, 1), end_point(2, 1), 0.1) || ...
       ~isWithin(current_point(3, 1), end_point(3, 1), 0.1) || ...
       ~isWithin(current_point(4, 1), end_point(4, 1), 0.01) || ...
       ~isWithin(current_point(5, 1), end_point(5, 1), 0.01) || ...
       ~isWithin(current_point(6, 1), end_point(6, 1), 0.01))
   ss = 0.25 * (end_point - current_point);
    current_point = eval(subs(sym_Op_S, ...
        horzcat(sym_a_vec, sym_alpha_vec, sym_d_vec, sym_theta_vec), ...
        horzcat(val_a_vec, val_alpha_vec, val_d_vec, val_theta_vec_temp)));
    x1(i, 1) = current_point(1, 1);
    y1(i, 1) = current_point(2, 1);
    z1(i, 1) = current_point(3, 1);
    val_Jacobian = eval(subs(sym_Jacobian, ...
        horzcat(sym_a_vec, sym_alpha_vec, sym_d_vec, sym_theta_vec), ...
        horzcat(val_a_vec, val_alpha_vec, val_d_vec, val_theta_vec_temp)));
    dq = pinv(val_Jacobian) * ss;
    val_theta_vec_temp = val_theta_vec_temp + dq.';
    disp('Current Point: ');
    disp(current_point);
    i = i + 1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generating a steady rate of change in theta values
% x1 is x values for path generated via IK
% x2 is x values for path generated via FK (given some known end state for
% theta
% x3 is x values for a straight-line path between start and end points
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
val_theta_vec_temp1 = [0, 0, 0, 0];
val_theta_vec_start = [0, 0, 0, 0];
val_theta_vec_final = val_theta_vec_temp;
current_point = eval(subs(sym_Op_S, ...
    horzcat(sym_a_vec, sym_alpha_vec, sym_d_vec, sym_theta_vec), ...
    horzcat(val_a_vec, val_alpha_vec, val_d_vec, val_theta_vec_start))); 
x2(1, 1) = current_point(1, 1);
y2(1, 1) = current_point(2, 1);
z2(1, 1) = current_point(3, 1);

index = 100;

x3(1, 1) = start_point(1, 1) + ((1 / index) * (end_point(1, 1) - start_point(1, 1)));
y3(1, 1) =  start_point(2, 1) + ((1 / index) * (end_point(2, 1) - start_point(2, 1)));
z3(1, 1) = start_point(3, 1) + ((1 / index) * (end_point(3, 1) - start_point(3, 1)));
for i = 1:index
    dq = (1 / index) * (val_theta_vec_final - val_theta_vec_start);
    val_theta_vec_temp1 = val_theta_vec_temp1 + dq;
    current_point = eval(subs(sym_Op_S, ...
        horzcat(sym_a_vec, sym_alpha_vec, sym_d_vec, sym_theta_vec), ...
        horzcat(val_a_vec, val_alpha_vec, val_d_vec, val_theta_vec_temp1)));
    x3(i, 1) = start_point(1, 1) + ((i / index) * (end_point(1, 1) - start_point(1, 1)));
    y3(i, 1) = start_point(2, 1) + ((i / index) * (end_point(2, 1) - start_point(2, 1)));
    z3(i, 1) = start_point(3, 1) + ((i / index) * (end_point(3, 1) - start_point(3, 1)));
    x2(i, 1) = current_point(1, 1);
    y2(i, 1) = current_point(2, 1);
    z2(i, 1) = current_point(3, 1);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure, hold on;
plot3(x1, y1, z1, 'r');
plot3(x2, y2, z2, 'b');
plot3(x3, y3, z3, 'g');
hold off;