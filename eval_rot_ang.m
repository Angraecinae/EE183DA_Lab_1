function [ output_eval ] = eval_rot_ang( input_rot_matrix, theta_sym, ...
    theta_val )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Takes 3x1 matrix representing phi, theta, psi symbolically and returns
% values in radians, theta_sym is a vector containing all variables
% theta_val is a vector containing desired values
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
output_eval = eval(subs(input_rot_matrix, ...
    theta_sym, theta_val));
end