function [ output_eval ] = eval_lin_mat( input_lin_matrix, theta_sym, ...
    theta_val )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Takes 3x1 matrix representing x, y, z symbolically and returns
% values, theta_sym is a vector containing all variables
% theta_val is a vector containing desired values
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
output_eval = eval(subs(input_lin_matrix, ...
    theta_sym, theta_val));
end