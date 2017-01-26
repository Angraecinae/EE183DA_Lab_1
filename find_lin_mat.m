function [ output_matrix ] = find_lin_mat( input_matrix )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Takes 4x4 transformation matrix and converts into 3x1 matrix representing
% x, y, z symbolically
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    output_matrix = [input_matrix(1, 4); ...
                     input_matrix(2, 4); ...
                     input_matrix(3, 4)];

end

