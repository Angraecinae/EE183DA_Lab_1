function [ output_matrix ] = find_rot_ang( input_matrix )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Takes 3x3 rotational matrix and converts into 3x1 matrix representing
% phi, theta, psi symbolically
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    phi = atan2(input_matrix(3, 2), input_matrix(3, 3));
    c2 = sqrt((input_matrix(3, 2)^2 + input_matrix(3, 3)^2));
    theta = atan2(-input_matrix(3, 1), c2);
    psi = atan2(input_matrix(2, 1), input_matrix(1, 1));
    output_matrix = [phi; ...
                     theta; ...
                     psi];
end