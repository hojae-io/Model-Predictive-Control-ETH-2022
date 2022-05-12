%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H_u, h_u, H_x, h_x] = generate_constraints(params)
    % YOUR CODE HERE
    s_max = params.constraints.MaxAbsPositionXZ;
    y_max = params.constraints.MaxAbsPositionY;
    u_max = params.constraints.MaxAbsThrust;
    
    nx = params.model.nx/2;
    nu = params.model.nu;
    
    H_x = [eye(nx),zeros(nx,nx);
    -eye(nx),zeros(nx,nx)];
    
    h_x = [s_max;y_max;s_max;s_max;y_max;s_max]; %fix this to generalize

    H_u = [eye(nu);
        -eye(nu)];
    h_u = u_max*ones(2*nu,1);
end