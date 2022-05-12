%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [s_max, y_max, u_max, J_u, df_max, vf_max, traj_feas] = traj_constraints(x,u,params)
    % YOUR CODE HERE
    s_limit = params.constraints.MaxAbsPositionXZ;
    y_limit = params.constraints.MaxAbsPositionY;
    u_limit = params.constraints.MaxAbsThrust;
    df_limit = params.constraints.MaxFinalPosDiff;
    vf_limit = params.constraints.MaxFinalVelDiff;
    
    Nt = length(u(1,:));
    s_max = max(abs([x(1,:),x(3,:)]));
    y_max = max(abs(x(2,:)));
    u_max = max(max(abs(u)));
    
    J_u = 0;
   
    for i=1:Nt
        J_u = J_u + u(:,i)'*u(:,i);
    end
    
    df_max = norm(x(1:3,Nt+1));
    vf_max = norm(x(4:6,Nt+1));
    traj_feas = false;
    if s_max<=s_limit && y_max<=y_limit && u_max<=u_limit ...
            && df_max<=df_limit &&vf_max<=vf_limit
        traj_feas = true;
    end    
end

