%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [tuning_struct, i_opt] = lqr_tuning(x0,Q,params)
    % YOUR CODE HERE
    M = length(Q(1,:));
    tuning_struct = struct.empty(M,0);
    
    for i=1:M
        tuning_struct(i,1).InitialCondition = x0;
        tuning_struct(i,1).Qdiag = Q(:,i);

        q = eye(6);
        for j=1:6
            q(j,j) = Q(j,i);
        end
        ctrl = LQR(q,eye(3),params);
        [x,u,~] = simulate(x0, ctrl, params);
        [s_max, y_max, u_max, J_u, df_max, vf_max, traj_feas] = traj_constraints(x,u,params);
        
        tuning_struct(i,1).MaxAbsPositionXZ = s_max;
        tuning_struct(i,1).MaxAbsPositionY = y_max;
        tuning_struct(i,1).MaxAbsThrust = u_max;
        tuning_struct(i,1).InputCost = J_u;
        tuning_struct(i,1).MaxFinalPosDiff = df_max;
        tuning_struct(i,1).MaxFinalVelDiff = vf_max;
        tuning_struct(i,1).TrajFeasible = traj_feas;
    
    end
    feas_index = [tuning_struct(:).TrajFeasible]==true;
    if feas_index==false
        i_opt = nan;
    else
        feas_cost = [tuning_struct(:).InputCost].*feas_index;
        [~,i_opt] = min(feas_cost);
    end
end