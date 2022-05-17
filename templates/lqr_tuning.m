%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [tuning_struct, i_opt] = lqr_tuning(x0,Q,params)
    cost_min = Inf;
    cost_min_idx = nan;
    tuning_struct = [];
    for i=1:size(Q,2)
        q = Q(:,i);

        ctrl = LQR(diag(q),eye(3),params);
        [X,U,~] = simulate(x0,ctrl,params);
        [s_max, y_max, u_max, J_u, df_max, vf_max, traj_feas] = traj_constraints(X,U,params);

        if traj_feas && J_u < cost_min
            cost_min = J_u;
            cost_min_idx = i;
        end

        data = struct(...
            'InitialCondition', x0, ...
            'Qdiag', q, ...
            'MaxAbsPositionXZ', s_max, ...
            'MaxAbsPositionY', y_max, ...
            'MaxAbsThrust', u_max, ...
            'InputCost', J_u, ...
            'MaxFinalPosDiff', df_max, ...
            'MaxFinalVelDiff', vf_max, ...
            'TrajFeasible', traj_feas...
        );
        tuning_struct = [tuning_struct; data];
    end
    i_opt = cost_min_idx;
end