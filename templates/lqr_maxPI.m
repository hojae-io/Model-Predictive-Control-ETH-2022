%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% BRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% INPUT:
%   Q, R: State and input weighting matrix
% OUTPUT:
%   H, h: Describes polytopic X_LQR = {x | H * x <= h}

function [H, h] = lqr_maxPI(Q,R,params)
    Hx = params.constraints.StateMatrix;
    hx = params.constraints.StateRHS;
    Hu = params.constraints.InputMatrix;
    hu = params.constraints.InputRHS;
    A = params.model.A;
    B = params.model.B;

    ctrl = LQR(Q,R,params);
    P = polytope([Hx;Hu*ctrl.K],[hx;hu]);
    Acl = A+B*ctrl.K;

    for i=1:1000
        P_ = P & domain(P,Acl);
        if P_ == P
            break
        end
        P = P_;
    end

    [H,h] = double(P);

end
