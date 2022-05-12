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
	% YOUR CODE HERE
    H_u = params.constraints.InputMatrix;
    h_u = params.constraints.InputRHS;
    H_x = params.constraints.StateMatrix;
    h_x = params.constraints.StateRHS;
    obj = LQR(Q,R,params);
    H = [H_x;H_u*obj.K];
    h = [h_x;h_u];
end

