%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef LQR
    properties
        K
    end
    
    methods
        %constructor
        function obj = LQR(Q,R,params)
            % YOUR CODE HERE
            A = params.model.A;
            B = params.model.B;
            [P,~,~] = idare(A,B,Q,R,[],[]);
            obj.K = -(B'*P*B+R)\B'*P*A;
        end
        
        function [u, u_info] = eval(obj,x)
            % YOUR CODE HERE
            u = obj.K*x;
            u_info = struct('ctrl_feas',true);
        end
    end
end