%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TS
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TS(Q,R,N,H,h,params)
            % YOUR CODE HERE      
            nu = params.model.nu;
            nx = params.model.nx;
            
            H_x = params.constraints.StateMatrix;
            h_x = params.constraints.StateRHS;
            H_u = params.constraints.InputMatrix;
            h_u = params.constraints.InputRHS;

            % define optimization variables
            U = sdpvar(repmat(nu,1,N),ones(1,N),'full');
            X = sdpvar(repmat(nx,1,N+1), ones(1,N+1), 'full');

            A = params.model.A;
            B = params.model.B;
            [P,~,~] = idare(A,B,Q,R,[],[]);
            
            constraints = [];
            objective = 0;

            for i=1:N
                u = U{i};
                objective = objective + X{i}'*Q*X{i} + u'*R*u;
                constraints = [constraints, H_x*X{i}<=h_x, H_u*U{i}<=h_u, X{i+1}==A*X{i} + B*u];
            end
            objective = objective +X{N+1}'*P*X{N+1};
            constraints = [constraints, H_x*X{N+1}<=h_x, H*X{N+1}<=h];
            
            opts = sdpsettings('verbose',1,'solver','quadprog');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,X{1},{U{1} objective});
        end

        function [u, u_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            tic;
            [optimizer_out,errorcode] = obj.yalmip_optimizer(x);
            solvetime = toc;
            [u, objective] = optimizer_out{:};

            feasible = true;
            if (errorcode ~= 0)
                feasible = false;
            end

            u_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end