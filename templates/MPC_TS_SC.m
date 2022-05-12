%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TS_SC
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TS_SC(Q,R,N,H,h,S,v,params)            
            % YOUR CODE HERE
            nu = params.model.nu;
            nx = params.model.nx;
            ne = length(S(:,1));

            H_x = params.constraints.StateMatrix;
            h_x = params.constraints.StateRHS;
            H_u = params.constraints.InputMatrix;
            h_u = params.constraints.InputRHS;


            % define optimization variables
            U = sdpvar(repmat(nu,1,N),ones(1,N),'full');
            X = sdpvar(repmat(nx,1,N+1), ones(1,N+1), 'full');
            e = sdpvar(repmat(ne,1,N+1),ones(1,N+1),'full');

            A = params.model.A;
            B = params.model.B;
            [P,~,~] = idare(A,B,Q,R,[],[]);
            
            constraints = [];
            objective = 0;
            
            for i=1:N
                u = U{i};
                objective = objective + X{i}'*Q*X{i} + u'*R*u+e{i}'*S*e{i}+v*norm(e{i}, inf);
                constraints = [constraints, H_x*X{i}<=h_x+e{i}, H_u*U{i}<=h_u, e{i}>=0, X{i+1}==A*X{i} + B*u];
            end
                
            objective = objective +X{N+1}'*P*X{N+1} + e{N+1}'*S*e{N+1}+v*norm(e{N+1}, inf);
            constraints = [constraints, H_x*X{N+1}<=h_x+e{N+1}, H*X{N+1}<=h, e{N+1}>=0];
            
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
