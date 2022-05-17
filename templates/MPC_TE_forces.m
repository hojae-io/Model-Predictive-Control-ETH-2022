%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TE_forces
    properties
        forces_optimizer
    end

    methods
        function obj = MPC_TE_forces(Q,R,N,params)
            % YOUR CODE HERE

            nu = params.model.nu;
            nx = params.model.nx;

            A = params.model.A;
            B = params.model.B;
            H_u = params.constraints.InputMatrix;
            h_u = params.constraints.InputRHS;
            H_x = params.constraints.StateMatrix;
            h_x = params.constraints.StateRHS;

            X = sdpvar(nx,N+1,'full');
            U = sdpvar(nu,N,'full');

            cost = 0;
            const = [];

            for i=1:N
                cost = cost + X(:,i)'*Q*X(:,i) + U(:,i)'*R*U(:,i);
                const = [const, X(:,i+1)==A*X(:,i)+B*U(:,i)];
                const = [const, H_x*X(:,i)<=h_x, H_u*U(:,i)<=h_u];
            end
            const = [const, X(:,N+1)==0];
            opts = getOptions('forcesSolver');
            opts.printlevel = 0;
            
            
            obj.forces_optimizer = optimizerFORCES(const, cost, opts, X(:,1), U(:,1), {'xinit'}, {'u0'});
            % YOUR CODE HERE
        end
        


        function [u, u_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            [optimizer_out,errorcode,info] = obj.forces_optimizer(x);
            u = optimizer_out;
            objective = info.pobj;
            solvetime = info.solvetime;

            feasible = true;
            if any(errorcode ~= 1)
                feasible = false;
                warning('MPC infeasible');
            end

            u_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end