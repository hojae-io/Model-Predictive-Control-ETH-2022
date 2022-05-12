%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Xt,Ut,u_info] = simulate(x0, ctrl, params)

% YOUR CODE HERE
% Hint: you can access the control command with ctrl.eval(x(:,i))
Nt = params.model.HorizonLength;
nx = params.model.nx;
nu = params.model.nu;
A = params.model.A;
B = params.model.B;
k = ctrl.K;

Xt = zeros(nx,Nt+1);
Ut = zeros(nu,Nt);
Xt(:,1) = x0;
Ut(:,1) = k*x0;

u_info = struct('ctrl_feas',ones(1,Nt));

for i=2:Nt
    x = Xt(:,i-1);
    u = Ut(:,i-1);
    
    Xt(:,i)=A*x+B*u;
    Ut(:,i)=k*Xt(:,i);
end
Xt(:,Nt+1)=A*Xt(:,Nt)+B*Ut(:,Nt);


end