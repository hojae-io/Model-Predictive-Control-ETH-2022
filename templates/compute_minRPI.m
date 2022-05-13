%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H_tube,h_tube,n_iter] = compute_minRPI(K_tube,params)
    % YOUR CODE HERE    
    A = params.model.A;
    B = params.model.B;
    Hw = params.constraints.DisturbanceMatrix;
    hw = params.constraints.DisturbanceRHS;
    W = Polyhedron(Hw, hw);
    
    Acl = A+B*K_tube;
    E_old = W;
  
    for i=1:1000
        E_new = plus(E_old, Acl^i*W);
        E_new.minHRep();
        
        if eq(E_new, E_old)
            n_iter = i;
            H_tube = E_new.A;
            h_tube = E_new.b;
            break
        end
        
        E_old = E_new;
    end
    
end