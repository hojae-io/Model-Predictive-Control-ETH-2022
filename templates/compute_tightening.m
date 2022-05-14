%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function params = compute_tightening(K_tube,H_tube,h_tube,params)  
	% YOUR CODE HERE
    Hx = params.constraints.StateMatrix;
    hx = params.constraints.StateRHS;
    Hu = params.constraints.InputMatrix;
    hu = params.constraints.InputRHS;
    
    state_polyhedron = Polyhedron(Hx, hx);
    input_polyhedron = Polyhedron(Hu, hu);
    E = Polyhedron(H_tube, h_tube);
    KE = K_tube * E;
    
    nominal_state_polyhedron = minus(state_polyhedron, E);
    nominal_input_polyhedron = minus(input_polyhedron, KE);
    
    params.constraints.StateMatrix = nominal_state_polyhedron.A;
    params.constraints.StateRHS = nominal_state_polyhedron.b;
    params.constraints.InputMatrix = nominal_input_polyhedron.A;
    params.constraints.InputRHS = nominal_input_polyhedron.b;
end