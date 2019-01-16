% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(norm(W_row3'*f - wdes(3), 1))
%   subject to
%     sum(f) == wdes(1)
%     W_row2'*f == wdes(2)
%     W_row4'*f == wdes(4)
%     FMIN <= f
%     f <= FMAX
%     W_row3'*f >= Wy_LB
%     W_row3'*f <= Wy_UB
%
% with variables
%        f   4 x 1
%
% and parameters
%     FMAX   1 x 1
%     FMIN   1 x 1
%   W_row2   4 x 1
%   W_row3   4 x 1
%   W_row4   4 x 1
%    Wy_LB   1 x 1
%    Wy_UB   1 x 1
%     wdes   4 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.FMAX, ..., params.wdes, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2018-03-26 22:22:07 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
