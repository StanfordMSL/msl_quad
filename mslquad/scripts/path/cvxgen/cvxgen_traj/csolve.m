% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(c4567_1, Q_1) + quad_form(c4567_2, Q_2) + quad_form(c4567_3, Q_3) + quad_form(c4567_4, Q_4) + quad_form(c4567_5, Q_5) + quad_form(c4567_6, Q_6) + quad_form(c4567_7, Q_7) + quad_form(c4567_8, Q_8))
%   subject to
%     c0123_1(1) == wpts(1)
%     tf0123_1'*c0123_1 + tf4567_1'*c4567_1 == wpts(2)
%     c0123_2(1) == wpts(2)
%     tf0123_2'*c0123_2 + tf4567_2'*c4567_2 == wpts(3)
%     c0123_3(1) == wpts(3)
%     tf0123_3'*c0123_3 + tf4567_3'*c4567_3 == wpts(4)
%     c0123_4(1) == wpts(4)
%     tf0123_4'*c0123_4 + tf4567_4'*c4567_4 == wpts(5)
%     c0123_5(1) == wpts(5)
%     tf0123_5'*c0123_5 + tf4567_5'*c4567_5 == wpts(6)
%     c0123_6(1) == wpts(6)
%     tf0123_6'*c0123_6 + tf4567_6'*c4567_6 == wpts(7)
%     c0123_7(1) == wpts(7)
%     tf0123_7'*c0123_7 + tf4567_7'*c4567_7 == wpts(8)
%     c0123_8(1) == wpts(8)
%     tf0123_8'*c0123_8 + tf4567_8'*c4567_8 == wpts(9)
%     c0123_1(2) == 0
%     2*c0123_1(3) == 0
%     6*c0123_1(4) == 0
%     24*c4567_1(1) == 0
%     final_v_c0123_3'*c0123_3 + final_v_c4567_3'*c4567_3 == 0
%     final_a_c0123_3'*c0123_3 + final_a_c4567_3'*c4567_3 == 0
%     final_j_c0123_3'*c0123_3 + final_j_c4567_3'*c4567_3 == 0
%     final_s_c0123_3'*c0123_3 + final_s_c4567_3'*c4567_3 == 0
%     final_v_c0123_4'*c0123_4 + final_v_c4567_4'*c4567_4 == 0
%     final_a_c0123_4'*c0123_4 + final_a_c4567_4'*c4567_4 == 0
%     final_j_c0123_4'*c0123_4 + final_j_c4567_4'*c4567_4 == 0
%     final_s_c0123_4'*c0123_4 + final_s_c4567_4'*c4567_4 == 0
%     final_v_c0123_5'*c0123_5 + final_v_c4567_5'*c4567_5 == 0
%     final_a_c0123_5'*c0123_5 + final_a_c4567_5'*c4567_5 == 0
%     final_j_c0123_5'*c0123_5 + final_j_c4567_5'*c4567_5 == 0
%     final_s_c0123_5'*c0123_5 + final_s_c4567_5'*c4567_5 == 0
%     final_v_c0123_6'*c0123_6 + final_v_c4567_6'*c4567_6 == 0
%     final_a_c0123_6'*c0123_6 + final_a_c4567_6'*c4567_6 == 0
%     final_j_c0123_6'*c0123_6 + final_j_c4567_6'*c4567_6 == 0
%     final_s_c0123_6'*c0123_6 + final_s_c4567_6'*c4567_6 == 0
%     final_v_c0123_7'*c0123_7 + final_v_c4567_7'*c4567_7 == 0
%     final_a_c0123_7'*c0123_7 + final_a_c4567_7'*c4567_7 == 0
%     final_j_c0123_7'*c0123_7 + final_j_c4567_7'*c4567_7 == 0
%     final_s_c0123_7'*c0123_7 + final_s_c4567_7'*c4567_7 == 0
%     final_v_c0123_8'*c0123_8 + final_v_c4567_8'*c4567_8 == 0
%     final_a_c0123_8'*c0123_8 + final_a_c4567_8'*c4567_8 == 0
%     final_j_c0123_8'*c0123_8 + final_j_c4567_8'*c4567_8 == 0
%     final_s_c0123_8'*c0123_8 + final_s_c4567_8'*c4567_8 == 0
%     v_cont_c0123_1'*c0123_1 + v_cont_c4567_1'*c4567_1 == c0123_2(2)
%     a_cont_c0123_1'*c0123_1 + a_cont_c4567_1'*c4567_1 == 2*c0123_2(3)
%     j_cont_c0123_1'*c0123_1 + j_cont_c4567_1'*c4567_1 == 6*c0123_2(4)
%     s_cont_c0123_1'*c0123_1 + s_cont_c4567_1'*c4567_1 == 24*c4567_2(1)
%     ds_cont_c0123_1'*c0123_1 + ds_cont_c4567_1'*c4567_1 == 120*c4567_2(2)
%     v_cont_c0123_2'*c0123_2 + v_cont_c4567_2'*c4567_2 == c0123_3(2)
%     a_cont_c0123_2'*c0123_2 + a_cont_c4567_2'*c4567_2 == 2*c0123_3(3)
%     j_cont_c0123_2'*c0123_2 + j_cont_c4567_2'*c4567_2 == 6*c0123_3(4)
%     s_cont_c0123_2'*c0123_2 + s_cont_c4567_2'*c4567_2 == 24*c4567_3(1)
%     ds_cont_c0123_2'*c0123_2 + ds_cont_c4567_2'*c4567_2 == 120*c4567_3(2)
%     v_cont_c0123_3'*c0123_3 + v_cont_c4567_3'*c4567_3 == c0123_4(2)
%     a_cont_c0123_3'*c0123_3 + a_cont_c4567_3'*c4567_3 == 2*c0123_4(3)
%     j_cont_c0123_3'*c0123_3 + j_cont_c4567_3'*c4567_3 == 6*c0123_4(4)
%     s_cont_c0123_3'*c0123_3 + s_cont_c4567_3'*c4567_3 == 24*c4567_4(1)
%     ds_cont_c0123_3'*c0123_3 + ds_cont_c4567_3'*c4567_3 == 120*c4567_4(2)
%     v_cont_c0123_4'*c0123_4 + v_cont_c4567_4'*c4567_4 == c0123_5(2)
%     a_cont_c0123_4'*c0123_4 + a_cont_c4567_4'*c4567_4 == 2*c0123_5(3)
%     j_cont_c0123_4'*c0123_4 + j_cont_c4567_4'*c4567_4 == 6*c0123_5(4)
%     s_cont_c0123_4'*c0123_4 + s_cont_c4567_4'*c4567_4 == 24*c4567_5(1)
%     ds_cont_c0123_4'*c0123_4 + ds_cont_c4567_4'*c4567_4 == 120*c4567_5(2)
%     v_cont_c0123_5'*c0123_5 + v_cont_c4567_5'*c4567_5 == c0123_6(2)
%     a_cont_c0123_5'*c0123_5 + a_cont_c4567_5'*c4567_5 == 2*c0123_6(3)
%     j_cont_c0123_5'*c0123_5 + j_cont_c4567_5'*c4567_5 == 6*c0123_6(4)
%     s_cont_c0123_5'*c0123_5 + s_cont_c4567_5'*c4567_5 == 24*c4567_6(1)
%     ds_cont_c0123_5'*c0123_5 + ds_cont_c4567_5'*c4567_5 == 120*c4567_6(2)
%     v_cont_c0123_6'*c0123_6 + v_cont_c4567_6'*c4567_6 == c0123_7(2)
%     a_cont_c0123_6'*c0123_6 + a_cont_c4567_6'*c4567_6 == 2*c0123_7(3)
%     j_cont_c0123_6'*c0123_6 + j_cont_c4567_6'*c4567_6 == 6*c0123_7(4)
%     s_cont_c0123_6'*c0123_6 + s_cont_c4567_6'*c4567_6 == 24*c4567_7(1)
%     ds_cont_c0123_6'*c0123_6 + ds_cont_c4567_6'*c4567_6 == 120*c4567_7(2)
%     v_cont_c0123_7'*c0123_7 + v_cont_c4567_7'*c4567_7 == c0123_8(2)
%     a_cont_c0123_7'*c0123_7 + a_cont_c4567_7'*c4567_7 == 2*c0123_8(3)
%     j_cont_c0123_7'*c0123_7 + j_cont_c4567_7'*c4567_7 == 6*c0123_8(4)
%     s_cont_c0123_7'*c0123_7 + s_cont_c4567_7'*c4567_7 == 24*c4567_8(1)
%     ds_cont_c0123_7'*c0123_7 + ds_cont_c4567_7'*c4567_7 == 120*c4567_8(2)
%
% with variables
%  c0123_1   4 x 1
%  c0123_2   4 x 1
%  c0123_3   4 x 1
%  c0123_4   4 x 1
%  c0123_5   4 x 1
%  c0123_6   4 x 1
%  c0123_7   4 x 1
%  c0123_8   4 x 1
%  c4567_1   4 x 1
%  c4567_2   4 x 1
%  c4567_3   4 x 1
%  c4567_4   4 x 1
%  c4567_5   4 x 1
%  c4567_6   4 x 1
%  c4567_7   4 x 1
%  c4567_8   4 x 1
%
% and parameters
%      Q_1   4 x 4    PSD
%      Q_2   4 x 4    PSD
%      Q_3   4 x 4    PSD
%      Q_4   4 x 4    PSD
%      Q_5   4 x 4    PSD
%      Q_6   4 x 4    PSD
%      Q_7   4 x 4    PSD
%      Q_8   4 x 4    PSD
% a_cont_c0123_1   4 x 1
% a_cont_c0123_2   4 x 1
% a_cont_c0123_3   4 x 1
% a_cont_c0123_4   4 x 1
% a_cont_c0123_5   4 x 1
% a_cont_c0123_6   4 x 1
% a_cont_c0123_7   4 x 1
% a_cont_c4567_1   4 x 1
% a_cont_c4567_2   4 x 1
% a_cont_c4567_3   4 x 1
% a_cont_c4567_4   4 x 1
% a_cont_c4567_5   4 x 1
% a_cont_c4567_6   4 x 1
% a_cont_c4567_7   4 x 1
% ds_cont_c0123_1   4 x 1
% ds_cont_c0123_2   4 x 1
% ds_cont_c0123_3   4 x 1
% ds_cont_c0123_4   4 x 1
% ds_cont_c0123_5   4 x 1
% ds_cont_c0123_6   4 x 1
% ds_cont_c0123_7   4 x 1
% ds_cont_c4567_1   4 x 1
% ds_cont_c4567_2   4 x 1
% ds_cont_c4567_3   4 x 1
% ds_cont_c4567_4   4 x 1
% ds_cont_c4567_5   4 x 1
% ds_cont_c4567_6   4 x 1
% ds_cont_c4567_7   4 x 1
% final_a_c0123_3   4 x 1
% final_a_c0123_4   4 x 1
% final_a_c0123_5   4 x 1
% final_a_c0123_6   4 x 1
% final_a_c0123_7   4 x 1
% final_a_c0123_8   4 x 1
% final_a_c4567_3   4 x 1
% final_a_c4567_4   4 x 1
% final_a_c4567_5   4 x 1
% final_a_c4567_6   4 x 1
% final_a_c4567_7   4 x 1
% final_a_c4567_8   4 x 1
% final_j_c0123_3   4 x 1
% final_j_c0123_4   4 x 1
% final_j_c0123_5   4 x 1
% final_j_c0123_6   4 x 1
% final_j_c0123_7   4 x 1
% final_j_c0123_8   4 x 1
% final_j_c4567_3   4 x 1
% final_j_c4567_4   4 x 1
% final_j_c4567_5   4 x 1
% final_j_c4567_6   4 x 1
% final_j_c4567_7   4 x 1
% final_j_c4567_8   4 x 1
% final_s_c0123_3   4 x 1
% final_s_c0123_4   4 x 1
% final_s_c0123_5   4 x 1
% final_s_c0123_6   4 x 1
% final_s_c0123_7   4 x 1
% final_s_c0123_8   4 x 1
% final_s_c4567_3   4 x 1
% final_s_c4567_4   4 x 1
% final_s_c4567_5   4 x 1
% final_s_c4567_6   4 x 1
% final_s_c4567_7   4 x 1
% final_s_c4567_8   4 x 1
% final_v_c0123_3   4 x 1
% final_v_c0123_4   4 x 1
% final_v_c0123_5   4 x 1
% final_v_c0123_6   4 x 1
% final_v_c0123_7   4 x 1
% final_v_c0123_8   4 x 1
% final_v_c4567_3   4 x 1
% final_v_c4567_4   4 x 1
% final_v_c4567_5   4 x 1
% final_v_c4567_6   4 x 1
% final_v_c4567_7   4 x 1
% final_v_c4567_8   4 x 1
% j_cont_c0123_1   4 x 1
% j_cont_c0123_2   4 x 1
% j_cont_c0123_3   4 x 1
% j_cont_c0123_4   4 x 1
% j_cont_c0123_5   4 x 1
% j_cont_c0123_6   4 x 1
% j_cont_c0123_7   4 x 1
% j_cont_c4567_1   4 x 1
% j_cont_c4567_2   4 x 1
% j_cont_c4567_3   4 x 1
% j_cont_c4567_4   4 x 1
% j_cont_c4567_5   4 x 1
% j_cont_c4567_6   4 x 1
% j_cont_c4567_7   4 x 1
% s_cont_c0123_1   4 x 1
% s_cont_c0123_2   4 x 1
% s_cont_c0123_3   4 x 1
% s_cont_c0123_4   4 x 1
% s_cont_c0123_5   4 x 1
% s_cont_c0123_6   4 x 1
% s_cont_c0123_7   4 x 1
% s_cont_c4567_1   4 x 1
% s_cont_c4567_2   4 x 1
% s_cont_c4567_3   4 x 1
% s_cont_c4567_4   4 x 1
% s_cont_c4567_5   4 x 1
% s_cont_c4567_6   4 x 1
% s_cont_c4567_7   4 x 1
% tf0123_1   4 x 1
% tf0123_2   4 x 1
% tf0123_3   4 x 1
% tf0123_4   4 x 1
% tf0123_5   4 x 1
% tf0123_6   4 x 1
% tf0123_7   4 x 1
% tf0123_8   4 x 1
% tf4567_1   4 x 1
% tf4567_2   4 x 1
% tf4567_3   4 x 1
% tf4567_4   4 x 1
% tf4567_5   4 x 1
% tf4567_6   4 x 1
% tf4567_7   4 x 1
% tf4567_8   4 x 1
% v_cont_c0123_1   4 x 1
% v_cont_c0123_2   4 x 1
% v_cont_c0123_3   4 x 1
% v_cont_c0123_4   4 x 1
% v_cont_c0123_5   4 x 1
% v_cont_c0123_6   4 x 1
% v_cont_c0123_7   4 x 1
% v_cont_c4567_1   4 x 1
% v_cont_c4567_2   4 x 1
% v_cont_c4567_3   4 x 1
% v_cont_c4567_4   4 x 1
% v_cont_c4567_5   4 x 1
% v_cont_c4567_6   4 x 1
% v_cont_c4567_7   4 x 1
%     wpts   9 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.Q_1, ..., params.wpts, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2018-05-12 17:48:34 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
