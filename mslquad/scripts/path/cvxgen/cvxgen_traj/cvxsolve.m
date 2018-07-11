% Produced by CVXGEN, 2018-05-12 17:48:34 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: cvxsolve.m.
% Description: Solution file, via cvx, for use with sample.m.
function [vars, status] = cvxsolve(params, settings)
if isfield(params, 'Q_1')
  Q_1 = params.Q_1;
elseif isfield(params, 'Q')
  Q_1 = params.Q{1};
else
  error 'could not find Q_1'
end
if isfield(params, 'Q_2')
  Q_2 = params.Q_2;
elseif isfield(params, 'Q')
  Q_2 = params.Q{2};
else
  error 'could not find Q_2'
end
if isfield(params, 'Q_3')
  Q_3 = params.Q_3;
elseif isfield(params, 'Q')
  Q_3 = params.Q{3};
else
  error 'could not find Q_3'
end
if isfield(params, 'Q_4')
  Q_4 = params.Q_4;
elseif isfield(params, 'Q')
  Q_4 = params.Q{4};
else
  error 'could not find Q_4'
end
if isfield(params, 'Q_5')
  Q_5 = params.Q_5;
elseif isfield(params, 'Q')
  Q_5 = params.Q{5};
else
  error 'could not find Q_5'
end
if isfield(params, 'Q_6')
  Q_6 = params.Q_6;
elseif isfield(params, 'Q')
  Q_6 = params.Q{6};
else
  error 'could not find Q_6'
end
if isfield(params, 'Q_7')
  Q_7 = params.Q_7;
elseif isfield(params, 'Q')
  Q_7 = params.Q{7};
else
  error 'could not find Q_7'
end
if isfield(params, 'Q_8')
  Q_8 = params.Q_8;
elseif isfield(params, 'Q')
  Q_8 = params.Q{8};
else
  error 'could not find Q_8'
end
if isfield(params, 'a_cont_c0123_1')
  a_cont_c0123_1 = params.a_cont_c0123_1;
elseif isfield(params, 'a_cont_c0123')
  a_cont_c0123_1 = params.a_cont_c0123{1};
else
  error 'could not find a_cont_c0123_1'
end
if isfield(params, 'a_cont_c0123_2')
  a_cont_c0123_2 = params.a_cont_c0123_2;
elseif isfield(params, 'a_cont_c0123')
  a_cont_c0123_2 = params.a_cont_c0123{2};
else
  error 'could not find a_cont_c0123_2'
end
if isfield(params, 'a_cont_c0123_3')
  a_cont_c0123_3 = params.a_cont_c0123_3;
elseif isfield(params, 'a_cont_c0123')
  a_cont_c0123_3 = params.a_cont_c0123{3};
else
  error 'could not find a_cont_c0123_3'
end
if isfield(params, 'a_cont_c0123_4')
  a_cont_c0123_4 = params.a_cont_c0123_4;
elseif isfield(params, 'a_cont_c0123')
  a_cont_c0123_4 = params.a_cont_c0123{4};
else
  error 'could not find a_cont_c0123_4'
end
if isfield(params, 'a_cont_c0123_5')
  a_cont_c0123_5 = params.a_cont_c0123_5;
elseif isfield(params, 'a_cont_c0123')
  a_cont_c0123_5 = params.a_cont_c0123{5};
else
  error 'could not find a_cont_c0123_5'
end
if isfield(params, 'a_cont_c0123_6')
  a_cont_c0123_6 = params.a_cont_c0123_6;
elseif isfield(params, 'a_cont_c0123')
  a_cont_c0123_6 = params.a_cont_c0123{6};
else
  error 'could not find a_cont_c0123_6'
end
if isfield(params, 'a_cont_c0123_7')
  a_cont_c0123_7 = params.a_cont_c0123_7;
elseif isfield(params, 'a_cont_c0123')
  a_cont_c0123_7 = params.a_cont_c0123{7};
else
  error 'could not find a_cont_c0123_7'
end
if isfield(params, 'a_cont_c4567_1')
  a_cont_c4567_1 = params.a_cont_c4567_1;
elseif isfield(params, 'a_cont_c4567')
  a_cont_c4567_1 = params.a_cont_c4567{1};
else
  error 'could not find a_cont_c4567_1'
end
if isfield(params, 'a_cont_c4567_2')
  a_cont_c4567_2 = params.a_cont_c4567_2;
elseif isfield(params, 'a_cont_c4567')
  a_cont_c4567_2 = params.a_cont_c4567{2};
else
  error 'could not find a_cont_c4567_2'
end
if isfield(params, 'a_cont_c4567_3')
  a_cont_c4567_3 = params.a_cont_c4567_3;
elseif isfield(params, 'a_cont_c4567')
  a_cont_c4567_3 = params.a_cont_c4567{3};
else
  error 'could not find a_cont_c4567_3'
end
if isfield(params, 'a_cont_c4567_4')
  a_cont_c4567_4 = params.a_cont_c4567_4;
elseif isfield(params, 'a_cont_c4567')
  a_cont_c4567_4 = params.a_cont_c4567{4};
else
  error 'could not find a_cont_c4567_4'
end
if isfield(params, 'a_cont_c4567_5')
  a_cont_c4567_5 = params.a_cont_c4567_5;
elseif isfield(params, 'a_cont_c4567')
  a_cont_c4567_5 = params.a_cont_c4567{5};
else
  error 'could not find a_cont_c4567_5'
end
if isfield(params, 'a_cont_c4567_6')
  a_cont_c4567_6 = params.a_cont_c4567_6;
elseif isfield(params, 'a_cont_c4567')
  a_cont_c4567_6 = params.a_cont_c4567{6};
else
  error 'could not find a_cont_c4567_6'
end
if isfield(params, 'a_cont_c4567_7')
  a_cont_c4567_7 = params.a_cont_c4567_7;
elseif isfield(params, 'a_cont_c4567')
  a_cont_c4567_7 = params.a_cont_c4567{7};
else
  error 'could not find a_cont_c4567_7'
end
if isfield(params, 'ds_cont_c0123_1')
  ds_cont_c0123_1 = params.ds_cont_c0123_1;
elseif isfield(params, 'ds_cont_c0123')
  ds_cont_c0123_1 = params.ds_cont_c0123{1};
else
  error 'could not find ds_cont_c0123_1'
end
if isfield(params, 'ds_cont_c0123_2')
  ds_cont_c0123_2 = params.ds_cont_c0123_2;
elseif isfield(params, 'ds_cont_c0123')
  ds_cont_c0123_2 = params.ds_cont_c0123{2};
else
  error 'could not find ds_cont_c0123_2'
end
if isfield(params, 'ds_cont_c0123_3')
  ds_cont_c0123_3 = params.ds_cont_c0123_3;
elseif isfield(params, 'ds_cont_c0123')
  ds_cont_c0123_3 = params.ds_cont_c0123{3};
else
  error 'could not find ds_cont_c0123_3'
end
if isfield(params, 'ds_cont_c0123_4')
  ds_cont_c0123_4 = params.ds_cont_c0123_4;
elseif isfield(params, 'ds_cont_c0123')
  ds_cont_c0123_4 = params.ds_cont_c0123{4};
else
  error 'could not find ds_cont_c0123_4'
end
if isfield(params, 'ds_cont_c0123_5')
  ds_cont_c0123_5 = params.ds_cont_c0123_5;
elseif isfield(params, 'ds_cont_c0123')
  ds_cont_c0123_5 = params.ds_cont_c0123{5};
else
  error 'could not find ds_cont_c0123_5'
end
if isfield(params, 'ds_cont_c0123_6')
  ds_cont_c0123_6 = params.ds_cont_c0123_6;
elseif isfield(params, 'ds_cont_c0123')
  ds_cont_c0123_6 = params.ds_cont_c0123{6};
else
  error 'could not find ds_cont_c0123_6'
end
if isfield(params, 'ds_cont_c0123_7')
  ds_cont_c0123_7 = params.ds_cont_c0123_7;
elseif isfield(params, 'ds_cont_c0123')
  ds_cont_c0123_7 = params.ds_cont_c0123{7};
else
  error 'could not find ds_cont_c0123_7'
end
if isfield(params, 'ds_cont_c4567_1')
  ds_cont_c4567_1 = params.ds_cont_c4567_1;
elseif isfield(params, 'ds_cont_c4567')
  ds_cont_c4567_1 = params.ds_cont_c4567{1};
else
  error 'could not find ds_cont_c4567_1'
end
if isfield(params, 'ds_cont_c4567_2')
  ds_cont_c4567_2 = params.ds_cont_c4567_2;
elseif isfield(params, 'ds_cont_c4567')
  ds_cont_c4567_2 = params.ds_cont_c4567{2};
else
  error 'could not find ds_cont_c4567_2'
end
if isfield(params, 'ds_cont_c4567_3')
  ds_cont_c4567_3 = params.ds_cont_c4567_3;
elseif isfield(params, 'ds_cont_c4567')
  ds_cont_c4567_3 = params.ds_cont_c4567{3};
else
  error 'could not find ds_cont_c4567_3'
end
if isfield(params, 'ds_cont_c4567_4')
  ds_cont_c4567_4 = params.ds_cont_c4567_4;
elseif isfield(params, 'ds_cont_c4567')
  ds_cont_c4567_4 = params.ds_cont_c4567{4};
else
  error 'could not find ds_cont_c4567_4'
end
if isfield(params, 'ds_cont_c4567_5')
  ds_cont_c4567_5 = params.ds_cont_c4567_5;
elseif isfield(params, 'ds_cont_c4567')
  ds_cont_c4567_5 = params.ds_cont_c4567{5};
else
  error 'could not find ds_cont_c4567_5'
end
if isfield(params, 'ds_cont_c4567_6')
  ds_cont_c4567_6 = params.ds_cont_c4567_6;
elseif isfield(params, 'ds_cont_c4567')
  ds_cont_c4567_6 = params.ds_cont_c4567{6};
else
  error 'could not find ds_cont_c4567_6'
end
if isfield(params, 'ds_cont_c4567_7')
  ds_cont_c4567_7 = params.ds_cont_c4567_7;
elseif isfield(params, 'ds_cont_c4567')
  ds_cont_c4567_7 = params.ds_cont_c4567{7};
else
  error 'could not find ds_cont_c4567_7'
end
if isfield(params, 'final_a_c0123_3')
  final_a_c0123_3 = params.final_a_c0123_3;
elseif isfield(params, 'final_a_c0123')
  final_a_c0123_3 = params.final_a_c0123{3};
else
  error 'could not find final_a_c0123_3'
end
if isfield(params, 'final_a_c0123_4')
  final_a_c0123_4 = params.final_a_c0123_4;
elseif isfield(params, 'final_a_c0123')
  final_a_c0123_4 = params.final_a_c0123{4};
else
  error 'could not find final_a_c0123_4'
end
if isfield(params, 'final_a_c0123_5')
  final_a_c0123_5 = params.final_a_c0123_5;
elseif isfield(params, 'final_a_c0123')
  final_a_c0123_5 = params.final_a_c0123{5};
else
  error 'could not find final_a_c0123_5'
end
if isfield(params, 'final_a_c0123_6')
  final_a_c0123_6 = params.final_a_c0123_6;
elseif isfield(params, 'final_a_c0123')
  final_a_c0123_6 = params.final_a_c0123{6};
else
  error 'could not find final_a_c0123_6'
end
if isfield(params, 'final_a_c0123_7')
  final_a_c0123_7 = params.final_a_c0123_7;
elseif isfield(params, 'final_a_c0123')
  final_a_c0123_7 = params.final_a_c0123{7};
else
  error 'could not find final_a_c0123_7'
end
if isfield(params, 'final_a_c0123_8')
  final_a_c0123_8 = params.final_a_c0123_8;
elseif isfield(params, 'final_a_c0123')
  final_a_c0123_8 = params.final_a_c0123{8};
else
  error 'could not find final_a_c0123_8'
end
if isfield(params, 'final_a_c4567_3')
  final_a_c4567_3 = params.final_a_c4567_3;
elseif isfield(params, 'final_a_c4567')
  final_a_c4567_3 = params.final_a_c4567{3};
else
  error 'could not find final_a_c4567_3'
end
if isfield(params, 'final_a_c4567_4')
  final_a_c4567_4 = params.final_a_c4567_4;
elseif isfield(params, 'final_a_c4567')
  final_a_c4567_4 = params.final_a_c4567{4};
else
  error 'could not find final_a_c4567_4'
end
if isfield(params, 'final_a_c4567_5')
  final_a_c4567_5 = params.final_a_c4567_5;
elseif isfield(params, 'final_a_c4567')
  final_a_c4567_5 = params.final_a_c4567{5};
else
  error 'could not find final_a_c4567_5'
end
if isfield(params, 'final_a_c4567_6')
  final_a_c4567_6 = params.final_a_c4567_6;
elseif isfield(params, 'final_a_c4567')
  final_a_c4567_6 = params.final_a_c4567{6};
else
  error 'could not find final_a_c4567_6'
end
if isfield(params, 'final_a_c4567_7')
  final_a_c4567_7 = params.final_a_c4567_7;
elseif isfield(params, 'final_a_c4567')
  final_a_c4567_7 = params.final_a_c4567{7};
else
  error 'could not find final_a_c4567_7'
end
if isfield(params, 'final_a_c4567_8')
  final_a_c4567_8 = params.final_a_c4567_8;
elseif isfield(params, 'final_a_c4567')
  final_a_c4567_8 = params.final_a_c4567{8};
else
  error 'could not find final_a_c4567_8'
end
if isfield(params, 'final_j_c0123_3')
  final_j_c0123_3 = params.final_j_c0123_3;
elseif isfield(params, 'final_j_c0123')
  final_j_c0123_3 = params.final_j_c0123{3};
else
  error 'could not find final_j_c0123_3'
end
if isfield(params, 'final_j_c0123_4')
  final_j_c0123_4 = params.final_j_c0123_4;
elseif isfield(params, 'final_j_c0123')
  final_j_c0123_4 = params.final_j_c0123{4};
else
  error 'could not find final_j_c0123_4'
end
if isfield(params, 'final_j_c0123_5')
  final_j_c0123_5 = params.final_j_c0123_5;
elseif isfield(params, 'final_j_c0123')
  final_j_c0123_5 = params.final_j_c0123{5};
else
  error 'could not find final_j_c0123_5'
end
if isfield(params, 'final_j_c0123_6')
  final_j_c0123_6 = params.final_j_c0123_6;
elseif isfield(params, 'final_j_c0123')
  final_j_c0123_6 = params.final_j_c0123{6};
else
  error 'could not find final_j_c0123_6'
end
if isfield(params, 'final_j_c0123_7')
  final_j_c0123_7 = params.final_j_c0123_7;
elseif isfield(params, 'final_j_c0123')
  final_j_c0123_7 = params.final_j_c0123{7};
else
  error 'could not find final_j_c0123_7'
end
if isfield(params, 'final_j_c0123_8')
  final_j_c0123_8 = params.final_j_c0123_8;
elseif isfield(params, 'final_j_c0123')
  final_j_c0123_8 = params.final_j_c0123{8};
else
  error 'could not find final_j_c0123_8'
end
if isfield(params, 'final_j_c4567_3')
  final_j_c4567_3 = params.final_j_c4567_3;
elseif isfield(params, 'final_j_c4567')
  final_j_c4567_3 = params.final_j_c4567{3};
else
  error 'could not find final_j_c4567_3'
end
if isfield(params, 'final_j_c4567_4')
  final_j_c4567_4 = params.final_j_c4567_4;
elseif isfield(params, 'final_j_c4567')
  final_j_c4567_4 = params.final_j_c4567{4};
else
  error 'could not find final_j_c4567_4'
end
if isfield(params, 'final_j_c4567_5')
  final_j_c4567_5 = params.final_j_c4567_5;
elseif isfield(params, 'final_j_c4567')
  final_j_c4567_5 = params.final_j_c4567{5};
else
  error 'could not find final_j_c4567_5'
end
if isfield(params, 'final_j_c4567_6')
  final_j_c4567_6 = params.final_j_c4567_6;
elseif isfield(params, 'final_j_c4567')
  final_j_c4567_6 = params.final_j_c4567{6};
else
  error 'could not find final_j_c4567_6'
end
if isfield(params, 'final_j_c4567_7')
  final_j_c4567_7 = params.final_j_c4567_7;
elseif isfield(params, 'final_j_c4567')
  final_j_c4567_7 = params.final_j_c4567{7};
else
  error 'could not find final_j_c4567_7'
end
if isfield(params, 'final_j_c4567_8')
  final_j_c4567_8 = params.final_j_c4567_8;
elseif isfield(params, 'final_j_c4567')
  final_j_c4567_8 = params.final_j_c4567{8};
else
  error 'could not find final_j_c4567_8'
end
if isfield(params, 'final_s_c0123_3')
  final_s_c0123_3 = params.final_s_c0123_3;
elseif isfield(params, 'final_s_c0123')
  final_s_c0123_3 = params.final_s_c0123{3};
else
  error 'could not find final_s_c0123_3'
end
if isfield(params, 'final_s_c0123_4')
  final_s_c0123_4 = params.final_s_c0123_4;
elseif isfield(params, 'final_s_c0123')
  final_s_c0123_4 = params.final_s_c0123{4};
else
  error 'could not find final_s_c0123_4'
end
if isfield(params, 'final_s_c0123_5')
  final_s_c0123_5 = params.final_s_c0123_5;
elseif isfield(params, 'final_s_c0123')
  final_s_c0123_5 = params.final_s_c0123{5};
else
  error 'could not find final_s_c0123_5'
end
if isfield(params, 'final_s_c0123_6')
  final_s_c0123_6 = params.final_s_c0123_6;
elseif isfield(params, 'final_s_c0123')
  final_s_c0123_6 = params.final_s_c0123{6};
else
  error 'could not find final_s_c0123_6'
end
if isfield(params, 'final_s_c0123_7')
  final_s_c0123_7 = params.final_s_c0123_7;
elseif isfield(params, 'final_s_c0123')
  final_s_c0123_7 = params.final_s_c0123{7};
else
  error 'could not find final_s_c0123_7'
end
if isfield(params, 'final_s_c0123_8')
  final_s_c0123_8 = params.final_s_c0123_8;
elseif isfield(params, 'final_s_c0123')
  final_s_c0123_8 = params.final_s_c0123{8};
else
  error 'could not find final_s_c0123_8'
end
if isfield(params, 'final_s_c4567_3')
  final_s_c4567_3 = params.final_s_c4567_3;
elseif isfield(params, 'final_s_c4567')
  final_s_c4567_3 = params.final_s_c4567{3};
else
  error 'could not find final_s_c4567_3'
end
if isfield(params, 'final_s_c4567_4')
  final_s_c4567_4 = params.final_s_c4567_4;
elseif isfield(params, 'final_s_c4567')
  final_s_c4567_4 = params.final_s_c4567{4};
else
  error 'could not find final_s_c4567_4'
end
if isfield(params, 'final_s_c4567_5')
  final_s_c4567_5 = params.final_s_c4567_5;
elseif isfield(params, 'final_s_c4567')
  final_s_c4567_5 = params.final_s_c4567{5};
else
  error 'could not find final_s_c4567_5'
end
if isfield(params, 'final_s_c4567_6')
  final_s_c4567_6 = params.final_s_c4567_6;
elseif isfield(params, 'final_s_c4567')
  final_s_c4567_6 = params.final_s_c4567{6};
else
  error 'could not find final_s_c4567_6'
end
if isfield(params, 'final_s_c4567_7')
  final_s_c4567_7 = params.final_s_c4567_7;
elseif isfield(params, 'final_s_c4567')
  final_s_c4567_7 = params.final_s_c4567{7};
else
  error 'could not find final_s_c4567_7'
end
if isfield(params, 'final_s_c4567_8')
  final_s_c4567_8 = params.final_s_c4567_8;
elseif isfield(params, 'final_s_c4567')
  final_s_c4567_8 = params.final_s_c4567{8};
else
  error 'could not find final_s_c4567_8'
end
if isfield(params, 'final_v_c0123_3')
  final_v_c0123_3 = params.final_v_c0123_3;
elseif isfield(params, 'final_v_c0123')
  final_v_c0123_3 = params.final_v_c0123{3};
else
  error 'could not find final_v_c0123_3'
end
if isfield(params, 'final_v_c0123_4')
  final_v_c0123_4 = params.final_v_c0123_4;
elseif isfield(params, 'final_v_c0123')
  final_v_c0123_4 = params.final_v_c0123{4};
else
  error 'could not find final_v_c0123_4'
end
if isfield(params, 'final_v_c0123_5')
  final_v_c0123_5 = params.final_v_c0123_5;
elseif isfield(params, 'final_v_c0123')
  final_v_c0123_5 = params.final_v_c0123{5};
else
  error 'could not find final_v_c0123_5'
end
if isfield(params, 'final_v_c0123_6')
  final_v_c0123_6 = params.final_v_c0123_6;
elseif isfield(params, 'final_v_c0123')
  final_v_c0123_6 = params.final_v_c0123{6};
else
  error 'could not find final_v_c0123_6'
end
if isfield(params, 'final_v_c0123_7')
  final_v_c0123_7 = params.final_v_c0123_7;
elseif isfield(params, 'final_v_c0123')
  final_v_c0123_7 = params.final_v_c0123{7};
else
  error 'could not find final_v_c0123_7'
end
if isfield(params, 'final_v_c0123_8')
  final_v_c0123_8 = params.final_v_c0123_8;
elseif isfield(params, 'final_v_c0123')
  final_v_c0123_8 = params.final_v_c0123{8};
else
  error 'could not find final_v_c0123_8'
end
if isfield(params, 'final_v_c4567_3')
  final_v_c4567_3 = params.final_v_c4567_3;
elseif isfield(params, 'final_v_c4567')
  final_v_c4567_3 = params.final_v_c4567{3};
else
  error 'could not find final_v_c4567_3'
end
if isfield(params, 'final_v_c4567_4')
  final_v_c4567_4 = params.final_v_c4567_4;
elseif isfield(params, 'final_v_c4567')
  final_v_c4567_4 = params.final_v_c4567{4};
else
  error 'could not find final_v_c4567_4'
end
if isfield(params, 'final_v_c4567_5')
  final_v_c4567_5 = params.final_v_c4567_5;
elseif isfield(params, 'final_v_c4567')
  final_v_c4567_5 = params.final_v_c4567{5};
else
  error 'could not find final_v_c4567_5'
end
if isfield(params, 'final_v_c4567_6')
  final_v_c4567_6 = params.final_v_c4567_6;
elseif isfield(params, 'final_v_c4567')
  final_v_c4567_6 = params.final_v_c4567{6};
else
  error 'could not find final_v_c4567_6'
end
if isfield(params, 'final_v_c4567_7')
  final_v_c4567_7 = params.final_v_c4567_7;
elseif isfield(params, 'final_v_c4567')
  final_v_c4567_7 = params.final_v_c4567{7};
else
  error 'could not find final_v_c4567_7'
end
if isfield(params, 'final_v_c4567_8')
  final_v_c4567_8 = params.final_v_c4567_8;
elseif isfield(params, 'final_v_c4567')
  final_v_c4567_8 = params.final_v_c4567{8};
else
  error 'could not find final_v_c4567_8'
end
if isfield(params, 'j_cont_c0123_1')
  j_cont_c0123_1 = params.j_cont_c0123_1;
elseif isfield(params, 'j_cont_c0123')
  j_cont_c0123_1 = params.j_cont_c0123{1};
else
  error 'could not find j_cont_c0123_1'
end
if isfield(params, 'j_cont_c0123_2')
  j_cont_c0123_2 = params.j_cont_c0123_2;
elseif isfield(params, 'j_cont_c0123')
  j_cont_c0123_2 = params.j_cont_c0123{2};
else
  error 'could not find j_cont_c0123_2'
end
if isfield(params, 'j_cont_c0123_3')
  j_cont_c0123_3 = params.j_cont_c0123_3;
elseif isfield(params, 'j_cont_c0123')
  j_cont_c0123_3 = params.j_cont_c0123{3};
else
  error 'could not find j_cont_c0123_3'
end
if isfield(params, 'j_cont_c0123_4')
  j_cont_c0123_4 = params.j_cont_c0123_4;
elseif isfield(params, 'j_cont_c0123')
  j_cont_c0123_4 = params.j_cont_c0123{4};
else
  error 'could not find j_cont_c0123_4'
end
if isfield(params, 'j_cont_c0123_5')
  j_cont_c0123_5 = params.j_cont_c0123_5;
elseif isfield(params, 'j_cont_c0123')
  j_cont_c0123_5 = params.j_cont_c0123{5};
else
  error 'could not find j_cont_c0123_5'
end
if isfield(params, 'j_cont_c0123_6')
  j_cont_c0123_6 = params.j_cont_c0123_6;
elseif isfield(params, 'j_cont_c0123')
  j_cont_c0123_6 = params.j_cont_c0123{6};
else
  error 'could not find j_cont_c0123_6'
end
if isfield(params, 'j_cont_c0123_7')
  j_cont_c0123_7 = params.j_cont_c0123_7;
elseif isfield(params, 'j_cont_c0123')
  j_cont_c0123_7 = params.j_cont_c0123{7};
else
  error 'could not find j_cont_c0123_7'
end
if isfield(params, 'j_cont_c4567_1')
  j_cont_c4567_1 = params.j_cont_c4567_1;
elseif isfield(params, 'j_cont_c4567')
  j_cont_c4567_1 = params.j_cont_c4567{1};
else
  error 'could not find j_cont_c4567_1'
end
if isfield(params, 'j_cont_c4567_2')
  j_cont_c4567_2 = params.j_cont_c4567_2;
elseif isfield(params, 'j_cont_c4567')
  j_cont_c4567_2 = params.j_cont_c4567{2};
else
  error 'could not find j_cont_c4567_2'
end
if isfield(params, 'j_cont_c4567_3')
  j_cont_c4567_3 = params.j_cont_c4567_3;
elseif isfield(params, 'j_cont_c4567')
  j_cont_c4567_3 = params.j_cont_c4567{3};
else
  error 'could not find j_cont_c4567_3'
end
if isfield(params, 'j_cont_c4567_4')
  j_cont_c4567_4 = params.j_cont_c4567_4;
elseif isfield(params, 'j_cont_c4567')
  j_cont_c4567_4 = params.j_cont_c4567{4};
else
  error 'could not find j_cont_c4567_4'
end
if isfield(params, 'j_cont_c4567_5')
  j_cont_c4567_5 = params.j_cont_c4567_5;
elseif isfield(params, 'j_cont_c4567')
  j_cont_c4567_5 = params.j_cont_c4567{5};
else
  error 'could not find j_cont_c4567_5'
end
if isfield(params, 'j_cont_c4567_6')
  j_cont_c4567_6 = params.j_cont_c4567_6;
elseif isfield(params, 'j_cont_c4567')
  j_cont_c4567_6 = params.j_cont_c4567{6};
else
  error 'could not find j_cont_c4567_6'
end
if isfield(params, 'j_cont_c4567_7')
  j_cont_c4567_7 = params.j_cont_c4567_7;
elseif isfield(params, 'j_cont_c4567')
  j_cont_c4567_7 = params.j_cont_c4567{7};
else
  error 'could not find j_cont_c4567_7'
end
if isfield(params, 's_cont_c0123_1')
  s_cont_c0123_1 = params.s_cont_c0123_1;
elseif isfield(params, 's_cont_c0123')
  s_cont_c0123_1 = params.s_cont_c0123{1};
else
  error 'could not find s_cont_c0123_1'
end
if isfield(params, 's_cont_c0123_2')
  s_cont_c0123_2 = params.s_cont_c0123_2;
elseif isfield(params, 's_cont_c0123')
  s_cont_c0123_2 = params.s_cont_c0123{2};
else
  error 'could not find s_cont_c0123_2'
end
if isfield(params, 's_cont_c0123_3')
  s_cont_c0123_3 = params.s_cont_c0123_3;
elseif isfield(params, 's_cont_c0123')
  s_cont_c0123_3 = params.s_cont_c0123{3};
else
  error 'could not find s_cont_c0123_3'
end
if isfield(params, 's_cont_c0123_4')
  s_cont_c0123_4 = params.s_cont_c0123_4;
elseif isfield(params, 's_cont_c0123')
  s_cont_c0123_4 = params.s_cont_c0123{4};
else
  error 'could not find s_cont_c0123_4'
end
if isfield(params, 's_cont_c0123_5')
  s_cont_c0123_5 = params.s_cont_c0123_5;
elseif isfield(params, 's_cont_c0123')
  s_cont_c0123_5 = params.s_cont_c0123{5};
else
  error 'could not find s_cont_c0123_5'
end
if isfield(params, 's_cont_c0123_6')
  s_cont_c0123_6 = params.s_cont_c0123_6;
elseif isfield(params, 's_cont_c0123')
  s_cont_c0123_6 = params.s_cont_c0123{6};
else
  error 'could not find s_cont_c0123_6'
end
if isfield(params, 's_cont_c0123_7')
  s_cont_c0123_7 = params.s_cont_c0123_7;
elseif isfield(params, 's_cont_c0123')
  s_cont_c0123_7 = params.s_cont_c0123{7};
else
  error 'could not find s_cont_c0123_7'
end
if isfield(params, 's_cont_c4567_1')
  s_cont_c4567_1 = params.s_cont_c4567_1;
elseif isfield(params, 's_cont_c4567')
  s_cont_c4567_1 = params.s_cont_c4567{1};
else
  error 'could not find s_cont_c4567_1'
end
if isfield(params, 's_cont_c4567_2')
  s_cont_c4567_2 = params.s_cont_c4567_2;
elseif isfield(params, 's_cont_c4567')
  s_cont_c4567_2 = params.s_cont_c4567{2};
else
  error 'could not find s_cont_c4567_2'
end
if isfield(params, 's_cont_c4567_3')
  s_cont_c4567_3 = params.s_cont_c4567_3;
elseif isfield(params, 's_cont_c4567')
  s_cont_c4567_3 = params.s_cont_c4567{3};
else
  error 'could not find s_cont_c4567_3'
end
if isfield(params, 's_cont_c4567_4')
  s_cont_c4567_4 = params.s_cont_c4567_4;
elseif isfield(params, 's_cont_c4567')
  s_cont_c4567_4 = params.s_cont_c4567{4};
else
  error 'could not find s_cont_c4567_4'
end
if isfield(params, 's_cont_c4567_5')
  s_cont_c4567_5 = params.s_cont_c4567_5;
elseif isfield(params, 's_cont_c4567')
  s_cont_c4567_5 = params.s_cont_c4567{5};
else
  error 'could not find s_cont_c4567_5'
end
if isfield(params, 's_cont_c4567_6')
  s_cont_c4567_6 = params.s_cont_c4567_6;
elseif isfield(params, 's_cont_c4567')
  s_cont_c4567_6 = params.s_cont_c4567{6};
else
  error 'could not find s_cont_c4567_6'
end
if isfield(params, 's_cont_c4567_7')
  s_cont_c4567_7 = params.s_cont_c4567_7;
elseif isfield(params, 's_cont_c4567')
  s_cont_c4567_7 = params.s_cont_c4567{7};
else
  error 'could not find s_cont_c4567_7'
end
if isfield(params, 'tf0123_1')
  tf0123_1 = params.tf0123_1;
elseif isfield(params, 'tf0123')
  tf0123_1 = params.tf0123{1};
else
  error 'could not find tf0123_1'
end
if isfield(params, 'tf0123_2')
  tf0123_2 = params.tf0123_2;
elseif isfield(params, 'tf0123')
  tf0123_2 = params.tf0123{2};
else
  error 'could not find tf0123_2'
end
if isfield(params, 'tf0123_3')
  tf0123_3 = params.tf0123_3;
elseif isfield(params, 'tf0123')
  tf0123_3 = params.tf0123{3};
else
  error 'could not find tf0123_3'
end
if isfield(params, 'tf0123_4')
  tf0123_4 = params.tf0123_4;
elseif isfield(params, 'tf0123')
  tf0123_4 = params.tf0123{4};
else
  error 'could not find tf0123_4'
end
if isfield(params, 'tf0123_5')
  tf0123_5 = params.tf0123_5;
elseif isfield(params, 'tf0123')
  tf0123_5 = params.tf0123{5};
else
  error 'could not find tf0123_5'
end
if isfield(params, 'tf0123_6')
  tf0123_6 = params.tf0123_6;
elseif isfield(params, 'tf0123')
  tf0123_6 = params.tf0123{6};
else
  error 'could not find tf0123_6'
end
if isfield(params, 'tf0123_7')
  tf0123_7 = params.tf0123_7;
elseif isfield(params, 'tf0123')
  tf0123_7 = params.tf0123{7};
else
  error 'could not find tf0123_7'
end
if isfield(params, 'tf0123_8')
  tf0123_8 = params.tf0123_8;
elseif isfield(params, 'tf0123')
  tf0123_8 = params.tf0123{8};
else
  error 'could not find tf0123_8'
end
if isfield(params, 'tf4567_1')
  tf4567_1 = params.tf4567_1;
elseif isfield(params, 'tf4567')
  tf4567_1 = params.tf4567{1};
else
  error 'could not find tf4567_1'
end
if isfield(params, 'tf4567_2')
  tf4567_2 = params.tf4567_2;
elseif isfield(params, 'tf4567')
  tf4567_2 = params.tf4567{2};
else
  error 'could not find tf4567_2'
end
if isfield(params, 'tf4567_3')
  tf4567_3 = params.tf4567_3;
elseif isfield(params, 'tf4567')
  tf4567_3 = params.tf4567{3};
else
  error 'could not find tf4567_3'
end
if isfield(params, 'tf4567_4')
  tf4567_4 = params.tf4567_4;
elseif isfield(params, 'tf4567')
  tf4567_4 = params.tf4567{4};
else
  error 'could not find tf4567_4'
end
if isfield(params, 'tf4567_5')
  tf4567_5 = params.tf4567_5;
elseif isfield(params, 'tf4567')
  tf4567_5 = params.tf4567{5};
else
  error 'could not find tf4567_5'
end
if isfield(params, 'tf4567_6')
  tf4567_6 = params.tf4567_6;
elseif isfield(params, 'tf4567')
  tf4567_6 = params.tf4567{6};
else
  error 'could not find tf4567_6'
end
if isfield(params, 'tf4567_7')
  tf4567_7 = params.tf4567_7;
elseif isfield(params, 'tf4567')
  tf4567_7 = params.tf4567{7};
else
  error 'could not find tf4567_7'
end
if isfield(params, 'tf4567_8')
  tf4567_8 = params.tf4567_8;
elseif isfield(params, 'tf4567')
  tf4567_8 = params.tf4567{8};
else
  error 'could not find tf4567_8'
end
if isfield(params, 'v_cont_c0123_1')
  v_cont_c0123_1 = params.v_cont_c0123_1;
elseif isfield(params, 'v_cont_c0123')
  v_cont_c0123_1 = params.v_cont_c0123{1};
else
  error 'could not find v_cont_c0123_1'
end
if isfield(params, 'v_cont_c0123_2')
  v_cont_c0123_2 = params.v_cont_c0123_2;
elseif isfield(params, 'v_cont_c0123')
  v_cont_c0123_2 = params.v_cont_c0123{2};
else
  error 'could not find v_cont_c0123_2'
end
if isfield(params, 'v_cont_c0123_3')
  v_cont_c0123_3 = params.v_cont_c0123_3;
elseif isfield(params, 'v_cont_c0123')
  v_cont_c0123_3 = params.v_cont_c0123{3};
else
  error 'could not find v_cont_c0123_3'
end
if isfield(params, 'v_cont_c0123_4')
  v_cont_c0123_4 = params.v_cont_c0123_4;
elseif isfield(params, 'v_cont_c0123')
  v_cont_c0123_4 = params.v_cont_c0123{4};
else
  error 'could not find v_cont_c0123_4'
end
if isfield(params, 'v_cont_c0123_5')
  v_cont_c0123_5 = params.v_cont_c0123_5;
elseif isfield(params, 'v_cont_c0123')
  v_cont_c0123_5 = params.v_cont_c0123{5};
else
  error 'could not find v_cont_c0123_5'
end
if isfield(params, 'v_cont_c0123_6')
  v_cont_c0123_6 = params.v_cont_c0123_6;
elseif isfield(params, 'v_cont_c0123')
  v_cont_c0123_6 = params.v_cont_c0123{6};
else
  error 'could not find v_cont_c0123_6'
end
if isfield(params, 'v_cont_c0123_7')
  v_cont_c0123_7 = params.v_cont_c0123_7;
elseif isfield(params, 'v_cont_c0123')
  v_cont_c0123_7 = params.v_cont_c0123{7};
else
  error 'could not find v_cont_c0123_7'
end
if isfield(params, 'v_cont_c4567_1')
  v_cont_c4567_1 = params.v_cont_c4567_1;
elseif isfield(params, 'v_cont_c4567')
  v_cont_c4567_1 = params.v_cont_c4567{1};
else
  error 'could not find v_cont_c4567_1'
end
if isfield(params, 'v_cont_c4567_2')
  v_cont_c4567_2 = params.v_cont_c4567_2;
elseif isfield(params, 'v_cont_c4567')
  v_cont_c4567_2 = params.v_cont_c4567{2};
else
  error 'could not find v_cont_c4567_2'
end
if isfield(params, 'v_cont_c4567_3')
  v_cont_c4567_3 = params.v_cont_c4567_3;
elseif isfield(params, 'v_cont_c4567')
  v_cont_c4567_3 = params.v_cont_c4567{3};
else
  error 'could not find v_cont_c4567_3'
end
if isfield(params, 'v_cont_c4567_4')
  v_cont_c4567_4 = params.v_cont_c4567_4;
elseif isfield(params, 'v_cont_c4567')
  v_cont_c4567_4 = params.v_cont_c4567{4};
else
  error 'could not find v_cont_c4567_4'
end
if isfield(params, 'v_cont_c4567_5')
  v_cont_c4567_5 = params.v_cont_c4567_5;
elseif isfield(params, 'v_cont_c4567')
  v_cont_c4567_5 = params.v_cont_c4567{5};
else
  error 'could not find v_cont_c4567_5'
end
if isfield(params, 'v_cont_c4567_6')
  v_cont_c4567_6 = params.v_cont_c4567_6;
elseif isfield(params, 'v_cont_c4567')
  v_cont_c4567_6 = params.v_cont_c4567{6};
else
  error 'could not find v_cont_c4567_6'
end
if isfield(params, 'v_cont_c4567_7')
  v_cont_c4567_7 = params.v_cont_c4567_7;
elseif isfield(params, 'v_cont_c4567')
  v_cont_c4567_7 = params.v_cont_c4567{7};
else
  error 'could not find v_cont_c4567_7'
end
wpts = params.wpts;
cvx_begin
  % Caution: automatically generated by cvxgen. May be incorrect.
  variable c4567_1(4, 1);
  variable c4567_2(4, 1);
  variable c4567_3(4, 1);
  variable c4567_4(4, 1);
  variable c4567_5(4, 1);
  variable c4567_6(4, 1);
  variable c4567_7(4, 1);
  variable c4567_8(4, 1);
  variable c0123_1(4, 1);
  variable c0123_2(4, 1);
  variable c0123_3(4, 1);
  variable c0123_4(4, 1);
  variable c0123_5(4, 1);
  variable c0123_6(4, 1);
  variable c0123_7(4, 1);
  variable c0123_8(4, 1);

  minimize(quad_form(c4567_1, Q_1) + quad_form(c4567_2, Q_2) + quad_form(c4567_3, Q_3) + quad_form(c4567_4, Q_4) + quad_form(c4567_5, Q_5) + quad_form(c4567_6, Q_6) + quad_form(c4567_7, Q_7) + quad_form(c4567_8, Q_8));
  subject to
    c0123_1(1) == wpts(1);
    tf0123_1'*c0123_1 + tf4567_1'*c4567_1 == wpts(2);
    c0123_2(1) == wpts(2);
    tf0123_2'*c0123_2 + tf4567_2'*c4567_2 == wpts(3);
    c0123_3(1) == wpts(3);
    tf0123_3'*c0123_3 + tf4567_3'*c4567_3 == wpts(4);
    c0123_4(1) == wpts(4);
    tf0123_4'*c0123_4 + tf4567_4'*c4567_4 == wpts(5);
    c0123_5(1) == wpts(5);
    tf0123_5'*c0123_5 + tf4567_5'*c4567_5 == wpts(6);
    c0123_6(1) == wpts(6);
    tf0123_6'*c0123_6 + tf4567_6'*c4567_6 == wpts(7);
    c0123_7(1) == wpts(7);
    tf0123_7'*c0123_7 + tf4567_7'*c4567_7 == wpts(8);
    c0123_8(1) == wpts(8);
    tf0123_8'*c0123_8 + tf4567_8'*c4567_8 == wpts(9);
    c0123_1(2) == 0;
    2*c0123_1(3) == 0;
    6*c0123_1(4) == 0;
    24*c4567_1(1) == 0;
    final_v_c0123_3'*c0123_3 + final_v_c4567_3'*c4567_3 == 0;
    final_a_c0123_3'*c0123_3 + final_a_c4567_3'*c4567_3 == 0;
    final_j_c0123_3'*c0123_3 + final_j_c4567_3'*c4567_3 == 0;
    final_s_c0123_3'*c0123_3 + final_s_c4567_3'*c4567_3 == 0;
    final_v_c0123_4'*c0123_4 + final_v_c4567_4'*c4567_4 == 0;
    final_a_c0123_4'*c0123_4 + final_a_c4567_4'*c4567_4 == 0;
    final_j_c0123_4'*c0123_4 + final_j_c4567_4'*c4567_4 == 0;
    final_s_c0123_4'*c0123_4 + final_s_c4567_4'*c4567_4 == 0;
    final_v_c0123_5'*c0123_5 + final_v_c4567_5'*c4567_5 == 0;
    final_a_c0123_5'*c0123_5 + final_a_c4567_5'*c4567_5 == 0;
    final_j_c0123_5'*c0123_5 + final_j_c4567_5'*c4567_5 == 0;
    final_s_c0123_5'*c0123_5 + final_s_c4567_5'*c4567_5 == 0;
    final_v_c0123_6'*c0123_6 + final_v_c4567_6'*c4567_6 == 0;
    final_a_c0123_6'*c0123_6 + final_a_c4567_6'*c4567_6 == 0;
    final_j_c0123_6'*c0123_6 + final_j_c4567_6'*c4567_6 == 0;
    final_s_c0123_6'*c0123_6 + final_s_c4567_6'*c4567_6 == 0;
    final_v_c0123_7'*c0123_7 + final_v_c4567_7'*c4567_7 == 0;
    final_a_c0123_7'*c0123_7 + final_a_c4567_7'*c4567_7 == 0;
    final_j_c0123_7'*c0123_7 + final_j_c4567_7'*c4567_7 == 0;
    final_s_c0123_7'*c0123_7 + final_s_c4567_7'*c4567_7 == 0;
    final_v_c0123_8'*c0123_8 + final_v_c4567_8'*c4567_8 == 0;
    final_a_c0123_8'*c0123_8 + final_a_c4567_8'*c4567_8 == 0;
    final_j_c0123_8'*c0123_8 + final_j_c4567_8'*c4567_8 == 0;
    final_s_c0123_8'*c0123_8 + final_s_c4567_8'*c4567_8 == 0;
    v_cont_c0123_1'*c0123_1 + v_cont_c4567_1'*c4567_1 == c0123_2(2);
    a_cont_c0123_1'*c0123_1 + a_cont_c4567_1'*c4567_1 == 2*c0123_2(3);
    j_cont_c0123_1'*c0123_1 + j_cont_c4567_1'*c4567_1 == 6*c0123_2(4);
    s_cont_c0123_1'*c0123_1 + s_cont_c4567_1'*c4567_1 == 24*c4567_2(1);
    ds_cont_c0123_1'*c0123_1 + ds_cont_c4567_1'*c4567_1 == 120*c4567_2(2);
    v_cont_c0123_2'*c0123_2 + v_cont_c4567_2'*c4567_2 == c0123_3(2);
    a_cont_c0123_2'*c0123_2 + a_cont_c4567_2'*c4567_2 == 2*c0123_3(3);
    j_cont_c0123_2'*c0123_2 + j_cont_c4567_2'*c4567_2 == 6*c0123_3(4);
    s_cont_c0123_2'*c0123_2 + s_cont_c4567_2'*c4567_2 == 24*c4567_3(1);
    ds_cont_c0123_2'*c0123_2 + ds_cont_c4567_2'*c4567_2 == 120*c4567_3(2);
    v_cont_c0123_3'*c0123_3 + v_cont_c4567_3'*c4567_3 == c0123_4(2);
    a_cont_c0123_3'*c0123_3 + a_cont_c4567_3'*c4567_3 == 2*c0123_4(3);
    j_cont_c0123_3'*c0123_3 + j_cont_c4567_3'*c4567_3 == 6*c0123_4(4);
    s_cont_c0123_3'*c0123_3 + s_cont_c4567_3'*c4567_3 == 24*c4567_4(1);
    ds_cont_c0123_3'*c0123_3 + ds_cont_c4567_3'*c4567_3 == 120*c4567_4(2);
    v_cont_c0123_4'*c0123_4 + v_cont_c4567_4'*c4567_4 == c0123_5(2);
    a_cont_c0123_4'*c0123_4 + a_cont_c4567_4'*c4567_4 == 2*c0123_5(3);
    j_cont_c0123_4'*c0123_4 + j_cont_c4567_4'*c4567_4 == 6*c0123_5(4);
    s_cont_c0123_4'*c0123_4 + s_cont_c4567_4'*c4567_4 == 24*c4567_5(1);
    ds_cont_c0123_4'*c0123_4 + ds_cont_c4567_4'*c4567_4 == 120*c4567_5(2);
    v_cont_c0123_5'*c0123_5 + v_cont_c4567_5'*c4567_5 == c0123_6(2);
    a_cont_c0123_5'*c0123_5 + a_cont_c4567_5'*c4567_5 == 2*c0123_6(3);
    j_cont_c0123_5'*c0123_5 + j_cont_c4567_5'*c4567_5 == 6*c0123_6(4);
    s_cont_c0123_5'*c0123_5 + s_cont_c4567_5'*c4567_5 == 24*c4567_6(1);
    ds_cont_c0123_5'*c0123_5 + ds_cont_c4567_5'*c4567_5 == 120*c4567_6(2);
    v_cont_c0123_6'*c0123_6 + v_cont_c4567_6'*c4567_6 == c0123_7(2);
    a_cont_c0123_6'*c0123_6 + a_cont_c4567_6'*c4567_6 == 2*c0123_7(3);
    j_cont_c0123_6'*c0123_6 + j_cont_c4567_6'*c4567_6 == 6*c0123_7(4);
    s_cont_c0123_6'*c0123_6 + s_cont_c4567_6'*c4567_6 == 24*c4567_7(1);
    ds_cont_c0123_6'*c0123_6 + ds_cont_c4567_6'*c4567_6 == 120*c4567_7(2);
    v_cont_c0123_7'*c0123_7 + v_cont_c4567_7'*c4567_7 == c0123_8(2);
    a_cont_c0123_7'*c0123_7 + a_cont_c4567_7'*c4567_7 == 2*c0123_8(3);
    j_cont_c0123_7'*c0123_7 + j_cont_c4567_7'*c4567_7 == 6*c0123_8(4);
    s_cont_c0123_7'*c0123_7 + s_cont_c4567_7'*c4567_7 == 24*c4567_8(1);
    ds_cont_c0123_7'*c0123_7 + ds_cont_c4567_7'*c4567_7 == 120*c4567_8(2);
cvx_end
vars.c0123_1 = c0123_1;
vars.c0123{1} = c0123_1;
vars.c0123_2 = c0123_2;
vars.c0123{2} = c0123_2;
vars.c0123_3 = c0123_3;
vars.c0123{3} = c0123_3;
vars.c0123_4 = c0123_4;
vars.c0123{4} = c0123_4;
vars.c0123_5 = c0123_5;
vars.c0123{5} = c0123_5;
vars.c0123_6 = c0123_6;
vars.c0123{6} = c0123_6;
vars.c0123_7 = c0123_7;
vars.c0123{7} = c0123_7;
vars.c0123_8 = c0123_8;
vars.c0123{8} = c0123_8;
vars.c4567_1 = c4567_1;
vars.c4567{1} = c4567_1;
vars.c4567_2 = c4567_2;
vars.c4567{2} = c4567_2;
vars.c4567_3 = c4567_3;
vars.c4567{3} = c4567_3;
vars.c4567_4 = c4567_4;
vars.c4567{4} = c4567_4;
vars.c4567_5 = c4567_5;
vars.c4567{5} = c4567_5;
vars.c4567_6 = c4567_6;
vars.c4567{6} = c4567_6;
vars.c4567_7 = c4567_7;
vars.c4567{7} = c4567_7;
vars.c4567_8 = c4567_8;
vars.c4567{8} = c4567_8;
status.cvx_status = cvx_status;
% Provide a drop-in replacement for csolve.
status.optval = cvx_optval;
status.converged = strcmp(cvx_status, 'Solved');
