% H = [1 -1
%      -1 2];
% f = [-2 -6]';
% b = [2 2 3]';
% A = [1 1;
%      -1 2;
%      2 1];
% LB = [0 0];
% [x,fval,exitflag] = quadprog(H,f,A,b,[],[],LB)

H = [0.6 2.1;
     2.1 6];
f = [-5 0]';
b = [5 2 3]';
A = [3 1;
     -1 2;
     2 -1];
LB = [0 0];
[x,fval,exitflag] = quadprog(H,f,A,b,[],[],LB)