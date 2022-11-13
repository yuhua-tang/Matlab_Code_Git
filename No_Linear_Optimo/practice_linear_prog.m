% f = [-1 2 -3 4]';
% b = [6;12;4];
% A = [1 1 3 -1;
%      2 3 -1 1;
%      1 0 2 1];
% Aeq = [1 1 1 1];
% beq = [2];
% LB = [0 0 0 0];
% [x,fval,exitflag] = linprog(f,A,b,Aeq,beq,LB)
 

f = [-5 -4 -6]';
b = [20 42 30]';
A = [1 -1 1;
     3 2 4;
     3 2 0];
LB = [1 1 1];
[x,fval,exitflag] = linprog(f,A,b,[],[],LB)