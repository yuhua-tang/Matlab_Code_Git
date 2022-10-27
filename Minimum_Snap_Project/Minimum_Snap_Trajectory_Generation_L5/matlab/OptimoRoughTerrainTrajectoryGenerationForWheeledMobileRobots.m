%code is for your ref
clc
clear all
 close all
 global v ;
 global L ;
 global ds;
 global target;
 global cost_threshold;
 global h;
 global max_iter;
 v = 10;
 L = 2.9;
 cost_threshold = 0.1;
 target = [6;2;0];
%  target = [6;2;-pi/6];
%  target = [6;2;pi/6];
 init_p = [6;0;0];
 ds = 0.1;
 max_iter =  50;
 h = [0.5;0.02;0.02];
 k0 = 0;
 figure
 optimize_trajectory(target, k0, init_p,max_iter,cost_threshold,h );
 
 function optimize_trajectory(target, k0, init_p,max_iter,cost_threshold,h )
    for i = 1:max_iter
        i
       [X,Y,YAW] = generate_trajectory(init_p(1),init_p(2),init_p(3),k0);
       dc = cal_diff(target,X(end),Y(end),YAW(end));
       cost = norm(dc-[0;0;0]);
       if cost < cost_threshold
           break;
       end
       J = cal_j(target, init_p, h, k0);
       dp  = - inv(J) * dc';
       alpha = learning_descent_step(dp,init_p,k0, target);
       alpha = 0.1;
       p = init_p+ alpha * dp;
       init_p = p;
       plot(X,Y,'-')
       drawnow
       pause(0.1)
       hold on
    end
 end
 
 function diff = cal_diff(target,x,y,yaw)
    diff(1) = target(1) - x ;
    diff(2) = target(2) - y;
    diff(3) = target(3) - yaw;
    reshape(diff,[3,1]);
 end
 
 function alpha = learning_descent_step(dp,p,k0, target)
    mincost = Inf;
    alpha_max = 0.5;
    alpha_min = 0.1;
    for alpha = alpha_min:0.1:alpha_max
       tp = p + alpha * dp; 
       [x_end,y_end,yaw_end] = generate_last_state(tp(1),tp(2),tp(3),k0);
       dc = cal_diff(target,x_end,y_end,yaw_end);
       cost = norm(dc-[0;0;0]);
       if cost < mincost
           alpha_opt = alpha;
           mincost = cost;
       end
    end
    
 
 end
 
 function J = cal_j(target, p ,h, k0)
 %p = [s,km,kf];
 %h = [delta_s, delta_km, delta_kf];
    [x_end_p,y_end_p,yaw_end_p] = generate_last_state(p(1)+h(1),p(2),p(3),k0);
    dp = cal_diff(target,x_end_p,y_end_p,yaw_end_p);
    [x_end_n,y_end_n,yaw_end_n] = generate_last_state(p(1)-h(1),p(2),p(3),k0);
    dn = cal_diff(target,x_end_n,y_end_n,yaw_end_n);
    d1  = (dp - dn)/(2*h(1));
    reshape(d1,[3,1]);
     
    [x_end_p,y_end_p,yaw_end_p] = generate_last_state(p(1),p(2)+h(2),p(3),k0);
    dp = cal_diff(target,x_end_p,y_end_p,yaw_end_p);
    [x_end_n,y_end_n,yaw_end_n] = generate_last_state(p(1),p(2)-h(2),p(3),k0);
    dn = cal_diff(target,x_end_n,y_end_n,yaw_end_n);
    d2  = (dp - dn)/(2*h(2));
    reshape(d2,[3,1]);
    
    [x_end_p,y_end_p,yaw_end_p] = generate_last_state(p(1),p(2),p(3)+h(3),k0);
    dp = cal_diff(target,x_end_p,y_end_p,yaw_end_p);
    [x_end_n,y_end_n,yaw_end_n] = generate_last_state(p(1),p(2),p(3)-h(3),k0);
    dn = cal_diff(target,x_end_n,y_end_n,yaw_end_n);
    d3  = (dp - dn)/(2*h(3));
    reshape(d1,[3,1]);
     
    J = [d1',d2',d3'];
 end

 function [x, y, yaw,v] = update(x, y, yaw,v,delta,dt,L)
    x = x + v * cos(yaw) * dt;
    y = y + v * sin(yaw) * dt;
    yaw = yaw + v / L * tan(delta) * dt;
    v = v;
end

function [X,Y,YAW] = generate_trajectory(s,km,kf,k0)
    ds = 0.1; 
    L = 2.9;
    n = s/ds;
    v = 10/3.6;
    time = s /v;
    horizon = time/n;
    t_input = [0;time/2;time];
    x_input = [k0;km;kf];
    para = quadratic_interpolation(t_input,x_input);
    x=0;y=0;yaw = 0;
    X = [x];Y = [y]; YAW = [yaw];
    for t=0:horizon:time
         delta = interp_refer(para,t);
         [x,y,yaw,v] = update(x, y, yaw,v,delta,horizon,L);
         X = [X;x];
         Y = [Y;y];
         YAW = [YAW;yaw];
    end
end

function [x_end,y_end,yaw_end] = generate_last_state(s,km,kf,k0)
     ds = 0.1; 
    L = 2.9;
    n = s/ds;
    v = 10/3.6;
    time = s /v;
    horizon = time/n;
    t_input = [0;time/2;time];
    x_input = [k0;km;kf];
    para = quadratic_interpolation(t_input,x_input);
    x=0;y=0;yaw = 0;
    X = [x];Y = [y]; YAW = [yaw];
    for t=0:horizon:time
         delta = interp_refer(para,t);
         [x,y,yaw,v] = update(x, y, yaw,v,delta,horizon,L);
         X = [X;x];
         Y = [Y;y];
         YAW = [YAW;yaw];
    end
    x_end = X(end);
    y_end = Y(end);
    yaw_end = YAW(end);
end

function para  = quadratic_interpolation(x,y)
     A = [1, x(1) x(1)^2;1, x(2) x(2)^2;1, x(3) x(3)^2];
     Y = [y(1);y(2);y(3)];
     para = inv(A) * Y;
end

function sol = interp_refer(para, time)
    sol = para(1) +  para(2) *time +  para(3) *time*time;
end