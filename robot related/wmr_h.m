%
% wmr_h.m
%
% [qnext,utrue]=wmr_nonh(q,u,ts,wcov): discrete time evolution of a wheeled mobile robot
% 
% q=current state
% u=current input
% u1 = forward velocity
% u2 = rotational velocity
% ts=sampling period (s)
% Gaussian odometry noise (zero mean)
%
% qnext=next state
% utrue=true input (with noise and disturbance)
% the input "u" for Nonholonomic motion

function [qnext,utrue]=wmr_h(q,u,ts,wcov)

m=length(u);
w=zeros(size(u));
w=normrnd(w,wcov(1:m));
utrue=u+w;
qnext(1)=q(1)+utrue(1)*ts;
qnext(2)=q(2)+utrue(2)*ts;

end
