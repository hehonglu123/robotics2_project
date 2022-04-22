%
% demonstration file for trapzoidal velocity profile
%

% final/initial positions, initial/final velocities
xf=1;xo=0;vo=0;vf=0;
%xf=1;xo=0;vo=1;vf=2;

% max vel/accel/decel
%vmax=2;amax=4;dmax=1;
vmax=1;amax=5;dmax=5;

% find the final time first
[x,v,a,ta,tb,tf]=trapgen(xo,xf,vo,vf,vmax,amax,dmax,0);
disp(['tf = ',num2str(tf)]);

% generate complete trajectory
N=100;
t=(0:tf/N:tf);
for i=1:length(t);
  [x(i),v(i),a(i),ta,tb,tf]=trapgen(xo,xf,vo,vf,vmax,amax,dmax,t(i));  
end

% plot trajectory
figure(1);plot(t,a);title('a');
figure(2);plot(t,v);title('v');
figure(3);plot(t,x);title('x');

