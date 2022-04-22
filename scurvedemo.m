%
% demonstration file for trapgen
%

% final/initial positions, initial/final velocities
xf=1;xo=0;vo=0;vf=0;
%xf=1;xo=0;vo=2;vf=2;

% max vel/accel/decel
% vmax=1;amax=5;dmax=5;
vmax=2;amax=5;dmax=5;

%portion of acceleration/deceleration for ramp up/down
pa1=.40;pa2=.40;
pb1=.40;pb2=.40;

% find the final time first
t=0;
[x,v,a,ta,tb,tf]=scurvegen(xo,xf,vo,vf,vmax,amax,dmax,...
    pa1,pa2,pb1,pb2,t);
disp(['tf = ',num2str(tf)]);

% generate complete trajectory
N=100;
t=(0:tf/N:tf);

for i=1:length(t)
  [x(i),v(i),a(i),ta,tb,tf]=scurvegen(xo,xf,vo,vf,vmax,amax,dmax,...
      pa1,pa2,pb1,pb2,t(i));  
end

% plot trajectory
figure(4);plot(t,a);title('a');
figure(5);plot(t,v);title('v');
figure(6);plot(t,x);title('x');


