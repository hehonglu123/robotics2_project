%
% s-curve velocity profile generator
%
% input:
%       xo = initial position
%       xf = final position
%       vo = initial velocity
%       vf = final velocity
%       vmax = max velocity
%       amax = max acceleration
%       dmax = max deceleration (positive number)
%       pa1 = portion of acceleration for ramp up
%       pa2 = portion of acceleration for ramp down
%       pb1 = portion of deceleration for ramp up
%       pb2 = portion of deceleration for ramp down
%       t = current time
%
% output:
%       x = position at time t
%       v = velocity at time t
%       a = acceleration at time t
%       ta = first switching time (from max accel to zero accel)
%       tb = second switching time (from zero accel to max decel)
%       tf = final arrival time
%
function [x,v,a,ta,tb,tf]=scurvegen(xo,xf,vo,vf,vmax,amax,dmax,...
      pa1,pa2,pb1,pb2,t)

if (vo>vmax)|(vf>vmax)
  error('vo or vf greater than vmax! ');
end

vmax=sign(xf-xo)*vmax;

if xf>xo
  am1=abs(amax);
  am2=-abs(dmax);
else
  am1=-abs(dmax);
  am2=abs(amax);
end

ta = abs(2*(vmax-vo)/(2-pa1-pa2)/am1);
ta1=pa1*ta; ta2=(1-pa2)*ta;
tf_tb = abs(2*(vmax-vf)/(2-pb1-pb2)/am2);
tb1_tb=pb1*tf_tb; tf_tb2=pb2*tf_tb;

term1=1/6*am1*(ta1^2-ta2^2+2*ta^2+2*ta*ta2-3*ta1*ta);
alpha=.5*am1*(ta2-ta1+ta);
term3=1/6*am2*((tb1_tb)^2 + 3*(-tf_tb2+tf_tb-tb1_tb)*(-tf_tb2+tf_tb)+...
    (tf_tb2)*(6*(-tf_tb2+tf_tb-(tb1_tb)/2)+2*(tf_tb2)));

tb=(xf-xo-(vo+alpha)*tf_tb-term1-term3+alpha*ta)/(alpha-vo);
tf=tf_tb+tb;

if tb<ta
  g1=am1/6*(pa1^2-(1-pa2)^2+2+2*(1-pa2)-3*pa1);
  g2=am2/6*((pb1^2 + 3*(-pb2+1-pb1)*(-pb2+1)+...
    pb2*(6*(-pb2+1-pb1/2)+2*pb2)));
  g3=am1/2*(2-pa2-pa1);
  alpha1 = abs(2/am1/(2-(pa1+pa2)));
  beta1  = abs(2/am2/(2-(pb1+pb2)));

  a=g1*alpha1^2+g2*beta1^2+g3*alpha1*beta1;
  b=-2*g1*alpha1^2*vo-2*g2*beta1^2*vf-g3*alpha1*beta1*(vo+vf)+...
      vo*(beta1+alpha1);
  c=g1*alpha1^2*vo^2+g2*beta1^2*vf^2+g3*alpha1*beta1*vo*vf+...
      xo-xf-vo*(beta1*vf+alpha1*vo);

  vm=roots([a b c]);vm=max(vm);

  ta=alpha1*(vm-vo);tb=ta;tf=tb+beta1*(vm-vf);
  tf_tb=tf-tb;tb1_tb=pb1*tf_tb; tf_tb2=pb2*tf_tb;
  ta1=pa1*ta;ta2=(1-pa2)*ta;

end
  
tb1=tb1_tb+tb;
tb2=tf-tf_tb2;

 if t<ta1 % 1 
   a=am1*t/ta1;
   v=.5*am1*t^2/ta1+vo;
   x=(1/6)*am1*t^3/ta1 + vo*t + xo;
 else
   v2o=.5*am1*ta1+vo;
   x2o=(1/6)*am1*ta1^2 + vo*ta1 + xo;
   if t<ta2 % 2
     a=am1;
     v=am1*(t-ta1)+v2o;
     x=.5*am1*(t-ta1)^2 + x2o + v2o*(t-ta1);
   else
     v3o=am1*(ta2-ta1)+v2o;
     x3o=.5*am1*(ta2-ta1)^2+x2o+v2o*(ta2-ta1);
     if t<ta % 3
       a=am1*(t-ta)/(ta2-ta);
       v=am1*(1-.5*(t-ta2)/(ta-ta2))*(t-ta2)+v3o;
       x=am1*(t-ta2)^2*(.5*(t-ta2)-(t-ta2)/(ta-ta2)/6)+x3o+v3o*(t-ta2);
       else
	 v4o=.5*am1*(ta-ta2)+v3o;
	 x4o=am1*(ta-ta2)^2/3+x3o+v3o*(ta-ta2);
	 if t<tb %4
	   a=0;
	   v=v4o;
	   x=v4o*(t-ta)+x4o;
	 else
	   v5o=v4o;
	   x5o=x4o+v4o*(tb-ta);
	   if t<tb1 %5
	     a=am2*(t-tb)/(tb1-tb);
	     v=.5*am2*(t-tb)^2/(tb1-tb)+v5o;
	     x=am2*(t-tb)^3/(tb1-tb)/6+v5o*(t-tb)+x5o;
	     else
	       v6o=.5*am2*(tb1-tb)+v5o;
	       x6o=am2*(tb1-tb)^2/6+x5o+v5o*(tb1-tb);
	       if t<tb2 %6
		 a=am2;
		 v=am2*(t-tb1)+v6o;
		 x=am2*.5*(t-tb1)^2+v6o*(t-tb1)+x6o;
	       else
		 v7o=am2*(tb2-tb1)+v6o;
		 x7o=.5*am2*(tb2-tb1)^2+x6o+v6o*(tb2-tb1);
		 if t<=tf %7
		   a=-am2*(t-tf)/(tf-tb2);
		   v=am2*(1-.5*(t-tb2)/(tf-tb2))*(t-tb2)+v7o;
		   x=am2*(t-tb2)^2*(.5-(t-tb2)/(tf-tb2)/6)+x7o+v7o*(t-tb2);
		 end
	       end
	     end
	   end
	 end
       end
     end

