%
% slam_kalman.m
%
% Given y and robot kinematics, estimate the robot position and orientation
% as well as the sensor locations using the Kalman filter
%
function [xhatnext,Pnext,Snext]=... 
    pose_est_slam(xhat,u,y,ts,N_scan,wcov,vcov,P,Sigma,robot,colobj)

N=length(xhat); %3q+3*4range+3*2bearing pose = 21 states
qhat=xhat(1:3);
[qbarnext,utrue]=wmr(qhat,u,ts,(zeros(size(wcov))));
xbarnext=xhat;
qbarnext=qbarnext';
xbarnext(1:3)=qbarnext;

yind=[1 2 3 4 7 8];     %only care about range & bearing observations for SLAM
A=eye(N,N);B=zeros(N,2);
A(1:3,1:3)=[1 0 -u(1)*sin(qhat(3));0 1 u(1)*cos(qhat(3)); 0 0 1];
B(1:3,1:2)=[cos(qhat(3)) 0 ; sin(qhat(3)) 0 ; 0 1 ];

V=diag(vcov(yind));
W=diag(wcov);

C=zeros(length(yind),size(A,1));
lW=colobj.obj{1}.X;
zW=0;
pL(:,1)=xhat(4:6);pL(:,2)=xhat(7:9);pL(:,3)=xhat(10:12);pL(:,4)=xhat(13:15);
pB(:,1)=xhat(16:18);pB(:,2)=xhat(19:21);

% C=[gradient_range(pL,qbarnext) [-gradient_range(pL(:,1),qbarnext);zeros(3,3)] [zeros(1,3); -gradient_range(pL(:,2),qbarnext); zeros(2,3)] [zeros(2,3); -gradient_range(pL(:,3),qbarnext); zeros(1,3)] [zeros(3,3); -gradient_range(pL(:,4),qbarnext)] zeros(4,6);
%    gradient_bearing(pB,qbarnext) zeros(2,12) [-gradient_bearing(pB(:,1),qbarnext);zeros(1,3)] [zeros(1,3); gradient_bearing(pB(:,2),qbarnext)]];

N_range=4;
N_bearing=2;
C=zeros(6,21);
C(1:N_range,1:3)=gradient_range(pL,qbarnext);
C(N_range+1:N_range+N_bearing,1:3)= gradient_bearing(pB,qbarnext);

C(1:N_range,4:4+3*N_range-1)=gradient_range_pL(pL,qhat);
C(N_range+1:N_range+N_bearing,16:21)=...
    gradient_bearing_pB(pB,qhat);

K=P*C'*inv(V);

ybar=h(qbarnext,pL,pB);



xhatnext=xbarnext+K*(y(yind)-ybar);
Pnext=inv(inv(Sigma)+C'*inv(V)*C);
Snext=A*P*A'+B*W*B';

end

% range and bearing sensor output map

function y=h(q,pL,pB)
    pR=[q(1:2); 0];
    y_range=vecnorm(pR-pL)';
    y_bearing=(atan2(pB(2,:)-q(2),pB(1,:)-q(1))-q(3))';
    y=[y_range;y_bearing];
end
% gradient of range sensor model

function grad_h=gradient_range(pL,q)
    pR=[q(1:2);0];
    grad_h=(-(pL-pR)./vecnorm(pL-pR))';    
end

% gradient of bearing sensor model

function grad_h=gradient_bearing(pB,q)
    pR=[q(1:2);0];
    grad_h=[([pB(2,:)'-pR(2) -(pB(1,:)'-pR(1))]'./vecnorm(pB(1:2,:)-pR(1:2)).^2)' ...
        -ones(size(pB,2),1)];
end


function grad_h=gradient_range_pL(pL,q)
    N_range=size(pL,2);
    pR=[q(1:2);0];
    grad_stack=((pL-pR)./vecnorm(pL-pR))'; 
    grad_h=zeros(N_range,3*N_range);
    for i=1:N_range;grad_h(i,(i-1)*3+1:3*i)=grad_stack(i,:);end
end

% gradient of bearing sensor model

function grad_h=gradient_bearing_pB(pB,q)
    N_bearing=size(pB,2);
    pR=[q(1:2);0];
    grad_stack=[([-(pB(2,:)'-pR(2)) (pB(1,:)'-pR(1))]'./vecnorm(pB(1:2,:)-pR(1:2)).^2)' ...
        zeros(size(pB,2),1)];
    grad_h=zeros(N_bearing,3*N_bearing);
    for i=1:N_bearing;grad_h(i,(i-1)*3+1:3*i)=grad_stack(i,:);end
end