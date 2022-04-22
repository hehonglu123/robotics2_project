%
% 2D formation control example based on the relative pose in a room
% 
clear all;close all;

% *****
% define room and robot
% *****
% define room
roomspec;
% define robot
rL=.4;rW=.2;rz=.1;
robot=collisionCylinder(rW,rz);
N=999;

% *****
% formation control part
% *****
% repeatable runs
rng(100);
% ask user for # of agents
p = 4;

D=[ -1  0  0;
     1 -1  0;
     0  1 -1;
     0  0  1];
dstar=2.2*rW;
zstar=repmat([0;dstar],p-1,1);
% show graph 
G=graph(-D*D'+diag(diag(D*D')));
figure(20);plot(G);
set(gcf,'position',[100,400,400,400]);
%
% for collision avoidance between agents, use fully connected graph
%
Dfull=(incidence(graph(ones(p,p)-diag(ones(p,1)))));
% *************************************
% # of links for the formation graph
l = size(D,2);
% # of states for each agent
n = 2;
% allocate space
q = zeros(n*p,N+1);
u = zeros(n*p,N);
z = zeros(n*l,N+1);
% initial condition
q(:,1) = rand(n*p,1);

q0=[.4;2;0];
q0=[rW+.1;2;0];
q0flat=zeros(n,p);
for i=1:p;q0flat(:,i)=[rW+.1;2.2*rW*i];end
q(:,1)=reshape(q0flat,n*p,1);
z(:,1) = kron(D',eye(n,n))*q(:,1);
% time step
ts = 1;
% time vector
t = (0:N)*ts;
% feedback gain
% for collision avoidance with each other
Kp_repel = 0.01*eye(n*p,n*p);
% for keeping the formation together
Kp_form = 0.1*eye(n*p,n*p);
% define repellent potential function 
% (only active for distance within qrange) 
qrange=.05;
psifun = @(x,dx) (x<dx).*(1./x-1./dx);
% lead agent feedback gain
Kp1 = .3;
% *****
% Show robot and room
% *****
fignum=1;
h_fig=roomshow(colobj,fignum);
axis('square');
% set up key press detection in figure
set(h_fig,'KeyPressFcn',@robotfcn);
keyvalue='a';
keypressed=0;
view(-90,90);axis([-1 11 -1 11 0 4])
for i=1:p;robotshow(robot,[q0flat(:,i);0],[.5,.1,.5]);end
hold off

% ****************
% control loop
% ****************
k=0;
while keyvalue~='q'
    k=k+1;
    while keypressed==0;pause(.05);end
    keypressed=0;
    switch keyvalue
        case 'leftarrow'
            vd_lead=[0;1];
        case 'rightarrow'
            vd_lead=[0;-1];
    end  
    vd = kron(ones(p,1),vd_lead);
    % ****
    % formation control part
    % ****
    % link difference vectors z = D^T * q 
    zk = kron(D',eye(n,n))*q(:,k);    
    % gradient to the formation
    phi = kron(D,eye(n))*(zk-zstar);
    % ****
    % repellent function to avoid inter-agent collision
    % ****
    zkfull=kron(Dfull',eye(n,n))*q(:,k);
    zkflat=reshape(zkfull,n,size(Dfull,2));
    distzk=vecnorm(zkflat);
    % if within qrange, push each other away along the distance vector
    psiflat = zkflat.*psifun(distzk,qrange)./distzk;
    psi = kron(Dfull,eye(n,n))*reshape(psiflat,n*size(Dfull,2),1);
    % ****
    % controller combines formation and repellent parts and add in desired
    % formation motion velocity
    % ****
    u(:,k) =- Kp_form * phi + vd;

    u_temp=u(:,k);
    u_temp(u_temp>0.5)=0.5;
    u_temp(u_temp<-0.5)=-0.5;
    u(:,k)=u_temp;


    % update agent position
    q(:,k+1) = q(:,k)+ts*u(:,k);
    % ****
    % check for collision
    % ****
    % check for each agent

%     q(:,k+1)=reshape(qflat,n*p,1);
    % update formation difference variable
    z(:,k+1) = kron(D',eye(n,n))*q(:,k+1);
    qkflat=reshape(q(:,k+1),n,p);
    clf;
    figure(1);hold on
    h_fig=roomshow(colobj,fignum);
    axis('square');
    view(-90,90);axis([-1 11 -1 11 0 4])
    for i=1:p;robotshow(robot,[qkflat(:,i);0],[.5,.1,.5]);end
    hold off;
end
