%
% 2D formation control example based on the relative pose in a room
% 
%
% contact force
%

% contact location
% Pci_q=[10,80,190,260]*pi/180;
Pci_q=[0, 180]*pi/180;
[Pci,e_ni]=contact_gen(Pci_q,load_x/2,load_y/2);

Pcorner=[load_x load_x -load_x -load_x load_x;...
    load_y -load_y -load_y load_y load_y]/2;
figure(60);plot(Pcorner(1,:),Pcorner(2,:),Pci(1,:),Pci(2,:),'x',...
    'LineWidth',2);
axis([-1,1,-1,1]);

% visualization of the load with robots attached
figure(40);
hold on;
roomshow(colobj,40);axis('square');
view(-90,90);axis([-1 11 -1 11 0 4]);
for i=1:3:31;
    Pcorner0=rot2(xc(1,i))*Pcorner+xc(2:3,i);
    Pci0=rot2(xc(1,i))*Pci+xc(2:3,i);
    plot(Pcorner0(1,:),Pcorner0(2,:),Pci0(1,:),Pci0(2,:),'x','LineWidth',2);
end;hold off

% grasp map
K=[0 -1;1 0]; % planar cross product
m=length(Pci_q); % # of fingers
%m=2;
for k=1:N
    R=rot2(xc(3,k));
    Pci_0=R*Pci;
    Pci_0_perp=K*Pci_0;
    e_ni_0=R*e_ni;
    % rigid grasp
    G1{k}=zeros(3,2*m);
    for i=1:m
        G1{k}(1,(i-1)*2+1:2*i)=Pci_0_perp(:,i)';
        G1{k}(2:3,(i-1)*2+1:2*i)=eye(2);
    end
    % >>> *****
% %     how to choose etaS so eta_n_1 (projected normal force at contact) is
% %     always positive (pointing inward)
% %     >>> *****
%     etaS=-20;
%     eta_1(:,k)=pinv(G1{k})*etaC(:,k)+etaS*null(G1{k})*ones(2*m-3,1);
%     eta_n_1(:,k)=sum(e_ni_0.*reshape(eta_1(:,k),[2,m]),1);

    %%%%%%%%%%%%%%%%%%%%% point contact without friction%%%%%%%%%%%%%%%%%%%
%     G2{k}=zeros(3,m);
%     for i=1:m
%         G2{k}(:,i)=G1{k}(:,(i-1)*2+1:2*i)*e_ni_0(:,i);
%     end
%     eta_2(:,k)=pinv(G2{k})*etaC(:,k);
%     neg_idx=find(eta_2(:,k)<0);
%     X=null(G2{k});
%     if min(X)<0
%         X=X+min(X)*ones(size(X));
%     end
%     %%%find minnimum squeeze force to keep force push inward, i.e fn>0
%     alpha=-min(eta_2(neg_idx,k)./X(neg_idx));
%     eta_2(:,k)=eta_2(:,k)+alpha.*X;
%     eta_n_2(:,k)=eta_2(:,k);  %normal already

    %%%%%%%%%%%%%%%%%%%% point contact with friction%%%%%%%%%%%%%%%%%%%%%%%%
    phi=pi/4*ones(1,m); % 45deg friction cone
    G3{k}=zeros(3,2*m);
    for i=1:m
        G3{k}(:,(i-1)*2+1:2*i)=G1{k}(:,(i-1)*2+1:2*i)*...
            [rot2(phi(i))*e_ni_0(:,i) rot2(-phi(i))*e_ni_0(:,i)];
    end  
    %%%find null space for squeezing force

    eta_3(:,k)=pinv(G3{k})*etaC(:,k);


%     m2=size(G3{k},2);
%     [X,FVAL,EXITFLAG,OUTPUT] = quadprog(G3{k}'*G3{k},zeros(m2,1),-eye(m2),-ones(m2,1));
% %     [X,FVAL,EXITFLAG,OUTPUT] = quadprog(eye(m2),zeros(m2,1),-eye(m2),-ones(m2,1),G3{k},zeros(3,1));
%     disp('*******************')%check null space
%     fprintf('minimum ||G x ||, x>0: %g, %g \n',FVAL,norm(G3{k}*X)^2);
%     disp('*******************')
%     fun = @(x,eta_m,eta_s) -min(min(eta_m+x*eta_s),0);  % The parameterized function.
% 
%     alpha = fminbnd(@(x) fun(x,pinv(G3{k})*etaC(:,k),X),0,10);
%     eta_3(:,k)=pinv(G3{k})*etaC(:,k)+alpha*X;
%     %calculate force in normal direction
%     eta_n_3(:,k)=sum((ones(2,m)*cos(pi/4.)).*reshape(eta_3(:,k),[2,m]),1);

end

% figure(21);plot(t(1:end-1),eta_n_1);legend('1','2','3','4');
figure(21);plot(t(1:end-1),eta_3);legend('1','2','3','4','5','6','7','8');

%
% grasping robots location
% 

%
% generate contacts based on load dimension and angle
%


function [P,Pn]=contact_gen(q,a,b)

P=zeros(2,length(q));
Pn=P;
for i=1:length(q)
    if abs(b*tan(q(i)))<a
        if(abs(q(i))<pi/2)
            P(1,i)=b*tan(q(i));
            P(2,i)=b;
            Pn(:,i)=[-1;0];
        else
            P(1,i)=-b*tan(q(i));
            P(2,i)=-b;
            Pn(:,i)=[1;0];
        end
    else
        if (q(i)<pi)&&(q(i)>0)
            P(1,i)=a;
            P(2,i)=a*tan(pi/2-q(i));
            Pn(:,i)=[0;-1];
        else
            P(1,i)=-a;
            P(2,i)=-a*tan(pi/2-q(i));
            Pn(:,i)=[0;1];
        end        
    end
end
end

%
% planar rotation
%

function R=rot2(q)

R=[cos(q) -sin(q); sin(q) cos(q)];

end 
