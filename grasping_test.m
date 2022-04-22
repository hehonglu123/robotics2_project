%%% Rigid, f1 only
% G=[0 -1; 1 0; 0 1];
% null(G)   %empty but ok due to rigid contact


%%%%frictionless contact
syms a b positive
%%% f1, f2, f3 only
% G=[0 -a b;1 -1 0; 0 0 1];
% null(G)     %empty, not ok may slip

% G=[-a b 0;-1 0 0; 0 1 -1];
% null(G)     %empty, not ok may slip

% G=[0 b 0;1 0 0; 0 1 -1];
% null(G)     %empty, not ok may slip
%%% frictionless contact f1, f2, f3 f4
% G=[0 -a b 0;1 -1 0 0; 0 0 1 -1];
% null(G)*a     %ok as long as a>0 and b>0

%%%% friction cone contact, f1 & f2 only
q=pi/4; %cone angle
%contact position
x1=-1;y1=0;
x2=1;y2=0;

f(:,1)=[1;0];
f(:,2)=[-1;0];

pC(:,1)=[x1;y1];
pC(:,2)=[x2;y2];


S=[0 -1;1 0];
tau(1)=(S*pC(:,1))'*rot2(q)*f(:,1);
tau(2)=(S*pC(:,1))'*rot2(-q)*f(:,1);
tau(3)=(S*pC(:,2))'*rot2(q)*f(:,2);
tau(4)=(S*pC(:,2))'*rot2(-q)*f(:,2);


G1(:,1)=[tau(1);rot2(q)*f(:,1)];
G1(:,2)=[tau(2);rot2(-q)*f(:,1)];
G1(:,3)=[tau(3);rot2(q)*f(:,2)];
G1(:,4)=[tau(4);rot2(-q)*f(:,2)];

m2=size(G1,2);

% minimizing ||G * x||^2, x>0
[X,FVAL,EXITFLAG,OUTPUT] = quadprog(G1'*G1,zeros(m2,1),-eye(m2),-ones(m2,1));
disp('*******************')
fprintf('minimum ||G x ||, x>0: %g, %g \n',FVAL,norm(G1*X)^2);
disp('*******************')
Gsym=sym(G1);
simplify(Gsym)
nullGsym=null(Gsym);

xi_vec=sym(zeros(size(nullGsym,2),1));
for i=1:size(nullGsym,2)
    x_i=['xi',num2str(i)];
    eval(['syms ',x_i]);
    eval(['xi_vec(i)=',x_i,';']);
end
disp(null(Gsym)*xi_vec);

if FVAL>.1;return;end
