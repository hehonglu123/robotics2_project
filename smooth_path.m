% load('grasp_path.mat')
roomspec;
lam=[0 cumsum(vecnorm(diff(despath')'))];

px=polyfit(lam,despath(1,:),10);
py=polyfit(lam,despath(2,:),10);

lam_dense=linspace(0,lam(end),100);
x=polyval(px,lam_dense);
y=polyval(py,lam_dense);

map = colobj2binary_map(colobj, 100, 0.8);
show(map)
hold on
plot(x,y,'o')
despath=[x;y];
save('smooth_path.mat','despath')