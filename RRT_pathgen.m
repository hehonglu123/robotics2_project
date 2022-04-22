roomspec;
ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);
load exampleMaps
map = colobj2binary_map(colobj, 100, 0.8);
sv.Map = map;
sv.ValidationDistance = 0.01;
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]];
planner = plannerRRT(ss,sv);
planner.MaxConnectionDistance = 0.1;
start = [2,2,0];
goal = [6,8,0];

[pthObj,solnInfo] = plan(planner,start,goal);
despath=pthObj.States(:,1:2)';
show(map)
hold on
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-'); % tree expansion
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2) % draw path

save('grasp_path.mat','despath')