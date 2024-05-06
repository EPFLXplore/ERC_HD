abc=[Commands.signals.values(:,1),Commands.signals.values(:,2),Commands.signals.values(:,3),Commands.signals.values(:,4),Commands.signals.values(:,5),Commands.signals.values(:,6)];
abc2=[ArmAngles.signals.values(:,1),ArmAngles.signals.values(:,2),ArmAngles.signals.values(:,3),ArmAngles.signals.values(:,4),ArmAngles.signals.values(:,5),ArmAngles.signals.values(:,6)];
steam = iddata(abc2,abc,0.05);
steam.InputName  = {'tCom';'bCom';'fCom';'wCom';'hCom';'gCom'};
steam.OutputName = {'tAng';'bAng';'fAng';'wAng';'hAng';'gAng'};
plot(steam(:,1))
plot(steam(:,2))
plot(steam(:,3))
imp=impulseest(steam,50);
clf, step(imp)

% id1=ssest(steam(1:1000))
% h = stepplot(imp,'b',id1,'r',2);

% showConfidence(h)
% compare(steam(900:1030),id1)
