function makeWorkSpacePlot()
    filePath="~/vrepWS/src/red_manipulation_step/trajectory_generator/logs/WorkSpaceTraj.log"
    results=read(filePath, -1, 4);
    i=1;
    angle=[results(:,i)*RAD2DEG,...
    results(:,i+1)*RAD2DEG,...
    results(:,i+2)*RAD2DEG,...
    results(:,i+3)*RAD2DEG]
    plot(angle(:,4), angle(:,3));
endfunction
//makeWorkSpacePlot();
//return;

filePath="~/vrepWS/src/red_manipulation_step/trajectory_generator/logs/JointSpaceTraj.log"
results=read(filePath, -1, 10);
RAD2DEG = 180/%pi;
RAD2DEG = 1;
i=1;
angle=[results(:,i)*RAD2DEG,...
results(:,i+1)*RAD2DEG,...
results(:,i+2)*RAD2DEG,...
results(:,i+3)*RAD2DEG,...
results(:,i+4)*RAD2DEG];
text='положение звена';

//function e=G(a,z),
//e=z(2)-a(1)*z(1)+a(1)*a(2)-a(1)*a(2)*%e^(-z(1)/a(2));
//endfunction
//att=[4;8];
//[k,error]=datafit(G,aim',att);
//model=k(1)*time-k(1)*k(2)+k(1)*k(2)*%e^(-time/k(2));
//deletefile("F:\temp1.txt")
//write("F:\temp1.txt", aim)
xset("font size", 3);
xtitle(text+' 1', 'Точка, ном.', 'Угол, [ ]');
subplot(321);
plot(angle(:,1), "r");
subplot(322);
xtitle('Положение звена 2', 'Точка, ном.', 'Угол, [ ]');
plot(angle(:,2), "r");
subplot(323);
xtitle(text+' 3', 'Точка, ном.', 'Угол, [ ]');
plot(angle(:,3), "r");
subplot(324);
xtitle(text+' 4', 'Точка, ном.', 'Угол, [ ]');
plot(angle(:,4), "r");
subplot(325);
xtitle(text+' 5', 'Точка, ном.', 'Угол, [ ]');
plot(angle(:,5), "r");
