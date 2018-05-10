directory = get_absolute_file_path("angles_plot.sce");
global results;
function makeWorkSpacePlot()
    global results;
    filePath=directory+"../logs/WorkSpaceTraj.log"
    results=read(filePath, -1, 10);
    i=1;
    i=4; //speed
    i=7; //acceleration
    pos=[results(:,i),...
    results(:,i+1),...
    results(:,i+2),...
    results(:,i+3)]
    xtitle('координаты гриппера', 'Время, [c]', 'Координата, [м]');
    plot(results(:,size(results,2)), pos(:,1),'b.');
    plot(results(:,size(results,2)), pos(:,2),'r*');
    plot(results(:,size(results,2)), pos(:,3),'g*');
    xgrid(4);
    legend('x','y','z',4);
endfunction

makeWorkSpacePlot();
return;

filePath=directory+"../logs/JointSpaceTraj.log"
results=read(filePath, -1, 26);
RAD2DEG = 180/%pi;
RAD2DEG = 1;
i=1;//angles
//i=6;//speed
angle=[results(:,i)*RAD2DEG,...
results(:,i+1)*RAD2DEG,...
results(:,i+2)*RAD2DEG,...
results(:,i+3)*RAD2DEG,...
results(:,i+4)*RAD2DEG];
text='положение звена';
time=results(:,size(results,2));
xset("font size", 3);
xtitle(text+' 1', 'Точка, ном.', 'Угол, [рад]');
subplot(321);
plot(time(:,1),angle(:,1), "r");
subplot(322);
xtitle('Положение звена 2', 'Точка, ном.', 'Угол, [рад]');
plot(time(:,1),angle(:,2), "r");
subplot(323);
xtitle(text+' 3', 'Точка, ном.', 'Угол, [рад]');
plot(time(:,1),angle(:,3), "r");
subplot(324);
xtitle(text+' 4', 'Точка, ном.', 'Угол, [рад]');
plot(time(:,1),angle(:,4), "r");
subplot(325);
xtitle(text+' 5', 'Точка, ном.', 'Угол, [рад]');
plot(time(:,1),angle(:,5), "r");
