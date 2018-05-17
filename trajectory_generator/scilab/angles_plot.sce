directory = get_absolute_file_path("angles_plot.sce");
global results;
function makeWorkSpacePlot()
    global results;
    filePath=directory+"../logs/WorkSpaceTraj.log"
    results=read(filePath, -1, 10);
    i=1;
    pos=[results(:,i),...
    results(:,i+1),results(:,i+2),results(:,i+3),...
    results(:,i+4),results(:,i+5),results(:,i+6),...
    results(:,i+7),results(:,i+8),results(:,i+9)];
    time=results(:,size(results,2));
    subplot(311);
    xtitle('координаты гриппера', 'Время, [c]', 'Координата, [м]');
    plot(time, pos(:,1),'b.');
    plot(time, pos(:,2),'r*');
    plot(time, pos(:,3),'g*');  
    xgrid(4);
    
    subplot(312);
    xtitle('скорость гриппера', 'Время, [c]', 'скорость, [м/с]');
    plot(time, pos(:,4),'b.');
    plot(time, pos(:,5),'r*');
    plot(time, pos(:,6),'g*');
    xgrid(4);
    subplot(313);
    xtitle('ускорение гриппера', 'Время, [c]', 'ускорение, [м/с2]');
    plot(time, pos(:,7),'b.');
    plot(time, pos(:,8),'r*');
    plot(time, pos(:,9),'g*');
    xgrid(4);
    legend(['x','y','z'],opt=3);
endfunction

makeWorkSpacePlot();
return;

filePath=directory+"../logs/JointSpaceTraj.log"
results=read(filePath, -1, 11);
RAD2DEG = 180/%pi;
RAD2DEG = 1;
i=1;//angles
i=6;//speed
angle=[results(:,i)*RAD2DEG,...
results(:,i+1)*RAD2DEG,...
results(:,i+2)*RAD2DEG,...
results(:,i+3)*RAD2DEG,...
results(:,i+4)*RAD2DEG];
if i==1 then
    text='положение звена';
else
    text='скорость звена';
end

time=results(:,size(results,2));
xset("font size", 3);
xtitle(text+' 1', 'Точка, ном.', 'Угол, [рад]');
subplot(321);
plot(time(:,1),angle(:,1), "r");
subplot(322);
xtitle(text+' 2', 'Точка, ном.', 'Угол, [рад]');
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
