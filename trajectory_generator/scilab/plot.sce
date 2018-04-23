function makeWorkSpacePlot()
    clear;
	RAD2DEG = 180/%pi;
	RAD2DEG = 1;
    filePath="~/uws/src/red_manipulation_step/arm_manipulation/logs/data.log"
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
filePath="~/uws/src/red_manipulation_step/arm_manipulation/logs/data.log"
results=read(filePath, -1, 26);
RAD2DEG = 180/%pi;
RAD2DEG = 1;
i=1;//angles
//i=6;//speed
angle=results;
text='положение звена';
time=results(:,size(results,2));
xset("font size", 3);
xtitle(text+' 1', 'Точка, ном.', 'Угол, [ ]');
subplot(321);
plot(time(:,1),angle(:,1), "r");
plot(time(:,1),angle(:,2), "b");
subplot(322);
xtitle('Положение звена 2', 'Точка, ном.', 'Угол, [ ]');
plot(time(:,1),angle(:,6), "r");
plot(time(:,1),angle(:,7), "b");
subplot(323);
xtitle(text+' 3', 'Точка, ном.', 'Угол, [ ]');
plot(time(:,1),angle(:,11), "r");
plot(time(:,1),angle(:,12), "b");
subplot(324);
xtitle(text+' 4', 'Точка, ном.', 'Угол, [ ]');
plot(time(:,1),angle(:,16), "r");
plot(time(:,1),angle(:,17), "b");
subplot(325);
xtitle(text+' 5', 'Точка, ном.', 'Угол, [ ]');
plot(time(:,1),angle(:,21), "r");
plot(time(:,1),angle(:,22), "b");
