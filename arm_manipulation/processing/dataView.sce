data = read("/home/senserlex/test_ws/src/red_manipulation_step/arm_manipulation/logs/data50.log", -1, 15);

gray = color(200, 200, 200);

angVel = [];
angles = data(:, 11:15);

for j = [2:size(data, 1)]
    angVel(j, :) = (angles(j, :) - angles(j - 1, :))/0.02;
    angAccel(j, :) = (angVel(j, :) - angVel(j - 1, :))/0.02
end

for i = [1:5]
    subplot(3, 5, i)
    plot(data(:, i + 10));
    xgrid(1, 1);
    titl = "\theta_{" + string(i) + "}$";
    title("$" + titl, "fontsize", 3);
    
    subplot(3, 5, i + 5)
    plot(data(:, i + 5));
    plot(angVel(:, i), "r");
    xgrid(1, 1);
    title("$\dot" + titl, "fontsize", 3);

    subplot(3, 5, i + 10)
    plot(data(:, i));
    plot(angAccel(:, i), "r");
    xgrid(1, 1);
    title("$\ddot" + titl, "fontsize", 3);
end
