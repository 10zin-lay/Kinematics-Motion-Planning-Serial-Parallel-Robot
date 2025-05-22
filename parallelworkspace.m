clear;

function p = Point(radius, angle)
    if ~isa(radius, 'sym')
        radius = sym(radius);
    end
    if ~isa(angle, 'sym')
        angle = sym(angle);
    end
    p = [radius * cos(angle); radius * sin(angle)];
end

function [angleA, angleB, legLength] = InverseKinematics(basePoint, platformPoint, linkA, linkB)
    legLength = sqrt((basePoint(1) - platformPoint(1))^2 + (basePoint(2) - platformPoint(2))^2);
    angleBaseToPlatform = atan2(platformPoint(2) - basePoint(2), platformPoint(1) - basePoint(1));
    angleOffset = acos((linkA^2 - linkB^2 + legLength^2) / (2 * legLength * linkA));
    angleA = angleBaseToPlatform + angleOffset;
    angleB = angleBaseToPlatform - angleOffset;
end

%% Coordinates of the base, platform points w.r.t the world reference frame {B}
syms centerX centerY orientation angleA angleB angleC jointA jointB jointC linkA linkB radiusP radiusB real

basePoint1 = Point(radiusB, pi/2);
basePoint2 = Point(radiusB, pi + pi / 6);
basePoint3 = Point(radiusB, 2 * pi - pi / 6);

platformJoint1 = Point(linkA, angleA);
platformJoint2 = Point(linkA, angleB);
platformJoint3 = Point(linkA, angleC);

platformLink1 = Point(linkB, jointA);
platformLink2 = Point(linkB, jointB);
platformLink3 = Point(linkB, jointC);

linkBase1 = basePoint1 + platformJoint1;
linkBase2 = basePoint2 + platformJoint2;
linkBase3 = basePoint3 + platformJoint3;

rotationMatrixBase = eye(2);
rotationMatrixPlatform = eye(2);

platformPoint1 = platformJoint1 + rotationMatrixPlatform * platformLink1;
platformPoint2 = platformJoint2 + rotationMatrixPlatform * platformLink2;
platformPoint3 = platformJoint3 + rotationMatrixPlatform * platformLink3;

% Platform centroid C (centerX, centerY, orientation)
% Transformation matrix of the centre C w.r.t frame {B}
rotationMatrixCenter = [cos(orientation) -sin(orientation);
                         sin(orientation)  cos(orientation)];

% Position of frame {C} w.r.t frame {B}
centerPosition = [centerX; centerY];

platformVertex1 = Point(radiusP, pi/2);
platformVertex2 = Point(radiusP, pi + pi / 6);
platformVertex3 = Point(radiusP, 2 * pi - pi / 6);

platformGlobalPoint1 = rotationMatrixCenter * platformVertex1 + centerPosition;
platformGlobalPoint2 = rotationMatrixCenter * platformVertex2 + centerPosition;
platformGlobalPoint3 = rotationMatrixCenter * platformVertex3 + centerPosition;

% Substitute values (convert symbolic to numeric)
linkBase1Func = matlabFunction(linkBase1, 'Vars', [radiusB, linkA, angleA]);
linkBase2Func = matlabFunction(linkBase2, 'Vars', [radiusB, linkA, angleB]);
linkBase3Func = matlabFunction(linkBase3, 'Vars', [radiusB, linkA, angleC]);

platformPoint1Func = matlabFunction(platformGlobalPoint1, 'Vars', [centerX, centerY, orientation, radiusP]);
platformPoint2Func = matlabFunction(platformGlobalPoint2, 'Vars', [centerX, centerY, orientation, radiusP]);
platformPoint3Func = matlabFunction(platformGlobalPoint3, 'Vars', [centerX, centerY, orientation, radiusP]);

basePoint1Func = matlabFunction(basePoint1, 'Vars', radiusB);
basePoint2Func = matlabFunction(basePoint2, 'Vars', radiusB);
basePoint3Func = matlabFunction(basePoint3, 'Vars', radiusB);

[angleA1Sym, angleA2Sym, length1Sym] = InverseKinematics(basePoint1, platformGlobalPoint1, linkA, linkB);
[angleB1Sym, angleB2Sym, length2Sym] = InverseKinematics(basePoint2, platformGlobalPoint2, linkA, linkB);
[angleC1Sym, angleC2Sym, length3Sym] = InverseKinematics(basePoint3, platformGlobalPoint3, linkA, linkB);

angleA1Func = matlabFunction(angleA1Sym, 'Vars', [centerX, centerY, orientation, radiusP, radiusB, linkB, linkA]);
angleA2Func = matlabFunction(angleA2Sym, 'Vars', [centerX, centerY, orientation, radiusP, radiusB, linkB, linkA]);
angleB1Func = matlabFunction(angleB1Sym, 'Vars', [centerX, centerY, orientation, radiusP, radiusB, linkB, linkA]);
angleB2Func = matlabFunction(angleB2Sym, 'Vars', [centerX, centerY, orientation, radiusP, radiusB, linkB, linkA]);
angleC1Func = matlabFunction(angleC1Sym, 'Vars', [centerX, centerY, orientation, radiusP, radiusB, linkB, linkA]);
angleC2Func = matlabFunction(angleC2Sym, 'Vars', [centerX, centerY, orientation, radiusP, radiusB, linkB, linkA]);

length1Func = matlabFunction(length1Sym, 'Vars', [centerX, centerY, orientation, radiusP, radiusB, linkB, linkA]);
length2Func = matlabFunction(length2Sym, 'Vars', [centerX, centerY, orientation, radiusP, radiusB, linkB, linkA]);
length3Func = matlabFunction(length3Sym, 'Vars', [centerX, centerY, orientation, radiusP, radiusB, linkB, linkA]);

%Design parameters of planar parallel robot
linkA = 170; 
linkB = 130; 
radiusP = 130; 
radiusB = 290; 
 
orientationValues = 79* pi / 180;
centerXValues = radiusB*cos(pi+pi/6):1:radiusB*cos(2*pi-pi/6);
centerYValues = radiusB*sin(pi+pi/6):1:radiusB*sin(pi/2);

%Initialize figure
figure;
hold on;
grid on;
xlabel('X-axis (mm)');
ylabel('Y-axis (mm)');
axis([-300 300 -150 300]);

%Plot initialization
trajectoryPlot = plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
platformPlot = plot(0, 0, 'r-', 'LineWidth', 1);
basePlot = plot(0, 0, 'b-', 'LineWidth', 1);
linkAPlot1 = plot(0, 0, 'g-', 'LineWidth', 1);
linkAPlot2 = plot(0, 0, 'g-', 'LineWidth', 1);
linkAPlot3 = plot(0, 0, 'g-', 'LineWidth', 1);
linkBPlot1 = plot(0, 0, 'm-', 'LineWidth', 1);
linkBPlot2 = plot(0, 0, 'm-', 'LineWidth', 1);
linkBPlot3 = plot(0, 0, 'm-', 'LineWidth', 1);

workspaceX = [];
workspaceY = [];
 
for i = 1:length(centerXValues)
    for j = 1:length(centerYValues)
        centerX = centerXValues(i);
        centerY = centerYValues(j);
        orientation = orientationValues;

        %Compute current positions
        platformPoint1Val = platformPoint1Func(centerX, centerY, orientation, radiusP);
        platformPoint2Val = platformPoint2Func(centerX, centerY, orientation, radiusP);
        platformPoint3Val = platformPoint3Func(centerX, centerY, orientation, radiusP);
    
        basePoint1Val = basePoint1Func(radiusB);
        basePoint2Val = basePoint2Func(radiusB);
        basePoint3Val = basePoint3Func(radiusB);
    
        angleA1Val = angleA1Func(centerX, centerY, orientation, radiusP, radiusB, linkB, linkA);
        angleA2Val = angleA2Func(centerX, centerY, orientation, radiusP, radiusB, linkB, linkA);
        angleB1Val = angleB1Func(centerX, centerY, orientation, radiusP, radiusB, linkB, linkA);
        angleB2Val = angleB2Func(centerX, centerY, orientation, radiusP, radiusB, linkB, linkA);
        angleC1Val = angleC1Func(centerX, centerY, orientation, radiusP, radiusB, linkB, linkA);
        angleC2Val = angleC2Func(centerX, centerY, orientation, radiusP, radiusB, linkB, linkA);
    
        linkBase1Val = linkBase1Func(radiusB, linkA, angleA1Val);
        linkBase2Val = linkBase2Func(radiusB, linkA, angleB1Val);
        linkBase3Val = linkBase3Func(radiusB, linkA, angleC1Val);
    
        %Update triangle vertices
        baseXCoords = [basePoint1Val(1), basePoint2Val(1), basePoint3Val(1), basePoint1Val(1)];
        baseYCoords = [basePoint1Val(2), basePoint2Val(2), basePoint3Val(2), basePoint1Val(2)];
    
        platformXCoords = [platformPoint1Val(1), platformPoint2Val(1), platformPoint3Val(1), platformPoint1Val(1)];
        platformYCoords = [platformPoint1Val(2), platformPoint2Val(2), platformPoint3Val(2), platformPoint1Val(2)];
    
        linkAPlot1X = [basePoint1Val(1), linkBase1Val(1)];
        linkAPlot1Y = [basePoint1Val(2), linkBase1Val(2)];
    
        linkAPlot2X = [basePoint2Val(1), linkBase2Val(1)];
        linkAPlot2Y = [basePoint2Val(2), linkBase2Val(2)]; 
    
        linkAPlot3X = [basePoint3Val(1), linkBase3Val(1)];
        linkAPlot3Y = [basePoint3Val(2), linkBase3Val(2)];
    
        linkBPlot1X = [linkBase1Val(1), platformPoint1Val(1)];
        linkBPlot1Y = [linkBase1Val(2), platformPoint1Val(2)];
    
        linkBPlot2X = [linkBase2Val(1), platformPoint2Val(1)];
        linkBPlot2Y = [linkBase2Val(2), platformPoint2Val(2)];
    
        linkBPlot3X = [linkBase3Val(1), platformPoint3Val(1)];
        linkBPlot3Y = [linkBase3Val(2), platformPoint3Val(2)];
    
        length1Val = length1Func(centerX, centerY, orientation, radiusP, radiusB, linkB, linkA); 
        length2Val = length2Func(centerX, centerY, orientation, radiusP, radiusB, linkB, linkA); 
        length3Val = length3Func(centerX, centerY, orientation, radiusP, radiusB, linkB, linkA); 

        if ((linkA-linkB <= length1Val && length1Val <= linkA+linkB) && ...
          (linkA-linkB <= length2Val && length2Val <= linkA+linkB) && ...
          (linkA-linkB <= length3Val && length3Val <= linkA+linkB) )
            workspaceX = [workspaceX, centerX];
            workspaceY = [workspaceY, centerY];

            % Update plots
            set(basePlot, 'XData', baseXCoords, 'YData', baseYCoords);
            set(platformPlot, 'XData', platformXCoords, 'YData', platformYCoords);
            set(trajectoryPlot, 'XData', workspaceX, 'YData', workspaceY);
            set(linkAPlot1, 'XData', linkAPlot1X, 'YData', linkAPlot1Y);
            set(linkAPlot2, 'XData', linkAPlot2X, 'YData', linkAPlot2Y);
            set(linkAPlot3, 'XData', linkAPlot3X, 'YData', linkAPlot3Y);
            set(linkBPlot1, 'XData', linkBPlot1X, 'YData', linkBPlot1Y);
            set(linkBPlot2, 'XData', linkBPlot2X, 'YData', linkBPlot2Y);
            set(linkBPlot3, 'XData', linkBPlot3X, 'YData', linkBPlot3Y);
        
            delete(findall(gcf, 'type', 'text'));
            
            pause(0.0001);
        end
    end
end
hold off;

%Final plot for trajectory and base
figure;
hold on;
grid on;
xlabel('X (mm)');
ylabel('Y (mm)');
title({sprintf('Workspace (\\alpha = %d)', orientationValues)});
axis([-300 300 -150 300]);

%Plot the trajectory
plot(workspaceX, workspaceY, 'r-', 'LineWidth', 2); % Trajectory in red

%Plot the base
plot(baseXCoords, baseYCoords, 'b-', 'LineWidth', 2); % Base in blue

legend({'Workspace', 'Base'}, 'Location', 'Best');
hold off;
