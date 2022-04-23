clear all
clc
close all

% =========================================================================
%                               IMPLEMENTACION
% =========================================================================

% Cargo los datos
scan = load('-ascii', 'laserscan.dat');
angle = linspace(-pi/2, pi/2, size(scan,2));
fov = pi;

pose_robot = [5 -7 -pi/4]; % respecto a la terna global
pose_LIDAR = [0.2 0 pi]; % respecto al cuerpo del robot


%%  ############################# INCISO 1 ################################
figura = figure();
set(gcf,'position',[400,250,1050,700])
hold on
plot(angle,scan,'o','LineWidth',1, 'Color', 'r')
% Configuraciones de ploteo
title('Mediciones en la terna de refencia del LIDAR')
set(gca(),'Box','on');
set(gca,'XTick',-pi/2:pi/10:pi/2) 
set(gca,'XTickLabel',{'-\pi/2','-2\pi/5','-3\pi/10','-\pi/5','-\pi/10','0','\pi/10','\pi/5','3\pi/10','2\pi/5','\pi/2'})
xlabel('Angulo \alpha [rad]', 'FontSize',11,'FontWeight','bold')
ylabel('Distancia sensada [m]', 'FontSize',11,'FontWeight','bold')
xlim([-pi/2 pi/2])
ylim([0 11])
grid on
%export_fig( "lectura_LIDAR" , '-png', '-transparent', '-native');

figura = figure();
set(gcf,'position',[400,250,750,500])
polarplot(angle,scan, 'o')
%thetaticks(-180:360/16:180)
thetalim([-180 180])
set(gca,'ThetaAxisUnits', 'radians') 
title('Mediciones en la terna de refencia del LIDAR (polar)')
%export_fig( "lectura_LIDAR_polar" , '-png', '-transparent', '-native');



%% ############################### INCISO 3 ###############################
A_lidar_robot = [rotZ(pose_LIDAR(3)) [pose_LIDAR(1) pose_LIDAR(2)]'; 0 0 1];
A_robot_global = [rotZ(pose_robot(3)) [pose_robot(1) pose_robot(2)]'; 0 0 1];

pose_LIDAR_global = A_robot_global * [pose_LIDAR(1:2) 1]';
scan_coords = [scan' .* cos(angle') , scan' .* sin(angle')];

% Proyecto las mediciones en la matriz global
for i = 1:length(scan)
    scan_global(i,:) = A_robot_global * A_lidar_robot * [scan_coords(i,:) 1]';
end

figura = figure();
set(gcf,'position',[600,350,650,500])
hold on
scatter(pose_robot(1), pose_robot(2), 'filled')
scatter(pose_LIDAR_global(1), pose_LIDAR_global(2), 'filled')
scatter(scan_global(:,1), scan_global(:,2),[], [0.2660 0.6740 0.1880])
% Configuraciones de ploteo
title('Posición del Robot, del LIDAR y de las mediciones en la terna global.')
set(gca(),'Box','on');
xlabel('Eje X - Terna global [m]', 'FontSize',10,'FontWeight','bold')
ylabel('Eje Y - Terna global [m]', 'FontSize',10,'FontWeight','bold')
leyenda=legend("Robot", "Sensor LIDAR", "Mediciones del sensor", "location", 'southeast');
set(leyenda, 'fontsize', 8);
grid on
%export_fig( "medicion_terna_global" , '-png', '-transparent', '-native');



% =========================================================================
%                               FUNCIONES
% =========================================================================

% Matriz de rotacion sobre el eje Z
function R = rotZ(th) 
    R = [cos(th) -sin(th); sin(th) cos(th)];
end