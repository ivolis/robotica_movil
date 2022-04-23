clear all
clc
close all

% =========================================================================
%                               IMPLEMENTACION
% =========================================================================

l = 0.5; % distancia entre ruedas del robot

% vl [m/s] | vr [m/s] | t [s]
actions = [0.1 0.5 2;
            0.5 0.1 2;
            0.2 -0.2 2;
            0.1 0.6 6;
            0.4 0.4 2];

initial_pose = [1.5 , 2.0 , pi/3];

% Para plotear la trayectoria debo divividr el problema en "n" intervalos
% del tiempo con valor tiempo/#puntos que realicen el mismo movimiento.
n = 50;
[pose1 , trayect1] = robot_movement(initial_pose(1), initial_pose(2), initial_pose(3), actions(1,1), actions(1,2), actions(1,3), l, n);
[pose2 , trayect2] = robot_movement(pose1(1), pose1(2), pose1(3), actions(2,1), actions(2,2), actions(2,3), l, n);
[pose3 , trayect3] = robot_movement(pose2(1), pose2(2), pose2(3), actions(3,1), actions(3,2), actions(3,3), l, n);
[pose4 , trayect4] = robot_movement(pose3(1), pose3(2), pose3(3), actions(4,1), actions(4,2), actions(4,3), l, n);
[pose5 , trayect5] = robot_movement(pose4(1), pose4(2), pose4(3), actions(5,1), actions(5,2), actions(5,3), l, n);


% =========================================================================
%                               PLOTEO
% =========================================================================

figura = figure();
set(gcf,'position',[600,350,650,500])
hold on
% Ploteos
plot(trayect1(:,1),trayect1(:,2),'LineWidth',1)
plot(trayect2(:,1),trayect2(:,2))
plot(trayect3(:,1),trayect3(:,2))
plot(trayect4(:,1),trayect4(:,2))
plot(trayect5(:,1),trayect5(:,2))
scatter(initial_pose(1), initial_pose(2), 'filled')
scatter(pose1(1), pose1(2), 'filled')
scatter(pose2(1), pose2(2), 'filled')
scatter(pose3(1), pose3(2), 80, 'd') % overlap
scatter(pose4(1), pose4(2), 'filled')
scatter(pose5(1), pose5(2), 'filled')
% Configuraciones de ploteo
title('Movimiento resultante del robot')
axis('equal');
set(gca(),'Box','on');
xlabel('Eje X - Terna global [m]', 'FontSize',10,'FontWeight','bold')
ylabel('Eje Y - Terna global [m]', 'FontSize',10,'FontWeight','bold')
leyenda=legend("Trayectoria 1", "Trayectoria 2", "Trayectoria 3", "Trayectoria 4", "Trayectoria 5", "Pose inicial", "Pose 1", "Pose 2", "Pose 3", "Pose 4", "Pose 5 (final)", "location", 'southeast');
set(leyenda, 'fontsize', 8);
grid on
%export_fig( "trayectoria_robot" , '-png', '-transparent', '-native');


% =========================================================================
%                               FUNCIONES
% =========================================================================

function [pose, trayect] = robot_movement(x, y, theta, v_l, v_r, t , l , n)
    
    trayect = zeros(n,3); % preallocating
    trayect(1,:) = [x , y , theta]; % 1er punto de la trayectoria es la pose de donde arranca
    dt = t/n;
    
    for i = 2:n
        [trayect(i,1), trayect(i,2), trayect(i,3)] = diffdrive(trayect(i-1,1), trayect(i-1,2) , trayect(i-1,3), v_l, v_r, dt, l);
    end
    
    % El valor final de dicha trayectoria sera claramente la pose siguiente del robot
    pose = trayect(end,:);

end


function [x_n , y_n , theta_n] = diffdrive(x, y , theta, v_l, v_r, t, l)

    if v_l == v_r
        v = (v_l + v_r)/2;
        x_n = x + v*cos(theta)*t;
        y_n = y + v*sin(theta)*t;
        theta_n = theta;
    else
        w = (v_r-v_l)/l;
        R = (l/2) * (v_l+v_r)/(v_r-v_l);
        ICC = [x - R*sin(theta), y + R*cos(theta)];

        steering_matrix = rotZ(w*t);
        aux_toIcc = [x - ICC(1) ; y - ICC(2) ; theta];
        aux_toPos = [ICC(1) ; ICC(2) ; w*t];

        new_pos = (steering_matrix*aux_toIcc + aux_toPos)';
        x_n = new_pos(1);
        y_n = new_pos(2);
        theta_n = new_pos(3);
    end
end


% Matriz de rotacion sobre el eje Z
function R = rotZ(th) 
    R = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
end
