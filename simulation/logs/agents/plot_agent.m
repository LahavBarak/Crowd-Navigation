clear
close all
clc
%% open file
T = readtable("agent_ORANGE.csv");
x = T.x;
y = T.y;
v_x = T.v_x;
v_y = T.v_y;
duration = T.duration;
sf_x = T.sf_x;
sf_y = T.sf_y;

%% plot graphs
% position
figure(1)
plot(x,y)
xlim([0 1200])
ylim([0 900])
set(gca,"YDir","reverse")
title("position")

% velocity over time
N = 1:length(x);
t = (0:numel(x)-1)/30;
figure(2)
subplot(3,1,1)
plot(t,v_x)
title("v_x")
subplot(3,1,2)
plot(t,v_y)
title("v_y")
subplot(3,1,3)
plot(t,duration)
title("duration")

% velocity and forces
figure(3)
subplot(2,2,1)
plot(t,v_x)
title("v_x")
subplot(2,2,2)
plot(t,v_y)
title("v_y")
subplot(2,2,3)
plot(t,sf_x)
title("sf_x")
subplot(2,2,4)
plot(t,sf_y)
title("sf_y")

%%
% figure(4)
% h = animatedline('LineWidth',2);
% axis([-40 40 -40 40])
% for k = 1:numel(v_x)
%     addpoints(h, v_x(k), v_y(k));
%     drawnow limitrate   % fast redraw
%     pause(0.02)         % adjust for speed
% end