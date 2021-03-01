%CHALLENGE: You will use LIDAR to detect and avoid obstacles. You can plan
%your entire path based on your intial LIDAR scan or dyanmically update
%your path as you go. 

clear all
close all
load('gauntletChallengeMapData.mat')


%DELIVERABLE 2: A contour plot of the potential field based on the LIDAR scan.
%DELIVERABLE 3: A quiver plot of the gradient of your potential field. 

%Build a mesh
[xcon,ycon]=meshgrid(-1.5:0.05:2.5,-3:0.05:1);
[xquiv,yquiv]=meshgrid(-1.5:0.3:2.5,-3:0.3:1);
%Define the symbolic variables we will being using
syms x_g y_g
%Initialize the potential and gradient components to 0
v = 0; 
fx = 0;
fy = 0;
fx_g = 0; 
fy_g = 0; 
%Loop through points on BOB and treat as sink points
disp('Creating contour and gradient plots')
for thet = 0:0.1:2*pi
    a = xc + bestr * cos(thet);
    b = yc + bestr * sin(thet);
    v = v + log(sqrt((xcon-a).^2 + (ycon-b).^2));
    fx = fx + (xquiv-a)./((xquiv-a).^2 + (yquiv-b).^2); 
    fy = fy + (yquiv-b)./((xquiv-a).^2 + (yquiv-b).^2); 
    fx_g = fx_g - (x_g-a)./((x_g-a).^2 + (y_g-b).^2);
    fy_g = fy_g - (y_g-b)./((x_g-a).^2 + (y_g-b).^2);
end
%Scale up BOB potential and gradient so that it has greater impact
scale = 12;
v = v * scale;
fx = fx * scale; 
fy = fy * scale; 
fx_g = fx_g * scale; 
fy_g = fy_g * scale; 
%Loop through points on obstacles and treat as source points
for k=1:1:8
    disp(k)
    xdist = bestEndPoints(2,1,k) - bestEndPoints(1,1,k);
    ydist = bestEndPoints(2,2,k) - bestEndPoints(1,2,k);
    dist = sqrt(xdist.^2 + ydist.^2) * 100; 
    dx = xdist/dist; 
    dy = ydist/dist; 
    xstart = bestEndPoints(1,1,k);
    ystart = bestEndPoints(1,2,k);
    for n=1:1:dist
        a = xstart + dx*n;
        b = ystart + dy*n;
        v = v - log(sqrt((xcon-a).^2 + (ycon-b).^2));
        fx = fx - (xquiv-a)./((xquiv-a).^2 + (yquiv-b).^2); 
        fy = fy - (yquiv-b)./((xquiv-a).^2 + (yquiv-b).^2);
        fx_g = fx_g + (x_g-a)./((x_g-a).^2 + (y_g-b).^2);
        fy_g = fy_g + (y_g-b)./((x_g-a).^2 + (y_g-b).^2);
    end
end

%Draw a contour plot of the potential field
figure(1)
contour(xcon,ycon,v,'k','ShowText','On')
hold on
viscircles([xc yc], bestr);
for kk=1:size(bestEndPoints,3)
    plot(bestEndPoints(:,1,kk), bestEndPoints(:,2,kk), 'r')
end
title('Contour Plot of Potential Field')
%So circles look like circles
axis equal

%Draw a quiver plot of the gradient of the potential field
figure(2)
quiver(xquiv,yquiv,fx,fy)
hold on
viscircles([xc yc], bestr);
for kk=1:size(bestEndPoints,3)
    plot(bestEndPoints(:,1,kk), bestEndPoints(:,2,kk), 'r')
end
title('Quiver Plot of Gradient of Potential Field')
%So circles look like circles
axis equal


%DELIVERABLE 4: A path of gradient descent from the starting point to the
%BOB

%Define initial position
pos=[0; 0];
x_g=pos(1);
y_g=pos(2);

%Calculate initial gradient vector components
g=[fx_g; fy_g];
g_i=subs(g);

step=0.0005;
g_i=double(step*g_i);

disp('Creating gradient descent plot')
figure(3)
viscircles([xc yc], bestr);
hold on
for kk=1:size(bestEndPoints,3)
    plot(bestEndPoints(:,1,kk), bestEndPoints(:,2,kk), 'r')
end
title('Expected Path of Gradient Descent')
%So circles look like circles
axis equal
%Make NEATO travel a predetermined number of steps until it hits the BOB
for i=1:8 
    disp(i)
    %Plot gradient vector
    h1=quiver(x_g,y_g,g_i(1),g_i(2), 'b')
    
    %Calculate next position
    pos=pos+g_i;
   
    %Recalculate gradient
    %lambda=lambda*delta; 
    x_g=pos(1);
    y_g=pos(2);
    g_new=subs(g);
    g_new=double(step*g_new);
    
    g_i=g_new;
end


%DELIVERABLE 5: A plot with the Gauntlet map, the intended path of gradient
%descent, and the actual path of gradient descent (calculated from the
%wheel encoder data). 

%Load in experimental data
load('gauntletpath2.mat')

time=dataset(:,1);
posL=dataset(:,2);
posR=dataset(:,3);

% Remove initial offset from data
time=time-time(1);
posL=posL-posL(1);
posR=posR-posL(1);

% Find velocities
dt=diff(time);
dL=diff(posL);
dR=diff(posR);

vL=dL./dt;
vR=dR./dt;

V=(vL+vR)./2;
d_num=0.235;
om=-(vL-vR)./d_num;

%Plot the experimental left and right wheel velocities as functions of time
r_exp=zeros(length(time),2);
theta_exp=zeros(length(time),1);

for n=1:length(dt)
    r_exp(n+1,1)=r_exp(n,1)+V(n)*cos(theta_exp(n))*dt(n);
    r_exp(n+1,2)=r_exp(n,2)+V(n)*sin(theta_exp(n))*dt(n);
    theta_exp(n+1)=theta_exp(n)+om(n)*dt(n);
end

for k=1:10:length(r_exp)
    h2=quiver(r_exp(k,1),r_exp(k,2),cos(theta_exp(k)),sin(theta_exp(k)),'g')
end
axis equal
legend([h1,h2],'Expected Path','Actual Path')
title('Intended vs Actual Path of Gradient Descent')
hold off
