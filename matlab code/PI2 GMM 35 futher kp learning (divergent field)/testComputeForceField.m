
nPoints = 100;
for cnt1 = 1: nPoints
    forceFieldTraj(1,cnt1) = -10* (1 - cnt1/nPoints);
    forceFieldTraj(2,cnt1) = sin(2*pi*(1 - cnt1/nPoints));
end

figure
hold on
scatter(forceFieldTraj(1,:),forceFieldTraj(2,:))

% ax_x=linspace(-12,2,100); 
% ax_y=linspace(-5,5,100);
% [x_tmp y_tmp]=meshgrid(ax_x,ax_y);


nSample = 20;
for cnt2 = 1:nSample
    x = cnt2*(0+5)/nSample - 5;
    for cnt3 = 1:nSample
        y = cnt3*(2+2)/nSample - 2;
        force = computeForceField([x;y],forceFieldTraj,0.2);
        quiver(x,y,force(1),force(2))
    end
end