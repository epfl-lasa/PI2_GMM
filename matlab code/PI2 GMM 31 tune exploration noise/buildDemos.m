function [demos, dt] = buildDemos()

duration = 10; %sec
dt = 0.1; % not passed to PI2 for now
n_demos = 10;

n_steps = duration/dt;
baseVelocity = [1;0];
baseStartPos = [0;0];
noise_std = 0.5;

demos = cell(1,n_demos);

for k = 1:n_demos
    
    traj = zeros(2,n_steps);
    traj(:,1) = baseStartPos + [0.5 0; 0 1]*randn(2,1);
    
    velNoiseTot = 0;
    for i = 2: n_steps
        velNoise = noise_std*randn(2,1);
        velNoiseTot = velNoiseTot + velNoise;
        velNoiseMean = velNoiseTot/i;
        traj(:,i) = traj(:,i-1) + (baseVelocity + velNoise - velNoiseMean) * dt;
    end
    demos{k} = traj;
end

figure
hold on
for k = 1:n_demos
    scatter(demos{k}(1,:),demos{k}(2,:),'.');
end
axis equal