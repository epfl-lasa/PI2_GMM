function [genCostSteps, totCostSteps, userCostSteps, R, viapointCostSteps, accelerationCostSteps, controlCostSteps, dirtCostSteps, forcesCostSteps, stiffCostSteps] = costFunc_diggingImpHWgravel(D,GMR,p)
% genCostSteps corresponds to S tilda before the summation of timesteps, totCostSteps corresponds to the
% S before the summation of timesteps, userCostSteps is the cost comming from
% the user defined cost function (before summation)

% implements a simple squared acceleration cost function with a penalty on
% the length of the parameter vector. Additionally, a via point has to be traversed at
% certain moment of time


n_rep = size(D,1);
n_subtrials = size(D,2);

n_steps_real = p.duration/p.dt;  % the duration of the core trajectory in time steps --
n_steps_convergence = p.duration_convergence/p.dt;     % everything beyond this time belongs to the terminal cost

%%% weights
QQ = 0.0001; % acceleration weight
R = 0.000000000001*eye(length(GMR.theta)); % "control cost" weight

viapointCostWeight = 10;
dirtCostWeight = 1;
forcesCostWeight = 0.000001;
stiffCostWeight = 0.0000005;

%---------------------------------------------------------------------------------------------------------------
% the "transition" cost

transCostSteps1 = zeros(n_steps_real,n_rep,n_subtrials);
transCostSteps2 = zeros(n_steps_real,n_rep,n_subtrials);

for k = 1:n_rep
    for m = 1:n_subtrials
        for n = 1:n_steps_real
            G = D(k,m).G(:,:,n);
            H = G/R*G'; % H can be 0 if h are zero because too far from Gaussian. Then M is NaN and so is cost
            B = G*G';
            if sum(D(k,m).h(:,n))
                M = R\G'/H*G;
                transCostSteps1(n,k,m) = 0.5 * (GMR.theta - GMR.theta0 + M*D(k,m).eps(:,n))'*R*(GMR.theta - GMR.theta0 + M*D(k,m).eps(:,n));
                transCostSteps1_lastValid =  transCostSteps1(n,k,m);
                transCostSteps2(n,k,m) = 0.5 * log(det(B));
                transCostSteps2_lastValid =  transCostSteps2(n,k,m);
            else
                transCostSteps1(n,k,m) = transCostSteps1_lastValid;  %  this happends when point is too far from Gaussians. Then point becomes immobile so it makes sens to give last valid cost. Probably not key part of code.
                transCostSteps2(n,k,m) = transCostSteps2_lastValid;
            end
        end
    end
end

%--------------------------------------------------------------------------
%%% "user defined" cost
viapointCostSteps = zeros(n_steps_real, n_rep, n_subtrials);
dirtCostSteps = zeros(n_steps_real, n_rep, n_subtrials);
accelerationCostSteps = zeros(n_steps_real, n_rep, n_subtrials);
forcesCostSteps = zeros(n_steps_real, n_rep, n_subtrials);
stiffCostSteps = zeros(n_steps_real, n_rep, n_subtrials);

for k=1:n_rep,
    for m= 1:n_subtrials

    real_qdd = [diff(smooth(diff(D(k,m).real_q(1,:)),100))' ; diff(smooth(diff(D(k,m).real_q(2,:)),100))' ; diff(smooth(diff(D(k,m).real_q(3,:)),100))'] / p.dt^2;
    real_qdd(:,end+1:end+2) = 0; % zero padding

      for n=1:n_steps_real,
        % cost during trajectory
        accelerationCostSteps(n,k)  =  QQ * norm(real_qdd(:,n))^2;
        Forces = D(k,m).real_F(:,n);
        Forces(3,:) = max(Forces(3,:),0);
        forcesCostSteps(n,k)        = forcesCostWeight*norm(Forces)^2;
        stiffCostSteps(n,k) = stiffCostWeight*D(k,m).kp(1,n);
      end

      % weight of dirt on shovel at the end
      
      dirtCostSteps(end,k,m) = dirtCostWeight*(0.35*9.8 + mean(D(k,m).real_F(3,end-50:end-40)));  % 300 grams is the max capacity for the shovel with dry gravel 
     
      % reach goal
      viapointCostSteps(end,k,m) = viapointCostSteps(end,k,m)+ viapointCostWeight*((D(k,m).real_q(1,n_steps_real)-0).^2+(D(k,m).real_q(2,n_steps_real)+0).^2 + (D(k,m).real_q(3,n_steps_real)+0).^2);

    end
end

forcesCostSteps(end-50:end) = 0; % compensate for stiffening acceleration induced forces

%--------------------------------------------------------------------------
% mean of the subtrials
viapointCostSteps = mean(viapointCostSteps,3);    
dirtCostSteps = mean(dirtCostSteps,3);
accelerationCostSteps = mean(accelerationCostSteps,3);
forcesCostSteps = mean(forcesCostSteps,3);
stiffCostSteps = mean(stiffCostSteps,3);
transCostSteps1 = mean(transCostSteps1,3);
transCostSteps2 = mean(transCostSteps2,3);
%--------------------------------------------------------------------------
userCostSteps = viapointCostSteps + accelerationCostSteps + dirtCostSteps + forcesCostSteps + stiffCostSteps;
controlCostSteps = transCostSteps1;
genCostSteps = transCostSteps1 + transCostSteps2 + userCostSteps;
totCostSteps = transCostSteps1 + userCostSteps;
