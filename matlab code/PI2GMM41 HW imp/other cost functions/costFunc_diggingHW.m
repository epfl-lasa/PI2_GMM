function [genCostSteps, totCostSteps, userCostSteps, R, viapointCostSteps, accelerationCostSteps, controlCostSteps, dirtCostSteps, forcesCostSteps] = costFunc_diggingHW(D,GMR,p)
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

% compute cost
% RR = D.RctrlCost; % disregard the lambda factor, to allow the transition
% cost to be scaled down;
QQ = 0.0001;
R = 0.000001*eye(length(GMR.theta));


viapointCostWeight = 10;
dirtCostWeight = 1;
forcesCostWeight = 0.00001;

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
% "user defined" cost
viapointCostSteps = zeros(n_steps_real, n_rep, n_subtrials);
dirtCostSteps = zeros(n_steps_real, n_rep, n_subtrials);
accelerationCostSteps = zeros(n_steps_real, n_rep, n_subtrials);
forcesCostSteps = zeros(n_steps_real, n_rep, n_subtrials);
for k=1:n_rep,
    for m= 1:n_subtrials

      for n=1:n_steps_real,
        % cost during trajectory
        accelerationCostSteps(n,k)  =  QQ * norm(D(k,m).qdd(:,n))^2;
        Forces = D(k,m).real_F(:,n);
        Forces(3,:) = max(Forces(3,:),0);
        forcesCostSteps(n,k)        = forcesCostWeight*norm(Forces)^2;
      end

      % weight of dirt on shovel at the end
      
      dirtCostSteps(end,k,m) = dirtCostWeight*(0.6*9.8 + D(k,m).real_F(3,end)); % 170 grams is the max capacity for the shovel (volume wise) / 550g for humid sand
     
      % reach goal
      viapointCostSteps(end,k,m) = viapointCostSteps(end,k,m)+ viapointCostWeight*((D(k,m).q(1,n_steps_real)-0).^2+(D(k,m).q(2,n_steps_real)+0).^2 + (D(k,m).q(3,n_steps_real)+0).^2);

    end
end

%--------------------------------------------------------------------------
% mean of the subtrials
viapointCostSteps = mean(viapointCostSteps,3);    
dirtCostSteps = mean(dirtCostSteps,3);
accelerationCostSteps = mean(accelerationCostSteps,3);
forcesCostSteps = mean(forcesCostSteps,3);
transCostSteps1 = mean(transCostSteps1,3);
transCostSteps2 = mean(transCostSteps2,3);
%--------------------------------------------------------------------------
userCostSteps = viapointCostSteps + accelerationCostSteps + dirtCostSteps + forcesCostSteps;
controlCostSteps = transCostSteps1;
genCostSteps = transCostSteps1 + transCostSteps2 + userCostSteps;
totCostSteps = transCostSteps1 + userCostSteps;
