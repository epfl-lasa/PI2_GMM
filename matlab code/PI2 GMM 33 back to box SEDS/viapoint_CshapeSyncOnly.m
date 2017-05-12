function [genCostSteps, totCostSteps, userCostSteps, R, viapointCostSteps, accelerationCostSteps, controlCostSteps, convergenceCost] = viapoint_Nshape1SyncOnly(D,GMR,p)
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
QQ = 0.0005;
R = 0.1*eye(length(GMR.theta));

viapointCostWeight = 1;
convergenceDistWeight = 0;
convergenceSpeedWeight = 0;

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
accelerationCostSteps = zeros(n_steps_real, n_rep, n_subtrials);
convergenceCost = zeros(n_steps_convergence, n_rep, n_subtrials);
for k=1:n_rep,
    for m= 1:n_subtrials

      for n=1:n_steps_real,
        % cost during trajectory
        accelerationCostSteps(n,k)  = 0.5 * QQ * norm(D(k,m).qdd(:,n))^2;
      end


      % pass through the viapoints
      
      
      [minVal, Index] =  min((D(k,m).q(1,1:n_steps_real)+30).^2+(D(k,m).q(2,1:n_steps_real)-100).^2);
      viapointCostSteps(Index,k,m) = viapointCostSteps(Index,k,m)+ viapointCostWeight* minVal;
      [minVal, Index] =  min((D(k,m).q(1,1:n_steps_real)+20).^2+(D(k,m).q(2,1:n_steps_real)-30).^2);
      viapointCostSteps(Index,k,m) = viapointCostSteps(Index,k,m)+ viapointCostWeight* minVal;
      
      % reach goal
      viapointCostSteps(end,k,m) = viapointCostSteps(end,k,m)+ viapointCostWeight*((D(k,m).q(1,n_steps_real)-0).^2+(D(k,m).q(2,n_steps_real)+0).^2);
      
      

      
      % cost for convergence, if p.duration.convergence >0
      for n = n_steps_real+1 : n_steps_real+n_steps_convergence
          convergenceCost(n-n_steps_real,k,m) = convergenceCost(n-n_steps_real,k,m)+  convergenceDistWeight*((D(k,m).q(1,n)-0).^2+(D(k,m).q(2,n)+0).^2);
          convergenceCost(n-n_steps_real,k,m) = convergenceCost(n-n_steps_real,k,m)+  convergenceSpeedWeight*(D(k,m).qd(1,n)+D(k,m).qd(2,n));
      end
    end
end

%--------------------------------------------------------------------------
% mean of the subtrials
viapointCostSteps = mean(viapointCostSteps,3);    
convergenceCost = mean(convergenceCost,3);
accelerationCostSteps = mean(accelerationCostSteps,3);
transCostSteps1 = mean(transCostSteps1,3);
transCostSteps2 = mean(transCostSteps2,3);
%--------------------------------------------------------------------------
userCostSteps = viapointCostSteps + accelerationCostSteps ;
userCostSteps(end,:,:) = userCostSteps(end,:,:) + sum(convergenceCost,1);
controlCostSteps = transCostSteps1;
genCostSteps = transCostSteps1 + transCostSteps2 + userCostSteps;
totCostSteps = transCostSteps1 + userCostSteps;
