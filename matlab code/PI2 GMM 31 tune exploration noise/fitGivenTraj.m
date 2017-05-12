function [genCostSteps, totCostSteps, userCostSteps, R] = fitGivenTraj(D,GMR)
% genCostSteps corresponds to S tilda before the summation of timesteps, totCostSteps corresponds to the
% S before the summation of timesteps, userCostSteps is the cost comming from
% the user defined cost function (before summation)

% implements a simple squared acceleration cost function with a penalty on
% the length of the parameter vector. Additionally, a via point has to be traversed at
% certain moment of time


n_rep = length(D);
n      = length(D(1).q);         % the length of a trajectory in time steps
n_real = D(1).duration/D(1).dt;  % the duration of the core trajectory in time steps --
                                 % everything beyond this time belongs to the terminal cost
                                 
givenTraj = load('givenTraj');
givenTraj = givenTraj.givenTraj

% compute cost
% RR = D.RctrlCost; % disregard the lambda factor, to allow the transition
% cost to be scaled down;
QQ = 0.001;
R = 1*eye(length(GMR.theta));

% the "transition" cost

   % !!! R should be a matrix
transCostSteps1 = zeros(n_real,n_rep);
transCostSteps2 = zeros(n_real,n_rep);
transCostSteps1_lastValid = 0;
transCostSteps2_lastValid = 0;

for k = 1:n_rep
    for i = 1:n_real
        G = D(k).G(:,:,i);
        H = G/R*G'; % H can be 0 if h are zero because too far from Gaussian. Then M is NaN and so is cost
        B = G*G';
        if sum(D(k).h(:,i))
            M = R\G'/H*G;
            transCostSteps1(i,k) = 0.5 * (GMR.theta - GMR.theta0 + M*D(k).eps(:,i))'*R*(GMR.theta - GMR.theta0 + M*D(k).eps(:,i));
            transCostSteps1_lastValid =  transCostSteps1(i,k);
            transCostSteps2(i,k) = 0.5 * log(det(B));
            transCostSteps2_lastValid =  transCostSteps2(i,k);
        else
            transCostSteps1(i,k) = transCostSteps1_lastValid;  %  this happends when point is too far from Gaussians. Then point becomes immobile so it makes sens to give last valid cost. Probably not key part of code.
            transCostSteps2(i,k) = transCostSteps2_lastValid;
        end
        
    end
end

%--------------------------------------------------------------------------
% "user defined" cost
userCostSteps  = zeros(n_real, n_rep);
for k=1:n_rep,
    
  for i=1:n_real,
    % cost during trajectory
    userCostSteps(i,k)  = 0.5 * QQ * norm(D(k).qdd(:,i))^2;
    userCostSteps(i,k)  = userCostSteps(i,k) + 10*D(k).kp(i);
    userCostSteps(i,k) = 1000* (D(k).q(1,i) - givenTraj(1,i))^2 + (D(k).q(2,i) - givenTraj(2,i))^2;
  end
    

end
%--------------------------------------------------------------------------

genCostSteps = transCostSteps1 + transCostSteps2 + userCostSteps;
%!!!!!!!
%genCostSteps = transCostSteps1 + userCostSteps;
totCostSteps = transCostSteps1 + userCostSteps;
