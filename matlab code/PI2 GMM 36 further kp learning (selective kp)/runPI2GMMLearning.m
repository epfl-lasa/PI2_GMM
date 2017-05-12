function [result] = runPI2GMMLearning(protocol, GMM, random_number_generator_state)

% This is an implementation of a variation of the Pi2 algorithm 
% (Theodorou E, Buchli J, Schaal S (2010) Reinforcement learning in high dimensional 
% state spaces: A path integral approach. Journal of Machine Learning Research) 
% where the control policiy (or virtual trajectory) is represented by GMR instead of DMPs
% 
% This code is adapted from Stefan Schaal's implementation of Pi2 (http://www-clmc.usc.edu/Resources/Software)

tic

global n_param;
global n_dim;         % number of dimension of the problem. Here we assume a 2D problem
global n_dim_kp;
n_dim = 2;
%n_dim_kp = n_dim;
n_dim_kp = 2;

% proces GMM data
global n_Gauss;
global n_in;
global n_out;
global n_subtrials;
global noisyStartPos;

n_in = n_dim;
n_out = n_dim + n_dim_kp; % &&
n_Gauss = length(GMM.Priors);
n_param = n_Gauss*(n_in*n_out+n_out);    % number of parameters in theta, or equivalently total of components of A and b for all the Gaussians &&
n_subtrials = 2;
noisyStartPos = false;

% run protocol
result = runProtocol(protocol,GMM);
result.random_number_generator_state = random_number_generator_state;

end

%--------------------------------------------------------------------------
function result = runProtocol(p,GMMinit)
% runs a particular protocol item, i.e., one line from the protocol specs
global GMM;
global GMR;
global n_param;
global n_dim;
global n_dim_kp;
global n_Gauss;
global n_in;
global n_out;
global n_steps_tot;
global eliteRolloutsTank;
global n_subtrials;
global noisyStartPos;

plotIntermediate = true;

% the policy
GMM.Mu      = GMMinit.Mu;
GMM.Sigma   = GMMinit.Sigma;
GMM.Priors  = GMMinit.Priors;

GMR.A         = zeros(n_out, n_in, n_Gauss);
GMR.b         = zeros(n_out, n_Gauss);
GMR.muInput   = zeros(n_in, n_Gauss);
GMR.sigmaInput = zeros(n_in, n_in, n_Gauss);
GMR.priors     = zeros(1, n_Gauss);                       % all GMR data is shared among roll-outs
GMR.theta      = zeros(n_param, 1);                       % theta is just A and b reshaped, and is the parametrization used (learned) along the algorithm. A and b are maintained and updated for monitoring
GMR.theta0     = zeros(n_param, 1);
GMR.cumulatedActivation = zeros(n_Gauss,1);               % sums the activation weight h of each Gaussian at each rollout and each update

computeGMM2GMR();   % A, b and theta receive here their initial values from the PfD GMM. They will then be updated at each RL iteration

GMR.theta0 = GMR.theta;
% the integration time step is set fixed to 10 milli seconds
dt = 0.1; %%% harmonize


% create the big data matrix for the roll-out data: We simply store all roll-out data in
% this matrix and evalute it late efficiently in vectorized from for learning updates
n_steps_real = p.duration/dt;
n_steps_convergence = p.duration_convergence/dt;
n_steps_tot = n_steps_real + n_steps_convergence;

D.traj_y        = zeros(n_dim, n_steps_tot);
D.traj_yd       = zeros(n_dim, n_steps_tot);
D.h             = zeros(n_Gauss, n_steps_tot);            % the gaussian "activation weights"
D.G             = zeros(n_out, n_param, n_steps_tot);     % the state dependent "control matrix"
D.eps           = zeros(n_param, n_steps_tot);            % the gaussian noise for the coefficient vector theta
D.theta_eps     = zeros(n_param, n_steps_tot);        % used only for reused roll-outs
if p.gaussian_centers == 2
    D.epsInputMu    = zeros(n_in, n_Gauss);         % used only with fixed noise so no n_step dimension
    D.epsInputSigma = zeros(n_in,n_in, n_Gauss);
end

D.q             = zeros(n_dim, n_steps_tot);% point mass pos
D.qd            = zeros(n_dim, n_steps_tot);% point mass vel
D.qdd           = zeros(n_dim, n_steps_tot);% point mass acc
D.u             = zeros(n_dim, n_steps_tot);% point mass command
D.kp            = zeros(n_dim_kp, n_steps_tot);% gaind of the PD controller


D_eval(1:n_subtrials)         = D;  % used for noiseless cost assessment
D_eval_newGMR  = D;
p_eval         = p;  % used for noiseless cost assessment
p_eval.rep     = 1;
p_eval.std     = 0;
p_eval.n_reuse = 0;

n_elite_tank = 10;
eliteRolloutsTank.D(1:n_elite_tank, 1:n_subtrials) = D;
eliteRolloutsTank.cost(1:n_elite_tank) = realmax;
eliteRolloutsTank.updateID(1:n_elite_tank) = 0;


D(1:p.rep,1:n_subtrials) = D;  % one data structure for each repetition

result.cost = zeros(p.updates+1,8);             % the cost with initial parameters is also recorded
result.thetas = zeros(p.updates+1,n_param);     % the initial parameters are also stored

if noisyStartPos
    startPosEps = randn(n_dim,1,n_subtrials); 
else
    startPosEps = zeros(n_dim,1,n_subtrials);
%     startPosEps(:,:,1) = [1;0];
%     startPosEps(:,:,2) = [0;0];
%     startPosEps(:,:,3) = [-1;0];
end

%FWindList = zeros(n_dim,n_subtrials);
FWindList = [[0;0],[0;10]];

nPoints = 100;
for cnt1 = 1: nPoints
    forceFieldTraj(1,cnt1) = -10* (1 - cnt1/nPoints);
    forceFieldTraj(2,cnt1) = sin(2*pi*(1 - cnt1/nPoints));
end

for i=1:p.updates,
  
  % perform one noiseless evaluation to get the cost
  D_eval=run_rollouts(D_eval,p_eval,1, forceFieldTraj, startPosEps,FWindList);
  
  % compute all costs in batch form, as this is faster in matlab
  %eval(sprintf('[genCostSteps_eval, totCostSteps_eval, userCostSteps_eval, R]=%s(D_eval,GMR);',p_eval.cost));
  eval(sprintf('[genCostSteps_eval, totCostSteps_eval, userCostSteps_eval, R, viapointCostSteps, accelerationCostSteps, controlCostSteps, convergenceCost]=%s(D_eval,GMR,p);',p_eval.cost));

 if i==1
   result.D_init = D_eval;
 end
     
  if  mod(i,10)== 1,
    disp(sprintf('%5d.genCost_eval = %f',i,sum(genCostSteps_eval)));
    disp(sprintf('%5d.totCost_eval = %f',i,sum(totCostSteps_eval)));
    disp(sprintf('%5d.userCost_eval = %f',i,sum(userCostSteps_eval)));
    toc
    tic
  end
  
  %result.cost(i,:) = [i*(p.rep-p.n_reuse)+p.n_reuse - 1,[sum(genCostSteps_eval), sum(totCostSteps_eval), sum(userCostSteps_eval)]];
  result.cost(i,:) = [double((i-1)*(p.rep-p.n_reuse)+p.n_reuse),sum(genCostSteps_eval), sum(totCostSteps_eval), sum(userCostSteps_eval), sum(viapointCostSteps), sum(accelerationCostSteps), sum(controlCostSteps), sum(convergenceCost)];
  result.thetas(i,:) = GMR.theta';
 
  % run learning roll-outs with a noise annealing multiplier !! noise
  % decreases along the number of updates
  noise_mult = double(p.updates - i)/double(p.updates);
  noise_mult = max([0.1 noise_mult]);
  D=run_rollouts(D,p,noise_mult, forceFieldTraj, startPosEps, FWindList);
  
  if (i > 1 && p.n_reuse > 0)
     D(1:p.n_reuse,:) = eliteRolloutsTank.D(1:p.n_reuse,:);
     for j = 1:p.n_reuse
         for m = 1:n_subtrials
              D(j,m).eps = D(j,m).theta_eps - repmat(GMR.theta,1,n_steps_tot);    % reused rollouts have their cost recomputed because eps has changed (???)
         end
     end
  end
  
  % compute all costs in batch from, as this is faster vectorized math in matlab
  eval(sprintf('[genCostSteps, totCostSteps, userCostSteps, R_unused, viapointCostSteps, accelerationCostSteps, controlCostSteps]=%s(D,GMR,p);',p.cost));
  
   % reuse of roll-out: the n_reuse best trials and re-evalute them the next update in
  % the spirit of importance sampling    !!! not operational for GMR, how
  % to deal with the fact that theta has been updated and that there is an
  % ambiguity as to what is theta and epsilon for the cost and update
  % calculation
  
  
  if i==1   
    storeElites(D,sum(totCostSteps,1),i); % place here so that no elites from current epoch are injected in rollouts
  else
     storeElites(D(p.n_reuse+1:end,:),sum(totCostSteps(:,p.n_reuse+1:end),1),i);  % rollouts that are already reuse are already in the elite tank
  end

  % visualization: plot roll-outs and nominal policy of this iteration
  if (((mod(i,10)==1 && plotIntermediate) || i==p.updates) && ~p.disable_plotting)                  
    plotGraphs(D,totCostSteps,p,D_eval,totCostSteps_eval,p_eval, i*2,forceFieldTraj)
  end
  
 
      % perform the PI2 update
  if  strcmp(p.PI2_type,'PI2_original') || strcmp(p.PI2_type,'PI2_no_M') || strcmp(p.PI2_type,'PI2_no_M_no_activWeight') || strcmp(p.PI2_type,'PI2_no_activWeight')
      updatePI2(D,p,totCostSteps,R); 
  elseif strcmp(p.PI2_type,'PI2_BB')
       updatePIBB(D,p,totCostSteps,R);
  elseif strcmp(p.PI2_type,'PI_WB')
       updatePIWB(D,p,totCostSteps,R);
  else
      updatePI2_alt(D,p,totCostSteps);
  end       
 
  
  % propagate update to A and b, only for monitoring and for streamlines
  % func and for SEDS noise building
  for j=1:n_Gauss
     GMR.A(:,:,j) = reshape(GMR.theta((j-1)*(n_out*n_in+n_out)+1:(j-1)*(n_out*n_in+n_out)+n_out*n_in),n_in,n_out)';
     GMR.b(:,j) = GMR.theta((j-1)*(n_out*n_in+n_out)+n_out*n_in+1:(j)*(n_out*n_in+n_out));
     [trash isNotNegDef] = chol(-(GMR.A(1:n_dim,:,j)+GMR.A(1:n_dim,:,j)')/2);
     if p.SEDS_constr && isNotNegDef
         'problem with SEDS constraints on A'
     end
     if p.SEDS_constr && sum(GMR.A(1:n_dim,:,j)*zeros(n_dim,1)+GMR.b(1:n_dim,j) ~= zeros(n_dim,1));
         'problem with SEDS constraints on b'
     end
  end

  result.p = p;
end

% perform the final noiseless evaluation to get the final cost
D_eval=run_rollouts(D_eval,p_eval,1, forceFieldTraj, startPosEps,FWindList);

eval(sprintf('[genCostSteps_eval, totCostSteps_eval, userCostSteps_eval, R_unused, viapointCostSteps_eval, accelerationCostSteps_eval, controlCostSteps_eval, convergenceCost_eval]=%s(D_eval,GMR,p);',p_eval.cost));
%eval(sprintf('[genCostSteps_eval, totCostSteps_eval, userCostSteps_eval]=%s(D_eval,GMR);',p_eval.cost));
disp(sprintf('%5d.genCost_eval = %f',i,sum(genCostSteps_eval)));
disp(sprintf('%5d.totCost_eval = %f',i,sum(totCostSteps_eval)));
disp(sprintf('%5d.userCost_eval = %f',i,sum(userCostSteps_eval)));

%record last update cost, parameters and simulation results
result.cost(p.updates+1,:) = [double(p.updates*(p.rep-p.n_reuse)+p.n_reuse),[sum(genCostSteps_eval), sum(totCostSteps_eval), sum(userCostSteps_eval), sum(viapointCostSteps_eval), sum(accelerationCostSteps_eval), sum(controlCostSteps_eval), sum(convergenceCost_eval)]];
%result.cost(p.updates+1,:) = [p.updates*(p.rep-p.n_reuse)+p.n_reuse,[sum(genCostSteps_eval), sum(totCostSteps_eval), sum(userCostSteps_eval)]];
result.thetas(p.updates+1,:) = GMR.theta';
result.Dfin = D_eval;
result.GMR = GMR;

%plot learning curve
if ~p.disable_plotting
    figure;
    hold on
    plot(result.cost(:,1),result.cost(:,3));   % tot cost
    plot(result.cost(:,1),result.cost(:,5),'r'); % viapoint cost
    plot(result.cost(:,1),result.cost(:,6),'g'); % acceleration cost
    plot(result.cost(:,1),result.cost(:,7),'k'); % control cost
    plot(result.cost(:,1),result.cost(:,8),'m'); % convergence cost
    xlabel('Number of roll outs');
    ylabel('TotCost');
    hold off
end


delta_theta = GMR.theta0- GMR.theta;
end
%-------------------------------------------------------------------------------
function updatePI2(D,p, totCostSteps,R)
% D is the data structure of all roll outs, and R the "user defined" cost matrix for these roll outs
global GMR;
global GMM;
global n_param;
global n_steps_tot;
global n_dim;
global n_Gauss;
global n_in;
global n_out;

if strcmp(p.PI2_type,'PI2_original') || strcmp(p.PI2_type,'PI2_no_activWeight')
    useMproj = true;
elseif strcmp(p.PI2_type,'PI2_no_M') || strcmp(p.PI2_type,'PI2_no_M_no_activWeight')
    useMproj = false;
else
    'problem'   % will crash because useMproj not defined, which is a good thing to detect the problem
end

if strcmp(p.PI2_type,'PI2_original') || strcmp(p.PI2_type,'PI2_no_M')
    useActivWeight = true;
elseif strcmp(p.PI2_type,'PI2_no_activWeight') ||  strcmp(p.PI2_type,'PI2_no_M_no_activWeight')
    useActivWeight = false;
else
    'problem'   
end

% computes the parameter update with PI2
[n_steps_real,n_rep] = size(totCostSteps);
% compute the accumulate cost   !! the matrix of the costs is turned upside
% down because the cumulative sum must be done backwards, i.e. it is the
% "cost to go" so the first timesteps include the cost of the later ones.
S = rot90(rot90(cumsum(rot90(rot90(totCostSteps))))); %%% this is S tilda in the paper

% gen
% [val bestrollout] = min(S(1,:));
% dtheta = D( bestrollout).eps(:,1); % fixed noise is same all along

maxS = max(S,[],2);
minS = min(S,[],2);
h = 10;
lambda = (maxS - minS)/h; % for information

expS = exp(-h*(S - minS*ones(1,n_rep))./((maxS-minS)*ones(1,n_rep)));

% % the probabilty of a trajectory 

P = expS./(sum(expS,2)*ones(1,n_rep));


% compute the projected noise term.
dthetaRollouts = zeros(n_param,n_steps_real,n_rep);
if p.gaussian_centers == 2
    dthetaRolloutsInputMu = zeros(n_in,n_Gauss,n,n_rep);
    dthetaRolloutsInputSigma = zeros(n_in,n_in,n_Gauss,n,n_rep);
end

for k=1:n_rep
    for i=1:n_steps_real
        if sum(isnan(D(k).h(:,i)))
            ;
        end
        if sum(D(k).h(:,i))
            G = D(k).G(:,:,i);
           if useMproj
              dthetaRollouts(:,i,k) = P(i,k)*(R\G'/(G/R*G')*G*D(k).eps(:,i));     
           else
              dthetaRollouts(:,i,k) = P(i,k)*D(k).eps(:,i);
           end
           if p.gaussian_centers == 2
                dthetaRolloutsInputMu(:,:,i,k) = P(i,k)*D(k).epsInputMu;
                dthetaRolloutsInputSigma(:,:,:,i,k) = P(i,k)*D(k).epsInputSigma;
            end
        else
            dthetaRollouts(:,i,k) = zeros(n_param,1); % the rollouts that have degenerated G matrix (because to far form the gaussians) are not taken into account for the update
        end
    end
end


% compute the parameter update per time step
dtheta = squeeze(sum( dthetaRollouts,3));
if p.gaussian_centers == 2
    dthetaInputMu = squeeze(sum( dthetaRolloutsInputMu,4));
    dthetaInputSigma = squeeze(sum(dthetaRolloutsInputSigma,5));
    if sum(sum(sum(isnan(dthetaInputMu)))) || sum(sum(sum(sum(isnan(dthetaInputSigma)))))
        'Problem';
    end
end

% average updates over time

% the time weighting matrix (note that this done based on the true duration of the
% movement, while the movement "recording" is done beyond D.duration). Empirically, this
% weighting accelerates learning
m = n_steps_real;
N = (m:-1:1)';
%N = ones(m,1);
%N = [N; ones(n-m,1)]; 
% N = [N; N];  
% the final weighting vector takes the kernel activation into account !!!no
W = (N*ones(1,n_param)).*1; %No activation weight
%%%% final best activation weight
%[stuff, bestRolloutTottraj] = max(P(1,:));
%W = N* reshape(repmat(D(bestRolloutTottraj).h',n_param/n_Gauss,1),n_param*n_steps,1)';
%%%synchro best activation weight
if useActivWeight
    for i = 1:n_steps_real
        [stuff, bestRolloutTottraj] = max(P(i,:));
        for j = 1:n_Gauss
            W(i,(n_param/n_Gauss)*(j-1)+1:(n_param/n_Gauss)*j) = D(bestRolloutTottraj).h(j,i)*N(i);
        end
    end
end

% ... and normalize through time
W = W./(ones(n_steps_real,1)*sum(W,1));
Z = isnan(W);
for i = 1:n_steps_real
    for j = 1:n_param
        if Z(i,j)
            W(i,j) = 0;
        end
        
    end
end


% compute the final parameter update for each DMP
dtheta = squeeze(sum(dtheta.*W',2));
if p.gaussian_centers == 2
    dthetaInputMu = squeeze(sum(dthetaInputMu.*permute(repmat(N,1,n_Gauss,n_in),[3 2 1]),3))/ sum(N);
    dthetaInputSigma = squeeze(sum(dthetaInputSigma.*permute(repmat(N,1,n_Gauss,n_in,n_in),[4 3 2 1]),4))/ sum(N);
end
if sum(isnan(dtheta))
    ;
end
GMR.theta = GMR.theta+dtheta;

if p.gaussian_centers == 2
    computeGMR2GMM();
    GMM.Mu(1:n_in,:) = GMM.Mu(1:n_in,:) + dthetaInputMu;  % cannot update directly GMR.InputMu and GMR.InputSigma because it also changes A and b terms
    GMM.Sigma(1:n_in,1:n_in,:) = GMM.Sigma(1:n_in,1:n_in,:) + dthetaInputSigma;
    
     for j = 1:n_Gauss                      % check that the covariance matrices of the GMM are still positive semi-definite. Otherwise cancel update
         [~,psdfCheck] = chol(GMM.Sigma(1:n_in,1:n_in,j));
         if psdfCheck
              GMM.Sigma(1:n_in,1:n_in,j) = GMM.Sigma(1:n_in,1:n_in,j)  - dthetaInputSigma(:,:,j);
         end
     end
    computeGMM2GMR();
    
end

% and update the parameters in the global GMR object

end

%-------------------------------------------------------------------------------
function dtheta = updatePIBB(D,p, totCostSteps, R)
% D is the data structure of all roll outs, and R the "user defined" cost matrix for these roll outs
global GMR;
global GMM;
global n_param;
global n_steps_tot;
global n_dim;
global n_Gauss;
global n_in;
global n_out;



% computes the parameter update with PI2
[n_steps_real,n_rep] = size(totCostSteps);
% compute the accumulate cost   !! the matrix of the costs is turned upside
% down because the cumulative sum must be done backwards, i.e. it is the
% "cost to go" so the first timesteps include the cost of the later ones.
S = sum(totCostSteps,1); %%% this is S tilda in the paper

maxS = max(S,[],2);
medS = median(S);
minS = min(S,[],2);
h = 10;
lambda = (maxS - minS)/h; % for information

expS = exp(-h*(S - minS*ones(1,n_rep))./((medS-minS)*ones(1,n_rep)+10e-100));

% % the probabilty of a trajectory 

P = expS./(sum(expS,2)*ones(1,n_rep));


% compute the projected noise term.
dthetaRollouts = zeros(n_param,n_rep);
for k = 1:n_rep
    dthetaRollouts(:,k) = P(k)*D(k).eps(:,1); % all eps are the same along time
end    

% compute the parameter update
dtheta = squeeze(sum( dthetaRollouts,2));

if sum(isnan(dtheta))
    ;
end

% and update the parameters in the global GMR object
GMR.theta = GMR.theta+dtheta;

end

function updatePIWB(D,p, totCostSteps,R)
% D is the data structure of all roll outs, and R the "user defined" cost matrix for these roll outs
global GMR;
global GMM;
global n_param;
global n_steps_tot;
global n_dim;
global n_Gauss;
global n_in;
global n_out;



% computes the parameter update with PI2
[n_steps_real,n_rep] = size(totCostSteps);
% compute the accumulate cost   !! the matrix of the costs is turned upside
% down because the cumulative sum must be done backwards, i.e. it is the
% "cost to go" so the first timesteps include the cost of the later ones.
S = rot90(rot90(cumsum(rot90(rot90(totCostSteps))))); %%% this is S tilda in the paper

% gen
% [val bestrollout] = min(S(1,:));
% dtheta = D( bestrollout).eps(:,1); % fixed noise is same all along

maxS = max(S,[],2);
minS = min(S,[],2);
h = 10;
lambda = (maxS - minS)/h; % for information

expS = exp(-h*(S - minS*ones(1,n_rep))./((maxS-minS)*ones(1,n_rep)));

% % the probabilty of a trajectory 

P = expS./(sum(expS,2)*ones(1,n_rep));


% compute the projected noise term.
dthetaRollouts = zeros(n_param,n_rep);


for r=1:n_rep
    for k=1:n_Gauss
        enteringTime = find(D(r).h(k,:)>0.1,1);
        dthetaRollouts((k-1)*n_param/n_Gauss+1:k*n_param/n_Gauss,r) = P(enteringTime,r)*D(r).eps((k-1)*n_param/n_Gauss+1:k*n_param/n_Gauss,1); % all eps are the same for a rollout
    end
end


% compute the parameter update 
dtheta = squeeze(sum( dthetaRollouts,2));

% and update the parameters in the global GMR object

GMR.theta = GMR.theta+dtheta;

end

%-------------------------------------------------------------------------------
function dtheta_reshaped = updatePI2_alt(D, p, totCostSteps)
global GMR;

[bsim_out, bR, method] = prepareForPI201(D,totCostSteps,p);
dtheta = PI2s(method,bsim_out,bR);

n_Gauss = length(GMR.priors);
n_in = size(GMR.muInput,1);
n_out = size(GMR.A,1);
n_param = length(GMR.theta);

for i = 1:n_Gauss;
    for j = 1:n_out
        dtheta_reshaped((i-1)*(n_in*n_out+n_out)+(j-1)*(n_in)+1 : (i-1)*(n_in*n_out+n_out)+(j-1)*(n_in)+n_in) = dtheta((i-1)*(n_in+1)+1: (i-1)*(n_in+1)+n_in,j); % for A terms
        dtheta_reshaped((i-1)*(n_in*n_out+n_out)+(j-1)+ n_in*n_out + 1) = dtheta((i-1)*(n_in+1) + n_in + 1, j); % for B terms
    end
end


GMR.theta = GMR.theta+dtheta_reshaped';
end

%-------------------------------------------------------------------------------
function D=run_rollouts(D,p,noise_mult, forceFieldTraj,varargin)
% a dedicated function to run multiple roll-outs using the specifictions in D and p
% noise_mult allows decreasing the noise with the number of roll-outs, which gives
% smoother converged performance (but it is not needed for convergence).
global GMR
global GMM
global n_param;
global n_dim;
global n_dim_kp;
global n_Gauss;
global n_in;
global n_out;
global n_steps_tot;
global n_subtrials;


GMRbackup = GMR;
GMMbackup = GMM;

% variation for the starting position for the subtrials (should it remain
% the same accross the updates? )
startPosEps = 0;
if nargin>3
    startPosEps = cell2mat(varargin(1));
end

fWindList = zeros(1,n_subtrials);
if nargin>4
    fWindList = cell2mat(varargin(2));
end

dt = p.dt;

% the simulated point mass
mass = 1;
damp = 1;

% run roll-outs
start = p.n_reuse + 1;
if (D(1).u(1,1) == 0) % indicates very first batch of rollouts 
  start = 1;
end

    for k=start:p.rep,

      %%% set up rollout specific fixed noise (if time variable noise
      %%% selected, this fixed noise is overwritten later in timestep loop
     % std_eps = computeStdRatio(p.std,GMR,0) * noise_mult; 
   %  if p.SEDS_constr
       std_eps_base =  p.std * noise_mult;                          % untuned
   %  else
         std_eps = computeStdRatio(p.std,GMR,0) * noise_mult;   % ratio tune, for SEDS only the noise for kp parameters will remain
   %  end
     %epsInit = randn(n_param,1).*std_eps.*repmat([1 1 1 1 1 1 1 1 1 0 0 0 1 1 1 0]', n_Gauss, 1);  % exploration for kp is set to 0 for now
     % epsInit = randn(n_param,1).*std_eps.*repmat([ 1 1 1 1 0 0 1 1 0]', n_Gauss, 1);  % exploration for kp is set to 0 for now
     % epsInit = randn(n_param,1).*std_eps.*repmat([ 1 1 1 1 20 20 1 1 100]', n_Gauss, 1);  % 
      epsInit = randn(n_param,1).*std_eps.*repmat([ 1*ones(1,n_dim*n_dim) 20*ones(1,n_dim*n_dim_kp) 1*ones(1,n_dim) 100*ones(1,n_dim_kp)]', n_Gauss, 1);  % compatible with multi-dim stiffness
      %epsInit = randn(n_param,1).*repmat([1 1 1 1 2 2 1 1 2]', n_Gauss, 1).*std_eps; % if the fixed_noise option is selected, epsilon will keep this value for every time step of the rolout
      %    epsInit = randn(n_param,1).*repmat([0 0 0 0 0 0 1 1 0]', n_Gauss, 1).*std_eps; %  B only
      
      if p.SEDS_constr                             % noise for kp is normally outside of here
          for j = 1:n_Gauss
              temp = randn(n_dim,n_dim).*std_eps_base;             % building symetric part d*S and trying decreasing d until (A+A')/2 + d*S is neg def
              S = triu(temp)+triu(temp,1)';
              A = GMR.A(1:n_dim,:,j);
              for d= 0:0.1:1
                  d = 1-d;
                  [trash notNegDef] = chol(-((A+A')./2 + d*S));
                  if ~notNegDef
                      S = S*d;
                      break;
                  end
              end                   % what to do if d reashes 0? resample S? add no symetric part? (done so now)
              epsA(:,:,j) = S;
              
              temp = randn(n_dim,n_dim).*std_eps_base;           % building random assymetric part (no conditions on this part)
              temp = triu(temp,1);
              AS = temp - temp';                  
              epsA(:,:,j) = epsA(:,:,j) + AS;
%               [trash notNegDef2] = chol(-(epsA(:,:,j)+epsA(:,:,j)')/2);
%               if notNegDef2
%                   'problem with noise'
%               end
              epsB(:,j) = -epsA(:,:,j)*zeros(n_dim,1); % goal is assumed to be 0,0
              
              epsInit((j-1)*(n_param/n_Gauss)+1:(j-1)*(n_param/n_Gauss)+n_dim^2) = reshape(epsA(:,:,j)',n_dim^2,1);
              epsInit((j-1)*(n_param/n_Gauss)+n_dim^2+n_in*n_dim_kp+1:(j-1)*(n_param/n_Gauss)+n_dim^2+n_in*n_dim_kp+n_dim_kp) = epsB(:,j);   % the n_in*n_dim_kp is there for the rows of kp params
          
%           noisyA = A + epsA(:,:,j);
%           [trash notNegDef3] = chol(-(noisyA+noisyA')/2);
%               if notNegDef3
%                   'problem with rollout A matrix'
%               end
          end
      end
          
      
     

        for m=1:n_subtrials

            fWind = fWindList(:,m);
            
            q    = p.start + startPosEps(:,:,m);
            qd   = zeros(n_dim,1);
            traj_y = p.start + startPosEps(:,:,m);

          % integrate for twice the duration to see converence behavior   !!! What
          % is our stop condition? We do not have a fixed duration



          for n=1:n_steps_tot  % !!! see how to do for first step


              eps = epsInit;
              if ~p.fixed_noise
               eps = [];%randn(n_param,1).*repmat([1 1 1 1 1 1 1 1 10]', n_Gauss, 1).*std_eps;
               
              end


              % after duration/dt no noise is added anymore (in case the simulation
              % lasts longer for convergence verification)
        %       if n > p.duration/dt,
        %         eps = 0*eps;
        %       end

              % integrate simulated 2D point mass with inverse dynamics control
              % based on GMR output

              h = zeros(n_Gauss,1);
              for i = 1:n_Gauss
                if p.feedback == 0
                    h(i) = GMR.priors(i) * my_mvnpdf(traj_y, GMR.muInput(:,i), GMR.sigmaInput(:,:,i));  % !!! first step
                end
                if p.feedback == 1
                    h(i) = GMR.priors(i) * my_mvnpdf(q, GMR.muInput(:,i), GMR.sigmaInput(:,:,i)); 
                end
              end
              if sum(h) == 0
                  disp('too far from GMM')
              else
                   h = h/sum(h);
              end
              
              if sum(isnan(h))
                  'problem';
              end
              
              GMR.cumulatedActivation = GMR.cumulatedActivation + h;
              if p.selective_noise

                 for i = 1:n_Gauss
                     [hMax, hMaxIndex] = max(h);
                    if i ~= hMaxIndex
                       eps((i-1)*n_param/n_Gauss+1:i*n_param/n_Gauss) = 0; 
                    end
                 end
              end

              G = zeros(n_out, n_param);
              lengthBloc = n_in*n_out+n_out;
              structG = zeros(n_out,lengthBloc);
              for i = 1:n_out
                if p.feedback == 0
                    structG(i,:) = [zeros(1,(i-1)*(n_in)) traj_y' zeros(1,((n_out)-i)*(n_in)) zeros(1,i-1) 1 zeros(1,(n_out)-i)];
                end
                if p.feedback == 1
                    structG(i,:) = [zeros(1,(i-1)*(n_in)) q' zeros(1,((n_out)-i)*(n_in)) zeros(1,i-1) 1 zeros(1,(n_out)-i)];
                end
              end
              for i = 1:n_Gauss
                  G(:,(i-1)*lengthBloc+1:i*lengthBloc) = h(i)*structG;
              end
              if sum(sum(isnan(G)))
                  ;
              end

              % debugging comparison ^^^^^^^^^^
%               traj_yd2 = 0;
%              for j=1:n_Gauss
%              h2(j) = GMR.priors(j) * mvnpdf(traj_y, GMR.muInput(:,j), GMR.sigmaInput(:,:,j));
%              traj_yd2 = traj_yd2 + h2(j)*(GMR.A(:,:,j)*traj_y + GMR.b(:,j));
%              end
              %^^^^^^^^^^^^^^^^^^^^^^^^

              outputs = G*(GMR.theta+eps);
              traj_yd = outputs(1:n_dim);
%               if (q(1)> 0.15 && q(1)< 0.17 && q(2) < 0.08  && q(2) > -0.24) || (q(1)> -0.17 && q(1)< 0.17 && (q(2) > -0.24 && q(2) < -0.22 ))
%                   traj_yd = traj_yd*0;
%                   qd = qd*0;
%               end
              traj_y = traj_y + traj_yd*dt;
                
              %kp = p.kp0;
              kp =  outputs(n_dim+1:end);
              kp = max(kp, zeros(n_dim_kp,1));   % kp should not be negative
              kd = 2*sqrt(kp);
              
              
              fWindCurrent = zeros(2,1);
              if q(1)> -5
                  fWindCurrent = fWind;
              end


              force = computeForceField(q,forceFieldTraj,10);

              u   =   kp.*(traj_y-q) + kd.*(traj_yd-qd);
              qdd = (u - qd * damp + fWindCurrent)/mass;
              qd  = qdd * dt + qd;
%               if (q(1)> 0.15 && q(1)< 0.17 && q(2) < 0.08  && q(2) > -0.24) || (q(1)> -0.17 && q(1)< 0.17 && (q(2) > -0.24 && q(2) < -0.22 ))
%                   traj_yd = traj_yd*0;
%                   qd = qd*0;
%               end
              q   = qd * dt + q;

              D(k,m).traj_y(:,n)   = traj_y;
              D(k,m).traj_yd(:,n)  = traj_yd;
              D(k,m).h(:,n)   = h;
              D(k,m).G(:,:,n) = G;
              D(k,m).eps(:,n) = eps;
              D(k,m).theta_eps(:,n) = GMR.theta+eps;

              D(k,m).q(:,n)   = q;
              D(k,m).qd(:,n)  = qd;
              D(k,m).qdd(:,n) = qdd;
              D(k,m).u(:,n)   = u;
              D(k,m).kp(:,n) = kp;

          end

          if p.gaussian_centers == 2 
             GMR = GMRbackup;  % recovering the initial unchanged GMR and GMM for next rollout. The noise applied to the GMM input parameters is saved in D for the update calculation
             GMM = GMMbackup; 
          end

        end
    end
    showRollouts = false;
    if showRollouts && p.rep>1 && ~p.disable_plotting
        for k = 1:p.rep
            allRolloutTrajs(k,:,1) = D(k,:).q(1,:);
            allRolloutTrajs(k,:,2) = D(k,:).q(2,:);
        end
        
        figure
        plot(allRolloutTrajs(:,:,1)',allRolloutTrajs(:,:,2)','.')

    end
end

%-------------------------------------------------------------------------------
function computeGMM2GMR
global GMM;
global GMR;
global n_Gauss;
global n_in;
global n_out;

    GMR.priors = GMM.Priors;
    for j=1:n_Gauss
        GMR.muInput(:,j)        = GMM.Mu(1:n_in,j);
        GMR.sigmaInput(:,:,j)   = GMM.Sigma(1:n_in,1:n_in,j);
        GMR.A(:,:,j)  = GMM.Sigma(n_in+1:n_in+n_out,1:n_in,j)/GMR.sigmaInput(:,:,j); 
        GMR.b(:,j)    = GMM.Mu(n_in+1:n_in+n_out,j) - GMR.A(:,:,j)*GMR.muInput(:,j);
        GMR.theta((j-1)*(n_out*n_in+n_out)+1:j*(n_out*n_in+n_out)) = [reshape(GMR.A(:,:,j)',n_out*n_in,1);GMR.b(:,j)];
    end
end

function computeGMR2GMM
    global GMM;
    global GMR;
    global n_Gauss;
    global n_in;
    global n_out
    
    % update A and b based on theta
      for j=1:n_Gauss
     GMR.A(:,:,j) = reshape(GMR.theta((j-1)*(n_out*n_in+n_out)+1:(j-1)*(n_out*n_in+n_out)+n_out*n_in),n_in,n_out)';
     GMR.b(:,j) = GMR.theta((j-1)*(n_out*n_in+n_out)+n_out*n_in+1:(j)*(n_out*n_in+n_out));
      end

    % compute GMM
    GMM.Prior = GMR.priors;
    for j = 1:n_Gauss
       GMM.Mu(1:n_in,j) = GMR.muInput(:,j);
       GMM.Sigma(1:n_in,1:n_in,j) = GMR.sigmaInput(:,:,j);
       GMM.Sigma(n_in+1:n_in+n_out,1:n_in,j) = GMR.A(:,:,j)*GMM.Sigma(1:n_in,1:n_in,j);
       GMM.Sigma(1:n_in,n_in+1:n_in+n_out,j) = GMM.Sigma(n_in+1:n_in+n_out,1:n_in,j)';
       GMM.Mu(n_in+1:n_in+n_out,j) = GMR.A(:,:,j)*GMR.muInput(:,j) + GMR.b(:,j);
    end
% Sigma_xdot does not need to be recomputed because it does not change. GMR
% does not hold any information about it because it does not use it.
end

function killNspawnGaussian
    %%%%%%%
    % This function recompute the position in input space of one specified Gaussian
    % of the GMM to put it in a area that covers datapoints that are poorly
    % represented by the other Gaussians. The purpose it to discover new
    % meaninful states that can give more flexibility to the GMR and make the learning process easier.
    %%%%
    global GMM;
    global eliteRolloutsTank;
    global n_Gauss;
    global n_in;
    global n_out
    
   Data = [];
    for i = 1:length(eliteRolloutsTank.cost)
        Data = [Data eliteRolloutsTank.D(:,i).traj_y]; 
    end
    
    %%% strip GMM to input dimensions only
    Mu = GMM.Mu(1:n_in,:); %%% it is assumed the input dimensions come first in the GMM
    Sigma = GMM.Sigma(1:n_in,1:n_in,:);
    Priors = GMM.Priors;  %%% the priors will actually remain untouches, i.e the new Gaussian will inherit the prior from the killed one. Better options?

    
     % E-step %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  Pxi = zeros(size(Data,2),n_Gauss);
  Pix = zeros(size(Data,2),n_Gauss);
  for i=1:n_Gauss
    %Compute probability p(x|i)
    Pxi(:,i) = gaussPDF(Data, Mu(:,i), Sigma(:,:,i));
  end
  %Compute posterior probability p(i|x)
  for j=1:size(Data,2)
    Pix(j,:) = (Priors.*Pxi(j,:))./(sum(Priors.*Pxi(j,:))+realmin);
  end
  %Compute cumulated posterior probability
  Egauss = sum(Pix);
  Edatapoints = sum(Pxi,2);
  
  [minLikelihood, killedGaussID] = min(Egauss)
  if minLikelihood < 1
      Mu(:,killedGaussID) = Data*(ones(length(Edatapoints),1)./(1+Edatapoints*100)) ./ sum(ones(length(Edatapoints),1)./(1+Edatapoints*100));
      %Sigma(:,:,killedGaussID) = diag(ones(n_in,1))*10; 
      Sigma(:,:,killedGaussID) = cov(Data');
  else
      return;
  end
  
    %%% reset killed Gaussian to center and high variance to cover all
    %%% (populated) space. This will enable the use of the M-step
   % Mu(:,killedGaussID) = mean(Data,2);
   % Sigma(:,:,killedGaussID) = diag(ones(n_in,1))*10;   %%% how to choose Sigma "big"?
    
  % and recompute p(x|i) and p(i|x)
  for i=1:n_Gauss
  %Compute probability p(x|i)
  Pxi(:,i) = gaussPDF(Data, Mu(:,i), Sigma(:,:,i));
  end
  %Compute posterior probability p(i|x)
  for j=1:size(Data,2)
    Pix(j,:) = (Priors.*Pxi(j,:))./(sum(Priors.*Pxi(j,:))+realmin);
  end
  %Compute cumulated posterior probability
  Egauss = sum(Pix);
  
  
  % M-step %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
%Update the priors
%Priors(KilledGaussID) = E(KilledGaussID) / nbData;
%Update the centers
%newMu = Data*Pix(:,killedGaussID) / Egauss(killedGaussID);

%Update the covariance matrices 
covtmp = zeros(n_in,n_in);

for j=1:size(Data,2)
  covtmp = covtmp + (Data(:,j)-Mu(:,killedGaussID))*(Data(:,j)-Mu(:,killedGaussID))'.*Pix(j,killedGaussID) * 1/(1+Edatapoints(j));
end
 Sigma(:,:,killedGaussID) = covtmp / Egauss(killedGaussID) / sum(ones(length(Edatapoints),1)./(1+Edatapoints))*size(Data,2);

%%% set the output dimensions to zero. The spawn of the new Gaussian will
%%% not perturbe the previous GMR at first. The output parameters will then
%%% be learned by PI2
newMu = [Mu(:,killedGaussID); zeros(n_out,1)];
newSigma = [Sigma(:,:,killedGaussID), zeros(n_in,n_out); zeros(n_out,n_in), zeros(n_out,n_out)];

GMM.Mu(:,killedGaussID) = newMu;
GMM.Sigma(:,:,killedGaussID) = newSigma;
end


function storeElites(D,cost,updateID)
global eliteRolloutsTank;
    
    updateID = [repmat(updateID,1,length(cost)) eliteRolloutsTank.updateID];
    D = [D; eliteRolloutsTank.D];
    cost = [cost eliteRolloutsTank.cost];
    
    [cost_new,inds]=sort(cost);
    
    for j=1:length(cost)
      D_new(j,:) = D(inds(j),:);
      updateID_new(j) = updateID(inds(j));
    end
    
    eliteRolloutsTank.D  = D_new(1:length(eliteRolloutsTank.cost),:);
    eliteRolloutsTank.cost = cost_new(1:length(eliteRolloutsTank.cost));
    eliteRolloutsTank.updateID = updateID_new(1:length(eliteRolloutsTank.cost));
end


%-------------------------------------------------------------------------------
function plotGraphs(D,totCostSteps,p,D_eval,totCostSteps_eval,p_eval, figID,forceFieldTraj)
% plots various graphs for one set of roll-outs and the noiseless realization

global GMR;
global n_param;
global n_dim;
global n_dim_kp;
global n_Gauss;
global n_in;
global n_out;
global n_subtrials;

gray = [0.5 0.5 0.5];

%dt = p.dt;
%T  = (1:length(D(1).q))'*dt;
T_real = 0.1:p.dt:p.duration;
T_tot = 0.1:p.dt:p.duration+p.duration_convergence;
TT  = zeros(length(D(1).q),p.rep);
TTT = zeros(length(D(1).q),p.rep*n_param);

%figure(figID);
figure
clf;

% pos, vel, acc
for j=1:n_dim;
  
  % desired traj position
  subplot(n_dim+1,5,(j-1)*5+1);
  
  for k=1:p.rep,
      TT(:,k)=D(k).traj_y(j,:);
  end
  plot(T_tot,D_eval(1).traj_y(j,:),'Color',gray,'LineWidth',2);
  hold on;
  plot(T_tot,TT);
  hold off;
  ylabel(sprintf('y_%d',j));
  
  title(sprintf('Dimension_%d',j));
  
  % desired traj velocity
  subplot(n_dim+1,5,(j-1)*5+2);
    
  for k=1:p.rep,
      TT(:,k)=D(k).traj_yd(j,:);
  end
  plot(T_tot,D_eval(1).traj_yd(j,:),'Color',gray,'LineWidth',2);
  hold on;
  plot(T_tot,TT);
  hold off;
  ylabel(sprintf('yd_%d',j));
  
  % desired traj acceleration    !!! replace by point mass acceleration?
  % (and reorder)
%   subplot(2*n_dim,5,(j-1)*10+3);
%   
%   for k=1:p.rep,
%       TT(:,k)=D(k).dmp(j).ydd;
%   end
%   plot(T,D_eval(1).dmp(j).ydd,'Color',gray,'LineWidth',2);
%   hold on;
%   plot(T,TT);
%   hold off;
%   ylabel(sprintf('ydd_%d',j));
  
  % point mass position
  subplot(n_dim+1,5,(j-1)*5+3);
  
  for k=1:p.rep,
      TT(:,k)=D(k).q(j,:);
  end
  plot(T_tot,D_eval(1).q(j,:),'Color',gray,'LineWidth',2);
  hold on;
  plot(T_tot,TT);
  hold off;
  ylabel(sprintf('q_%d',j));

  % point mass velocity
  subplot(n_dim+1,5,(j-1)*5+4);
    
  for k=1:p.rep,
      TT(:,k)=D(k).qd(j,:);
  end
  plot(T_tot,D_eval(1).qd(j,:),'Color',gray,'LineWidth',2);
  hold on;
  plot(T_tot,TT);
  hold off;
  ylabel(sprintf('qd_%d',j));


  % kp
  subplot(n_dim+1,5, (j-1)*5+5);
  if n_dim_kp == 1
      dim_kp = 1;
  else
      dim_kp = j;
  end
  for k=1:p.rep,
      TT(:,k)=D(k).kp(dim_kp,:);
  end
  plot(T_tot,D_eval(1).kp(dim_kp,:),'Color',gray,'LineWidth',2);
  hold on;
  plot(T_tot,TT);
  hold off;
  ylabel(sprintf('kp_%d',j));
  
end
  
  % the cost
  subplot(n_dim+1,5, n_dim*5+2);

  plot(T_real,totCostSteps_eval,'Color',gray,'LineWidth',2);
  hold on;
  plot(T_real,totCostSteps);
  ylabel(sprintf('step cost r'));
  
  % the cumulative cost
 subplot(n_dim+1,5, n_dim*4+3);

  S_eval = rot90(rot90(cumsum(rot90(rot90(totCostSteps_eval)))));
  S = rot90(rot90(cumsum(rot90(rot90(totCostSteps)))));

  plot(T_real,S_eval,'Color',gray,'LineWidth',2);
  hold on;
  plot(T_real,S);
  ylabel(sprintf('R=sum(r)'));
  hold off

%   subplot(n_dim+1,4, n_dim*4+3);
% 
%   bar(GMR.theta);
%   ylabel('theta');
%   axis('tight');
 


%figure(figID+1);
figure
hold on
if n_dim == 2;
    for m = 1:n_subtrials
        plot(D_eval(1,m).q(1,1:p.duration/p.dt),D_eval(1,m).q(2,1:p.duration/p.dt),'b.');
        plot(D_eval(1,m).traj_y(1,1:p.duration/p.dt),D_eval(1,m).traj_y(2,1:p.duration/p.dt),'r.');
        plot(D_eval(1,m).q(1,p.duration/p.dt+1:end),D_eval(1,m).q(2,p.duration/p.dt+1:end),'c.');
        plot(D_eval(1,m).traj_y(1,p.duration/p.dt+1:end),D_eval(1,m).traj_y(2,p.duration/p.dt+1:end),'y.');
    end  
    xlabel('x1')
    ylabel('x2');
elseif n_dim == 3;
    for m = 1:n_subtrials
        plot3(D_eval(1,m).q(1,1:p.duration/p.dt),D_eval(1,m).q(2,1:p.duration/p.dt),D_eval(1,m).q(3,1:p.duration/p.dt),'b.');
        plot3(D_eval(1,m).traj_y(1,1:p.duration/p.dt),D_eval(1,m).traj_y(2,1:p.duration/p.dt),D_eval(1,m).traj_y(3,1:p.duration/p.dt),'r.');
        plot3(D_eval(1,m).q(1,p.duration/p.dt+1:end),D_eval(1,m).q(2,p.duration/p.dt+1:end),D_eval(1,m).q(3,p.duration/p.dt+1:end),'c.');
        plot3(D_eval(1,m).traj_y(1,p.duration/p.dt+1:end),D_eval(1,m).traj_y(2,p.duration/p.dt+1:end),D_eval(1,m).traj_y(3,p.duration/p.dt+1:end),'y.');
    end
    xlabel('x1')
    ylabel('x2');
    zlabel('x3'); 
end
title('path of point mass & desired traj');
    
if n_dim == 2;
    GMRreduced = GMR;
    GMRreduced.A = GMR.A(1:n_dim,:,:);    % get rid of kp dimension for using streamlines
    GMRreduced.b = GMR.b(1:n_dim,:,:);
    currentAxis = axis;
    showStreamlines(GMRreduced,[-11 ,1 -5 ,5],50); %%% line
    %showStreamlines(GMRreduced,[-8 ,4 -4 ,8],50); %%% box
    %showStreamlines(GMRreduced,[-60 ,20 -30 ,50],50); %%% P
    %showStreamlines(GMRreduced,[-160 ,20 -120 ,20],50); %%% N
    %showStreamlines(GMRreduced,axis + 30*sign(axis),50);
    %showStreamlines(GMRreduced,axis,50);
    %showStreamlines(GMRreduced,currentAxis + 0.8*[currentAxis(1)-currentAxis(2) currentAxis(2)-currentAxis(1) currentAxis(3)-currentAxis(4) currentAxis(4)-currentAxis(3)],50);
end


% scatter(forceFieldTraj(1,:),forceFieldTraj(2,:),'k')
% nSample = 20;
% for cnt2 = 1:nSample
%     x = cnt2*(0+10)/nSample - 10;
%     for cnt3 = 1:nSample
%         y = cnt3*(2+2)/nSample - 2;
%         force = computeForceField([x;y],forceFieldTraj,0.2);
%         quiver(x,y,force(1),force(2),'g')
%     end
% end
    
% learning param !!!

if n_dim ==2
    my_plotGMM(GMR.muInput, GMR.sigmaInput, 'r', 1);
    scatter(GMR.muInput(1,:),GMR.muInput(2,:),[],'r')
elseif n_dim ==3
    scatter3(GMR.muInput(1,:),GMR.muInput(2,:),GMR.muInput(3,:),[],'r')
end
hold off

drawnow;
end
