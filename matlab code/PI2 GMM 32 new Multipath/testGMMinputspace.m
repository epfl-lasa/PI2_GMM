close all


%[demos, dt] = buildDemos();

%p = readProtocol('protocol_batch.txt');


    load('NShape.mat')
    n_demos = size(demos,2);
    n_steps = size(demos{1},2);
    kp0 = 1;

    nbGauss = 4;
    useSEDS = false;

    clear trainingData; %otherwise conflict if a SEDS protocol follows a non SEDS one
    if useSEDS
        % defining dataset without kp for SEDS
        for cnt2 = 1:1%n_demos
      trainingData(:,(cnt2-1)*(n_steps-1)+1:cnt2*(n_steps-1)) = [demos{cnt2}(1,1:n_steps-1); demos{cnt2}(2,1:n_steps-1); diff(demos{cnt2}(1,:))/dt; diff(demos{cnt2}(2,:)/dt)];
      %trainingData(:,(cnt2-1)*(n_steps-1)+1:cnt2*(n_steps-1)) = [demos{cnt2}(1,1:n_steps-1); demos{cnt2}(2,1:n_steps-1)];
        end
            options.tol_mat_bias = 10^-6; % A very small positive scalar to avoid
                                          % instabilities in Gaussian kernel [default: 10^-15]

            options.display = 1;          % An option to control whether the algorithm
                                          % displays the output of each iterations [default: true]

            options.tol_stopping=10^-10;  % A small positive scalar defining the stoppping
                                          % tolerance for the optimization solver [default: 10^-10]

            options.max_iter = 500;       % Maximum number of iteration for the solver [default: i_max=1000]

            options.objective = 'direction';    % 'likelihood': use likelihood as criterion to
                                          % optimize parameters of GMM
                                          % 'mse': use mean square error as criterion to
                                          % optimize parameters of GMM
                                          % 'direction': minimize the angle between the
                                          % estimations and demonstrations (the velocity part)
                                          % to optimize parameters of GMM                              
                                          % [default: 'mse']
    else
        % defining dataset with kp for GMM
        for cnt2 = 1:1%n_demos
          % trainingData(:,(cnt2-1)*(n_steps-1)+1:cnt2*(n_steps-1)) = [demos{cnt2}(1,1:n_steps-1); demos{cnt2}(2,1:n_steps-1); diff(demos{cnt2}(1,:))/dt; diff(demos{cnt2}(2,:)/dt); kp0*ones(1,n_steps-1)];
              trainingData(:,(cnt2-1)*(n_steps-1)+1:cnt2*(n_steps-1)) = [demos{cnt2}(1,1:n_steps-1); demos{cnt2}(2,1:n_steps-1)];
        end
    end

    if true
        if useSEDS
            [Priors0, Mu0, Sigma0] = initialize_SEDS(trainingData, nbGauss); %finding an initial guess for GMM's parameter
            [GMM.Priors, GMM.Mu, GMM.Sigma]=SEDS_Solver(Priors0, Mu0, Sigma0,trainingData,options); %running SEDS optimization solver

       
        else
            [Priors0, Mu0, Sigma0] = EM_init_kmeans(trainingData, nbGauss);
            [GMM.Priors, GMM.Mu, GMM.Sigma, Pix] = EM(trainingData, Priors0, Mu0, Sigma0);
        end
    end
    % figure
    % hold on
    % scatter(trainingData(1,:),trainingData(2,:))
    % scatter(GMM.Mu(1,:),GMM.Mu(2,:))
    % hold off

    
figure
hold on
scatter(trainingData(1,:),trainingData(2,:))
my_plotGMM(GMM.Mu(1:2,:), GMM.Sigma(1:2,1:2,:), 'r', 1);

%     for cnt2 = 1:n_runs
% 
%         if relearnGMM
%             if useSEDS
%                 [Priors0, Mu0, Sigma0] = initialize_SEDS(trainingData, nbGauss); %finding an initial guess for GMM's parameter
%                 [GMM.Priors, GMM.MuBasic, GMM.SigmaBasic]=SEDS_Solver(Priors0, Mu0, Sigma0,trainingData,options); %running SEDS optimization solver
%                 GMM.Mu = [GMM.MuBasic; ones(1,nbGauss)*kp0];
%                 for j = 1:nbGauss
%                     GMM.Sigma(:,:,j) = [GMM.SigmaBasic(:,:,j) zeros(4,1); zeros(1,4) 1]; % the value for diag term for kp is not important as it is not used to compute A or b
%                 end 
%             else
%                 [Priors0, Mu0, Sigma0] = EM_init_kmeans(trainingData, nbGauss);
%                 [GMM.Priors, GMM.Mu, GMM.Sigma, Pix] = EM(trainingData, Priors0, Mu0, Sigma0);
%             end
%         end
% 
%         results(cnt1,cnt2) = runPI2GMMLearning(p(cnt1),GMM);
%         costEvoAllRuns(:,cnt1,cnt2) = results(cnt1,cnt2).cost(:,3);
%       %  testResults(cnt2) = testLearnPolicy(results(cnt2).GMR);
%     end
%     costEvoRolloutIndex = results(cnt1,1).cost(:,1);
%     costEvoMean = mean(costEvoAllRuns(:,:,cnt2),2);
%     costEvoStd = std(costEvoAllRuns(:,:,cnt2),0,2);
% 
%     figure
%     hold on
%     plot(costEvoRolloutIndex, costEvoMean);
%     plot(costEvoRolloutIndex, costEvoMean+costEvoStd, '.');
%     plot(costEvoRolloutIndex, costEvoMean-costEvoStd, '.');
%     s = strcat('Cost evolution mean and std, ', int2str(n_runs),' runs');
%     title(s)
%     hold off
% 
%     figure
%     hold on
%     for cnt3=1:n_runs
%     scatter(1,results(cnt1,cnt3).cost(end,2))
%     end
%     results_this_protocol = results(cnt1,:);
%     s_file = strcat('results_protocol ',int2str(cnt1));
%     save(s_file, 'results_this_protocol');
% end
% 
% save('results_all_protocols', 'results');

% with mvnpdf and 10 interations of 10 roll-outs, spends 19.377 total time
% (11.028 self) for 268800 cals