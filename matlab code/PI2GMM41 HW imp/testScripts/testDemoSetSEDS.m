close all

nbGauss = 3;
%demos = a.dataPoints;
dt = 0.1;
n_demos = size(demos,2);
n_in = 2;
n_out = 2;
[x0 , xT, trainingData, index] = preprocess_demos(demos,dt,0.0001);


options.tol_mat_bias = 10^-1; % A very small positive scalar to avoid
                                          % instabilities in Gaussian kernel [default: 10^-15]

            options.display = 1;          % An option to control whether the algorithm
                                          % displays the output of each iterations [default: true]

            options.tol_stopping=10^-6;  % A small positive scalar defining the stoppping
                                          % tolerance for the optimization solver [default: 10^-10]

            options.max_iter = 500;       % Maximum number of iteration for the solver [default: i_max=1000]

            options.objective = 'likelihood';    % 'likelihood': use likelihood as criterion to
            


[Priors0, Mu0, Sigma0] = initialize_SEDS(trainingData, nbGauss); %finding an initial guess for GMM's parameter
[GMM.Priors, GMM.Mu, GMM.Sigma]=SEDS_Solver(Priors0, Mu0, Sigma0,trainingData,options); %running SEDS optimization solver


            
            
 GMR.priors = GMM.Priors;
    for j=1:nbGauss
        GMR.muInput(:,j)        = GMM.Mu(1:n_in,j);
        GMR.sigmaInput(:,:,j)   = GMM.Sigma(1:n_in,1:n_in,j);
        GMR.A(:,:,j)  = GMM.Sigma(n_in+1:n_in+n_out,1:n_in,j)/GMR.sigmaInput(:,:,j); 
        GMR.b(:,j)    = GMM.Mu(n_in+1:n_in+n_out,j) - GMR.A(:,:,j)*GMR.muInput(:,j);
        GMR.theta((j-1)*(n_out*n_in+n_out)+1:j*(n_out*n_in+n_out)) = [reshape(GMR.A(:,:,j)',n_out*n_in,1);GMR.b(:,j)];
        
        outputVariance(:,:,j) = zeros(2,2);
        for k=1:nbGauss
            h(k) = GMR.priors(k) * mvnpdf(GMM.Mu(1:n_in,j), GMM.Mu(1:n_in,k), GMM.Sigma(1:n_in,1:n_in,k));
        end
        h = h/sum(h);
        for k=1:nbGauss
            outputVariance(:,:,j) =  outputVariance(:,:,j) + h(k)^2*(GMM.Sigma(n_in+1:n_in+n_out,n_in+1:n_in+n_out,k) - GMM.Sigma(n_in+1:n_in+n_out,1:n_in,k)/GMM.Sigma(1:n_in,1:n_in,k)*GMM.Sigma(1:n_in,n_in+1:n_in+n_out,k));
        end
        
    end
    
figure
showStreamlines(GMR,[-1 1 -1 1],200)
hold on
scatter(trainingData(1,:), trainingData(2,:),'m.')
scatter(GMR.muInput(1,:),GMR.muInput(2,:),[],'r')
my_plotGMM(GMR.muInput, GMR.sigmaInput, 'r', 1);
my_plotGMM(GMR.muInput, outputVariance, 'g', 1);
