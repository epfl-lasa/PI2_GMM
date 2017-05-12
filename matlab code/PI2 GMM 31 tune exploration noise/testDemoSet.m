close all

nbGauss = 4;
demos = a.dataPoints;
dt = 0.1;
n_demos = size(demos,2);
n_in = 2;
n_out = 2;

sum_steps = 0;
for cnt2 = 1:n_demos
    n_steps = size(demos{cnt2},2);
    sum_steps = sum_steps + n_steps-1;
    trainingData(:,sum_steps - n_steps+1 +1:sum_steps) = [demos{cnt2}(1,1:n_steps-1); demos{cnt2}(2,1:n_steps-1); diff(demos{cnt2}(1,:))/dt; diff(demos{cnt2}(2,:)/dt)];
end

 [Priors0, Mu0, Sigma0] = EM_init_kmeans(trainingData, nbGauss);
 [GMM.Priors, GMM.Mu, GMM.Sigma, Pix] = EM(trainingData, Priors0, Mu0, Sigma0);
 
 
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
hold on
scatter(trainingData(1,:), trainingData(2,:),'m')
scatter(GMR.muInput(1,:),GMR.muInput(2,:),[],'r')
my_plotGMM(GMR.muInput, GMR.sigmaInput, 'r', 1);
my_plotGMM(GMR.muInput, outputVariance, 'g', 1);
showStreamlines(GMR,axis,50)




%%%%%%%%%%%%%%%%%% linear system
x = [trainingData(1,:)', trainingData(2,:)', ones(size(trainingData,2),1)]\[trainingData(3,:)', trainingData(4,:)']; 
GMR2.priors = 1;
GMR2.muInput        = [0;0];
GMR2.sigmaInput   = eye(2);
GMR2.A  = [x(1,1) x(2,1); x(1,2) x(2,2)]; 
GMR2.b    = [x(3,1); x(3,2)];
%GMR2.theta((j-1)*(n_out*n_in+n_out)+1:j*(n_out*n_in+n_out)) = [reshape(GMR.A(:,:,j)',n_out*n_in,1);GMR.b(:,j)];

figure
showStreamlines(GMR2,[-10 10 -10 10],50)
hold on
scatter(trainingData(1,:), trainingData(2,:),'m')


figure
scatter3(trainingData(1,:),trainingData(2,:),trainingData(3,:))
title('v1')
figure
scatter3(trainingData(1,:),trainingData(2,:),trainingData(4,:))
title('v2')

figure
icolor = ceil((trainingData(4,:)+abs(min(trainingData(4,:))))/(max(trainingData(4,:))+abs(min(trainingData(4,:))))*256)+1;
scatter3(trainingData(1,:),trainingData(2,:),trainingData(3,:),trainingData(4,:)+abs(min(trainingData(4,:)))+1,icolor)