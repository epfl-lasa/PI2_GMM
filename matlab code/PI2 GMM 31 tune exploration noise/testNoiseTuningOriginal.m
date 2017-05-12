
n_Gauss = size(GMR.A,3);
n_in = size(GMR.A,2);
n_out = size(GMR.b,1);
n_params = n_Gauss*(n_in*n_out+n_out);

vNoiseNormRec = zeros(15,15,10);
for n = 1:10
    n
   %epsilonRec = produceExplorationNoise(1,GMR,0);
   epsilonRec = produceExplorationNoiseRatio(1,GMR,0);
   epsilonBasic = randn(n_params,1)*1;
   
   for i=1:1:200
       for j = 1:1:200
              traj_y = [-150+i*1;-150+j*1];
              
              %%%%build G
              h = zeros(n_Gauss,1);
              for k = 1:n_Gauss
                    h(k) = GMR.priors(k) * my_mvnpdf(traj_y, GMR.muInput(:,k), GMR.sigmaInput(:,:,k));
              end
              if sum(h) == 0
                  disp('too far from GMM')
              else
                   h = h/sum(h);
              end
           
              G = zeros(n_out, n_params);
              lengthBloc = n_in*n_out+n_out;
              structG = zeros(n_out,lengthBloc);
              for k = 1:n_out
                    structG(k,:) = [zeros(1,(k-1)*(n_in)) traj_y' zeros(1,((n_out)-k)*(n_in)) zeros(1,k-1) 1 zeros(1,(n_out)-k)];
              end
              for k = 1:n_Gauss
                  G(:,(k-1)*lengthBloc+1:k*lengthBloc) = h(k)*structG;
              end
              
              %%%% velocity noise vector
              outputsRec = G*(epsilonRec);
              vNoiseNormRec(i,j,n) = norm(outputsRec(1:2));
              outputsBasic = G*(epsilonBasic);
              vNoiseNormBasic(i,j,n) = norm(outputsBasic(1:2));
              
              %%% baseline velocity
              outputsBaseline = G*GMR.theta;
              vBaselineNorm(i,j) = norm(outputsBaseline(1:2));
              
              vNoiseNormRatioRec = vNoiseNormRec/vBaselineNorm(i,j);
              vNoiseNormRatioBasic = vNoiseNormBasic/vBaselineNorm(i,j);
       
       end
   end
   
end



% vNoiseNormVarRec = var(vNoiseNormRec,0,3);
% vNoiseNormMeanRec = mean(vNoiseNormRec,3);
% figure
% hold on
% surf(-149:50,-149:50,-vNoiseNormMeanRec')
% cmap = colormap;
% cmap = flipud(cmap);
% colormap(cmap);
% my_plotGMM(GMR.muInput, GMR.sigmaInput, 'r', 1);
% 
% 
% vNoiseNormVarBasic = var(vNoiseNormBasic,0,3);
% vNoiseNormMeanBasic = mean(vNoiseNormBasic,3);
% figure
% hold on
% surf(-149:50,-149:50,-vNoiseNormMeanBasic')
% cmap = colormap;
% cmap = flipud(cmap);
% colormap(cmap);
% my_plotGMM(GMR.muInput, GMR.sigmaInput, 'r', 1);

vNoiseNormRatioVarRec = var(vNoiseNormRatioRec,0,3);
vNoiseNormRatioMeanRec = mean(vNoiseNormRatioRec,3);
figure
hold on
surf(-149:50,-149:50,-vNoiseNormRatioMeanRec')
cmap = colormap;
cmap = flipud(cmap);
colormap(cmap);
my_plotGMM(GMR.muInput, GMR.sigmaInput, 'r', 1);


vNoiseNormRatioVarBasic = var(vNoiseNormRatioBasic,0,3);
vNoiseNormRatioMeanBasic = mean(vNoiseNormRatioBasic,3);
figure
hold on
surf(-149:50,-149:50,-vNoiseNormRatioMeanBasic')
cmap = colormap;
cmap = flipud(cmap);
colormap(cmap);
my_plotGMM(GMR.muInput, GMR.sigmaInput, 'r', 1);


figure
hold on
surf(-149:50,-149:50,-vBaselineNorm')
cmap = colormap;
cmap = flipud(cmap);
colormap(cmap);
my_plotGMM(GMR.muInput, GMR.sigmaInput, 'r', 1);


