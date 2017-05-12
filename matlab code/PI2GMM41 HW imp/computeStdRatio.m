% This function computes the std ratio for the noise applied to the
% parameters of the different Gaussians based on their location and the
% desired ratio.

function standDev = computeStdRatio(desiredMeanNormRatio, GMR,useSEDS)
n_Gauss = size(GMR.A,3);
n_in = size(GMR.A,2);
n_out = size(GMR.b,1);
n_params = n_Gauss*(n_in*n_out+n_out);

standDev = zeros(n_params,1);

for k = 1: n_Gauss
    
    
  h = zeros(n_Gauss,1);
  for m = 1:n_Gauss
        h(m) = GMR.priors(m) * my_mvnpdf(GMR.muInput(:,k), GMR.muInput(:,m), GMR.sigmaInput(:,:,m));
  end
  h = h/sum(h);
  refVeloAtCenter = 0;
  for m = 1:n_Gauss
        refVeloAtCenter = refVeloAtCenter + h(m)*(GMR.A(:,:,m)*GMR.muInput(:,k)+GMR.b(:,m));
  end
  normRefVeloAtCenter = norm(refVeloAtCenter(1:2));
  for i = 1:n_in
      distanceNormalizor = max([abs(GMR.muInput(i,k)) 0.01]);
      standDevA(:,i) = [ones(n_in,1)*sqrt(2/3/pi)*normRefVeloAtCenter*desiredMeanNormRatio/distanceNormalizor ; ones((n_out-n_in),1)*sqrt(2/3/pi)*desiredMeanNormRatio/distanceNormalizor] ;  % for kp parameters it should not be multiplied by velocity
  end
  standDevB = [ones(n_in,1)*sqrt(2/3/pi)*normRefVeloAtCenter*desiredMeanNormRatio;  ones((n_out-n_in),1)*sqrt(2/3/pi)*desiredMeanNormRatio]; % for kp parameters it should not be multiplied by velocity
  standDev((k-1)*(n_in*n_out+n_out)+1:k*(n_in*n_out+n_out)) = [reshape(standDevA',n_in*n_out,1); standDevB];
end
end