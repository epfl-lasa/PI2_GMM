function explorationNoiseVector = produceExplorationNoise(desiredMeanNorm, GMR,useSEDS)
n_Gauss = size(GMR.A,3);
n_in = size(GMR.A,2);
n_out = size(GMR.b,1);
n_params = n_Gauss*(n_in*n_out+n_out);

standDev = zeros(n_params,1);

for k = 1: n_Gauss
    for i = 1:n_in
        standDevA(:,i) = ones(n_out,1)*sqrt(2/3/pi)*desiredMeanNorm/GMR.muInput(i,k);
    end
    standDevB = ones(n_out,1)*sqrt(2/3/pi)*desiredMeanNorm;
    standDev((k-1)*(n_in*n_out+n_out)+1:k*(n_in*n_out+n_out)) = [reshape(standDevA',n_in*n_out,1); standDevB];
end

explorationNoiseVector = randn(n_params,1).*standDev;
end