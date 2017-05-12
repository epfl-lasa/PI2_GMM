function [bsim_out, bR, method] = prepareForPI201(D,totCostSteps,p)
global GMR;
n_Gauss = length(GMR.priors);
n_in = size(GMR.muInput,1);
n_out = size(GMR.A,1);
n_param = length(GMR.theta);
n_rollout = length(D);
N = p.duration/p.dt;

if strcmp(p.PI2_type,'PI2_01')
    method = 'PI2_01';
elseif strcmp(p.PI2_type,'PI2_01b')
    method = 'PI2_01b';
elseif strcmp(p.PI2_type,'PI2_BB')
    method = 'PI_BB';
end

for k = 1:n_rollout
    
    bsim_out(k).t = [1:N]*p.dt ;%zeros(1,N));
    bsim_out(k).x = D(k).q(:,1:N);
    bsim_out(k).u = D(k).u(:,1:N);  %unused


    for i = 1:n_Gauss
        for j = 1:n_out   
            bsim_out(k).Controller.theta((i-1)*(n_in+1)+1:(i-1)*(n_in+1)+n_in,j) = GMR.theta((i-1)*(n_in*n_out+n_out)+(j-1)*n_in+1 : (i-1)*(n_in*n_out+n_out)+(j-1)*n_in+n_in); % taking care of A terms
            bsim_out(k).Controller.theta((i-1)*(n_in+1)+n_in+1,j) = GMR.theta((i-1)*(n_param/n_Gauss)+ n_in*n_out + (j-1)+1); % taking care of b terms
            for t = 1:N
                 bsim_out(k).eps((i-1)*(n_in+1)+1:(i-1)*(n_in+1)+n_in,j, t) = D(k).eps((i-1)*(n_in*n_out+n_out)+(j-1)*n_in+1 : (i-1)*(n_in*n_out+n_out)+(j-1)*n_in+n_in , t);
                 bsim_out(k).eps((i-1)*(n_in+1)+n_in+1,j,t) = D(k).eps((i-1)*(n_param/n_Gauss)+ n_in*n_out + (j-1)+1 , t);
            end
        end
    
        bsim_out(k).Controller.BaseFnc = @BaseFnc;
         bsim_out(k).Controller.BaseFncFake = @BaseFncFake;
    end
end

bR = totCostSteps'; % or '

end

function f = BaseFnc(t,x)
    global GMR;
    n_Gauss = length(GMR.priors);

    for cnt0 = 1:length(t)
                
        h = zeros(n_Gauss,1);
      for i = 1:n_Gauss
            h(i) = GMR.priors(i) * my_mvnpdf(x(:,cnt0), GMR.muInput(:,i), GMR.sigmaInput(:,:,i)); 
      end
      if sum(h) == 0
          disp('too far from GMM')
      else
           h = h/sum(h);
      end
        
        local_lin = [1; x(:,cnt0)] * h';
        
        f(:,cnt0) = local_lin(:);
    end

end

function f = BaseFncFake(t,x)
    global GMR;
    n_Gauss = length(GMR.priors);

    for cnt0 = 1:length(t)
                
        h = zeros(n_Gauss,1);
      for i = 1:n_Gauss
            h(i) = GMR.priors(i) * my_mvnpdf(x(:,cnt0), GMR.muInput(:,i), GMR.sigmaInput(:,:,i)); 
      end
      if sum(h) == 0
          disp('too far from GMM')
      else
           h = h/sum(h);
      end
        
        local_lin = [1; ones(length(x(:,cnt0)),1)] * h';
        
        f(:,cnt0) = local_lin(:);
    end

end