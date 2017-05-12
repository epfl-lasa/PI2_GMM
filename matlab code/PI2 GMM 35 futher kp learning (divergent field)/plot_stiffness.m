xmin = -12;
xmax = 2;
ymin = -2;
ymax = 2;
res = 1000;


n_Gauss = size(results_this_protocol(1).GMR.muInput,2);
n_in  = size(results_this_protocol(1).GMR.muInput,1);
n_out = size(results_this_protocol(1).GMR.b,1);
n_param = n_Gauss*(n_in*n_out+n_out); 

K = zeros(res+1,res+1,2);

for cnt0 = 1:length(results_this_protocol)
GMR = results_this_protocol(cnt0).GMR;

    
for cnt1 = 1:res+1
    for cnt2 = 1:res+1
        traj_y = [xmin + (cnt1-1)*(xmax-xmin)/res; ymin + (cnt2-1)*(ymax-ymin)/res];
   
             h = zeros(n_Gauss,1);
              for i = 1:n_Gauss

                    h(i) = GMR.priors(i) * my_mvnpdf(traj_y, GMR.muInput(:,i), GMR.sigmaInput(:,:,i));  % !!! first step

              end
              if sum(h) == 0
                  disp('too far from GMM')
              else
                   h = h/sum(h);
              end
              
              if sum(isnan(h))
                  'problem';
              end
              

              G = zeros(n_out, n_param);
              lengthBloc = n_in*n_out+n_out;
              structG = zeros(n_out,lengthBloc);
              for i = 1:n_out

                    structG(i,:) = [zeros(1,(i-1)*(n_in)) traj_y' zeros(1,((n_out)-i)*(n_in)) zeros(1,i-1) 1 zeros(1,(n_out)-i)];

              end
              for i = 1:n_Gauss
                  G(:,(i-1)*lengthBloc+1:i*lengthBloc) = h(i)*structG;
              end

              outputs = G*(GMR.theta);

              kp =  max(outputs(n_in+1:end),0);
              
              K(cnt1,cnt2,1) = K(cnt1,cnt2,1) + kp(1);
              K(cnt1,cnt2,2) = K(cnt1,cnt2,2) + kp(2);
    
    end
end
    
end


figure
hold on
surf(xmin:(xmax-xmin)/res:xmax,ymin:(ymax-ymin)/res:ymax,-K(:,:,1)');
title('kp1')
cmap = colormap;
cmap = flipud(cmap);
colormap(cmap);
my_plotGMM(GMR.muInput, GMR.sigmaInput, 'r', 1);
figure
hold on
surf(xmin:(xmax-xmin)/res:xmax,ymin:(ymax-ymin)/res:ymax,-K(:,:,2)');
title('kp2')
cmap = colormap;
cmap = flipud(cmap);
colormap(cmap);
my_plotGMM(GMR.muInput, GMR.sigmaInput, 'r', 1);

% K = K/res *0.01;
%     
% for cnt1 = 1:res+1
%     for cnt2 = 1:res+1
%         if K(cnt1,cnt2,1) > 0 && K(cnt1,cnt2,2) > 0
%         traj_y = [xmin + (cnt1-1)*(xmax-xmin)/res; ymin + (cnt2-1)*(ymax-ymin)/res];
%             rectangle('Position',[traj_y(1),traj_y(2),K(cnt1,cnt2,1),K(cnt1,cnt2,2)],'Curvature',[1,1]);
%         end
%     end
% end
