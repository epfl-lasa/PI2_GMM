function D=get_traj_and_stiff_from_policy(theta,GMR,protocol)
% This function takes a parameter vector, a GMR (for the necessary input-space information) and a protocol structure 
% to create a text file containing the tajectory and
% stiffness profile to be used with the trajectory_publisher package to
% execute the behavior on the KUKA.

% TO extract .txt file from previously saved results:
% Load a results_protocol.mat file, and then execute get_traj_and_stiff_from_policy(results_this_protocol.thetas(xxx,:),results_this_protocol.GMR,results_this_protocol.p)
% xxx being the number of the policy (i.e. the number of updates that were
% made to obtain this policy).

n_dim_pos = 2;
n_dim_angle = 1;
n_dim = n_dim_pos + n_dim_angle;
%n_dim_kp = n_dim;
n_dim_kp = 1;
n_in = n_dim_pos;
n_out = n_dim + n_dim_kp;
n_Gauss = length(GMR.priors);
n_param = n_Gauss*(n_in*n_out+n_out);

dt = p.dt;
n_steps_tot = p.duration/dt;

traj_y = p.start;
            

for n=1:n_steps_tot  

              h = zeros(n_Gauss,1);
              for i = 1:n_Gauss
                if p.feedback == 0
                    h(i) = GMR.priors(i) * my_mvnpdf(traj_y(1:n_in), GMR.muInput(:,i), GMR.sigmaInput(:,:,i));  % !!! first step
                end
                if p.feedback == 1
                    h(i) = GMR.priors(i) * my_mvnpdf(q(1:n_in), GMR.muInput(:,i), GMR.sigmaInput(:,:,i)); 
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
              

              %%% compute control matrix G
              G = zeros(n_out, n_param);
              lengthBloc = n_in*n_out+n_out;
              structG = zeros(n_out,lengthBloc);
              for i = 1:n_out
                if p.feedback == 0
                    structG(i,:) = [zeros(1,(i-1)*(n_in)) traj_y(1:n_in)' zeros(1,((n_out)-i)*(n_in)) zeros(1,i-1) 1 zeros(1,(n_out)-i)];
                end
                if p.feedback == 1
                    structG(i,:) = [zeros(1,(i-1)*(n_in)) q(1:n_in)' zeros(1,((n_out)-i)*(n_in)) zeros(1,i-1) 1 zeros(1,(n_out)-i)];
                end
              end
              for i = 1:n_Gauss
                  G(:,(i-1)*lengthBloc+1:i*lengthBloc) = h(i)*structG;
              end
              if sum(sum(isnan(G)))
                  ;
              end

              %%% compute state dependent desired velocity and stiffness
              outputs = G*theta';
              traj_yd = outputs(1:n_dim);   % desired velocity
              traj_y = traj_y + traj_yd*dt; % desired position

                
              if n_dim_kp == 0
                  kp = p.kp0;
              else
                  kp =  outputs(end - n_dim_kp +1:end);
              end
              kp = max(kp, 50);   % stiffness. should not be negative , not smaller than a minimum
              kp = min(kp, 700);  % safety
              kd = 2*sqrt(kp);     % damping
              

              D.traj_y(:,n)   = traj_y; % desired position
              D.traj_yd(:,n)  = traj_yd; % desired velociy
              D.kp(:,n) = kp;

end
          
    
              %%% visualization of rollout
              figure
              subplot(3,3,1)
              hold on
              scatter(D.traj_y(1,:),D.traj_y(2,:),'r.')
              if p.rep == 1
                  title('Updated policy: y vs z')
              else
                  title('Next rollout: y vs z')
              end
              % plot scene
              subplot(3,3,2)
              hold on
              plot(D.traj_y(3,:),'r.')
              title('angle')
              subplot(3,3,3)
              hold on
              plot(D.kp(1,:),'r.') 

           %%% write to file for robot to execute [0 x1 x2 cos(angle/2) sin(angle/2) 0 0]
            qw = cos(p.offset(4)/2)*cos((D.traj_y(3,:)+p.offset(5))/2)*cos(p.offset(6)/2) + sin(p.offset(4)/2)*sin((D.traj_y(3,:)+p.offset(5))/2)*sin(p.offset(6)/2);
            qx = -cos(p.offset(4)/2)*sin((D.traj_y(3,:)+p.offset(5))/2)*sin(p.offset(6)/2) + sin(p.offset(4)/2)*cos((D.traj_y(3,:)+p.offset(5))/2)*cos(p.offset(6)/2);
            qy = cos(p.offset(4)/2)*sin((D.traj_y(3,:)+p.offset(5))/2)*cos(p.offset(6)/2) + sin(p.offset(4)/2)*cos((D.traj_y(3,:)+p.offset(5))/2)*sin(p.offset(6)/2);
            qz = cos(p.offset(4)/2)*cos((D.traj_y(3,:)+p.offset(5))/2)*sin(p.offset(6)/2) - sin(p.offset(4)/2)*sin((D.traj_y(3,:)+p.offset(5))/2)*cos(p.offset(6)/2);
            rolloutTrajAndImp = [ ones(n_steps_tot,1).*p.offset(1) , D.traj_y(1,:)'+ p.offset(2) , D.traj_y(2,:)' + p.offset(3), qx', qy' ,qz',qw', D.kp(1,:)'];  % [px py pz qx qy qz qw k(1d)], the simulation works in the XZ plane, axis [0 1 0] angle representation is transformed to quaternion
            save('./traj.txt', 'rolloutTrajAndImp','-ascii');


end
