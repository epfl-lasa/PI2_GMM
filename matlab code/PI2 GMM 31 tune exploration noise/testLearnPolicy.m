function [testResults] = testLearnPolicy(GMR)
n_trials = 1;
n_dim = 2;
D = run_rollouts([0; 0], n_trials, 100, GMR); % !!! Start pos is set to 0 0
[genCostSteps, totCostSteps, userCostSteps, R_unused, viapointCostSteps, accelerationCostSteps, controlCostSteps]=viapointLineWithSTurn(D,GMR);


figure;
hold on
for m = 1:n_trials
    plot(D(1,m).q(1,:),D(1,m).q(2,:),'b.');
    plot(D(1,m).traj_y(1,:),D(1,m).traj_y(2,:),'r.');
end
xlabel('q_1 & y_1')
ylabel('q_2 & y_2');
title('2D path of point mass & desired traj');
GMRreduced = GMR;
GMRreduced.A = GMR.A(1:n_dim,:,:);    % get rid of kp dimension for using streamlines
GMRreduced.b = GMR.b(1:n_dim,:,:);
showStreamlines(GMRreduced,[-5 15 -10 10],50);
% learning param !!!
scatter(GMR.muInput(1,:),GMR.muInput(2,:),[],'r')
my_plotGMM(GMR.muInput, GMR.sigmaInput, 'r', 1);
hold off

testResults.meanTotCost = sum(totCostSteps);
testResults.Dtest = D;

end

function D=run_rollouts(startPos, n_trials, n_steps, GMR)
% a dedicated function to run multiple roll-outs using the specifictions in D and p
% noise_mult allows decreasing the noise with the number of roll-outs, which gives
% smoother converged performance (but it is not needed for convergence).

n_param = size(GMR.theta,1);
n_dim = 2;
n_Gauss = size(GMR.A,3);
n_in = size(GMR.A,2);
n_out = size(GMR.A,1);




dt = 0.1;
feedback = 1;
% the simulated point mass
mass = 1;
damp = 1;
fWind = 0;

        for m=1:n_trials
            startPosEps = randn(2,1)*0;
            
            q    = startPos + startPosEps;
            qd   = zeros(2,1);
            traj_y = startPos + startPosEps;

          % integrate for twice the duration to see converence behavior   !!! What
          % is our stop condition? We do not have a fixed duration



          for n=1:n_steps,   % !!! see how to do for first step


              % integrate simulated 2D point mass with inverse dynamics control
              % based on GMR output

              h = zeros(n_Gauss,1);
              for i = 1:n_Gauss
                if feedback == 0
                    h(i) = GMR.priors(i) * my_mvnpdf(traj_y, GMR.muInput(:,i), GMR.sigmaInput(:,:,i));  % !!! first step
                end
                if feedback == 1
                    h(i) = GMR.priors(i) * my_mvnpdf(q, GMR.muInput(:,i), GMR.sigmaInput(:,:,i)); 
                end
              end
              if sum(h) == 0
                  disp('too far from GMM')
              else
                   h = h/sum(h);
              end


              G = zeros(n_out, n_param);
              lengthBloc = n_in*n_out+n_out;
              structG = zeros(n_out,lengthBloc);
              for i = 1:n_out
                if feedback == 0
                    structG(i,:) = [zeros(1,(i-1)*(n_in)) traj_y' zeros(1,((n_out)-i)*(n_in)) zeros(1,i-1) 1 zeros(1,(n_out)-i)];
                end
                if feedback == 1
                    structG(i,:) = [zeros(1,(i-1)*(n_in)) q' zeros(1,((n_out)-i)*(n_in)) zeros(1,i-1) 1 zeros(1,(n_out)-i)];
                end
              end
              for i = 1:n_Gauss
                  G(:,(i-1)*lengthBloc+1:i*lengthBloc) = h(i)*structG;
              end
              if sum(sum(isnan(G)))
                  ;
              end

              outputs = G*GMR.theta;
              traj_yd = outputs(1:n_dim);
              traj_y = traj_y + traj_yd*dt;

              kp = 1; % outputs(n_dim+1);
              if kp<0
                  kp = 0;
              end
              kd = 2*sqrt(kp);

              fWindCurrent = 0;
              if q(1)> 5
                  fWindCurrent = fWind;
              end

              u   =   kp*(traj_y-q) + kd*(traj_yd-qd);
              qdd = (u - qd * damp + fWindCurrent)/mass;
              qd  = qdd * dt + qd;
              q   = qd * dt + q;

              D(m).traj_y(:,n)   = traj_y;
              D(m).traj_yd(:,n)  = traj_yd;
              D(m).h(:,n)   = h;
              D(m).G(:,:,n) = G;
              D(m).eps(:,n) = zeros(n_param,1);

              D(m).q(:,n)   = q;
              D(m).qd(:,n)  = qd;
              D(m).qdd(:,n) = qdd;
              D(m).u(:,n)   = u;
              D(m).kp(n) = kp;
              D(m).duration = 10;
              D(m).dt = 0.1;
          end

        end
    end
