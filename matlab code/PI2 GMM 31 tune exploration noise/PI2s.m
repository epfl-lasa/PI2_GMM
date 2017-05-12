function dtheta = PI2s(method,bsim_out,bR)

n_dof  = size(bsim_out(1).Controller.theta,2);
n_agent = 3;       % Number of agent per input  !!!Modified
n_bfs  = size(bsim_out(1).Controller.theta,1)/n_agent;
n_time = size(bR,2);
n_reps = size(bR,1);

show_test_graph = 0;

switch method
    case 'PI2'
        PI2_ave_mode = 4;
    case 'PI2_01'
        PI2_ave_mode = 5;
   case 'PI2_01b'
        PI2_ave_mode = 6;
    case 'PI_BB'
        PI2_ave_mode = 3;
    case 'PI_BB++'
        PI2_ave_mode = 3;
end
    

dtheta1 = zeros(n_bfs*n_agent,n_dof);
dtheta2 = zeros(n_bfs*n_agent,n_dof);
for cnt1 = 1:n_dof
    % Reward function
    R = permute(bR,[4 3 2 1]);
    R = repmat(R,[1 n_agent 1 1]);
    R_new = zeros(1,n_agent,n_time,n_reps);
    Meps  = zeros(n_bfs,n_agent,n_time,n_reps);
    eps   = zeros(n_bfs,n_agent,n_time,n_reps);
    Apsi  = zeros(n_bfs,n_agent,n_time,n_reps);
    
    for cnt2 = 1:length(bsim_out)
        sim_out = bsim_out(cnt2);
        T = sim_out.t;
        X = sim_out.x;
        U = sim_out.u;
        e = sim_out.eps(:,cnt1,:);
      %  e = ones(size(e,1),size(e,2),size(e,3));         %  !!!!!!!!!!!!! test
        e = my_reshape(squeeze(e),n_bfs,n_agent,1);
        temp_Psi = sim_out.Controller.BaseFnc(T,X);
        temp_PsiNoX = sim_out.Controller.BaseFncFake(T,X); % added
        Psi = my_reshape(temp_Psi,n_bfs,n_agent,1);
        PsiNoX = my_reshape(temp_PsiNoX,n_bfs,n_agent,1); % added
        theta = sim_out.Controller.theta(:,cnt1);
        n_temp = length(T);
        theta = my_reshape(theta,n_bfs,n_agent,n_temp);
        
        psiTeps = sum(Psi.*e,1);
        
        RR = repmat( [1 1*ones(1,n_agent-1)], [n_bfs 1 n_temp] );   %!!! modified to fit my tests
        
        temp1 = (1./RR) .* Psi .* repmat(psiTeps,[n_bfs 1 1]);
        temp2 = repmat( sum( Psi ./ RR .* Psi ,1 ),[n_bfs 1 1]);
        
        Meps(:,:,1:n_temp,cnt2) = temp1 ./ (temp2+1e-9);
        if sum(sum(sum(sum(isnan(Meps)))))   % !!! modified
            ;
        end
        eps (:,:,1:n_temp,cnt2) = e;
        Apsi(:,:,1:n_temp,cnt2) = repmat(Psi(:,1,:),[1 n_agent 1]);
        ApsiNoX(:,:,1:n_temp,cnt2) = repmat(PsiNoX(:,1,:),[1 n_agent 1]); % added
        
%         theta_hat = theta + Meps(:,:,1:n_temp,cnt2);
        % Equalizing the size for unsuccessful trials
%         temp = repmat(theta_hat(:,:,end),[1 1 n_time-n_temp]);
%         theta_hat = cat(3,theta_hat,temp);
%         temp = repmat(RR(:,:,end),[1 1 n_time-n_temp]);
%         RR = cat(3,RR,temp);
        % Calculating the augmented reward
        %R_new(:,:,:,cnt2) = R(:,:,:,cnt2)+ sum( RR.*(theta_hat.*theta_hat),1 ); % modified, we allready have the augmented cost so we do not need R_new
        R_new(:,:,:,cnt2) = R(:,:,:,cnt2);  % modified, we allready have the augmented cost so we do not need R_new
%         if sum(isnan(theta_hat))    %!!! modified
%             ;
%         end
        
        %     temp = squeeze(sum( RR.*(theta_hat.*theta_hat),1 ));
        %     plot(1:n_time-1,squeeze(R(:,:,1:n_time-1,cnt2))',...
        %         1:n_time-1,temp(:,1:n_time-1)','--')
    end
    
    % compute the accumulate cost
    temp1 = R_new(:,:,n_time:-1:1,:);       %
    temp2 = cumsum(temp1,3);
    S = temp2(:,:,n_time:-1:1,:);
    
    % compute the exponentiated cost with the special trick to automatically
    % adjust the lambda scaling parameter
    maxS = repmat( max(S,[],4), [1 1 1 n_reps]);
    minS = repmat( min(S,[],4), [1 1 1 n_reps]);
    medS = repmat( median(S,4), [1 1 1 n_reps]);
    
    h = 10; % this is the scaling parameters in side of the exp() function (see README.pdf)
    expS = exp( -h*(S-minS)./(medS-minS+10e-100) );         %!!! modified from medS to maxS and added small value to denominator in case medS == minS
    
    % the probabilty of a trajectory
    P = expS./( repmat( sum(expS,4), [1 1 1 n_reps]) );
    
    % TEST: checking for P(\tau)
    if show_test_graph == 1
        test1(R,R_new,S,expS,P)
    end
    % TEST: comparing g'*Meps ans g'*eps
    if show_test_graph == 1
        test2(bsim_out,Meps,eps,n_bfs,n_agent)
    end
    
    % compute the projected noise term. It is computationally more efficient to break this
    % operation into inner product terms.
    PMeps = repmat(P,[n_bfs 1 1 1]) .* Meps;
    Peps  = repmat(P,[n_bfs 1 1 1]) .* eps;
    PApsi = repmat(P,[n_bfs 1 1 1]) .* Apsi;
    PApsiNoX = repmat(P,[n_bfs 1 1 1]) .* ApsiNoX;   % added
    
    % compute the parameter update per time step
    temp_dtheta1 = sum(PMeps,4);
    temp_dtheta2 = sum(Peps,4);
    temp_W       = sum(PApsi,4);
    temp_WnoX       = sum(PApsiNoX,4);    % added
    
    % TEST: comparing time-varing thetas from Meps and eps
    if show_test_graph == 1
        test3(bsim_out,temp_dtheta1,temp_dtheta2,n_bfs,n_agent)
    end
    
    
    % average updates over time
    switch PI2_ave_mode
        case 1      % The uniform weighting
            W = ones(n_bfs,n_agent,n_time);
        case 2      % The time weighting matrix
            N = n_time:-1:1;
            N = permute(N,[1 3 2]);
            W = repmat(N,[n_bfs n_agent 1]);
        case 3      % PIBB like method
            W = cat(3, ones(n_bfs,n_agent,1),zeros(n_bfs,n_agent,n_time-1));
        case 4      % The time weighting matrix which takes the kernel activation into account
            % The time weighting matrix
            N = n_time:-1:1;
            N = permute(N,[1 3 2]);
            % Activation kerne;
            [~,temp_ind] = min(sum(bR,2));
            sim_out = bsim_out(temp_ind);
            temp_T = 0:(n_time-1);
            T = sim_out.t(1)+temp_T*(sim_out.t(2)-sim_out.t(1));
            X = [sim_out.x repmat(sim_out.x(:,end),[1 n_time-length(sim_out.t)])];
            temp_Psi = sim_out.Controller.BaseFnc(T,X);                         % !!! not here, this is for PI2 original... (Modified BaseFnc to BaseFncFake)
            Psi = my_reshape(temp_Psi,n_bfs,n_agent,1);
            % the final weighting vector takes the kernel activation into account
            W = repmat(N,[n_bfs n_agent 1]) .* Psi;
        case 5
            W = temp_W;  
        case 6
            W = temp_WnoX;  % added
    end
    
    % ... and normalize through time
    W = W./( repmat(sum(W,3),[1 1 n_time]) );      
    Z = isnan(W);                                    %!!! Modified
    for i = 1:size(W,1)
        for j = 1:size(W,2)
            for k = 1:size(W,3)
                if Z(i,j)
                    W(i,j,k) = 0;
                end
            end
        end
    end
    % compute the final parameter update for each DMP
     temp_dtheta1 = sum( temp_dtheta1 .* W, 3 );   %     !!!!!!!!
     temp_dtheta2 = sum( temp_dtheta2 .* W, 3 );   %     !!!!!!!!
    
%  temp_dtheta1 = sum( temp_dtheta1 , 3 );        
 % temp_dtheta2 = sum( temp_dtheta2 , 3 ); 
   
    temp_dtheta1 = temp_dtheta1';
    temp_dtheta2 = temp_dtheta2';
    
    dtheta1(:,cnt1) = temp_dtheta1(:);          % P*M*eps
    dtheta2(:,cnt1) = temp_dtheta2(:);          % P*eps
    
end

% TEST: comparing time_averaged thetas from Meps and eps
if show_test_graph == 1
    test4(bsim_out,dtheta1,dtheta2)
end


if show_test_graph == 1
    test5(bsim_out(end),dtheta2)
end

if strcmpi(method,'PI2_01') || strcmpi(method,'PI2_01b') || strcmpi(method,'PI2')  % modified added PI2_01b
    dtheta = dtheta1;
elseif strcmpi(method,'PI_BB') || strcmpi(method,'PI_BB++')
    dtheta = dtheta2;
end
if sum(sum(isnan(dtheta))) % !!! modified
    ;
end

end


function B = my_reshape(A,n_bfs,n_agent,time_rep)

temp_B = zeros(n_bfs,size(A,2),n_agent);
for cnt1 = 1:n_agent
    index = ( 1:n_agent:size(A,1) ) + (cnt1-1);   %!!! Modified 11 to n_agent
    temp_B(:,:,cnt1) = A(index,:);
end

B = permute(temp_B,[1 3 2]);

B = repmat(B,[1 1 time_rep]);

end



%% Test function for the PI2
% checking for P(\tau)
function test1(R,R_new,S,expS,P)
    figure(11); plot( squeeze(R(1,1,:,:)) ); title('R')
    figure(12); plot( squeeze(R_new(1,1,:,:)) ); title('R new')
    figure(13); plot( squeeze(S(1,1,:,:)) ); title('S')
    figure(14); plot( squeeze(expS(1,1,:,:)) ); title('expS')
    figure(15); plot( squeeze(P(1,1,:,:)) ); title('P')
end

% comparing g'*Meps ans g'*eps
function test2(bsim_out,Meps,eps,n_bfs,n_agent)
    delta = zeros(size(Meps,3),size(Meps,4));
    for cnt0 = 1:size(Meps,4)

        sim_out = bsim_out(cnt0);
        T = sim_out.t;
        X = sim_out.x;
        Psi = sim_out.Controller.BaseFnc(T,X);
        Psi = my_reshape(Psi,n_bfs,n_agent,1);

        temp1 = sum(sum(Psi .* Meps(:,:,:,cnt0),2),1);
        temp1 = squeeze(temp1);

        temp2 = sum(sum(Psi .* eps(:,:,:,cnt0),2),1);
        temp2 = squeeze(temp2);

        delta(:,cnt0) = abs( (temp1-temp2) ./ temp1 );
    end

    figure(21)
    legend
    plot(delta)
end

% comparing time-varing thetas from Meps and eps
function test3(bsim_out,temp_dtheta1,temp_dtheta2,n_bfs,n_agent)
    sim_out = bsim_out(end);
    T = sim_out.t;
    X = sim_out.x;
    Psi = sim_out.Controller.BaseFnc(T,X);
    Psi = my_reshape(Psi,n_bfs,n_agent,1);
        
    temp1 = squeeze( sum(sum(Psi .* temp_dtheta1,2),1) );
    temp2 = squeeze( sum(sum(Psi .* temp_dtheta2,2),1) );
    delta = abs( (temp1-temp2) ./ temp1 );

    figure(31)
    legend
    plot(delta)
end

% comparing time_averaged thetas from Meps and eps
function test4(bsim_out,dtheta1,dtheta2)
    sim_out = bsim_out(end);
    T = sim_out.t;
    X = sim_out.x;
    Psi = sim_out.Controller.BaseFnc(T,X);
    Psi = permute( repmat(Psi,[1 1 3]), [1 3 2]);

    temp1 = squeeze( sum( repmat(dtheta1,[1 1 length(T)]) .* Psi ,1) );
    temp2 = squeeze( sum( repmat(dtheta2,[1 1 length(T)]) .* Psi ,1) );

    delta = abs( (temp1-temp2) ./ temp1 );
    figure(41)
    legend
    plot(delta')
end


function test5(sim_out,dtheta2)
    temp = abs(dtheta2-sim_out.eps(:,:,10)) ./ abs(sim_out.eps(:,:,10));
    figure(51)
    bar(temp)
    
    T = sim_out.t;
    X = sim_out.x;
    U = sim_out.u;
    Psi = sim_out.Controller.BaseFnc(T,X);
    Psi = repmat(permute(Psi,[1 3 2]),[1 3 1]);
    temp = Psi .* repmat(sim_out.Controller.theta+dtheta2,[1 1 length(T)]);
%     temp = Psi .* ( repmat(sim_out.Controller.theta,[1 1 length(T)])+sim_out.eps );
    Up = squeeze(sum(temp,1));
    figure(52)
    plot(T,U','-',T,Up','--')
end
