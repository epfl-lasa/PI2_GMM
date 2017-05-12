resultsBatch = results_this_protocol;
for i = 1:length(resultsBatch)
  % costEvo(:,i) = resultsBatch(i).cost(:,3);
  costEvo(:,:,i) = resultsBatch(i).cost;
   finalCost(i) = resultsBatch(i).cost(end,3);
   finalDesTraj1(i,:) = resultsBatch(i).Dfin.traj_y(1,:);
   finalDesTraj2(i,:) = resultsBatch(i).Dfin.traj_y(2,:);
   finalActualTraj1(i,:) = resultsBatch(i).Dfin.q(1,:);
   finalActualTraj2(i,:) = resultsBatch(i).Dfin.q(2,:);
   stiffness1(i,:) = resultsBatch(i).Dfin.kp(1,:);
  stiffness2(i,:) = resultsBatch(i).Dfin.kp(2,:);
  % stiffness2(i,:) = stiffness1(i,:);
end

costEvoMean = mean(costEvo,3);
costEvoStd = std(costEvo,0,3);
costEvoRolloutIndex = resultsBatch(1).cost(:,1);

figure
hold on
plot(costEvoRolloutIndex, costEvoMean(:,3));
plot(costEvoRolloutIndex, costEvoMean(:,3)+costEvoStd(:,3), '.');
plot(costEvoRolloutIndex, costEvoMean(:,3)-costEvoStd(:,3), '.');
plot(costEvoRolloutIndex, costEvoMean(:,5),'r');
plot(costEvoRolloutIndex, costEvoMean(:,5)+costEvoStd(:,5), '.r');
plot(costEvoRolloutIndex, costEvoMean(:,5)-costEvoStd(:,5), '.r');
plot(costEvoRolloutIndex, costEvoMean(:,8),'m');
plot(costEvoRolloutIndex, costEvoMean(:,8)+costEvoStd(:,8), '.m');
plot(costEvoRolloutIndex, costEvoMean(:,8)-costEvoStd(:,8), '.m');
plot(costEvoRolloutIndex, costEvoMean(:,6),'g');
plot(costEvoRolloutIndex, costEvoMean(:,6)+costEvoStd(:,6), '.g');
plot(costEvoRolloutIndex, costEvoMean(:,6)-costEvoStd(:,6), '.g');
plot(costEvoRolloutIndex, costEvoMean(:,7),'k');
plot(costEvoRolloutIndex, costEvoMean(:,7)+costEvoStd(:,7), '.k');
plot(costEvoRolloutIndex, costEvoMean(:,7)-costEvoStd(:,7), '.k');
hold off
axis([1 1000 0 10])

costEvoMed = median(costEvo,3);
sortedCostEvo = sort(costEvo,3);
costEvoUpperQuartile = sortedCostEvo(:,:,ceil(length(resultsBatch)*0.75));
costEvoLowerQuartile = sortedCostEvo(:,:,ceil(length(resultsBatch)*0.25));
costEvoRolloutIndex = resultsBatch(1).cost(:,1);

% figure
% hold on
% plot(costEvoRolloutIndex, costEvoMed(:,3));
% plot(costEvoRolloutIndex, costEvoUpperQuartile(:,3) , '.');
% plot(costEvoRolloutIndex, costEvoLowerQuartile(:,3) , '.');
% plot(costEvoRolloutIndex, costEvoMed(:,5),'r');
% plot(costEvoRolloutIndex, costEvoUpperQuartile(:,5) , '.r');
% plot(costEvoRolloutIndex, costEvoLowerQuartile(:,5) , '.r');
% plot(costEvoRolloutIndex, costEvoMed(:,8),'g');
% plot(costEvoRolloutIndex, costEvoUpperQuartile(:,8) , '.g');
% plot(costEvoRolloutIndex, costEvoLowerQuartile(:,8) , '.g');
% plot(costEvoRolloutIndex, costEvoMed(:,6),'k');
% plot(costEvoRolloutIndex, costEvoUpperQuartile(:,6) , '.k');
% plot(costEvoRolloutIndex, costEvoLowerQuartile(:,6) , '.k');
% plot(costEvoRolloutIndex, costEvoMed(:,7),'m');
% plot(costEvoRolloutIndex, costEvoUpperQuartile(:,7) , '.m');
% plot(costEvoRolloutIndex, costEvoLowerQuartile(:,7) , '.m');
% hold off

figure
hold on
%plot(finalDesTraj1',finalDesTraj2','.r')
plot(finalActualTraj1',finalActualTraj2')
hold off

figure
hold on
subplot(2,1,1)
plot(stiffness1')
subplot(2,1,2)
plot(stiffness2')
hold off

meanStiff1 = mean(stiffness1,1);
meanStiff2 = mean(stiffness2,1);
stdStiff1 = std(stiffness1,0,1);
stdStiff2 = std(stiffness2,0,1);

figure
subplot(2,1,1)
hold on
plot(meanStiff1)
plot(meanStiff1+stdStiff1,'.')
plot(meanStiff1-stdStiff1,'.')
hold off
subplot(2,1,2)
hold on
plot(meanStiff2)
plot(meanStiff2+stdStiff2,'.')
plot(meanStiff2-stdStiff2,'.')
hold off


figure
hold on
for i = 1:length(resultsBatch)
    scatter(1,finalCost(i))
end
hold off