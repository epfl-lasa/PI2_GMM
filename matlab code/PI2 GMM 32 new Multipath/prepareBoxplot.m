%%
j=1
labs{1} = 'Tuned 0025';
labs{2} = 'Tuned 005';
labs{3} = 'Tuned 01';
labs{4} = 'Tuned 02';
labs{5} = 'Tuned 04';
labs{6} = 'Tuned 08';
labs{7} = 'Untuned 00025';
labs{8} = 'Untuned 0005';
labs{9} = 'Untuned 001';
labs{10} = 'Untuned 002';
labs{11} = 'Untuned 004';
labs{12} = 'Untuned 008';

%%
j=1
labs{1} = 'PI2';
labs{2} = 'PI2 no M';
labs{3} = 'PI2 01';
labs{4} = 'PI2 BB';
%%
j=1
labs{1} = 'without activation weight';
labs{2} = 'with activation weight';
%%
j=1
labs{1} = 'SEDS';
labs{2} = 'GMR';
%%
j=1
labs{1} = 'PI2';
labs{2} = 'PIBB';
%%
j=1
labs{1} = '0.01';
labs{2} = '0.025';
labs{3} = '0.05';
labs{4} = '0.1';
labs{5} = '0.2';
labs{6} = '0.3';
labs{7} = '0.4';
%%
for i=1:length(results_this_protocol)
finalCosts(i,j) = results_this_protocol(i).cost(end,3);
shortTimeCosts(i,j) = results_this_protocol(i).cost(10,3);
end
j=j+1
%%
figure
boxplot(finalCosts,'notch','on','labels',labs)
ylabel('cost at final iteration')
title('N-shape 3')
%scatter(ones(1,length(finalCosts)),finalCosts)
%hold off
figure
boxplot(shortTimeCosts,'notch','on','labels',labs)
title('N-shape 3')
ylabel('Cost at 10th iteration')
%%
figure
boxplot(finalCosts,'notch','on')