%% C shape

hold on
scatter([-30 -20 0],[100 30 0], 80, 'g')
scatter([-30 -20 0],[100 30 0], 80, 'gx')

%% N shape 5 
hold on
scatter([-120 -60 -30 0],[-10 -40 -110 0], 80, 'g')
scatter([-120 -60 -30 0],[-10 -40 -110 0], 80, 'gx')

%% N shape 4 
hold on
scatter([-100 10 0],[-50 -70 0], 80, 'g')
scatter([-100 10 0],[-50 -70 0], 80, 'gx')

%% N shape 3 
hold on
scatter([-90 -60 0],[-10 -130 0], 80, 'g')
scatter([-90 -60 0],[-10 -130 0], 80, 'gx')

%% N shape 1 
hold on
scatter([-100 0 0],[-50 -70 0], 80, 'g')
scatter([-100 0 0],[-50 -70 0], 80, 'gx')

%% P shape 3 
hold on
scatter([-45 -10 10 0],[20 30 15 0], 80, 'g')
scatter([-45 -10 10 0],[20 30 15 0], 80, 'gx')

%% Line 1 
hold on
scatter([200 125 100 0],[50 60 20 0], 80, 'g')
scatter([200 125 100 0],[50 60 20 0], 80, 'gx')

%% Line L Turn 
hold on
scatter([2 4 6 6 6 6],[0 0 0 2 4 6], 80, 'g')
scatter([2 4 6 6 6 6],[0 0 0 2 4 6], 80, 'gx')

%% Line Bump 
hold on
scatter([2 4 6 8 10],[0 0 3 0 0 ], 80, 'g')
scatter([2 4 6 8 10],[0 0 3 0 0 ], 80, 'gx')

%% Plot box, start and goal
hold on
 line([-2.5, -2.5], [-2.5, 2]);
 line([-2.5, 2.5], [-2.5, -2.5]);
 line([2.5, 2.5], [-2.5, 2]);
  line([-1.5, -1.5], [-1.5, 2]);
 line([-1.5, 1.5], [-1.5, -1.5]);
 line([1.5, 1.5], [-1.5, 2]);
 line([-2.5, -1.5], [2, 2]);
 line([2.5, 1.5], [2, 2]);
 scatter(0,0,'sk')
 scatter(0,0,'xk')
 scatter(-6,-2,'sk')
 scatter(-6,-2,'xk')
 
 %% Two Sides
 hold on
 scatter([-6 -4 -2 0 2 4 6],[2 0 -2 0 -3 -3 0], 80, 'g')
 
  %% Two Sides 2
 hold on
%  scatter([-6 -4 -2 0 2 3 5 6],[-3 0 3 0 0 -4 -4 0], 80, 'g')
%  scatter([-6 -4 -2 0 2 3 5 6],[-3 0 3 0 0 -4 -4 0], 80, 'gx')
 scatter([-6 -4 -2],[-3 0 3], 80, 'g')
 scatter([2 3 5 6],[0 -4 -4 0], 80, 'g')
 scatter([0],[0], 80, 'g')