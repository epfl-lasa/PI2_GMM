x = 0:0.1:10;
y1 = normpdf(x,2,1);
y2 = normpdf(x,8,1)*2;
y3 =  normpdf(x,2,1)./(normpdf(x,2,1)+normpdf(x,8,1))*1 + normpdf(x,8,1)./(normpdf(x,2,1)+normpdf(x,8,1))*5;

figure
hold on
plot(x,y1)
plot(x,y2)
plot(x,y3,'r')