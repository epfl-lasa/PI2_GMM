clear all
hold on
k = 1;
for i = -5:0.5:2
    for j = -3:0.5:3
        X(k) = i;
        Y(k) = j;
        U(k)= 0;
        V(k) = 5;
        k = k+1;
    end
end

quiver(X,Y,U,V);