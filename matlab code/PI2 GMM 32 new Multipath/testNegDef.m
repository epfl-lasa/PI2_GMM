
for i = 1:100
    
    N = randn(4,4);
    L = triu(N);
    A = L*L';
    l = eig(A);
    if prod(l>0)
        ;
    else
        'error'
    end
end