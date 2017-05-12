function y = my_mvnpdf(x,mu,sigma)
   if size(x,2) > 1
       x = x';
   end
   if size(mu,2) > 1
       mu =  mu';
   end
   %y = 1/sqrt((2*pi).^(2*size(x,1))*norm(sigma))*exp(-1/2*(x-mu)'/sigma*(x-mu));
   y = exp(-1/2*(x-mu)'/sigma*(x-mu))./sqrt((2*pi).^size(x,1)*det(sigma));
end