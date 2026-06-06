function [result] = Taylor(n,m,sigma,miu,sigma_max)
    num=0;
    den=0;
    k=pi/(miu*sigma_max)^m;
for i=1:n
        num=num+(-1)^(i-1)*((k)^(2*i)*sigma^(2*(m*i-1))/factorial(2*i));
        den=den+(-1)^(i-1)*((2)^(2*i)*sigma^(2*(i-1))/factorial(2*i));
end
    result=num/den;
end