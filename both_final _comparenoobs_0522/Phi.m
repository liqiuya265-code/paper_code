function [result] = Phi(sigma,sigma_m,n)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
sigma=sigma/sigma_m;
if (sigma>1)||(sigma<-1)
    result=1;
else
    result=1/2*(1-cos((pi*abs(sigma)^n)));
end