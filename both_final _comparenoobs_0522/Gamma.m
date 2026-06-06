function [result] = Gamma(sigma,sigma_max,miu,m)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
sigma=sigma/sigma_max;
if (sigma<miu)|(sigma==miu)
    result=(1-cos(pi*(sigma/miu)^m))/2;
else
    result=1;
end