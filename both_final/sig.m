function [result] = sig(epsilon,k)
    result=sign(epsilon)*abs(epsilon)^k;
end