clc;clear
eta=0.5;
N=1:6;
for i=1:6
prob(i)=(1-eta)^N(i)+N(i)*eta*(1-eta)^(N(i)-1);
end
