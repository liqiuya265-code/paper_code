function x_new = RK4(t,x,Ay,Az,h,Vm)

k1 = dx_get(t,x,Ay,Az,Vm);
k2=  dx_get(t+h/2,x+h*k1/2,Ay,Az,Vm);
k3=  dx_get(t+h/2,x+h*k2/2,Ay,Az,Vm);
k4=  dx_get(t+h,  x+h*k3,  Ay,Az,Vm);
x_new=x+h*(k1+2*k2+2*k3+k4)/6;