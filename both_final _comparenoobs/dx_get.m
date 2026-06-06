function dx = dx_get(t,x,Ay,Az,Vm)
    dx(1)=-Vm*cos(x(4))*cos(x(5));
    dx(2)=-Vm*sin(x(4))/x(1);
    dx(3)=-Vm*cos(x(4))*sin(x(5))/(x(1)*cos(x(2)));
    dx(4)=Az/Vm-dx(2)*cos(x(5))-dx(3)*sin(x(5))*sin(x(2));
    dx(5)=Ay/(Vm*cos(x(4)))-dx(2)*tan(x(4))*sin(x(5))-dx(3)*cos(x(2))+dx(3)*tan(x(4))*cos(x(5))*sin(x(2));
    dx=dx';
end