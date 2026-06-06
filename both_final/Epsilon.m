function [epsilon] = Epsilon(tgo,a,j)
switch j
    case 1
        matrix=[0 0 0 0;1 -1 0 0;1 0 -1 0;1 0 0 -1];
        epsilon=a(1,:)*(matrix*tgo');
    case 2
        matrix=[-1 1 0 0;0 0 0 0;0 1 -1 0;0 1 0 -1];
        epsilon=a(2,:)*(matrix*tgo');
    case 3
        matrix=[-1 0 1 0;0 -1 1 0;0 0 0 0;0 0 1 -1];
        epsilon=a(3,:)*(matrix*tgo');
    case 4
        matrix=[-1 0 0 1;0 -1 0 1;0 0 -1 1;0 0 0 0];
        epsilon=a(4,:)*(matrix*tgo');
end