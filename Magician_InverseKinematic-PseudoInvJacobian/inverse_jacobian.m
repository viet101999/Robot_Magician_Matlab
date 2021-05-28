j1 = 0; j2 = 0; j3 = 0; j4=0; j5=0; j6=0; j7=0
j=[j1 j2 j3 j4 j5 j6 j7;1 0 1 0 1 0 1;0 1 0 1 0 1 0;90 -90 90 -90 90 -90 90];
FK=DHkine(j);

pos_start=[3;0;1];

pos_target=[2;0;2];

%pos_target=evalin('base','pos_target');
jstart=[0;0;0];
j1=jstart(1,1);
j2=jstart(2,1);
j3=jstart(3,1);
if pos_target==pos_start
   movement=0;
else
   movement=1;
   delta=divelo(pos_start,pos_target);
end

while movement==1
j4=360-(j2+j3);
j=[j1 j2 j3 j4;1 0 0 0;0 1 1 1;90 0 0 0];
FK=DHkine(j);
Jac=Jacobian(FK);
Jacinv=pinv(Jac);
dXYZ=delta(1:3,1)/10;
dTheta=Jacinv*dXYZ;
dTheta1=radtodeg(dTheta(1,1));
dTheta2=radtodeg(dTheta(2,1));
dTheta3=radtodeg(dTheta(3,1));
j1=j1+dTheta1;
j2=j2+dTheta2;
j3=j3+dTheta3;

pos_new=[FK(1,16);FK(2,16);FK(3,16)];
delta=divelo(pos_new,pos_target);
EucError=delta(5,1)^2;
OrinError=delta(6,1)^2;

if EucError <10^-12
   movement=0; 
   jstart=[j1;j2;j3];
end 

% if all(delta(:) <= 0.1)
%    movement=0; 
% end 

    
%dEucXY=delta(4,1);
%dEucXYZ=delta(5,1);
end
jstart=[j1;j2;j3]