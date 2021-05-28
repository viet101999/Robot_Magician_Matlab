%initial parameter
%[j0 j1 j2 j3;d0 d1 d2 d3;a0 a1 a2 a3;t0 t1 t2 t3]
%theta3=360-theta1-theta2
j1 = 90;
j2 = 118;
j3 = -142;
j4=360-(j2+j3);
l1 = 102.03; l2 = 177.5; l3 = 190; l4 = 81.3; l5 = 30.6; l6 = 0; %l6 = 2.4;
j=[j1 j2 j3 j4;l1 0 0 l6;l5 l2 l3 l4;90 0 0 0];
FK=DHkine(j);
Q=XYZkine(FK);
J=Jacobian(FK); 
plot3(Q(1,:),Q(2,:),Q(3,:),'-o','LineWidth',2,'MarkerSize',6,'MarkerFaceColor',[0.5,0.5,0.5]);grid on;%axis([-31,31,-31,31,0,31]);
text(Q(1,5),Q(2,5),Q(3,5),['  (', num2str(Q(1,5),3), ', ', num2str(Q(2,5),3),', ', num2str(Q(3,5),3), ')']);
title('3DOF Pseudo Inverse Jacobian Inverse Kinematic')
xlabel('X Axis');
ylabel('Y Axis');
zlabel('Z Axis');
axis([-500 500 -100 500 0 500]);
h = rotate3d;
h.Enable = 'on';
h.ActionPostCallback = @mypostcallback;
assignin('base','FK',FK);
pause(0.1);