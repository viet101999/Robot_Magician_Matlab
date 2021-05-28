function [ FK ] = FKdraw( j1,j2,j3 )
%initial parameter
%[j0 j1 j2 j3;d0 d1 d2 d3;a0 a1 a2 a3;t0 t1 t2 t3]
%theta3=360-theta1-theta2
j4=360-(j2+j3);
j=[j1 j2 j3 j4;102.03 0 0 2.4;30.6 177.5 190 81.3;90 0 0 0];
FK=DHkine(j);
Q=XYZkine(FK);
%J=Jacobian(FK); 
plot3(Q(1,:),Q(2,:),Q(3,:),'-o','LineWidth',2,'MarkerSize',6,'MarkerFaceColor',[0.5,0.5,0.5]);
grid on;%axis([-31,31,-31,31,0,31]);
text(Q(1,5),Q(2,5),Q(3,5),['  (', num2str(Q(1,5),3), ', ', num2str(Q(2,5),3),', ', num2str(Q(3,5),3), ')']);
title('3DOF Magican')
xlabel('X Axis');
ylabel('Y Axis');
zlabel('Z Axis');
axis([-500 500 -100 500 0 500]);
h = rotate3d;
h.Enable = 'on';
h.ActionPostCallback = @mypostcallback;
assignin('base','FK',FK);
pause(0.1);
end

function mypostcallback(obj,evd)
%disp('A rotation is about to occur.');
ax_properties = get(gca);
assignin('base','pov',ax_properties.View);
end

%use evalin('base',a) to get variable a from workspace 
%assignin('base','a_rms',a_rms) to write variable a_rms to workspace