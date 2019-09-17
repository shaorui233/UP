
t = linspace(0,dt*N,length(Qs));

figure();
subplot(2,2,1);
plot(t,rad2deg(Qs(3,:)),t,rad2deg(Qs(10,:))); title('Pitch');
xlabel('Time (seconds)'); ylabel('Deg, Deg/s'); legend('Pitch Angle', 'Pitch Rate');

subplot(2,2,2);
plot(t,taus');
title('Torque'); legend('Front hip','Front knee','Rear hip','Rear knee'); grid on;


subplot(2,2,3);
plot(t,Qs(7 + (4:7),:)'); title('Velocity'); legend('Front hip','Front knee','Rear hip','Rear knee');

subplot(2,2,4);
plot(t,Qs(4:7,:)'); title('Position'); legend('Front hip','Front knee','Rear hip','Rear knee');