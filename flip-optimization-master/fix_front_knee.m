plot(Qs(5,:)');

knee_zone = 50;

q5 = Qs(5,1:knee_zone);
qd5 = Qs(5+7,1:knee_zone);

qd5(q5 < 1.5) = 0;
q5(q5 < 1.5) = 1.5;


qfixed = [q5 Qs(5,knee_zone+1:end)];
qdfixed = [qd5 Qs(5+7,knee_zone+1:end)];

subplot(2,1,1);
plot([qfixed; Qs(5,:)]');
subplot(2,1,2);
plot([qdfixed; Qs(12,:)]');

Qs(5,:) = qfixed;
Qs(12,:) = qdfixed;