params = get_robot_params(1);
model = get_robot_model(params);

nb = model.NB;
total_mass = 0;
for b = 1:nb
    [mb,~,~] = mcI(model.I{b});
    total_mass = total_mass + mb;
end

t = 0:dt:(N-1)*dt;
fz = Ffs(2,:) + Frs(2,:);
az = -fz/(total_mass) - 9.81;

vz = dt * cumsum(az);
pz = dt * cumsum(vz);

plot(t,pz,t,Qs(2,:)'); legend('integrated','optimized');

total_mass
[~,~,I_body] = mcI(model.I{3})