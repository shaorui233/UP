figure(888);
N = size(Qs,2);
N_interp = 200;
x = 0:N-1;
x_i = linspace(0,N-1,N_interp);
Q_interp = zeros(7,N_interp);
f_interp = zeros(4,N_interp);
for k = 1:size(Qs,1)
    Q_interp(k,:) = interp1(x,Qs(k,:),x_i);
    
end
f_in = [Frs;Ffs];
for k = 1:4
    f_interp(k,:) = [interp1(x,f_in(k,:),x_i)];
end
%v = VideoWriter('fliip3.avi');
%open(v);
%K(N) = struct('cdata',[],'colormap',[]);

c00 = [-10;0];
c1 = pfi{2} - [.15;0];
c0 = [c1(1); -2];
c2 = c1 + [3;0];

for i = 1:N_interp-1
    [~,~,p0,~,~,pf0,~,~,~] = all_the_dynamics(model,Q_interp(1:7,i),Q_interp(8:end,i),zero_force,0);
    anim(:,i) = [pf0{2};p0{7};p0{6};p0{6};p0{4};p0{5};pf0{1}];
    ca = reshape(anim(:,i),3,[]);
    plot(ca(1,:),ca(3,:),'-mo',...
    'LineWidth',2,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor',[.49 1 .63],...
     'MarkerSize',10); axis(.5*[-3 1 -1.4 3]); 
 hold on;
 c_box = [c0 c1 c2];
 plot(c_box(1,:),c_box(2,:));
 hold off;
 hold on;
 
 f_graph1 = [pf0{2}([1 3]);pf0{2}([1 3]) + -.001 * f_interp(1:2,i)];
 f_graph2 = [pf0{1}([1 3]);pf0{1}([1 3]) + -.001 * f_interp(3:4,i)];
 
 cf1 = reshape(f_graph1,2,[]);
 cf2 = reshape(f_graph2,2,[]);
 
 plot(cf1(1,:),cf1(2,:));
 plot(cf2(1,:),cf2(2,:));
 hold off;
%pause(.01);
drawnow;
%K(i) = getframe(888);
     %writeVideo(v,K(i));
end
%close(v);