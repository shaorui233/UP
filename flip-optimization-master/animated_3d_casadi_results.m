figure(555);
N = size(Qs,2);
N_interp = 200;
x = 0:N-1;
x_i = linspace(0,N-1,N_interp);
Q_interp = zeros(7,N_interp);
for k = 1:size(Qs,1)
    Q_interp(k,:) = interp1(x,Qs(k,:),x_i);
    
end

a_pts_1 = [4 3 2 1 5 6 7 8];
a_pts_2 = a_pts_1 + 8;
a_pts_3 = [1 5 13 9 1];
v = VideoWriter('spin2.avi');
open(v);
K(N) = struct('cdata',[],'colormap',[]);

for i = 1:N_interp-1
    [~,~,p0,~,~,pf0,~,~,~] = all_the_dynamics(model,Q_interp(1:18,i),Q_interp(19:end,i),zero_force,0);
         anim(:,i) = [p0{7};p0{8};p0{9};pf0{1};
         p0{10};p0{11};p0{12};pf0{2};
         p0{13};p0{14};p0{15};pf0{3};
         p0{16};p0{17};p0{18};pf0{4}];
    c1 = reshape(anim(:,i),3,[]);

    
    plot3(c1(1,a_pts_1),c1(2,a_pts_1),c1(3,a_pts_1),'-mo',...
    'LineWidth',2,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor',[.49 1 .63],...
    'MarkerSize',10); hold on;
    plot3(c1(1,a_pts_2),c1(2,a_pts_2),c1(3,a_pts_2),'-mo',...
    'LineWidth',2,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor',[.49 1 .63],...
    'MarkerSize',10);
    plot3(c1(1,a_pts_3),c1(2,a_pts_3),c1(3,a_pts_3),'-mo',...
    'LineWidth',2,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor',[.49 1 .63],...
    'MarkerSize',10); hold off; grid on;
axis(.3 * [-2 2 -2 2 -1 3]);
view(20,80);
    drawnow;
 
%pause(.01);
drawnow;
K(i) = getframe(555);
     writeVideo(v,K(i));
end

close(v);