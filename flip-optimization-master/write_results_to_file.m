function  write_results_to_file( Qs, taus, Ffs, Frs, filename, dt, N )

data = [Qs; taus; Ffs; Frs]; 
N_interp = 1000 * dt * N; % number of .001 sec intervals
data_interp = zeros(size(data,1),N_interp);

x        = 0:N-1;
x_interp = linspace(0,N-1,N_interp);

for k = 1:size(data,1)
    data_interp(k,:) = interp1(x,data(k,:),x_interp);
end

fid = fopen(filename,'w');
fwrite(fid,data_interp,'single');

end