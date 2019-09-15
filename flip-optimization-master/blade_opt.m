% add casadi library (this probably only works on Linux)
addpath(genpath('casadi'));

% add spatial v2 library (should work on all OS)
addpath(genpath('spatial_v2'));

s = 150;
n_pts = 20;
ds = 15;
mp = round((n_pts)/2);

opti = casadi.Opti();

X = opti.variable(n_pts,n_pts);

opti.subject_to(X(:) > 0);
opti.subject_to(X(:) < 1);

moi = 0;
plast = 0;
bite = 0;
allgrad = 0;
for i = 1:n_pts
    for j = 1:n_pts
        x_p = ((i - mp)/n_pts) * s;
        y_p = ((j - mp)/n_pts) * s;
        r2 = x_p^2 + y_p^2;
        moi = X(i,j) * r2 / (ds^2 * 10) + moi ;
        bite = bite + (plast - X(i,j))^2;
        plast = X(i,j);
        if(r2 < 500) 
            opti.subject_to(X(i,j) > 0.6)
        end
        
        if(i + 1 <= n_pts)
            allgrad = allgrad + (X(i,j) - X(i+1,j))^2;
        end
        
        if(i - 1 >= 1)
            allgrad = allgrad + (X(i,j) - X(i-1,j))^2;
        end
        
        if(j + 1 <= n_pts)
            allgrad = allgrad + (X(i,j) - X(i,j+1))^2;
        end
        
        if(j - 1 >= 1)
            allgrad = allgrad + (X(i,j) - X(i,j-1))^2;
        end
        
    end
end



opti.minimize(-moi - bite + allgrad);
opti.subject_to(sum(X(:)) == n_pts * n_pts / 2);

opti.solver('ipopt');
sol = opti.solve();
Xs = sol.value(X);
mois = sol.value(moi)
surf(double(Xs > 0.5));