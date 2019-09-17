function write_dynamics_to_file( H,C,p,pf,Jf,Jdqdf,vf )

q = sym('q',[7 1],'real');
qd = sym('qd',[7 1],'real');
f_front = sym('ff',[3 1],'real');
%f_front = [0; 0; 0; f_front];
f_rear  = sym('fr',[3 1],'real');
%f_rear = [0; 0; 0; f_rear];

Q = [q;qd];
F = [f_front;f_rear];


parfor i = 1:7
    if i == 1
    matlabFunction(H,'File','dynamics_out/H_sym','Vars',{q});
    disp('Done with H');
    end
    
    if i == 2
    matlabFunction(C,'File','dynamics_out/C_sym','Vars',{[Q;F]});
    
    disp('Done with C');
    end
     
    if i == 3
    write_cell_to_file(p,'p_sym',{q}); 
    disp('Done with p');
    end
     
    if i == 4
    write_cell_to_file(pf,'pf_sym',{q});
    disp('Done with pf');
    end
     
    if i == 5
    write_cell_to_file(Jf,'Jf_sym',{q});
    disp('Done with Jf');
    end
     
    if i == 6
    write_cell_to_file(Jdqdf,'Jdqdf_sym',{[q, qd]});
    disp('Done with Jdqdf');
    end
    
    if i == 7
    write_cell_to_file(vf,'vf_sym',{[q,qd]});
    disp('Done with vf');
    end
end
end

