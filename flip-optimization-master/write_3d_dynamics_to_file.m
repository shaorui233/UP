function  write_3d_dynamics_to_file( H,C,p,pf,Jf,Jdqdf,vf  )
q = sym('q',[18 1],'real');
qd = sym('qd',[18 1],'real');

Q = [q;qd];
parfor i = 1:7
    if i == 1
    matlabFunction(H,'File','3d_dynamics_out/H_sym','Vars',{q});
    disp('Done with H');
    end
    
    if i == 2
    matlabFunction(C,'File','3d_dynamics_out/C_sym','Vars',{[Q]});
    
    disp('Done with C');
    end
     
    if i == 3
    write_3d_cell_to_file(p,'p_sym',{q}); 
    disp('Done with p');
    end
     
    if i == 4
    write_3d_cell_to_file(pf,'pf_sym',{q});
    disp('Done with pf');
    end
     
    if i == 5
    write_3d_cell_to_file(Jf,'Jf_sym',{q});
    disp('Done with Jf');
    end
     
    if i == 6
    write_3d_cell_to_file(Jdqdf,'Jdqdf_sym',{[q, qd]});
    disp('Done with Jdqdf');
    end
    
    if i == 7
    write_3d_cell_to_file(vf,'vf_sym',{[q,qd]});
    disp('Done with vf');
    end
end



end

