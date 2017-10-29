function [dq_all] = get_dynamics(q1_all,q2_all,q3_all,dq1_all,dq2_all,dq3_all,u1_all,u2_all)

model_params;

for i = 1:length(q1_all);
    
q1 = q1_all(i) ;
q2 = q2_all(i);
q3 = q3_all(i);
dq1 = dq1_all(i);
dq2 = dq2_all(i) ;
dq3 = dq3_all(i);
u_th1 = u1_all(i);
u_th2 = u2_all(i) ;

    ff = ff_acrobat(q1,q2,q3,dq1,dq2,dq3,m1,m2,m3,l1,l2,l3,r1,r2,r3,u_th1,u_th2,g);
    M = M_acrobat(q1,q2,q3,dq1,dq2,dq3,m1,m2,m3,l1,l2,l3,r1,r2,r3,u_th1,u_th2,g);

    ddq = M\ff;
    
    dq_all(i,:) = [dq1 dq2 dq3 ddq'];
end