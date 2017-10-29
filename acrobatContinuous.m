function phaseout = acrobatContinuous(input)

q1_all = input.phase.state(:,1);
q2_all = input.phase.state(:,2);
q3_all = input.phase.state(:,3);
dq1_all = input.phase.state(:,4);
dq2_all = input.phase.state(:,5);
dq3_all = input.phase.state(:,6);
u1_all = input.phase.control(:,1);
u2_all = input.phase.control(:,2);

dq_all = get_dynamics(q1_all,q2_all,q3_all,dq1_all,dq2_all,dq3_all,u1_all,u2_all);


phaseout.dynamics  = dq_all;
phaseout.integrand = u1_all.^2+u2_all.^2;
end