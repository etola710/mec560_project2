close all
model_params

time = solution.phase.time;
states = solution.phase.state;
controls = solution.phase.control;

pp_q1=spline(time,states(:,1));
pp_q2=spline(time,states(:,2));
pp_q3=spline(time,states(:,3));
pp_dq1=spline(time,states(:,4));
pp_dq2=spline(time,states(:,5));
pp_dq3=spline(time,states(:,6));

pp_u1=spline(time,controls(:,1));
pp_u2=spline(time,controls(:,2));

pp_states.q1 = pp_q1;
pp_states.q2 = pp_q2;
pp_states.q3 = pp_q3;
pp_states.dq1 = pp_dq1;
pp_states.dq2 = pp_dq2;
pp_states.dq3 = pp_dq3;

pp_controls.u1= pp_u1;
pp_controls.u2= pp_u2;

dt_sim = .01;
tf = time(end);
N_sim = tf/dt_sim;

t_new = (0:1/(N_sim-1):1)*tf;

filename = ['DC_gamma_' num2str(gamma) '.gif'];

figure;

for i=1:1:length(t_new)
    q1_st = ppval(t_new(i),pp_states.q1);
    q2_st = ppval(t_new(i),pp_states.q2);
    q3_st = ppval(t_new(i),pp_states.q3);
    dq1_st = ppval(t_new(i),pp_states.dq1);
    dq2_st = ppval(t_new(i),pp_states.dq2);
    dq3_st = ppval(t_new(i),pp_states.dq3);
    
    u_th1=0;
    u_th2=0;
    joint1=J1(q1_st,q2_st,q3_st,dq1_st,dq2_st,dq3_st,m1,m2,m3,l1,l2,l3,r1,r2,r3,u_th1,u_th2,g);
    joint2=J2(q1_st,q2_st,q3_st,dq1_st,dq2_st,dq3_st,m1,m2,m3,l1,l2,l3,r1,r2,r3,u_th1,u_th2,g);
    joint3=J3(q1_st,q2_st,q3_st,dq1_st,dq2_st,dq3_st,m1,m2,m3,l1,l2,l3,r1,r2,r3,u_th1,u_th2,g);
    
    mass1=P1(q1_st,q2_st,q3_st,dq1_st,dq2_st,dq3_st,m1,m2,m3,l1,l2,l3,r1,r2,r3,u_th1,u_th2,g);
    mass2=P2(q1_st,q2_st,q3_st,dq1_st,dq2_st,dq3_st,m1,m2,m3,l1,l2,l3,r1,r2,r3,u_th1,u_th2,g);
    mass3=P3(q1_st,q2_st,q3_st,dq1_st,dq2_st,dq3_st,m1,m2,m3,l1,l2,l3,r1,r2,r3,u_th1,u_th2,g);
    
    plot(joint1(1),joint1(2),'ko')
    hold on
    axis([-10 10 -10 10])
    
    plot(0,0,'ko')
    plot(joint2(1),joint2(2),'ko')
    plot(joint3(1),joint3(2),'ko')
    plot(mass1(1),mass1(2),'ro')
    plot(mass2(1),mass2(2),'ro')
    plot(mass3(1),mass3(2),'ro')
    
    line([0 joint1(1)],[0 joint1(2)])
    line([joint1(1) joint2(1)],[joint1(2),joint2(2)])
    line([joint2(1) joint3(1)],[joint2(2),joint3(2)])
        
    xlabel('X')
    ylabel('Y')
    title(['Cost = t_f + ' num2str(gamma) ' \times U^2' ])
    text(.5,2.5,['t = ' num2str(t_new(i))] )
    hold off
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if i  == 1;
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',.01);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',.01);
    end
    
    pause(.1)
end