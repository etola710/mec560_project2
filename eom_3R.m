clc
close all
clear all

addpath('Screws','fcn_support','screw_theory')
% Defining symbols
syms m1 m2 m3 l1 l2 l3 r1 r2 r3;
syms q1 q2 q3 dq1 dq2 dq3
syms tau1 tau2 tau3
syms u_th1 u_th2 g real;
syms q10  q20 q30;

vars = {'q1', 'q2', 'q3' , 'dq1', 'dq2' , 'dq3',...
    'm1','m2','m3','l1','l2','l3','r1','r2','r3',...
    'u_th1','u_th2' , 'g'};
%{
j=[1 1 1]; %3R
mass=[m1 m2 m3];
dim(:,1)=[l1 wd ht]';
dim(:,2)=[l2 wd ht]';
dim(:,3)=[l3 wd ht]';
%angles
th=[q1 q2 q3];
th_d=[dq1 dq2 dq3];
%joint axes
w(:,1) = [ 0 0 -1]';
w(:,2) = w(:,1);
w(:,3) = w(:,1);
%positions
p(:,1)=[-r1 0 0 1]';
p(:,2)=[(-l1-r2) 0 0 1]';
p(:,3)=[(-l1-l2-r3) 0 0 1]';
%ref config
g0(:,:,1)=sym_tran(eye(3),p(:,1));
g0(:,:,2)=sym_tran(eye(3),p(:,2));
g0(:,:,3)=sym_tran(eye(3),p(:,3));

q_v = [q1;q2;q3];
dq_v = [dq1;dq2;dq3];

M=manip_inertial_mat(mass,dim,g0,th,th_d,j,w,p);
C=D2C(M,q_v,dq_v);

V=0;
[gst(:,:,1),~]= manipdkin(g0(:,:,1),w,p(1:3,:),j(1),th);
[gst(:,:,2),~]= manipdkin(g0(:,:,2),w,p(1:3,:),j(1:2),th);
[gst(:,:,3),~]= manipdkin(g0(:,:,3),w,p(1:3,:),j,th);

for i= 1:length(j)
    V=V+mass(i)*g*gst(1,4,i);
end

N=Gvector(V,q_v);
    %}
    %{\
    % Position vectors
    P1 = [r1 * cos(q1);
        r1 * sin(q1)];
    
    P2 = [l1 * cos(q1) + r2 * cos(q1+q2);
        l1 * sin(q1)+ r2 * sin(q1+q2)];
    
    P3 = [l1 * cos(q1) + l2*cos(q1+q2) + r3* cos(q1+q2+q3);
        l1 * sin(q1)+ l2 * sin(q1+q2) + r3*sin(q1+q2+q3)];
    
    %joint positions
    J1=[l1 * cos(q1) ; l1 * sin(q1)];
    J2=[l1 * cos(q1) + l2*cos(q1+q2) ; 
        l1 * sin(q1)+ l2 * sin(q1+q2)];
    J3=[l1 * cos(q1) + l2*cos(q1+q2) + l3* cos(q1+q2+q3);
        l1 * sin(q1)+ l2 * sin(q1+q2) + l3*sin(q1+q2+q3)];
    
    write_kin=1;
    if write_kin == 1
        write_file(P1,'P1.m',vars);
        write_file(P2,'P2.m',vars);
        write_file(P3,'P3.m',vars);
        write_file(J1,'J1.m',vars);
        write_file(J2,'J2.m',vars);
        write_file(J3,'J3.m',vars);
    end
    
    q_v = [q1;q2;q3];
    dq_v = [dq1;dq2;dq3];
    %
    
    % Taking derivative to compute velocities
    V1 = get_vel(P1,q_v,dq_v);
    V2 = get_vel(P2,q_v,dq_v);
    V3 = get_vel(P3,q_v,dq_v);
    
    % Computing Kinetic energy and potential energy
    KE1 =simplify(1/2*m1*V1.'*V1);
    KE2 =simplify(1/2*m2*V2.'*V2);
    KE3 =simplify(1/2*m3*V3.'*V3);
    
    PE1 = m1*g*P1(2);
    PE2 = m2*g*P2(2);
    PE3 = m3*g*P3(2);
    
    % Define Lagrangian
    T = KE1 + KE2 + KE3;
    V = PE1 + PE2 + PE3;
    %}
    %{\
    [M,C,N] = get_mat(T, V, q_v,dq_v);
    
    M = simplify(M);
    C = simplify(C);
    N = simplify(N);
    %}
    %{
% Now express this in the form of dx/dt = f(x,u)
x = [q1;q2;q3;dq1;dq2;dq3]; % Vector of state space
ddq0 = [0;0;0]; % Vector of SS joint accelerations
x0 = [q10;q20;q30;0;0;0]; % Vector of SS joint angles and velocites
tau_v = [tau1;tau2;tau3]; % Vector of torques
    %}
    %{
% Function to calculate Linearized representation
[A_lin,B_lin] = linearize_DCG(M,C,N,x,tau_v,x0,ddq0);
A_lin = simplify(A_lin);
B_lin = simplify(B_lin);
size(A_lin)
size(B_lin)

    %}
    %{\
    U=[0;u_th1;u_th2];
    
    ff = U - C*dq_v - N;
    
    write_kin = 1;
    if write_kin == 1
        write_file(ff,'ff_acrobat.m',vars); % Writing FF
        write_file(M,'M_acrobat.m',vars); % Writing M
        %{
    write_file(T,'KE_acrobat.m',vars); % Writing KE
    write_file(V,'PE_acrobat.m',vars); % Writing PE
        %}
    end
    %}
    %{
    
    syms q1_d q2_d q3_d 'real'
    syms dq1_d dq2_d dq3_d ddq1_d ddq2_d ddq3_d 'real'
    syms tau1_d tau2_d tau3_d 'real'
    syms u_th1_d u_th2_d 'real'
    
    vars_d = {'q1_d', 'q2_d', 'q3_d' , 'dq1_d', 'dq2_d' , 'dq3_d',...
        'm1','m2','m3','l1','l2','l3','r1','r2','r3',...
        'u_th1_d','u_th2_d' , 'g'};
    
    tau_d=[u_th1_d ;u_th2_d];
    tau_v=[u_th1 ;u_th2];
    
    X_q = [q1 ;q2 ;q3 ;dq1 ;dq2 ;dq3];
    X_q_d = [q1_d ;q2_d ;q3_d ;dq1_d ;dq2_d ;dq3_d];
    
    [dfdq,dMdq,dfdu] = linearize_MF(M,ff,X_q,tau_v,X_q_d,tau_d);
    
    display('Testing dMidq, should retun 0s')
    M_d=subs(M,[q1 q2 q3],[q1_d q2_d q3_d]);
    M_d=simplify(simplify(M_d));
    %Mi=inv(M);
    
    for i = 1:length(X_q)/2
        dmidq(:,:,i) = subs(diff(inv(M),X_q(i)),[q1 q2 q3],[q1_d q2_d q3_d]);
        dmidq_c(:,:,i)=-inv(M_d)*dMdq(:,:,i)*inv(M_d);
        %simplify(dmidq(:,:,i)-dmidq_c(:,:,i));
        if norm(simplify(dmidq(:,:,i)-dmidq_c(:,:,i)))==0
            display('Direct calculation and analytical results returned the same values')
            display('dMidq calculated correctly')
        else
            display('Direct calculation and analytical results returned DIFFERENT values')
            display('CHECK CALCULATIONS of dMidq')
        end
    end
    
    
    
    Miff = Mi*ff;
    Miff = simplify(Miff);
    
    display('Testing dMiffdq, should retun 0s')
    for i = 1:length(X_q)/2
        
        dmiffdq(:,i) = subs(diff(Miff,X_q(i)),[q1 q2 q3],[q1_d q2_d q3_d]);
        
        Miff_col1 = dmidq(:,:,i)*subs(ff, [q1 q2 q3],[q1_d q2_d q3_d]);
        Miff_col2 = subs(M\diff(ff,X_q(i)),[q1 q2 q3],[q1_d q2_d q3_d]);
        
        dmiffdq_c(:,i) =Miff_col1 + Miff_col2 ;
    end
    
    if norm(simplify(dmiffdq_c-dmiffdq))==0
        display('Direct calculation and analytical results returned the same values')
        display('dMiffdq calculated correctly')
    else
        display('Direct calculation and analytical results returned DIFFERENT values')
        display('CHECK CALCULATIONS of dMiffdq')
    end
    
%}