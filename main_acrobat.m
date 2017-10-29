%-------------------------- Cart-pole Problem -----------------------%
%--------------------------------------------------------------------%
clear all
close all
clc

addpath fcn_models

initial = [-(pi/2) 0 0 0 0 0];
final =[(pi/2) 0 0 0 0 0];
% Auxillary data:
%-------------------------------------------------------------------%
%-------------------- Data Required by Problem ---------------------%
%-------------------------------------------------------------------%
% Some model related constants
gamma = .0001;
auxdata.gamma = gamma ;


% Parameters:
%-------------------------------------------------------------------%
%-------------------- Data Required by Problem ---------------------%
%-------------------------------------------------------------------%
%{\
% Some model related constants
ndof     = 3;
nstates  = 2*ndof;
joints   = {'q1' 'q2' 'q3'};
njoints  = size(joints,2);

dofnames = {'q1','q2','q3'};
N  = nstates;        % number of state variables in model
Nu = 2;         % number of controls
%}
%-------------------------------------------------------------------%
%----------------------------- Bounds ------------------------------%
%-------------------------------------------------------------------%

%-------------------------------------------------------------------%
t0  = 0;
tf  = 5;

xMin =[-2*pi deg2rad(-40) deg2rad(-40) -100 -100 -100];       %minimum of coordinates
xMax =[2*pi deg2rad(120) deg2rad(120) 100 100 100];       %maximum of coordinates

uMin = [-10 -10]; %minimum of torques
uMax = [10 10]; %maximum of torques

% setting up bounds
bounds.phase.initialtime.lower  = 0;
bounds.phase.initialtime.upper  = 0;
bounds.phase.finaltime.lower    = 0.05; 
bounds.phase.finaltime.upper    = tf;
bounds.phase.initialstate.lower = initial;
bounds.phase.initialstate.upper = initial;
bounds.phase.state.lower        = xMin;
bounds.phase.state.upper        = xMax;
bounds.phase.finalstate.lower   = final;
bounds.phase.finalstate.upper   = final;
bounds.phase.control.lower      = uMin; 
bounds.phase.control.upper      = uMax; 
bounds.phase.integral.lower     = 0;
bounds.phase.integral.upper     = 10000;



%-------------------------------------------------------------------%
%--------------------------- Initial Guess -------------------------%
%-------------------------------------------------------------------%
rng(0);

q1Guess = [initial(1);final(1)]; 
q2Guess = [initial(2);final(2)]; 
q3Guess = [initial(3);final(3)];
dq1Guess = [initial(4);final(4)];
dq2Guess = [initial(5);final(5)];
dq3Guess = [initial(6);final(6)];
%{\
u1Guess = [0;0];
u2Guess =[0;0];
%}
tGuess = [0;tf]; 
state_guess = [q1Guess, q2Guess, q3Guess,dq1Guess,dq2Guess,dq3Guess];
control_guess =[u1Guess u2Guess];
guess.phase.time  = tGuess;
guess.phase.state = state_guess;
guess.phase.control = control_guess;
guess.phase.integral = 0;

% 
% load solution.mat
% guess  = solution

%-------------------------------------------------------------------%
%--------------------------- Problem Setup -------------------------%
%-------------------------------------------------------------------%
setup.name                        = 'acrobat-Problem';
setup.functions.continuous        = @acrobatContinuous;
setup.functions.endpoint          = @acrobatEndpoint;
setup.bounds                      = bounds;
setup.auxdata                     = auxdata;
setup.functions.report            = @report;
setup.guess                       = guess;
setup.nlp.solver                  = 'ipopt';
setup.derivatives.supplier        = 'sparseCD';
setup.derivatives.derivativelevel = 'first';
setup.scales.method               = 'none';
setup.derivatives.dependencies    = 'full';
setup.mesh.method                 = 'hp-PattersonRao';
setup.mesh.tolerance              = .1;
setup.method                      = 'RPM-Integration';

%-------------------------------------------------------------------%
%------------------- Solve Problem Using GPOPS2 --------------------%
%-------------------------------------------------------------------%
output = gpops2(setup);
output.result.nlptime
solution = output.result.solution;

%-------------------------------------------------------------------%
%--------------------------- Plot Solution -------------------------%
%-------------------------------------------------------------------%

plotacrobat;
