clc
clear
% 6-DOF nonlinear model for Twin-Engine UAV 
% Twin Engine 
% units: speed (ft/sec), weight (lb)

% ===============================================
% Flight condition:
ro=0.00215213; %(slug/ft^3)
alphao=3.67/57.3; % (rad)  
ao=alphao;
uo=74.8;%90; % (ft/sec)(CL1=0.537, CLo=0.522)
wo=uo*tan(ao);
vo=0;
Vto=sqrt(uo^2+vo^2+wo^2); 
U0=Vto;
gammao=0/57.3; %(rad)
thetao=alphao+gammao;
fi0=0;
dEo=-4.4766; %-2.704/57.3; % initial elevator deflection (rad)
dTo=8.17/57.3;
%dTo=0;

% 1. Weight and inertia
W=70; %(lb) 
g=32.2; % (ft/sec^2)
m=W/g; %(slug)
Ix=9.358; %10.2; % (slug-ft^2) 
Iy=5.350; %8.6;  % (slug-ft^2)
Iz=12.39; %6.5;  % (slug-ft^2)
Ixz=1.11; %3;   % (slug-ft^2)
Ixy=0;   %  symmetric
Iyz=0;

% From page 87 of Stevens:
Ixd=Ix*[cos(alphao)]^2+Iz*[sin(alphao)]^2-Ixz*sin(2*alphao);
Iyd=Iy;
Izd=Ix*[sin(alphao)]^2+Iz*[cos(alphao)]^2+Ixz*sin(2*alphao);
Ixzd=0.5*(Ix-Iz)*[sin(2*alphao)]+Ixz*cos(2*alphao);
landa=Ix*Iz-Ixz^2; % page 31 of Stevens
mu=(Izd*Ixd)/landa;
sigma=(Izd*Ixzd)/landa;
nu=Izd/landa;

% Page 80 of Stevens (to be used in 6DOF nonlinear ODEs of motion)
G=Ix*Iz-Ixz^2;
c1=[(Iy-Iz)*Iz-Ixz^2]/G;
c2=[(Ix-Iy+Iz)*Ixz]/G;
c3=Iz/G;
c4=Ixz/G;
c5=(Iz-Ix)/Iy;
c6=Ixz/Iy;
c7=1/Iy;
c8=[Ix*(Ix-Iy)+Ixz^2]/G;
c9=Ix/G;

% 2. Geometry: ------------------------
S=22.146;
s = S; % (ft^2) wing reference area
St=4.21875; % ft^2
Svt=0.87115+0.1995;%1.28; % (ft^2)
b=11.02; %14.0;   % (ft) wing span
C=2; %1.29; % (ft) Wing chord
AR=b/C;   % wing aspect ratio
pi=3.14;
cbar=C;

% 3. nondim stability and control derivative (unit: 1/rad)
% From AAA---------------:
CLo=0.522;
Cmo=-0.17;
Cmowf=Cmo;
CDo=0.032;

CLa= 4.817; %4.580152; %5.237;
Cma= -.963; %-0.677; %-1.0195;
CDa=0.149;

CLad= 2.647; %1.55; 
Cmad= -9.9; %-6.5;
CDad=0;

CLu= .005; %0.0021;
Cmu = .00028; %0;
CDu = 0;

CLq= 6.034; %6.902; %8.44; 
Cmq= -22.23; %-7.225; %-30.84;
Cmq= -7.225; %-30.84;

CDq=0.0;

CLde= 0.86; %(0.007640*57.3); 
% Cmde= -3.6; %-0.967167 
Cmde= -1.467167; 
CDde=0.0363;

Clb=-0.0717; %-0.079;  
Cyb=-0.26557; %-0.314; 
Cnb=0.062324; %0.048;

Clp=-0.4164; %-0.57;      
Cyp=-0.121995; %-0.089;    
Cnp=-0.032818; %-0.066; 

Clr=0.1393; %0.1618; 
Cyr=0.12987; %0.1077; 
Cnr=-0.094971; %-0.049;

Clda=0.284552; %0.227; 
Cyda=0.0;        
Cnda=0; %-0.03;
Cybd=0; %???

Cldr=0.0095; %     
Cydr=0.145141; %0.1771; %
Cndr=-0.066583; %-0.0716; %      

umax=uo+50; % (ft/sec)
CL1=(2*W)/(ro*S*uo^2);
K=1/(pi*0.9*AR); % % induced drag factor 
CD1=CDo + (K*CL1^2);
q1=0.5*ro*Vto^2;
q = q1;
qbar = q1;
% Coupling derivatives:
Cxb=-(2*Svt)/(pi*S);
betao=0.0;
Xb=Cxb*betao*.5*ro*uo*uo*S;
Zfi=0.1*W;
Xdt=547.36; % engine model (CT)
P=10.3*550; % power (ft*lb/sec) 
eta_p=0.75; % propeller efficiency
Fto=0.55*P*eta_p/uo;  % initial thrust (lb)

phi=0;
beta=0;
CT=47.36; %(T=CT*dT)
qS=0.5*ro*uo^2*S;

k1=1;  k5=-2; % dT 100000,-2
k2=1;k6=-3; %dE  10, -3
k3=1; k7=1;   % dA 1000, 100
k4=0.5;  k8=-1;    % dR 0.5, -100 

kmatrix1=[1  0   0   0
          0   1  0   0
          0   0   1  0
          0   0   0   .5];

kmatrix2=[-2  0   0   0
          0   -3  0   0
          0   0   1  0
          0   0   0   -1];

Cbar=C;
C1=Cbar/(2*uo);
C2=b/(2*uo);

%Data

Data = xlsread('Fri May 24 14-54-11 2013e.xlsx');
Time = Data(829:4557,7) ;


P = Data(829:4557,32);
Q = Data(829:4557,33);
R = Data(829:4557,34);
Phi = Data(829:4557,38);
Theta = Data(829:4557,39);
Psi = Data(829:4557,40);

len=length(Time);


Aileron_Input = Data(829:4557,51);
Elevator_Input = Data(829:4557,52);
Throttle_Input = Data(829:4557,53);
Throttle_Input=zeros(len,1);
Rudder_Input = Data(829:4557,54);

freq= 10 ;%Hz
sampling_time=1/freq;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Aileron Input
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
row_s = 1567; %Start
row_e = 1624;
% row_s = 1733; %Start
% row_e = 1805;
% row_s = 2563; %Start
% row_e = 2628;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Elevator Input
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% row_s = 908; %Start
% row_e = 980;
% row_s = 1068; %Start
% row_e = 1148;
% row_s = 2380; %Start
% row_e = 2437;


p_data=P(row_s:row_e);
t_data=Time(row_s:row_e);
phi_data=Phi(row_s:row_e);

sim('Sim_Non_linear_Model_Twin_Engine_UAV')

input_aileron = ScopeData4.signals(1).values;
input_elevator = ScopeData4.signals(3).values;
input_throttle = ScopeData4.signals(2).values;
input_rudder = ScopeData4.signals(4).values;

t2=ScopeData1.time;
p_sim=ScopeData1.signals(2).values;
r_sim=ScopeData1.signals(4).values;
q_sim=ScopeData1.signals(8).values;
phi_sim=ScopeData1.signals(3).values;
psi_sim=ScopeData1.signals(5).values;
theta_sim=ScopeData1.signals(9).values;

  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Lateral-Directional Response
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1),
subplot(3,1,1),plot(t2,input_aileron*57.3,'k-','linewidth',3),
    ylabel('\delta_A (deg)','fontsize',25),grid,set(gca,'fontsize',25),title('Airplane Lateral-Directional Response','fontsize',25)
subplot(3,1,2),plot(t2,p_sim*57.3,'k--','linewidth',3),...
    ylabel(' p (deg/sec)','fontsize',25),legend('Flight Data','Simulation'),grid,set(gca,'fontsize',25)
% subplot(3,1,3),plot(t1,R(row_s:row_e)*57.3,'k-',t2,r_sim*57.3,'k--','linewidth',3),...
%     ylabel(' r (deg/sec)','fontsize',25),xlabel('Time (sec)','fontsize',25),grid,set(gca,'fontsize',25)
% subplot(5,2,2),plot(t1,Phi(row_s:row_e)*57.3,'-',t2,phi_sim*57.3,'--','linewidth',3)
% subplot(5,2,4),plot(t1,Psi(row_s:row_e)*57.3,'-',t2,psi_sim*57.3,'--','linewidth',3),xlabel('Time (sec)')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Longitudinal Response
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% figure(2),
% subplot(2,1,1),plot(t1,input_elevator(:,2)*57.3,'k-','linewidth',3),xlabel('Time (sec)'),...
%     ylabel('\delta_E (deg)','fontsize',25),grid,set(gca,'fontsize',25),title('Airplane Longitudinal Response','fontsize',25)
% subplot(2,1,2),plot(t1,-Q(row_s:row_e)*57.3,'k-',t2,q_sim*57.3+10,'k--','linewidth',3),...
%     ylabel(' q (deg/sec)','fontsize',25),xlabel('Time (sec)','fontsize',25),...
%     legend('Flight Data','Simulation'),grid,set(gca,'fontsize',25)


