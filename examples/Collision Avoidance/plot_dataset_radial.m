%© Norwegian University of Science and Technology (NTNU),
%Department of Marine Technology.
%The software is developed under the ORCAS Project.
%Author: Tobias Rye Torben.

clear all; close all; clc;

load datasets/radial.mat;
theta = -160:1:160;

%% Setup STL requirements
phi_safety = '[](!collision)';
phi_ho = '[](headon -> (time_to_turn -> (port_low /\ port_high)))';
phi_mission = '[]stay_on_track';
phi_colreg = ['[]( headon -> (time_to_turn -> (port_low /\ port_high))', ...
              '/\ (giveway -> (time_to_turn -> !cpa_small))', ...
              '/\ (overtaking -> (time_to_turn -> !cpa_small)) )'];

%Requirement parameters
d_min = 50.0;
cpa_min = 50.0;
tcpa_turn = 15.0;
reciprocal_int = 6.0;
e_max = 400;

%Normalizing factors
nf_tcpa = 10.0;
nf_beta_r = 10.0;
nf_dist = 100.0;
nf_e = 400.0;
nf_cpa = 100;

%Predicates

%Collision
i=1;
pred_safety(i).str = 'collision';
pred_safety(i).A = [1];
pred_safety(i).b = d_min;
pred_safety(i).proj = 1;
pred_safety(i).Normalized = 1;
pred_safety(i).NormBounds = nf_dist;

%COLREG
i=1;
pred_ho(i).str = 'headon';
pred_ho(i).A = [-1];
pred_ho(i).b = 0;
pred_ho(i).proj = 1;

i=i+1;
pred_ho(i).str = 'time_to_turn';
pred_ho(i).A = [1];
pred_ho(i).b = tcpa_turn;
pred_ho(i).proj = 2;
pred_ho(i).Normalized = 1;
pred_ho(i).NormBounds = nf_tcpa;

i=i+1;
pred_ho(i).str = 'port_low';
pred_ho(i).A = [1];
pred_ho(i).b = -10;
pred_ho(i).proj = 3;
pred_ho(i).Normalized = 1;
pred_ho(i).NormBounds = nf_beta_r;

i=i+1;
pred_ho(i).str = 'port_high';
pred_ho(i).A = [-1];
pred_ho(i).b = 170;
pred_ho(i).proj = 3;
pred_ho(i).Normalized = 1;
pred_ho(i).NormBounds = nf_beta_r;

%Mission
i=1;
pred_mission(i).str = 'stay_on_track';
pred_mission(i).A = [1];
pred_mission(i).b = e_max;
pred_mission(i).proj = 1;
pred_mission(i).Normalized = 1;
pred_mission(i).NormBounds = nf_e;

%Colreg
i=1;
pred_colreg(i).str = 'headon';
pred_colreg(i).A = [-1];
pred_colreg(i).b = 0;
pred_colreg(i).proj = 1;

i=i+1;
pred_colreg(i).str = 'giveway';
pred_colreg(i).A = [-1];
pred_colreg(i).b = 0;
pred_colreg(i).proj = 2;

i=i+1;
pred_colreg(i).str = 'overtaking';
pred_colreg(i).A = [-1];
pred_colreg(i).b = 0;
pred_colreg(i).proj = 3;

i=i+1;
pred_colreg(i).str = 'port_low';
pred_colreg(i).A = [1];
pred_colreg(i).b = -10;
pred_colreg(i).proj = 6;
pred_colreg(i).Normalized = 1;
pred_colreg(i).NormBounds = nf_beta_r;

i=i+1;
pred_colreg(i).str = 'port_high';
pred_colreg(i).A = [-1];
pred_colreg(i).b = 170;
pred_colreg(i).proj = 6;
pred_colreg(i).Normalized = 1;
pred_colreg(i).NormBounds = nf_beta_r;

i=i+1;
pred_colreg(i).str = 'time_to_turn';
pred_colreg(i).A = [1];
pred_colreg(i).b = tcpa_turn;
pred_colreg(i).proj = 4;
pred_colreg(i).Normalized = 1;
pred_colreg(i).NormBounds = nf_tcpa;

i=i+1;
pred_colreg(i).str = 'cpa_small';
pred_colreg(i).A = [1];
pred_colreg(i).b = cpa_min;
pred_colreg(i).proj = 5;
pred_colreg(i).Normalized = 1;
pred_colreg(i).NormBounds = nf_cpa;



for i = 1:numel(dataset)
    [headon overtaking overtaken standon giveway] = colreg_selector( ...
                                                    dataset(i).cpa, dataset(i).tcpa, ...
                                                    dataset(i).eta, dataset(i).obs_eta{1});

    w_ho = [headon; dataset(i).tcpa; dataset(i).beta];
    w_safety = dataset(i).obs_dist;
    w_mission = abs(dataset(i).eta(2,:));
    w_colreg = [headon;giveway;overtaking;dataset(i).tcpa;dataset(i).cpa;dataset(i).beta];
    t = dataset(i).t;
    rho_safety(i) = dp_taliro(phi_safety, pred_safety,w_safety',t');
    rho_ho(i) = dp_taliro(phi_ho, pred_ho,w_ho',t');
    rho_mission(i) = dp_taliro(phi_mission, pred_mission,w_mission',t');
    rho_colreg(i) = dp_taliro(phi_colreg, pred_colreg,w_colreg',t');
end

subplot(3,1,1)
plot(rho_safety);hold on;
plot(rho_ho);
plot(rho_mission);
plot(rho_colreg);
figure;
%%
[~,i] = min(abs(theta-(130)));
[headon overtaking overtaken standon giveway] = colreg_selector( ...
                                                    dataset(i).cpa, dataset(i).tcpa, ...
                                                    dataset(i).eta, dataset(i).obs_eta{1});
w_safety = dataset(i).obs_dist;
w_ho = [headon; dataset(i).tcpa; dataset(i).beta];
w_mission = abs(dataset(i).eta(2,:));
w_colreg = [headon;giveway;overtaking;dataset(i).tcpa;dataset(i).cpa;dataset(i).beta];
t = dataset(i).t;
online_safety = online_robustness(phi_safety(3:end), pred_safety, w_safety, t);
online_mission = online_robustness(phi_mission(3:end), pred_mission, w_mission, t);
online_colreg = online_robustness(phi_colreg(3:end), pred_colreg, w_colreg, t);
visualizeSim(dataset(i), online_safety, online_colreg, online_mission, 10, [-400 400], [-500 300], 'figures/radial_mission_falsification');