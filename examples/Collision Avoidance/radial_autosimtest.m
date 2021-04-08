clear all; close all; clc;
rng default
global dataset

%% Description
%This scripts runs verification against Safety, Mission and Colreg
%requirements for Case 1 (Obstacle on direct radial collision course) in "Automatic Simulation-based Testing of
%Autonomous Ships using Gaussian Processes and Temporal Logic".

%% AutoSimTest Hyper parameters
hypers.maxNumberOfSimulations = 3000;
hypers.n_seed = 1;
hypers.n_conf = 3.0;
hypers.kappa = 2;
hypers.kernelfun = 'ardmatern52';
hypers.sigma_n = 0.02;
hypers.sigma = 0.5;
hypers.length_scales = [10];

%% Setup STL requirements
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

%STL formulas
phi_safety = '[](!collision)';
phi_mission = '[]stay_on_track';
phi_colreg = ['[]( headon -> (time_to_turn -> (port_low /\ port_high))', ...
    '/\ (giveway -> (time_to_turn -> !cpa_small))', ...
    '/\ (overtaking -> (time_to_turn -> !cpa_small)) )'];

%STL Predicates
%Collision
i=1;
pred_safety(i).str = 'collision';
pred_safety(i).A = [1];
pred_safety(i).b = d_min;
pred_safety(i).proj = 1;
pred_safety(i).Normalized = 1;
pred_safety(i).NormBounds = nf_dist;

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


%% Scenario Parameters
theta = SimParameter('\theta [deg]', -160:1:160);
parameters = [theta];

%% Dataset (if using prerun simulations)
load('datasets/radial.mat');

%% Run Automatic Testing Algorithm
[~, rho_mean_safety, rho_var_safety, rho_obs_safety, P_safety, P_obs_safety, fig_safety] = auto_sim_test(@draw_from_dataset_safety, parameters, hypers, phi_safety, pred_safety, 'figures/radial_safety.avi');
[~, rho_mean_mission, rho_var_mission, rho_obs_mission, P_mission, P_obs_mission, fig_mission] = auto_sim_test(@draw_from_dataset_mission, parameters, hypers, phi_mission, pred_mission, 'figures/radial_mission.avi');
[~, rho_mean_colreg, rho_var_colreg, rho_obs_colreg, P_colreg, P_obs_colreg, fig_colreg] = auto_sim_test(@draw_from_dataset_colreg, parameters, hypers, phi_colreg, pred_colreg, 'figures/radial_colreg.avi');

%% Plot against ground truth
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
    rho_mission(i) = dp_taliro(phi_mission, pred_mission,w_mission',t');
    rho_colreg(i) = dp_taliro(phi_colreg, pred_colreg,w_colreg',t');
end

figure(fig_safety);
h = plot(dataset_parameters(1).Values,rho_safety, 'LineWidth', 1);
legend('GP Mean', '99% confidence interval',  'Observed points', 'Ground truth');
print('figures/radial_safety', '-depsc')
print('figures/radial_safety', '-dpng')

figure(fig_mission);
h = plot(dataset_parameters(1).Values,rho_mission, 'LineWidth', 1);
legend('GP Mean', '99% confidence interval',  'Observed points', 'Ground truth');
print('figures/radial_mission', '-depsc')
print('figures/radial_mission', '-dpng')

figure(fig_colreg);
h = plot(dataset_parameters(1).Values,rho_colreg, 'LineWidth', 1);
legend('GP Mean', '99% confidence interval',  'Observed points', 'Ground truth');
print('figures/radial_colreg', '-depsc')
print('figures/radial_colreg', '-dpng')

%% Statistical Validation of Safety Verification
[rows,cols] = size(rho_mean_safety);
for i = 1:rows
    for j = 1:cols
        norm_offset(i,j) = (rho_safety(j,i)-rho_mean_safety(i,j))/sqrt(rho_var_safety(i,j));
    end
end
figure;
subplot(1,2,1)
histogram(norm_offset, 140,'Normalization', 'pdf');
pd = fitdist(norm_offset(:),'Normal')
hold on;
x = -5:0.01:5;
plot(x,normpdf(x,pd.mu,pd.sigma), 'LineWidth', 1);
plot(x,normpdf(x,0,1), 'LineWidth', 1);
legend('Observed histogram', 'Fitted Normal PDF', 'Theoretical PDF');
xlim([-5 5])
subplot(1,2,2);
probplot(norm_offset);
print('figures/validation_radial_safety', '-dpng')
print('figures/validation_radial_safety', '-depsc')

%% Helper functions
function [w,t] = draw_from_dataset_safety(parameters)
    global dataset
    theta = -160:1:160;
    [~,j] = min(abs(theta-parameters(1)));
    i = 1;
    t = dataset(i,j).t';
    w = dataset(i,j).obs_dist';
end

function [w,t] = draw_from_dataset_mission(parameters)
    global dataset
    theta = -160:1:160;
    [~,j] = min(abs(theta-parameters(1)));
    i = 1;
    t = dataset(i,j).t';
    w = abs(dataset(i,j).eta(2,:))';
end

function [w,t] = draw_from_dataset_colreg(parameters)
    global dataset
    theta = -160:1:160;
    [~,j] = min(abs(theta-parameters(1)));
    i = 1;
    [headon overtaking overtaken standon giveway] = colreg_selector( ...
        dataset(i,j).cpa, dataset(i,j).tcpa, ...
        dataset(i,j).eta, dataset(i,j).obs_eta{1});
    t = dataset(i,j).t';
    w = [headon;giveway;overtaking;dataset(i,j).tcpa;dataset(i,j).cpa;dataset(i,j).beta]';
end