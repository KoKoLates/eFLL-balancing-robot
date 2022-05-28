% Building Fls
clear all, close all  clc

% Generate new Fuzzy Interface System
sys = mamfis('System');

% Adding Inputs
sys = addInput(sys, 'input', 'Angle', [-50, 50]);

% Adding Membership function for the first Inputs
sys = addMF(sys, 'input', 1, 'N', 'trampf', [-50, -50, -50, 50]);
sys = addMF(sys, 'input', 1, 'P', 'trampf', [-50, 50, 50, 50]);

% Adding Inputs
sys = addInput(sys, 'input', 'Velocity', [-50, 50]);

% Adding Membership function for the second Inputs
sys = addMF(sys, 'input', 2, 'N', 'trampf', [-50, -50, -50, 50]);
sys = addMF(sys, 'input', 2, 'P', 'trampf', [-50, 50, 50, 50]);

% Adding Outputs
sys = addInput(sys, 'input', 'Power', [-255, 255]);

% Adding Membership function for the first Outputs
sys = addMF(sys, 'output', 1, 'N', 'trampf', [-255, -255, -255, 0]);
sys = addMF(sys, 'output', 1, 'Z', 'trampf', [-255, 0, 0, 255]);
sys = addMF(sys, 'output', 1, 'P', 'trampf', [0, 255, 255, 255]);

% Adding the rules for the system
rule1 = [1 1 3 1 1];
rule2 = [1 2 2 1 1];
rule3 = [2 1 2 1 1];
rule4 = [2 2 1 1 1];
ruleList = [rule1, rule2, rule3 ,rule4];
system = addRule(sys, ruleList);
showrule(system);

% Plot the membership function and surface rule
plotmf(sys, 'input', 1)
figure
plotmf(sys, 'input', 2)
figure
plotmf(sys, 'output', 1)
figure
gensurf(system)

