% Building Fls
clear all, close all  clc

% Generate new Fuzzy Interface System
sys = mamfis('System');

% Adding Inputs
sys = addInput(sys, 'input', 'Angle', [-20, 20]);

% Adding Membership function for the first Inputs
sys = addMF(sys, 'input', 1, 'NN', 'trampf', [-20, -20, -15, -10]);
sys = addMF(sys, 'input', 1, 'NM', 'trampf', [-15, -10, -10, -5]);
sys = addMF(sys, 'input', 1, 'ZZ', 'trampf', [-10, 0, 0, 10]);
sys = addMF(sys, 'input', 1, 'PM', 'trampf', [5, 10, 10, 15]);
sys = addMF(sys, 'input', 1, 'PP', 'trampf', [10, 15, 20, 20]);

% Adding Inputs
sys = addInput(sys, 'input', 'Velocity', [-50, 50]);

% Adding Membership function for the second Inputs
sys = addMF(sys, 'input', 2, 'NN', 'trampf', [-50, -50, -30, -15]);
sys = addMF(sys, 'input', 2, 'NM', 'trampf', [-30, -20, -20, -10]);
sys = addMF(sys, 'input', 2, 'ZZ', 'trampf', [-15, 0, 0, 15]);
sys = addMF(sys, 'input', 2, 'PM', 'trampf', [10, 20, 20, 30]);
sys = addMF(sys, 'input', 2, 'PP', 'trampf', [15, 30, 50, 50]);

% Adding Outputs
sys = addInput(sys, 'input', 'Power', [-255, 255]);

% Adding Membership function for the first Outputs
sys = addMF(sys, 'output', 1, 'NN', 'trampf', [-255, -255, -255, -50]);
sys = addMF(sys, 'output', 1, 'NM', 'trampf', [-175, -100, -100, -20]);
sys = addMF(sys, 'output', 1, 'ZZ', 'trampf', [-100, -10, 10, 100]);
sys = addMF(sys, 'output', 1, 'PM', 'trampf', [20, 100, 100, 175]);
sys = addMF(sys, 'output', 1, 'PP', 'trampf', [50, 255, 255, 255]);

% Adding the rules for the system
rule1_1 = [1 1 5 1 1];
rule1_2 = [2 1 5 1 1];
rule1_3 = [3 1 5 1 1];
rule1_4 = [2 1 5 1 1];
rule1_5 = [2 2 5 1 1];
rule1_6 = [3 1 5 1 1];

rule2_1 = [1 4 4 1 1];
rule2_2 = [2 3 4 1 1];
rule2_3 = [3 2 4 1 1];
rule2_4 = [4 1 4 1 1];
rule2_5 = [5 2 2 1 1];
rule2_6 = [4 3 2 1 1];
rule2_7 = [3 4 2 1 1];
rule2_8 = [2 5 2 1 1];

rule3_1 = [1 5 3 1 1];
rule3_2 = [2 4 3 1 1];
rule3_3 = [3 3 3 1 1];
rule3_4 = [4 2 3 1 1];
rule3_5 = [5 1 3 1 1];

rule4_1 = [5 3 1 1 1];
rule4_2 = [5 4 1 1 1];
rule4_3 = [5 5 1 1 1];
rule4_4 = [4 4 1 1 1];
rule4_5 = [4 5 1 1 1];
rule4_6 = [3 5 1 1 1];


ruleList = [rule1_1, rule1_2, rule1_3, rule1_4, rule1_5, rule1_6,... 
    rule2_1, rule2_2, rule2_3, rule2_4, rule2_5, rule2_6, rule2_7, rule2_8, ... 
    rule3_1, rule3_2, rule3_3, rule3_4, rule3_5, ...
    rule4_1, rule4_2, rule4_3, rule4_4, rule4_5, rule4_6...
    ];
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

