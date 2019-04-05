clear all
clc
close all

%%
syms u
knots = [0 0 0 1 1];

N_00 = 0;
N_10 = 0;
N_20 = u + (1-u);