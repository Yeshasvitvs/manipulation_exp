clear all;
close all;
clc;

WBT_robotName = '1r_manipulator_scaled_down';
WBT_wbiList = 'ROBOT_DYNAMIC_MODEL_JOINTS';

%%Homogeneous tranform between base frame and world frame
%base_pose = [1,0,0,0;
%             0,1,0,0;
%             0,0,1,0;
%             0,0,0,1];
         
%jonit_config = [1.5; 1.57];

%%Loading Simulink Model
model='sample_1r1p.slx';
load_system(model);
sim(model);
