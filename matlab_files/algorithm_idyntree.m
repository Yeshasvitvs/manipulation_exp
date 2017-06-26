close all; 
clear all;
clc;

filename = '/home/yeshi/projects/manipulation_exp/manipulation/data/rmotion2.txt';
data = importdata(filename);

mdlLoader = iDynTree.ModelLoader();
mdlLoader.loadModelFromFile('/home/yeshi/gazebo-yeshi/1r_2link/model.urdf');

mdl = mdlLoader.model();
sensorsList = mdlLoader.sensors();
mdl.toString()

link1 = mdl.getLink(mdl.getLinkIndex('1r_2link__first_link'));
link2 = mdl.getLink(mdl.getLinkIndex('1r_2link__second_link'));

%%Getting Link Mass, CoM and Body Inertia
I1 = link1.getInertia();
M1 = I1.asMatrix();
M1.toString();
M11 = I1.asVector();
M11.toString();
m1 = I1.getMass();
com1 = I1.getCenterOfMass();
% com1.getVal(0);

I2 = link2.getInertia();
M2 = I2.asMatrix();
M2.toString();
M22 = I2.asVector();
M22.toString();
m2 = I1.getMass();
com2 = I2.getCenterOfMass();

%%Getting number of FT sensors
nrOfFTSensors = mdlLoader.sensors().getNrOfSensors(iDynTree.SIX_AXIS_FORCE_TORQUE);

dt = 0.01; %%This is fixed step for derivation
%%TODO Setting the base frame

%%Loop when data is processed when object is moved in gazebo
for i=1:1:1000
    H_A_1 = iDynTree.getWorldLinkTransform(1);
end
