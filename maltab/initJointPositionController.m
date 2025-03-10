%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2016 CoDyCo
%  * @author: Daniele Pucci, Gabriele Nava
%  * Permission is granted to copy, distribute, and/or modify this program
%  * under the terms of the GNU General Public License, version 2 or any
%  * later version published by the Free Software Foundation.
%  *
%  * A copy of the license can be found at
%  * http://www.robotcub.org/icub/license/gpl.txt
%  *
%  * This program is distributed in the hope that it will be useful, but
%  * WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  * Public License for more details
%  */
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% NOTE: THIS SCRIPT IS RUN AUTOMATICALLY WHEN THE USER STARTS THE ASSOCIATED
% SIMULINK MODEL. NO NEED TO RUN THIS SCRIPT EVERY TIME.
clearvars;
clc;

% Add path to local source code
addpath('./src/')

%% GENERAL SIMULATION INFO
%
% If you are simulating the robot with Gazebo, remember that it is required
% to launch Gazebo as follows:
% 
%     gazebo -slibgazebo_yarp_clock.so
% 
% and properly set the environmental variable YARP_ROBOT_NAME in the .bashrc.

% Simulation time in seconds. For long simulations, put an high number 
% (not inf) for allowing automatic code generation
Config.SIMULATION_TIME = 600000;

% Controller period [s]
Config.tStep           = 0.01;

%% PRELIMINARY CONFIGURATION
%

% DATA PROCESSING
%
% Save the Matlab workspace after stopping the simulation
Config.SAVE_WORKSPACE         = true;

%datasetType = 'Classical';
datasetType = 'Dynamical';
%datasetType = 'Hyperbolic';

dataset = [datasetType, '/log.mat'];
load(['data/' dataset]);

scalingFactor = 1;

time = [stateTime(1):0.01:stateTime(end) * scalingFactor];
ss = spline(stateTime * scalingFactor, jointsConfiguraion, time);
desiredJointsPosition = timeseries(ss', time);

desiredBaseOrientation = timeseries(baseQuaternion', stateTime * scalingFactor);
desiredBasePosition = timeseries(basePosition', stateTime * scalingFactor);

dlmwrite(['../cpp/txtDatasets/', datasetType,'/jointDataset.txt'],desiredJointsPosition.Data,'delimiter',' ','newline','pc')


% Verify that the integration time has been respected during the simulation

% Run robot-specific configuration parameters
%run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/configRobot.m')); 

