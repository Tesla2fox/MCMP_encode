%**********************************************************************************************************************
% Author: Pei-Qiu Huang, Yong Wang, Kezhi Wang, and Kun Yang
% Last Edited: 09/02/2019
% Eamil: pqhuang@csu.edu.cn; ywang@csu.edu.cn; kezhi.wang@northumbria.ac.uk; and kunyang@essex.ac.uk
% Reference: Differential Evolution with a Variable Population Size for Deployment Optimization in a UAV-Assisted
% IoT Data Collection System, IEEE Transactions on Emerging Topics in Computational Intelligence, In press.
%**********************************************************************************************************************

clc;
clear;

for run = 1:1
    %% Parameter setting
    Para.NIoT    = 100;
    Para.NSP     = Para.NIoT;
    Para.Xmin    = 0;
    Para.Xmax    = 1000;
    Para.Ymin    = 0;
    
    Para.Ymax    = 1000;
    Para.Hmin    = 200;
    Para.Hmax    = 200;
    
    Para.p       = 0.1;
    Para.K       = 5;
    Para.rho     = 1e-6;
    Para.sigma   = 1e-28;
    Para.B       = 1e6;
    Para.D       = load(['Data\D_',num2str(Para.NIoT),'.dat']);
    
    Para.ph      = 1000;
    Para.evaluations = 0;
    Para.maxEvaluations = 100000;
    
    Para.F = 0.6;
    Para.CR = 0.5;
    
    %% Initialization of UAV's Deployment
    IoTPosition = load(['Data\IoTPosition_',num2str(Para.NIoT),'.dat']);
    C = 1;
    while C~=0
        UAVPosition = [Para.Xmin+(Para.Xmax-Para.Xmin)*rand(Para.NSP,1),Para.Ymin+(Para.Ymax-Para.Ymin)*rand(Para.NSP,1),Para.Hmin+(Para.Hmax-Para.Hmin)*rand(Para.NSP,1)];
        [Fbest,C] = Fitness(UAVPosition,IoTPosition,Para);
        Para.evaluations = Para.evaluations + 1;
    end
    
    while(Para.evaluations < Para.maxEvaluations)
        
        %% Initialization of Stop Points
        candidatePos = DE(UAVPosition, Para);
        
        %% Update of UAV's Deployment
        for i = 1:floor(size(candidatePos,1))
            % Insert
            InsUAVPosition = [UAVPosition;candidatePos(i,:)];
            [FIns,~] = Fitness(InsUAVPosition,IoTPosition,Para);
            
            % Replace
            r = randi(size(UAVPosition,1));
            RepUAVPosition = UAVPosition;
            RepUAVPosition(r,:) = candidatePos(i,:);
            [FRep,~] = Fitness(RepUAVPosition,IoTPosition,Para);
            
            % Remove
            r = randi(size(UAVPosition,1));
            RemUAVPosition = UAVPosition;
            RemUAVPosition(r,:) = [];
            [FRem,~] = Fitness(RemUAVPosition,IoTPosition,Para);
            
            [FitnessImprove,index] = min([FIns-Fbest,FRep-Fbest,FRem-Fbest]);
            
            if FitnessImprove<0 && index == 1
                UAVPosition = InsUAVPosition;
                Fbest = FIns;
            elseif FitnessImprove<0 && index == 2
                UAVPosition = RepUAVPosition;
                Fbest = FRep;
            elseif (FitnessImprove<0 && index == 3) || (FRem-Fbest==0)
                UAVPosition = RemUAVPosition;
                Fbest = FRem;
            end
            Para.evaluations = Para.evaluations +3;
        end
    end
    Fbest(run)
end
