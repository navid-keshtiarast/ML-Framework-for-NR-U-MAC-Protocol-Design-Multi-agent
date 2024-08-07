%% Set input parameters

gNbNums = 18; %number of GNBs
ueNums = 18; %number of UEs
roomHeight = 10; %height of the factory in meter
carrierFrequency = 5.945; %in GHz 
bandwidth = 20e6;
utHeight = 1.5; %height of user terminal in meter
factorySize = 'big'; %factory size : big;small
clutterType = 'high'; %clutter type : low;high
factoryScenario = 'InF-SL'; %factory scenario : InF-SL, InF-DL, InF-SH, InF-DH
onlyLOS = true; %false = multipath propagation are also taken into account

seedNum = 1;
algorithmType = 'twister';

%% Set factory layout

[clutterDensity,clutterHeight,clutterSize] = setClutterConfiguration(clutterType);
[factorySizeVector,gNbDistance] = setFactoryLayout(factorySize,roomHeight);

%% Set gNBs layout

[gNbLayoutMatrix] = setGnbLayout(factoryScenario,gNbDistance,gNbNums);

plot(gNbLayoutMatrix(:,1),gNbLayoutMatrix(:,2), 'b*');
grid on;

%% Set UEs layout
%TODO : uniform height between user equipments??

% assumes that ueHeight = 1.5
[ueLayoutMatrix] = setUeLayout4(factorySizeVector,gNbLayoutMatrix,seedNum);

plot(ueLayoutMatrix(:,1),ueLayoutMatrix(:,2), 'b*');
grid on;

%% UE distance test

% ueDistance = zeros(ueNums,ueNums);
% 
% for i = 1: ueNums
%     for j = 1:ueNums
%         ueDistance(i,j) = calculate3dDistance(ueLayoutMatrix(i,:),ueLayoutMatrix(j,:));
%     end
% end

plot(ueLayoutMatrix(:,1),ueLayoutMatrix(:,2), 'b*','Color','red');
grid on;
hold on;

plot(gNbLayoutMatrix(:,1),gNbLayoutMatrix(:,2), 'b*');
hold on;

for i = 1 : 18
    plot([gNbLayoutMatrix(i,1) ueLayoutMatrix(i,1)], [gNbLayoutMatrix(i,2) ueLayoutMatrix(i,2)])
end

for i = 1 : 18
        for j = 1 : 18
            dist(i,j) = calculate3dDistance(gNbLayoutMatrix(i,:),ueLayoutMatrix(j,:));
        end
end


%% Calculate path loss matrix in dB

[pathlossMatrixGnb,gNbNums,~] = calculatePathloss(gNbLayoutMatrix, gNbLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);
[pathlossMatrixGnbUe,gNbNums,ueNums] = calculatePathloss(gNbLayoutMatrix, ueLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);
[pathlossMatrixUe,ueNums,~] = calculatePathloss(ueLayoutMatrix, ueLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);

%% Calculate LOS probability

[probabilityLOSMatrix,gNbNums,ueNums] = calculateLOSProbability(gNbLayoutMatrix,ueLayoutMatrix,utHeight,clutterHeight,clutterDensity,clutterSize, factoryScenario);

%% Create output file 

outputPath = '/home/inets/Desktop/A/matlab/pathlossTrace';
%Output file for UE distances
writematrix(ueLayoutMatrix,append(outputPath,'/ueCoordinates.txt'));
%Output file for AP-AP pathloss
writematrix(pathlossMatrixGnb,append(outputPath,'/pathlossMatrixGnb.txt'));
%Output file for AP-UE pathloss
writematrix(pathlossMatrixGnbUe,append(outputPath,'/pathlossMatrixGnbUe.txt'));
%Output file for UE-UE pathloss
writematrix(pathlossMatrixUe,append(outputPath,'/pathlossMatrixUe.txt'));

%% Create output files group

    [clutterDensity,clutterHeight,clutterSize] = setClutterConfiguration(clutterType);
    [factorySizeVector,gNbDistance] = setFactoryLayout(factorySize,roomHeight);
    [gNbLayoutMatrix] = setGnbLayout(factoryScenario,gNbDistance, gNbNums);
    [pathlossMatrixGnb,gNbNums,~] = calculatePathloss(gNbLayoutMatrix, gNbLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);
    
    outputPath = '/home/inets/Desktop/A/workspace-nr/ns-3-dev/pathlossTrace/onlyLOS';
    writematrix(gNbLayoutMatrix,append(outputPath,'/gNbLayoutMatrix.txt'),'Delimiter','space');
    writematrix(pathlossMatrixGnb,append(outputPath,'/pathlossMatrixGnbGnb.txt'),'Delimiter','space');

    for i = 1:100
        seedNum = i;
        apIndex = randperm(18);
        writematrix(apIndex,append(outputPath,'/apIndex_',num2str(i),'.txt'),'Delimiter','space')
        [ueLayoutMatrix] = setUeLayout4(factorySizeVector,gNbLayoutMatrix,seedNum);
        [pathlossMatrixGnbUe,gNbNums,ueNums] = calculatePathloss(gNbLayoutMatrix, ueLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);
        [pathlossMatrixUe,ueNums,~] = calculatePathloss(ueLayoutMatrix, ueLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);
        
        writematrix(ueLayoutMatrix,append(outputPath,'/ueCoordinates_',num2str(i),'.txt'),'Delimiter','space');
        writematrix(pathlossMatrixGnbUe,append(outputPath,'/pathlossMatrixGnbUe_',num2str(i),'.txt'),'Delimiter','space');
        writematrix(pathlossMatrixUe,append(outputPath,'/pathlossMatrixUeUe_',num2str(i),'.txt'),'Delimiter','space');
    end

    [clutterDensity,clutterHeight,clutterSize] = setClutterConfiguration(clutterType);
    [factorySizeVector,gNbDistance] = setFactoryLayout(factorySize,roomHeight);
    [gNbLayoutMatrix] = setGnbLayout(factoryScenario,gNbDistance, gNbNums);
    [pathlossMatrixGnb,gNbNums,~] = calculatePathloss(gNbLayoutMatrix, gNbLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);
    
    outputPath = '/home/inets/Desktop/A/workspace-nr/ns-3-dev/pathlossTrace/notOnlyLOS';
    writematrix(gNbLayoutMatrix,append(outputPath,'/gNbLayoutMatrix.txt'),'Delimiter','space');
    writematrix(pathlossMatrixGnb,append(outputPath,'/pathlossMatrixGnbGnb.txt'),'Delimiter','space');

    for i = 1:100
        seedNum = i;
        apIndex = randperm(18);
        writematrix(apIndex,append(outputPath,'/apIndex_',num2str(i),'.txt'),'Delimiter','space')
        [ueLayoutMatrix] = setUeLayout4(factorySizeVector,gNbLayoutMatrix,seedNum);
        [pathlossMatrixGnbUe,gNbNums,ueNums] = calculatePathloss(gNbLayoutMatrix, ueLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);
        [pathlossMatrixUe,ueNums,~] = calculatePathloss(ueLayoutMatrix, ueLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);
        
        writematrix(ueLayoutMatrix,append(outputPath,'/ueCoordinates_',num2str(i),'.txt'),'Delimiter','space');
        writematrix(pathlossMatrixGnbUe,append(outputPath,'/pathlossMatrixGnbUe_',num2str(i),'.txt'),'Delimiter','space');
        writematrix(pathlossMatrixUe,append(outputPath,'/pathlossMatrixUeUe_',num2str(i),'.txt'),'Delimiter','space');
    end
