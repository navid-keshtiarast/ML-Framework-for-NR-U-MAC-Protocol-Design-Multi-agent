%% Configurations

gNbNums = 6; %number of GNBs
ueNums = gNbNums; %number of UEs
roomHeight = 10; %height of the factory in meter
carrierFrequency = 5.945; %in GHz 
bandwidth = 20e6;
utHeight = 1.5; %height of user terminal in meter
factorySize = 'wifi'; %factory size : big;small
factoryScenario = 'InF-SL'; %factory scenario : InF-SL, InF-DL, InF-SH, InF-DH
onlyLOS = true; %false = multipath propagation are also taken into account

seedNum = 1;
algorithmType = 'twister';

%% Set factory layout
% 
% [factorySizeVector,gNbDistance] = setFactoryLayout(factorySize,roomHeight);
% 
% %% Set gNBs layout
% 
% [gNbLayoutMatrix] = setGnbLayoutWifi(factoryScenario,gNbDistance,gNbNums);
% 
% plot(gNbLayoutMatrix(:,1),gNbLayoutMatrix(:,2), 'b*');
% xlim([0 100])
% ylim([0 100])
% grid on;
% 
% %% Set UEs layout
% %TODO : uniform height between user equipments??
% 
% % assumes that ueHeight = 1.5
% [ueLayoutMatrix] = setUeLayout3(factorySizeVector,ueNums,seedNum);
% 
% plot(ueLayoutMatrix(:,1),ueLayoutMatrix(:,2), 'b*','Color','red');
% grid on;
% hold on;
% 
% plot(gNbLayoutMatrix(:,1),gNbLayoutMatrix(:,2), 'b*');
% hold on;
% grid on;
% 
% for i = 1 : gNbNums
%     plot([gNbLayoutMatrix(i,1) ueLayoutMatrix(i,1)], [gNbLayoutMatrix(i,2) ueLayoutMatrix(i,2)])
%     plot([gNbLayoutMatrix(i,1) ueLayoutMatrix(i,1)], [gNbLayoutMatrix(i,2) ueLayoutMatrix(i,2)])
% end

%% Create output files scenario 6

    [factorySizeVector,gNbDistance] = setFactoryLayout(factorySize,roomHeight);

    outputPath = '/home/navid/NS3_AI_RECMAC/ns-allinone-3.38/ns-3.38/freespacePL/Scenario3';
    [gNbLayoutMatrix] = setGnbLayoutWifi(factoryScenario,gNbDistance,gNbNums);
    [pathlossMatrixGnb,gNbNums,~] = calculatePathloss(gNbLayoutMatrix, gNbLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);
    writematrix(gNbLayoutMatrix,append(outputPath,'/gNbLayoutMatrix.txt'),'Delimiter','space');
    writematrix(pathlossMatrixGnb,append(outputPath,'/pathlossMatrixGnbGnb.txt'),'Delimiter','space');

    for i = 1:1000
        seedNum = i;
        apIndex = randperm(gNbNums);
        writematrix(apIndex,append(outputPath,'/apIndex_',num2str(i),'.txt'),'Delimiter','space')
        [ueLayoutMatrix] = setUeLayout3(factorySizeVector,ueNums,seedNum);
        ueLayoutMatrix = sortrows(ueLayoutMatrix,[1 2]);
   
        [pathlossMatrixGnbUe,gNbNums,ueNums] = calculatePathloss(gNbLayoutMatrix, ueLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);
        [pathlossMatrixUe,ueNums,~] = calculatePathloss(ueLayoutMatrix, ueLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);
  
        writematrix(ueLayoutMatrix,append(outputPath,'/ueCoordinates_',num2str(i),'.txt'),'Delimiter','space');
        writematrix(pathlossMatrixGnbUe,append(outputPath,'/pathlossMatrixGnbUe_',num2str(i),'.txt'),'Delimiter','space');
        writematrix(pathlossMatrixUe,append(outputPath,'/pathlossMatrixUeUe_',num2str(i),'.txt'),'Delimiter','space');
    end