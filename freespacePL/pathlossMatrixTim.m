%% Configurations

gNbNums = 3; %number of GNBs
ueNums = gNbNums; %number of UEs
roomHeight = 2.5; %height of the factory in meter
carrierFrequency = 5.945; %in GHz 
bandwidth = 20e6;
ueHeight = 0.5; %height of user terminal in meter

seedNum = 1;
algorithmType = 'twister';

outputPath = '/home/inets/Desktop/A/ns-allinone-3.38/ns-3.38/freespacePL/Tim';

%% Set room layout

roomSizeVector = [10 10 roomHeight];

%% Set AP layout

gNbLayoutMatrix = zeros(gNbNums,3);

xVector = [4 6 8];
yVector = [4 4 4];
gNbHeight = 0.5;

gNbLayoutMatrix(:,1) = xVector(:);
gNbLayoutMatrix(:,2) = yVector(:);
gNbLayoutMatrix(:,3) = gNbHeight;

plot(gNbLayoutMatrix(:,1),gNbLayoutMatrix(:,2), 'b*');

%% Set Layout UEs

ueLayoutMatrix = zeros(ueNums,3);

xVector = [4 6 8];
yVector = [2 2 2];

ueLayoutMatrix(:,1) = xVector(:);
ueLayoutMatrix(:,2) = yVector(:);
ueLayoutMatrix(:,3) = ueHeight;

hold on
plot(ueLayoutMatrix(:,1),ueLayoutMatrix(:,2), 'b*');

%% Calculate Pathloss

[pathlossMatrixGnb,gNbNums,~] = calculatePathloss(gNbLayoutMatrix, gNbLayoutMatrix, carrierFrequency);
[pathlossMatrixGnbUe,gNbNums,ueNums] = calculatePathloss(gNbLayoutMatrix, ueLayoutMatrix, carrierFrequency);
[pathlossMatrixUe,ueNums,~] = calculatePathloss(ueLayoutMatrix, ueLayoutMatrix, carrierFrequency);

%% Output to Matrix
apIndex = randperm(gNbNums);
writematrix(apIndex,append(outputPath,'/apIndex.txt'),'Delimiter','space')

writematrix(gNbLayoutMatrix,append(outputPath,'/gNbLayoutMatrix.txt'),'Delimiter','space');
writematrix(pathlossMatrixGnb,append(outputPath,'/pathlossMatrixGnbGnb.txt'),'Delimiter','space');

writematrix(ueLayoutMatrix,append(outputPath,'/ueCoordinates.txt'),'Delimiter','space');
writematrix(pathlossMatrixGnbUe,append(outputPath,'/pathlossMatrixGnbUe.txt'),'Delimiter','space');
writematrix(pathlossMatrixUe,append(outputPath,'/pathlossMatrixUeUe.txt'),'Delimiter','space');
