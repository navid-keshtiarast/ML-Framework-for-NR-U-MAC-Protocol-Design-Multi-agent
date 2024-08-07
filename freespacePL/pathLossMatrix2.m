
gNbNums = 18; %number of GNBs
ueNums = gNbNums; %number of UEs
roomHeight = 10; %height of the factory in meter
carrierFrequency = 5.945; %in GHz 
bandwidth = 20e6;
utHeight = 1.5; %height of user terminal in meter
factorySize = 'custom'; %factory size : big;small
factoryScenario = 'InF-SL'; %factory scenario : InF-SL, InF-DL, InF-SH, InF-DH
onlyLOS = true; %false = multipath propagation are also taken into account

seedNum = 1;
algorithmType = 'twister';

%% Set factory layout

[factorySizeVector,gNbDistance] = setFactoryLayout(factorySize,roomHeight);

%% Set gNBs layout

[gNbLayoutMatrix] = setGnbLayout(factoryScenario,gNbDistance,gNbNums);

plot(gNbLayoutMatrix(:,1),gNbLayoutMatrix(:,2), 'b*');
grid on;

gNbLayoutMatrix2 = zeros(gNbNums*3,3);

for i = 0 : gNbNums - 1
    for j = 1 : 3
        gNbLayoutMatrix2(i*3+j,:) = gNbLayoutMatrix(i+1,:);
    end
end

%% Set UEs layout
%TODO : uniform height between user equipments??

% assumes that ueHeight = 1.5
[ueLayoutMatrix] = setUeLayout3(factorySizeVector,ueNums,seedNum);

plot(ueLayoutMatrix(:,1),ueLayoutMatrix(:,2), 'b*');

ueLayoutMatrix2 = zeros(ueNums*3,3);

for i = 0 : ueNums - 1
    for j = 1 : 3
        ueLayoutMatrix2(i*3+j,:) = ueLayoutMatrix(i+1,:);
    end
end

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

%Test connect one to one random
for i = 1 : 18
    plot([gNbLayoutMatrix(i,1) ueLayoutMatrix(i,1)], [gNbLayoutMatrix(i,2) ueLayoutMatrix(i,2)])
    plot([gNbLayoutMatrix(i,1) ueLayoutMatrix(i,1)], [gNbLayoutMatrix(i,2) ueLayoutMatrix(i,2)])
end

ueLayoutMatrix1 = sortrows(ueLayoutMatrix,[1 2]);

plot(ueLayoutMatrix1(:,1),ueLayoutMatrix1(:,2), 'b*','Color','red');
grid on;
hold on;

plot(gNbLayoutMatrix(:,1),gNbLayoutMatrix(:,2), 'b*');
hold on;

for i = 1 : 18
    plot([gNbLayoutMatrix(i,1) ueLayoutMatrix1(i,1)], [gNbLayoutMatrix(i,2) ueLayoutMatrix1(i,2)])
    plot([gNbLayoutMatrix(i,1) ueLayoutMatrix1(i,1)], [gNbLayoutMatrix(i,2) ueLayoutMatrix1(i,2)])
end

[ueLayoutMatrix2] = setUeLayout3(factorySizeVector,ueNums,seedNum);
ueLayoutMatrix2 = sortrows(ueLayoutMatrix2,[1 2]);
[gNbLayoutMatrix2] = setUeLayout3(factorySizeVector,gNbNums,seedNum+100);
gNbLayoutMatrix2 = sortrows(gNbLayoutMatrix2,[1 2]);

plot(ueLayoutMatrix2(:,1),ueLayoutMatrix2(:,2), 'b*','Color','red');
grid on;
hold on;

plot(gNbLayoutMatrix2(:,1),gNbLayoutMatrix2(:,2), 'b*');
hold on;

for i = 1 : 18
    plot([gNbLayoutMatrix2(i,1) ueLayoutMatrix2(i,1)], [gNbLayoutMatrix2(i,2) ueLayoutMatrix2(i,2)])
    plot([gNbLayoutMatrix2(i,1) ueLayoutMatrix2(i,1)], [gNbLayoutMatrix2(i,2) ueLayoutMatrix2(i,2)])
end
%% Compare two maximum distance

minDist1 = 0;
minDist2 = 0;

ueLayoutMatrix1 = sortrows(ueLayoutMatrix,[1 2]);


for i = 1 : 18
    temp = calculate3dDistance(gNbLayoutMatrix(i,:),ueLayoutMatrix(i,:))
    minDist1 = minDist1 + temp;
end

for i = 1 : 18
    temp = calculate3dDistance(gNbLayoutMatrix(i,:),ueLayoutMatrix1(i,:))
    minDist2 = minDist2 + temp;
end

%% Sort for more than two nodes

%Each with one nodes
[cIdx] = unique(knnsearch(gNbLayoutMatrix,ueLayoutMatrix),'stable');
% brute force knnsearch to do without replacements
startingK = 2;
while length(cIdx)<length(ueLayoutMatrix(1:18,:))
    [cIdx] = unique(knnsearch(gNbLayoutMatrix,ueLayoutMatrix,'k',startingK),'stable');
    startingK = startingK + 1;
end

ueLayoutMatrix1 = ueLayoutMatrix(cIdx,:);

plot(ueLayoutMatrix_new(:,1),ueLayoutMatrix_new(:,2), 'b*','Color','red');
grid on;
hold on;

plot(gNbLayoutMatrix(:,1),gNbLayoutMatrix(:,2), 'b*');
hold on;

for i = 1 : 18
    plot([gNbLayoutMatrix(i,1) ueLayoutMatrix(i,1)], [gNbLayoutMatrix(i,2) ueLayoutMatrix(i,2)])
    hold on
end

%Each with more than nodes
[cIdx] = knnsearch(ueLayoutMatrix(1:18,:),gNbLayoutMatrix);
[cIdx2] = knnsearch(ueLayoutMatrix(19:36,:),gNbLayoutMatrix);

%% Calculate path loss matrix in dB

[pathlossMatrixGnb,gNbNums,~] = calculatePathloss(gNbLayoutMatrix2, gNbLayoutMatrix2, carrierFrequency, onlyLOS, factoryScenario);
[pathlossMatrixGnbUe,gNbNums,ueNums] = calculatePathloss(gNbLayoutMatrix2, ueLayoutMatrix2, carrierFrequency, onlyLOS, factoryScenario);
[pathlossMatrixUe,ueNums,~] = calculatePathloss(ueLayoutMatrix2, ueLayoutMatrix2, carrierFrequency, onlyLOS, factoryScenario);
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

%% Create output files scenario 1

    [factorySizeVector,gNbDistance] = setFactoryLayout(factorySize,roomHeight);
    [gNbLayoutMatrix] = setGnbLayout(factoryScenario,gNbDistance, gNbNums);
    [pathlossMatrixGnb,gNbNums,~] = calculatePathloss(gNbLayoutMatrix, gNbLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);
    
    outputPath = '/home/inets/Desktop/A/workspace-nr/ns-3-dev/freespacePL/Scenario1';
    writematrix(gNbLayoutMatrix,append(outputPath,'/gNbLayoutMatrix.txt'),'Delimiter','space');
    writematrix(pathlossMatrixGnb,append(outputPath,'/pathlossMatrixGnbGnb.txt'),'Delimiter','space');

    for i = 1:100
        seedNum = i;
        apIndex = randperm(18);
        writematrix(apIndex,append(outputPath,'/apIndex_',num2str(i),'.txt'),'Delimiter','space')
        [ueLayoutMatrix] = setUeLayout3(factorySizeVector,ueNums,seedNum);
        [pathlossMatrixGnbUe,gNbNums,ueNums] = calculatePathloss(gNbLayoutMatrix, ueLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);
        [pathlossMatrixUe,ueNums,~] = calculatePathloss(ueLayoutMatrix, ueLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);
        
        writematrix(ueLayoutMatrix,append(outputPath,'/ueCoordinates_',num2str(i),'.txt'),'Delimiter','space');
        writematrix(pathlossMatrixGnbUe,append(outputPath,'/pathlossMatrixGnbUe_',num2str(i),'.txt'),'Delimiter','space');
        writematrix(pathlossMatrixUe,append(outputPath,'/pathlossMatrixUeUe_',num2str(i),'.txt'),'Delimiter','space');
    end

%% Create output files scenario 1a

    [factorySizeVector,gNbDistance] = setFactoryLayout(factorySize,roomHeight);
    [gNbLayoutMatrix] = setGnbLayout(factoryScenario,gNbDistance, gNbNums);
    gNbLayoutMatrix2 = zeros(gNbNums*3,3);

    for gnbOuterLoop = 0 : gNbNums-1
        for gnbInnerLoop = 1 : 3
            gNbLayoutMatrix2(gnbOuterLoop*3+gnbInnerLoop,:) = gNbLayoutMatrix(gnbOuterLoop+1,:);
        end
    end
    
    [pathlossMatrixGnb,~,~] = calculatePathloss(gNbLayoutMatrix2, gNbLayoutMatrix2, carrierFrequency, onlyLOS, factoryScenario);
    
    outputPath = '/home/inets/Desktop/A/workspace-nr/ns-3-dev/freespacePL/Scenario4';
    writematrix(gNbLayoutMatrix2,append(outputPath,'/gNbLayoutMatrix.txt'),'Delimiter','space');
    writematrix(pathlossMatrixGnb,append(outputPath,'/pathlossMatrixGnbGnb.txt'),'Delimiter','space');

    for i = 1:10
        seedNum = i;
        apIndex = randperm(18);
        apIndex2 = zeros(1,18*3);
        for apOuterLoop = 0 : gNbNums-1
            for apInnerLoop = 1 : 3
                apIndex2(1,apOuterLoop*3+apInnerLoop) = apIndex(1,apOuterLoop+1);
            end
        end

        writematrix(apIndex2,append(outputPath,'/apIndex_',num2str(i),'.txt'),'Delimiter','space')

        [ueLayoutMatrix] = setUeLayout3(factorySizeVector,ueNums,seedNum);
        ueLayoutMatrix2 = zeros(ueNums*3,3);

        for ueOuterLoop = 0 : ueNums - 1
            for ueInnerLoop = 1 : 3
                ueLayoutMatrix2(ueOuterLoop*3+ueInnerLoop,:) = ueLayoutMatrix(ueOuterLoop+1,:);
            end
        end

        [pathlossMatrixGnbUe,~,~] = calculatePathloss(gNbLayoutMatrix2, ueLayoutMatrix2, carrierFrequency, onlyLOS, factoryScenario);
        [pathlossMatrixUe,~,~] = calculatePathloss(ueLayoutMatrix2, ueLayoutMatrix2, carrierFrequency, onlyLOS, factoryScenario);
        
        writematrix(ueLayoutMatrix2,append(outputPath,'/ueCoordinates_',num2str(i),'.txt'),'Delimiter','space');
        writematrix(pathlossMatrixGnbUe,append(outputPath,'/pathlossMatrixGnbUe_',num2str(i),'.txt'),'Delimiter','space');
        writematrix(pathlossMatrixUe,append(outputPath,'/pathlossMatrixUeUe_',num2str(i),'.txt'),'Delimiter','space');
    end

    %% Create output files scenario 5

    [factorySizeVector,gNbDistance] = setFactoryLayout(factorySize,roomHeight);
    [gNbLayoutMatrix] = setGnbLayout(factoryScenario,gNbDistance, gNbNums);
    gNbLayoutMatrix2 = zeros(gNbNums*2,3);

    for gnbOuterLoop = 0 : gNbNums-1
        for gnbInnerLoop = 1 : 2
            gNbLayoutMatrix2(gnbOuterLoop*2+gnbInnerLoop,:) = gNbLayoutMatrix(gnbOuterLoop+1,:);
        end
    end
    
    [pathlossMatrixGnb,~,~] = calculatePathloss(gNbLayoutMatrix2, gNbLayoutMatrix2, carrierFrequency, onlyLOS, factoryScenario);
    
    outputPath = '/home/inets/Desktop/A/workspace-nr/ns-3-dev/freespacePL/Scenario5';
    writematrix(gNbLayoutMatrix2,append(outputPath,'/gNbLayoutMatrix.txt'),'Delimiter','space');
    writematrix(pathlossMatrixGnb,append(outputPath,'/pathlossMatrixGnbGnb.txt'),'Delimiter','space');

    for i = 1:10
        seedNum = i;
        apIndex = randperm(18);
        apIndex2 = zeros(1,18*2);
        for apOuterLoop = 0 : gNbNums-1
            for apInnerLoop = 1 : 2
                apIndex2(1,apOuterLoop*2+apInnerLoop) = apIndex(1,apOuterLoop+1);
            end
        end

        writematrix(apIndex2,append(outputPath,'/apIndex_',num2str(i),'.txt'),'Delimiter','space')

        [ueLayoutMatrix] = setUeLayout3(factorySizeVector,ueNums,seedNum);
        ueLayoutMatrix2 = zeros(ueNums*2,3);

        for ueOuterLoop = 0 : ueNums - 1
            for ueInnerLoop = 1 : 2
                ueLayoutMatrix2(ueOuterLoop*2+ueInnerLoop,:) = ueLayoutMatrix(ueOuterLoop+1,:);
            end
        end

        [pathlossMatrixGnbUe,~,~] = calculatePathloss(gNbLayoutMatrix2, ueLayoutMatrix2, carrierFrequency, onlyLOS, factoryScenario);
        [pathlossMatrixUe,~,~] = calculatePathloss(ueLayoutMatrix2, ueLayoutMatrix2, carrierFrequency, onlyLOS, factoryScenario);
        
        writematrix(ueLayoutMatrix2,append(outputPath,'/ueCoordinates_',num2str(i),'.txt'),'Delimiter','space');
        writematrix(pathlossMatrixGnbUe,append(outputPath,'/pathlossMatrixGnbUe_',num2str(i),'.txt'),'Delimiter','space');
        writematrix(pathlossMatrixUe,append(outputPath,'/pathlossMatrixUeUe_',num2str(i),'.txt'),'Delimiter','space');
    end
   %% Create output files scenario 2

    [factorySizeVector,gNbDistance] = setFactoryLayout(factorySize,roomHeight);
    [gNbLayoutMatrix] = setGnbLayout(factoryScenario,gNbDistance, gNbNums);
    [pathlossMatrixGnb,gNbNums,~] = calculatePathloss(gNbLayoutMatrix, gNbLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);
    
    outputPath = '/home/inets/Desktop/A/workspace-nr/ns-3-dev/freespacePL/Scenario2';
    writematrix(gNbLayoutMatrix,append(outputPath,'/gNbLayoutMatrix.txt'),'Delimiter','space');
    writematrix(pathlossMatrixGnb,append(outputPath,'/pathlossMatrixGnbGnb.txt'),'Delimiter','space');

    for i = 1:100
        seedNum = i;
        apIndex = randperm(18);
        writematrix(apIndex,append(outputPath,'/apIndex_',num2str(i),'.txt'),'Delimiter','space')
        [ueLayoutMatrix] = setUeLayout3(factorySizeVector,ueNums,seedNum);
        ueLayoutMatrix = sortrows(ueLayoutMatrix,[1 2]);
        [pathlossMatrixGnbUe,gNbNums,ueNums] = calculatePathloss(gNbLayoutMatrix, ueLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);
        [pathlossMatrixUe,ueNums,~] = calculatePathloss(ueLayoutMatrix, ueLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);
        
        writematrix(ueLayoutMatrix,append(outputPath,'/ueCoordinates_',num2str(i),'.txt'),'Delimiter','space');
        writematrix(pathlossMatrixGnbUe,append(outputPath,'/pathlossMatrixGnbUe_',num2str(i),'.txt'),'Delimiter','space');
        writematrix(pathlossMatrixUe,append(outputPath,'/pathlossMatrixUeUe_',num2str(i),'.txt'),'Delimiter','space');
    end
     %% Create output files scenario 3

    [factorySizeVector,gNbDistance] = setFactoryLayout(factorySize,roomHeight);

    outputPath = '/home/inets/Desktop/A/workspace-nr/ns-3-dev/freespacePL/Scenario3';

    for i = 1:100
        seedNum = i;
        apIndex = randperm(18);
        writematrix(apIndex,append(outputPath,'/apIndex_',num2str(i),'.txt'),'Delimiter','space')
        [gNbLayoutMatrix] = setUeLayout3(factorySizeVector,ueNums,seedNum+100);
        gNbLayoutMatrix = sortrows(gNbLayoutMatrix,[1 2]);
        [ueLayoutMatrix] = setUeLayout3(factorySizeVector,ueNums,seedNum);
        ueLayoutMatrix = sortrows(ueLayoutMatrix,[1 2]);
        [pathlossMatrixGnb,gNbNums,~] = calculatePathloss(gNbLayoutMatrix, gNbLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);
        [pathlossMatrixGnbUe,gNbNums,ueNums] = calculatePathloss(gNbLayoutMatrix, ueLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);
        [pathlossMatrixUe,ueNums,~] = calculatePathloss(ueLayoutMatrix, ueLayoutMatrix, carrierFrequency, onlyLOS, factoryScenario);
        
        writematrix(gNbLayoutMatrix,append(outputPath,'/gNbLayoutMatrix_',num2str(i),'.txt'),'Delimiter','space');
        writematrix(pathlossMatrixGnb,append(outputPath,'/pathlossMatrixGnbGnb_',num2str(i),'.txt'),'Delimiter','space');
        writematrix(ueLayoutMatrix,append(outputPath,'/ueCoordinates_',num2str(i),'.txt'),'Delimiter','space');
        writematrix(pathlossMatrixGnbUe,append(outputPath,'/pathlossMatrixGnbUe_',num2str(i),'.txt'),'Delimiter','space');
        writematrix(pathlossMatrixUe,append(outputPath,'/pathlossMatrixUeUe_',num2str(i),'.txt'),'Delimiter','space');
    end