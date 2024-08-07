function [gNbLayoutMatrix] = setGnbLayout(factoryScenario,gNbDistance, gNbNums)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
gNbLayoutMatrix = zeros(18,3);
xVector = zeros(18,1);
yVector = zeros(18,1);

    for rowLoop = 1 : 3
        for colLoop = 1: 6
            xVector(colLoop+6*(rowLoop-1),1) = gNbDistance/2 + (colLoop-1)*gNbDistance;
            yVector(colLoop+6*(rowLoop-1),1) = gNbDistance/2 + (rowLoop-1)*gNbDistance;
        end
    end

    if factoryScenario == "InF-SL" || factoryScenario == "InF-DL"
        gNbHeight = 1.5;
    elseif factoryScenario == "InF-SH" || factoryScenario == "InF-DH"
        gNbHeight = 8;
    else
        message('Unauthorized parameter!!')
    end

    gNbLayoutMatrix(:,1) = xVector(:);
    gNbLayoutMatrix(:,2) = yVector(:);
    gNbLayoutMatrix(:,3) = gNbHeight;

    gNbLayoutMatrix = sortrows(gNbLayoutMatrix,[1 2]);

    gNbLayoutMatrix = gNbLayoutMatrix(1:gNbNums,:);
end