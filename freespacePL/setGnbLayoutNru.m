function [gNbLayoutMatrix] = setGnbLayoutNru(factoryScenario,gNbDistance, gNbNums)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
gNbLayoutMatrix = zeros(gNbNums,3);
xVector = zeros(gNbNums,1);
yVector = zeros(gNbNums,1);

xVector = [25 55 85 115 145 175 25 55 85 115 145 175 25 55 85 115 145 175];
yVector = [50 50 50 50 50 50 100 100 100 100 100 100 150 150 150 150 150 150];

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