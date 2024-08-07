function [probabilityMatrix,gNbNums,ueNums] = calculateLOSProbability(gNbLayoutMatrix,ueLayoutMatrix,utHeight,clutterHeight,clutterDensity,clutterSize, factoryScenario)
%CALCULATELOSPROBABILITY Summary of this function goes here
%   Detailed explanation goes here
gNbNums = size(gNbLayoutMatrix,1);
ueNums = size(ueLayoutMatrix,1);

probabilityMatrix = zeros(gNbNums,ueNums);

    for i = 1 : gNbNums
        for j = 1 : ueNums
            dist = calculate2dDistance(gNbLayoutMatrix(i,(1:2)),ueLayoutMatrix(j,(1:2)));
            if dist ~= 0
                if factoryScenario == "InF-SL" || factoryScenario == "InF-DL"
                    kSubsce = -clutterSize/(log(1-clutterDensity));
                elseif factoryScenario == "InF-SH" || factoryScenario == "InF-DH"
                    kSubsce = (-clutterSize/(log(1-clutterDensity)))*((gNbLayoutMatrix(i,3)-utHeight)/(clutterHeight-utHeight));
                else
                    message('Unauthorized factoryScenario parameter!!')
                end
                probabilityMatrix(i,j) = exp(-dist/kSubsce);
            else
                probabilityMatrix(i,j) = 1;
            end
        end
    end
end

