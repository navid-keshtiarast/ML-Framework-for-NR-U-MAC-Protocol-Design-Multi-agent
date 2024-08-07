function [pathlossMatrix,unitNum1,unitNum2] = calculatePathloss(layoutMatrix1, layoutMatrix2, frequency, ~, ~)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
unitNum1 = size(layoutMatrix1,1);
unitNum2 = size(layoutMatrix2,1);

pathlossMatrix = zeros(unitNum1,unitNum2);

    for i = 1 : unitNum1
        for j = 1 : unitNum2
            dist = calculate3dDistance(layoutMatrix1(i,:),layoutMatrix2(j,:));
            if dist ~= 0
                pathlossLOS = 20*log10(dist/1000) + 20*log10(frequency) + 92.45;
                pathlossMatrix(i,j) = pathlossLOS;
            else
                pathlossMatrix(i,j) = 0;
            end
        end
    end
end