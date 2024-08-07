function [ueLayoutMatrix] = setUeLayout4(factorySizeVector,gNbLayoutMatrix,seedNum)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    rng(seedNum);
    
    ueNums = size(gNbLayoutMatrix,1);
    ueLayoutMatrix = zeros(ueNums,3);
    ueLayoutMatrix(:,3) = 1.5;
    allowableDistance = 1;
    maxDistance = 30;
    
    counter = 1;
    
    while counter < ueNums+1
        %randomX = randi([ueLayoutMatrix(counter-1,1)-1 ueLayoutMatrix(counter-1,1)]+1,1,1);
        bitGen = (rand>=0.5);
        if bitGen == 1
            signum = 1;
        else 
            signum = -1;
        end
        
            randomX = (gNbLayoutMatrix(counter,1)) + randi([-30,30],1,1);
            randomY = (gNbLayoutMatrix(counter,2)) + randi([-30,30],1,1);

        distance = sqrt((randomX-ueLayoutMatrix(:,1)).^2 + (randomY-ueLayoutMatrix(:,2)).^2);
        actualDistance = sqrt((randomX-gNbLayoutMatrix(counter,1)).^2 + (randomY-gNbLayoutMatrix(counter,2)).^2);

        minDistance = min(distance);
        if minDistance >= allowableDistance && actualDistance <= maxDistance && randomX > 0 && randomX < factorySizeVector(1) && randomY > 0 && randomY < factorySizeVector(2)
            ueLayoutMatrix(counter,1) = randomX;
            ueLayoutMatrix(counter,2) = randomY;
            counter = counter+1;  
        end
    end
end