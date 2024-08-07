function [ueLayoutMatrix] = setUeLayout4(factorySizeVector,gNbLayoutMatrix,seedNum)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    rng(seedNum);
    
    ueNums = size(gNbLayoutMatrix,1);
    ueLayoutMatrix = zeros(ueNums,3);
    ueLayoutMatrix(:,3) = 1.5;
    allowableDistance = 1;
    
    counter = 1;
    
    while counter < ueNums+1
        %randomX = randi([ueLayoutMatrix(counter-1,1)-1 ueLayoutMatrix(counter-1,1)]+1,1,1);
        bitGen = (rand>=0.5);
        if bitGen == 1
            signum = 1;
        else 
            signum = -1;
        end
        
            randomX = (gNbLayoutMatrix(counter,1)-1) + 2*rand(1,1);
            randomY = signum*sqrt(1-(randomX-gNbLayoutMatrix(counter,1))^2)+gNbLayoutMatrix(counter,2);

        distance = sqrt((randomX-ueLayoutMatrix(:,1)).^2 + (randomY-ueLayoutMatrix(:,2)).^2);
    
        minDistance = min(distance);
        if minDistance >= allowableDistance && randomX > 0 && randomX < factorySizeVector(1) && randomY > 0 && randomY < factorySizeVector(2)
            ueLayoutMatrix(counter,1) = randomX;
            ueLayoutMatrix(counter,2) = randomY;
            counter = counter+1;  
        end
    end
end