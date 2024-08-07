function [ueLayoutMatrix] = setUeLayout(factorySizeVector,ueNums,seedNum)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    rng(seedNum);
    
    ueLayoutMatrix = zeros(ueNums,3);
    ueLayoutMatrix(:,3) = 1.2;
    allowableDistance = 1;
    
    %ueLayoutMatrix(1,1) = randi([1 factorySizeVector(1)],1,1); 
    ueLayoutMatrix(1,1) = factorySizeVector(1).*rand(1,1);
    %ueLayoutMatrix(1,2) = randi([1 factorySizeVector(2)],1,1);
    ueLayoutMatrix(1,2) = factorySizeVector(2).*rand(1,1);
    
    counter = 2;
    
    while counter < ueNums+1
        %randomX = randi([ueLayoutMatrix(counter-1,1)-1 ueLayoutMatrix(counter-1,1)]+1,1,1);
        bitGen = (rand>=0.5);
        if bitGen == 1
            signum = 1;
        else 
            signum = -1;
        end
        
%         directionGen = (rand>=0.5);
%         if directionGen == 1
            randomX = (ueLayoutMatrix(counter-1,1)-1) + 2*rand(1,1);
            randomY = signum*sqrt(1-(randomX-ueLayoutMatrix(counter-1,1))^2)+ueLayoutMatrix(counter-1,2);
%         else
%             randomY = (ueLayoutMatrix(counter-1,2)-1) + 2*rand(1,1);
%             randomX = signum*sqrt(1-(randomY-ueLayoutMatrix(counter-1,2))^2)+ueLayoutMatrix(counter-1,1);
%         end

        distance = sqrt((randomX-ueLayoutMatrix(:,1)).^2 + (randomY-ueLayoutMatrix(:,2)).^2);
    
        minDistance = min(distance);
        if minDistance >= allowableDistance && randomX > 0 && randomX < factorySizeVector(1) && randomY > 0 && randomY < factorySizeVector(2)
            ueLayoutMatrix(counter,1) = randomX;
            ueLayoutMatrix(counter,2) = randomY;
            counter = counter+1;
        end
    end
end

