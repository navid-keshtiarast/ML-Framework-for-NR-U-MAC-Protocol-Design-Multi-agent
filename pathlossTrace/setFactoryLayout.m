function [factorySizeVector,gNbDistance] = setFactoryLayout(factorySize,roomHeight)
%SETFACTORYLAYOUT Summary of this function goes here
%   Detailed explanation goes here
    if factorySize == "small"
        factorySizeVector = [120 60 roomHeight];
        gNbDistance = 20;
    elseif factorySize == "big"
        factorySizeVector = [300 150 roomHeight];
        gNbDistance = 50;
    else
        message('Unauthorized parameter!!')
    end
end

