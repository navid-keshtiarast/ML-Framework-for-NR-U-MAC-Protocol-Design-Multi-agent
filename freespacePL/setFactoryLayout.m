function [factorySizeVector,gNbDistance] = setFactoryLayout(factorySize,roomHeight)
%SETFACTORYLAYOUT Summary of this function goes here
%   Detailed explanation goes here
    if factorySize == "small"
        factorySizeVector = [120 60 roomHeight];
        gNbDistance = 20;
    elseif factorySize == "big"
        factorySizeVector = [300 150 roomHeight];
        gNbDistance = 50;
    elseif factorySize == "custom"
        factorySizeVector = [200 200 roomHeight];
        gNbDistance = 30;
    elseif factorySize == "wifi"
        factorySizeVector = [100 100 roomHeight];
        gNbDistance = 40;
    else
        message('Unauthorized parameter!!')
    end
end

