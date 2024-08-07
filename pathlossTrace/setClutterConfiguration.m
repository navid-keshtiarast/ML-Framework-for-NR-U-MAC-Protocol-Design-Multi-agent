function [clutterDensity,clutterHeight,clutterSize] = setClutterConfiguration(clutterType)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    if clutterType == "low"
        clutterDensity = 0.2;
        clutterHeight = 2;
        clutterSize = 10;
    elseif clutterType == "high"
        clutterDensity = 0.6;
        clutterHeight = 6;
        clutterSize = 2;
    else
        message('Unauthorized parameter!!')
    end
end

