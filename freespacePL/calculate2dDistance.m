function [dist2d] = calculate2dDistance(positionVector1,positionVector2)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

temp = (positionVector2(1)-positionVector1(1))^2 + (positionVector2(2)-positionVector1(2))^2;
dist2d = sqrt(temp);

end