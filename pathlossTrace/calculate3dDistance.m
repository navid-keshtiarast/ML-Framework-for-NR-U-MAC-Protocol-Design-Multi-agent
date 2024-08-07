function [dist3d] = calculate3dDistance(positionVector1,positionVector2)
%CALCULATEDISTANCE Summary of this function goes here
%   Detailed explanation goes here

temp = (positionVector2(1)-positionVector1(1))^2 + (positionVector2(2)-positionVector1(2))^2 + (positionVector2(3)-positionVector1(3))^2;
dist3d = sqrt(temp);

end

