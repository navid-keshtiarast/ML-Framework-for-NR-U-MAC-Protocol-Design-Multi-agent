function [pathlossMatrix,unitNum1,unitNum2] = calculatePathloss(layoutMatrix1, layoutMatrix2, frequency, onlyLOS, factoryScenario)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
unitNum1 = size(layoutMatrix1,1);
unitNum2 = size(layoutMatrix2,1);

pathlossMatrix = zeros(unitNum1,unitNum2);

    for i = 1 : unitNum1
        for j = 1 : unitNum2
            dist = calculate3dDistance(layoutMatrix1(i,:),layoutMatrix2(j,:));
            if dist ~= 0
                pathlossLOS = 31.84 + 21.50*log10(dist)+19*log10(frequency);
                if onlyLOS == true
                    pathlossMatrix(i,j) = pathlossLOS;
                else
                    switch factoryScenario
                        case "InF-SL"
                            pathlossNLOS = 33 + 25.5*log10(dist) + 20*log10(frequency);
                            pathlossMatrix(i,j) = max(pathlossLOS,pathlossNLOS);
                        case "InF-DL"
                            pathlossNLOS = 18.6 + 35.7*log10(dist) + 20*log10(frequency);
                            pathlossMatrix(i,j) = max(pathlossLOS,pathlossNLOS);
                        case "InF-SH"
                            pathlossNLOS = 32.4 + 23*log10(dist) + 20*log10(frequency);
                            pathlossMatrix(i,j) = max(pathlossLOS,pathlossNLOS);
                        case "InF-DH"
                            pathlossNLOS = 33.63 + 21.9*log10(dist) + 20*log10(frequency);
                            pathlossMatrix(i,j) = max(pathlossLOS,pathlossNLOS);
                        otherwise
                            disp('Unauthorized factoryScenario parameter!!')
                    end
                end
            else
                pathlossMatrix(i,j) = 0;
            end
        end
    end
end