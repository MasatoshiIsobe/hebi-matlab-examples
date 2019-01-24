% Get Spring Stiffness from the modules
%
% Dave Rollinson
% Jan 2019

log = HebiUtils.loadGroupLog('logs/2019-01-22_09-01-44.523','view','debug');

numModules = size(log.position,2);

polyOrder = 5;
springCoeffs = nan( numModules, polyOrder+1 );
stiffnessCoeffs = nan( numModules, polyOrder );

for i = 1:numModules
    springCoeffs(i,:) = polyfit( -log.deflection(:,i), log.effort(:,i), ...
                                 polyOrder );
    stiffnessCoeffs(i,:) = polyder( springCoeffs(i,:) );                         
end
    
save('latestStiffnessCoeffs','stiffnessCoeffs');