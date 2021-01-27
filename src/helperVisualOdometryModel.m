%%
% *|helperVisualOdometryModel|*
%
% Compute visual odometry measurement from ground truth input and
% parameters struct. To model the uncertainty in the scaling between
% subsequent frames of the monocular camera, a constant scaling factor
% combined with a random drift is applied to the ground truth position.
function [posVO, orientVO, paramsVO] ...
    = helperVisualOdometryModel(pos, orient, paramsVO)

% Extract model parameters. 
scaleVO = paramsVO.scale;
sigmaN = paramsVO.sigmaN;
tau = paramsVO.tau;
sigmaB = paramsVO.sigmaB;
sigmaA = sqrt((2/tau) + 1/(tau*tau))*sigmaB;
b = paramsVO.driftBias;

% Calculate drift. 
b = (1 - 1/tau).*b + randn(1,3)*sigmaA;
drift = randn(1,3)*sigmaN + b;
paramsVO.driftBias = b;

% Calculate visual odometry measurements.
posVO = scaleVO*pos + drift;
orientVO = eul2rotm(scaleVO*orient + drift);
end
