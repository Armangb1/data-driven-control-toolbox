% +MFAC  Model-Free Adaptive Control (CFDL / PFDL / FFDL)
%
% Classes (matlab.System, usable as MATLAB objects or Simulink blocks).
%   MFACController  - Generalized MFAC controller. CFDL and PFDL are
%                     special cases of the FFDL implementation:
%                       CFDL  <=>  Ly=0, Lu=1  (default)
%                       PFDL  <=>  Ly=0, Lu=L
%                       FFDL  <=>  general Ly, Lu
%                     Pseudo-order vector H(t) = [y(t..t-Ly+1), u(t..t-Lu+1)]',
%                     projection-algorithm PPD update with reset, and the
%                     Hou & Jin control law with weighting factors rho, eta,
%                     mu, lambda. Two outputs: u and the scalar phi(Ly+1)
%                     ("current-input" PPD).
%
% Examples:
%   ddc.mfac.MFACController                          % CFDL (default)
%   ddc.mfac.MFACController('Ly',0,'Lu',3,'PhiInit',0.01*ones(1,3), ...
%                           'Rho',0.3*ones(1,3))    % PFDL, L=3
%   ddc.mfac.MFACController('Ly',2,'Lu',2,'PhiInit',0.01*ones(4,1), ...
%                           'Rho',0.3*ones(4,1))    % FFDL
%
% See also ddc.common, ddc.str, ddc.deepc.
