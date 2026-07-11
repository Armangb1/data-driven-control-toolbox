% +COMMON  Shared utilities for the Data-Driven Control Toolbox
%
% Functions.
%   HankelBuilder               - Build a (block) Hankel matrix from data.
%   checkPersistencyExcitation  - Verify persistency-of-excitation rank condition.
%
% Classes (matlab.System, usable as MATLAB objects or Simulink blocks).
%   DataBuffer                  - Sliding-window FIFO signal buffer.
%   RLSEstimator                - Recursive least squares parameter estimator.
%   ExcitationSignalGenerator   - PRBS / random excitation signal source.
%
% See also ddc.deepc, ddc.mfac, ddc.str, ddc.ufc, ddc.spsa, ddc.vrft.
