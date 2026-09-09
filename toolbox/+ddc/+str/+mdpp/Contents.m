% +STR.+MDPP  Minimum-Degree Pole Placement (discrete-time, Algorithm 3.1)
%
% Design engine for pole-placement self-tuning regulators: given a plant
% model and the desired closed-loop / observer polynomials, it computes the
% polynomial feedback controller R*u = T*uc - S*y.
%
% Functions.
%   mdpp_design         - Main design entry point (Algorithm 3.1).
%   toPoly              - Convert a polynomial, tf, zpk, or ss into num/den row vectors.
%   factorB             - Split B = Bplus * Bminus into cancelable/non-cancelable parts.
%   solveDiophantine    - Solve A*R' + Bminus*S = Ao*Am via a Sylvester matrix.
%   buildController     - Assemble R, T and verify causality.
%   trimLeadingZeros    - Strip leading zero coefficients from a polynomial row vector.
%
% All polynomials use row vectors in descending powers (MATLAB convention).
% Errors are thrown as MException with identifiers under 'MDPP:*'.
%
% Model objects (tf / zpk / ss):
%   Every polynomial argument (A, B, Am, Bm, Ao) may instead be a SISO tf,
%   zpk, or ss object from the Control System Toolbox.  B / Bm take the model
%   numerator (descending powers of q), A / Am / Ao take the denominator.
%   If A is a model object it carries both, so B may be passed as [] and is
%   derived automatically.  Continuous-time objects are accepted but raise a
%   'MDPP:ContinuousModel' warning, since the stability semantics are
%   discrete-time (unit disk) -- convert with c2d first.  These conversions
%   require the Control System Toolbox only when an object is actually
%   passed; plain polynomial inputs work in base MATLAB.
%
% Error identifiers.
%   MDPP:InvalidInput                - Non-monic A, empty/non-finite polynomials, unsupported type.
%   MDPP:FactorizationMismatch       - conv(Bplus, Bminus) != B.
%   MDPP:IncompatibleReferenceModel  - Degree mismatch, or B- does not divide Bm.
%   MDPP:NotCoprime                  - A and B- share a root -> singular Sylvester matrix.
%   MDPP:DiophantineResidual         - Solved equation residual exceeds tolerance.
%   MDPP:NonCausalController         - deg(R) < deg(S) or leading coeff(R) ~= 0.
%   MDPP:ContinuousModel             - Warning: continuous-time object passed for discrete-time design.
%
% See also ddc.str.IndirectSTRController, ddc.str.DirectSTRController.