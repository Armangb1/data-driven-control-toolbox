function [R, T, AchievedBm, warnings] = buildController(Rprime, Bplus, Bminus, Ao, Bm, tol, autoAugmentBm, S)
% buildController  Construct final controller polynomials R, T and verify causality.
%
%   [R, T, AchievedBm, warnings] = ...
%       ddc.str.mdpp.buildController(Rprime, Bplus, Bminus, Ao, Bm, tol, autoAugmentBm, S)
%
%   Given the Diophantine solution R' and the B-factorization, builds:
%     R = R' * Bplus        (controller denominator)
%     T = Ao * Bmprime      (feedforward polynomial; Bm = Bminus * Bmprime)
%
%   Causality is verified structurally before returning: deg(R) >= deg(S)
%   (when S is supplied) and deg(R) >= deg(T), and the leading coefficient of
%   R must be nonzero. This is the safety net that catches upstream
%   bookkeeping errors (wrong deg(Ao), mis-sized Bmprime, etc.) that would
%   otherwise produce a non-realizable implicit control law R*u = T*uc - S*y.
%
%   Inputs:
%     Rprime        - Diophantine solution numerator
%     Bplus         - Cancelable part of plant B (monic)
%     Bminus        - Non-cancelable part of plant B
%     Ao            - Observer polynomial
%     Bm            - Desired closed-loop reference model numerator
%     tol           - Numerical tolerance
%     autoAugmentBm - If true, automatically augment Bm with Bminus factors
%     S             - Optional feedback polynomial (for full causality check)
%
%   Outputs:
%     R             - Controller denominator polynomial
%     T             - Controller feedforward polynomial
%     AchievedBm    - Bm actually realized (may differ from input if auto-augmented)
%     warnings      - Cell array of warning strings

    if nargin < 7 || isempty(autoAugmentBm), autoAugmentBm = false; end
    if nargin < 6 || isempty(tol),           tol = 1e-9;            end
    if nargin < 8,                           S = [];                end

    Bplus  = ddc.str.mdpp.trimLeadingZeros(Bplus(:).');
    Bminus = ddc.str.mdpp.trimLeadingZeros(Bminus(:).');
    Ao     = ddc.str.mdpp.trimLeadingZeros(Ao(:).');
    Bm     = ddc.str.mdpp.trimLeadingZeros(Bm(:).');
    Rprime = ddc.str.mdpp.trimLeadingZeros(Rprime(:).');

    warnings = {};
    AchievedBm = Bm;

    % --- Derive Bmprime from Bm = Bminus * Bmprime ---
    % A scalar Bminus means no non-cancelable zeros: any Bm is realizable.
    if isscalar(Bminus)
        % Bminus = b0 (constant): no non-cancelable zeros, so any Bm works.
        Bmprime = Bm / Bminus(1);
        AchievedBm = Bm;
    else
        % Bminus is a genuine polynomial with degree >= 1. Bm must contain
        % every non-cancelable plant zero: Bm = Bminus * Bmprime exactly.
        if length(Bm) >= length(Bminus)
            [Bmprime, remainder] = deconv(Bm, Bminus);
            Bmprime   = ddc.str.mdpp.trimLeadingZeros(Bmprime);
            remainder = ddc.str.mdpp.trimLeadingZeros(remainder);

            if norm(remainder, Inf) > tol
                if autoAugmentBm
                    % Auto-augment rule: choose Bmprime as a flat (degree-)
                    % polynomial scaled so that polyval(AchievedBm, 1) equals
                    % the DC gain of the user's requested Bm. This preserves
                    % the requested steady-state gain while guaranteeing the
                    % non-cancelable zeros appear in the reference model.
                    nBmprime = length(Bm) - length(Bminus) + 1;
                    if nBmprime < 1, nBmprime = 1; end
                    Bmprime = ones(1, nBmprime);
                    AchievedBm = conv(Bminus, Bmprime);
                    dcTarget = polyval(Bm, 1);
                    dcActual = polyval(AchievedBm, 1);
                    if abs(dcActual) > tol
                        Bmprime = Bmprime * (dcTarget / dcActual);
                        AchievedBm = conv(Bminus, Bmprime);
                    end
                    warnings{end+1} = sprintf( ...
                        ['AutoAugmentBm: Bm replaced. Bminus does not divide the requested Bm ', ...
                         '(remainder norm = %.2e). AchievedBm = Bminus*Bmprime with preserved ', ...
                         'DC gain.'], norm(remainder, Inf));
                else
                    error('MDPP:IncompatibleReferenceModel', ...
                        ['Bminus does not divide Bm (remainder norm = %.2e). ', ...
                         'The reference model numerator Bm must contain all non-cancelable ', ...
                         'plant zeros (Bminus) as factors. Set AutoAugmentBm=true to allow ', ...
                         'automatic augmentation.'], norm(remainder, Inf));
                end
            else
                Bmprime = ddc.str.mdpp.trimLeadingZeros(Bmprime);
                AchievedBm = Bm;
            end
        else
            if autoAugmentBm
                nBmprime = max(length(Bm) - length(Bminus) + 1, 1);
                Bmprime = ones(1, nBmprime);
                AchievedBm = conv(Bminus, Bmprime);
                dcTarget = polyval(Bm, 1);
                dcActual = polyval(AchievedBm, 1);
                if abs(dcActual) > tol
                    Bmprime = Bmprime * (dcTarget / dcActual);
                    AchievedBm = conv(Bminus, Bmprime);
                end
                warnings{end+1} = ...
                    'AutoAugmentBm: Bm shorter than Bminus; augmented with gain-preserving Bmprime.';
            else
                error('MDPP:IncompatibleReferenceModel', ...
                    'Bm (deg %d) is shorter than Bminus (deg %d).', ...
                    length(Bm)-1, length(Bminus)-1);
            end
        end
    end

    % --- Build the final controller polynomials ---
    R = ddc.str.mdpp.trimLeadingZeros(conv(Rprime, Bplus));
    T = ddc.str.mdpp.trimLeadingZeros(conv(Ao, Bmprime));

    % --- Causality verification before returning ---
    nR = length(R) - 1;
    nT = length(T) - 1;
    okT = nR >= nT;

    nS = -Inf;
    okS = true;
    if ~isempty(S)
        S = ddc.str.mdpp.trimLeadingZeros(S(:).');
        nS  = length(S) - 1;
        okS = nR >= nS;
    end

    leadingR = R(1);
    okLead = abs(leadingR) > tol;

    if ~(okT && okS && okLead)
        error('MDPP:NonCausalController', ...
            ['Controller is non-causal: deg(R)=%d, deg(S)=%d, deg(T)=%d, ', ...
             'leading coeff of R = %.4e. Check compatibility conditions ', ...
             '(deg(Ao), deg(Bm), Bminus | Bm).'], ...
            nR, nS, nT, leadingR);
    end