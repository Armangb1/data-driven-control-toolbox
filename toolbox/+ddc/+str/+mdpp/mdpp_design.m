function result = mdpp_design(A, B, Am, Bm, Ao, varargin)
% MDPP_DESIGN  Minimum-degree pole placement (Algorithm 3.1).
%
%   result = ddc.str.mdpp.mdpp_design(A, B, Am, Bm, Ao, Name, Value, ...)
%
%   Designs a polynomial-based feedback controller R*u = T*uc - S*y for the
%   plant A(q)*y = B(q)*u such that the closed-loop characteristic polynomial
%   is Bplus(q)*Ao(q)*Am(q), where Am is the desired tracking dynamics and Ao
%   the observer polynomial. This implements the discrete-time pole placement
%   design of Astron and Wittenmark, "Adaptive Control", Algorithm 3.1.
%
%   Inputs (each may be a polynomial row vector, or a SISO tf/zpk/ss object
%   from the Control System Toolbox; see ddc.str.mdpp.toPoly for the extraction rules):
%     A  - Plant denominator (monic, descending powers: [1 a1 a2 ... an])
%          If A is a tf/zpk/ss object and B is empty, B is derived from the
%          model numerator.
%     B  - Plant numerator (descending powers, may carry leading zeros).
%          may be omitted ([]) when A is a tf/zpk/ss object.
%     Am - Desired closed-loop characteristic polynomial (deg = deg(A));
%          denominator part of a tf/zpk/ss is used.
%     Bm - Desired reference model numerator (deg = deg(B));
%          numerator part of a tf/zpk/ss is used.
%     Ao - Observer polynomial (deg = deg(A) - deg(Bplus) - 1);
%          denominator part of a tf/zpk/ss is used.
%
%   Name-Value options:
%     'StabilityMargin'  (default 1.0)   abs(root) < margin => stable
%     'DampingMin'       (default [])    optional min damping-ratio threshold
%     'AutoAugmentBm'     (default false) auto-augment Bm with Bminus factors
%     'Tol'               (default 1e-9) numerical tolerance
%     'CondWarnThreshold' (default 1e8)  warn if Sylvester cond exceeds this
%
%   Output:
%     result - struct with fields
%       Bplus, Bminus       — factors of B
%       Rprime, S           — Diophantine solution
%       R, T                — final controller polynomials
%       AchievedBm          — Bm actually realized
%       IsCausal            — logical (true unless impossible, in which case
%                              an MDPP:NonCausalController error is thrown)
%       Warnings            — cellstr of non-fatal warnings
%
%   Throws MException with identifiers under 'MDPP:*' on unrecoverable
%   problems (see README for the identifier catalog).

    % --- Name-Value options ---
    p = inputParser;
    p.addParameter('StabilityMargin', 1.0);
    p.addParameter('DampingMin', []);
    p.addParameter('AutoAugmentBm', false);
    p.addParameter('Tol', 1e-9);
    p.addParameter('CondWarnThreshold', 1e8);
    p.parse(varargin{:});
    opts = p.Results;

    tol      = opts.Tol;
    margin   = opts.StabilityMargin;
    dampMin  = opts.DampingMin;
    autoAug  = opts.AutoAugmentBm;
    condWarn = opts.CondWarnThreshold;

    % --- Convert tf/zpk/ss inputs to polynomial row vectors ---
    % A is denominator-like: if it is a model object it carries both A (den)
    % and B (num). B may then be omitted ([]) and is derived from the model.
    isPlantModel = isa(A, 'tf') || isa(A, 'zpk') || isa(A, 'ss');
    [A_num, A_den] = ddc.str.mdpp.toPoly(A);
    A = A_den;
    if isempty(B)
        if isPlantModel
            B = A_num;
        else
            error('MDPP:InvalidInput', ...
                'B must be provided when A is a plain polynomial (B = [] is only allowed when A is a tf/zpk/ss model).');
        end
    else
        [B_num, ~] = ddc.str.mdpp.toPoly(B);
        B = B_num;
    end
    [~, Am_den] = ddc.str.mdpp.toPoly(Am);
    [Bm_num, ~] = ddc.str.mdpp.toPoly(Bm);
    [~, Ao_den] = ddc.str.mdpp.toPoly(Ao);
    Am = Am_den;
    Bm = Bm_num;
    Ao = Ao_den;

    % --- Length / coefficient validation (before any algebra) ---
    A  = ddc.str.mdpp.trimLeadingZeros(A(:).');
    B  = ddc.str.mdpp.trimLeadingZeros(B(:).');
    Am = ddc.str.mdpp.trimLeadingZeros(Am(:).');
    Bm = ddc.str.mdpp.trimLeadingZeros(Bm(:).');
    Ao = ddc.str.mdpp.trimLeadingZeros(Ao(:).');

    if isempty(A) || isempty(B) || isempty(Am) || isempty(Bm) || isempty(Ao)
        error('MDPP:InvalidInput', 'All polynomial inputs must be non-empty.');
    end
    if any(~isfinite(A)) || any(~isfinite(B)) || any(~isfinite(Am)) || ...
       any(~isfinite(Bm)) || any(~isfinite(Ao))
        error('MDPP:InvalidInput', 'Polynomial inputs contain non-finite coefficients.');
    end
    if B == 0
        error('MDPP:InvalidInput', 'B polynomial is zero.');
    end

    % A must be monic; auto-normalize otherwise (with a warning).
    if abs(A(1) - 1) > tol
        warning('MDPP:InvalidInput', ...
            'A is not monic (A(1) = %.6g). Auto-normalizing by dividing by A(1).', A(1));
        A = A / A(1);
        A = ddc.str.mdpp.trimLeadingZeros(A);
    end

    nA  = length(A) - 1;
    nAm = length(Am) - 1;
    nB  = length(B) - 1;
    nBm = length(Bm) - 1;

    % --- Compatibility: deg(Am) == deg(A) ---
    if nAm ~= nA
        error('MDPP:IncompatibleReferenceModel', ...
            'deg(Am) = %d must equal deg(A) = %d.', nAm, nA);
    end

    % --- Compatibility: deg(Bm) == deg(B) ---
    if nBm ~= nB
        error('MDPP:IncompatibleReferenceModel', ...
            'deg(Bm) = %d must equal deg(B) = %d.', nBm, nB);
    end

    % --- Step 1: factor B = Bplus * Bminus ---
    [Bplus, Bminus] = ddc.str.mdpp.factorB(B, margin, dampMin, tol);

    nBplus = length(Bplus) - 1;

    % --- Compatibility: deg(Ao) == deg(A) - deg(Bplus) - 1 ---
    nAo = length(Ao) - 1;
    expectedAo = nA - nBplus - 1;
    if nAo ~= expectedAo
        error('MDPP:IncompatibleReferenceModel', ...
            'deg(Ao) = %d must equal deg(A) - deg(Bplus) - 1 = %d - %d - 1 = %d.', ...
            nAo, nA, nBplus, expectedAo);
    end

    warnings = {};

    % --- Compatibility: Bminus must divide Bm (fail fast, before Step 2) ---
    % If Bminus has any non-trivial factors, the reference model numerator must
    % contain every non-cancelable plant zero; otherwise the closed-loop zeros
    % cannot be realized (they cannot be canceled).
    if ~isscalar(Bminus)
        if length(Bm) >= length(Bminus)
            [~, bmRem] = deconv(Bm, Bminus);
            if norm(bmRem, Inf) > tol && ~autoAug
                error('MDPP:IncompatibleReferenceModel', ...
                    ['Bminus does not divide Bm (remainder norm = %.2e). ', ...
                     'The reference model numerator Bm must contain all non-cancelable ', ...
                     'plant zeros (Bminus) as factors (Bm = Bminus * Bmprime). ', ...
                     'Set AutoAugmentBm=true to allow automatic augmentation.'], ...
                    norm(bmRem, Inf));
            end
        elseif ~autoAug
            error('MDPP:IncompatibleReferenceModel', ...
                'Bm (deg %d) is shorter than Bminus (deg %d).', ...
                length(Bm)-1, length(Bminus)-1);
        end
    end

    % --- Step 2: solve the Diophantine equation ---
    if isscalar(Bminus)
        % Section 8 fast path: all zeros cancelable (Bminus = b0 scalar).
        % The equation A*R' + b0*S = Ao*Am reduces to polynomial division:
        % R' = quotient(Ao*Am / A), S = remainder(Ao*Am / A) / b0.
        b0 = Bminus(1);
        Bplus = B / b0;              % re-derive monic Bplus
        Bplus = Bplus / Bplus(1);

        C = conv(Ao, Am);
        [Rprime, rem] = deconv(C, A);
        Rprime = ddc.str.mdpp.trimLeadingZeros(Rprime);

        S = ddc.str.mdpp.trimLeadingZeros(rem) / b0;
        if length(S) < nA
            S = [zeros(1, nA - length(S)), S];
        end
    else
        [Rprime, S, diophWarnings] = ddc.str.mdpp.solveDiophantine(A, Bminus, Ao, Am, tol, condWarn);
        warnings = [warnings, diophWarnings];
        Rprime = ddc.str.mdpp.trimLeadingZeros(Rprime);
        S      = ddc.str.mdpp.trimLeadingZeros(S);
    end

    % --- Step 3: build the controller and verify causality ---
    [R, T, AchievedBm, ctrlWarnings] = ddc.str.mdpp.buildController( ...
        Rprime, Bplus, Bminus, Ao, Bm, tol, autoAug, S);
    warnings = [warnings, ctrlWarnings];

    R = ddc.str.mdpp.trimLeadingZeros(R);
    T = ddc.str.mdpp.trimLeadingZeros(T);
    % S is kept at its canonical degree (length nA, deg nA - 1) so that the
    % controller representation and the causality check use the intended
    % MDPP degrees (deg R == deg S == deg T == deg A - 1).

    % --- Explicit causality verification (safety net) ---
    nR = length(R) - 1;
    nS = length(S) - 1;
    nT = length(T) - 1;
    isCausal = (nR >= nS) && (nR >= nT) && (abs(R(1)) > tol);
    if ~isCausal
        error('MDPP:NonCausalController', ...
            'Controller is non-causal: deg(R)=%d, deg(S)=%d, deg(T)=%d, R(1)=%.4e.', ...
            nR, nS, nT, R(1));
    end

    % --- Assemble the result ---
    result.Bplus      = Bplus;
    result.Bminus     = Bminus;
    result.Rprime     = Rprime;
    result.S          = S;
    result.R          = R;
    result.T          = T;
    result.AchievedBm = AchievedBm;
    result.IsCausal   = true;
    result.Warnings   = warnings(:);
end