function [Bplus, Bminus] = factorB(B, stabilityMargin, dampingMin, tol)
% factorB  Factor B(q) = Bplus(q) * Bminus(q) into cancelable and non-cancelable parts.
%
%   [Bplus, Bminus] = ddc.str.mdpp.factorB(B, stabilityMargin, dampingMin, tol)
%
%   B must be a row vector of polynomial coefficients in descending powers.
%   Bplus is monic and collects the factors whose roots are cancelable
%   (stable and well-damped); Bminus absorbs the remaining factors together
%   with the leading coefficient of B, so that conv(Bplus, Bminus) == B.
%
%   A zero of B is classified as cancelable iff:
%     1) abs(root) < stabilityMargin  (strictly inside the stability region), AND
%     2) if dampingMin is supplied, the mapped continuous-time damping ratio
%        zeta = -cos(angle(log(root))) satisfies zeta >= dampingMin.
%   Roots on or outside the stability boundary (within tol) are always
%   non-cancelable regardless of dampingMin.
%
%   This separation is structurally necessary: canceling a non-minimum-phase
%   or poorly-damped zero would place a matching unstable or nearly unstable
%   pole into the controller, destroying internal stability even though the
%   nominal input-output transfer function looks correct.

    if nargin < 4 || isempty(tol),           tol = 1e-9;     end
    if nargin < 3 || isempty(dampingMin),     dampingMin = []; end
    if nargin < 2 || isempty(stabilityMargin), stabilityMargin = 1; end

    B = ddc.str.mdpp.trimLeadingZeros(B(:).');
    if isscalar(B) && B == 0
        error('MDPP:InvalidInput', 'B polynomial is zero.');
    end
    if any(~isfinite(B))
        error('MDPP:InvalidInput', 'B polynomial contains non-finite coefficients.');
    end

    % Scalar B (no roots): all zeros are trivially cancelable.
    % Keep Bplus monic (=1) and put the leading coefficient into Bminus so
    % conv(Bplus, Bminus) == B.
    if isscalar(B)
        Bplus  = 1;
        Bminus = B(1);
        return;
    end

    r = roots(B);
    cancelable = isCancelableRoot(r, stabilityMargin, dampingMin, tol);

    % Partition the roots into cancelable / non-cancelable sets, pairing
    % complex-conjugate roots explicitly so that poly() on each subset
    % produces real (or near-real) coefficient vectors.
    used = false(size(r));
    cancelRoots    = [];
    nonCancelRoots = [];

    for i = 1:numel(r)
        if used(i), continue; end
        if isreal(r(i)) || abs(imag(r(i))) < tol
            used(i) = true;
            if cancelable(i)
                cancelRoots(end+1) = r(i); %#ok<AGROW>
            else
                nonCancelRoots(end+1) = r(i); %#ok<AGROW>
            end
        else
            % Find the conjugate partner (same cancelability, r_j = conj(r_i)).
            partnerIdx = find(~used & (cancelable == cancelable(i)) & ...
                abs(r - conj(r(i))) < tol, 1);
            if isempty(partnerIdx)
                error('MDPP:InvalidInput', ...
                    'Cannot pair complex-conjugate root %.6g%+.6gi.', ...
                    real(r(i)), imag(r(i)));
            end
            used(i) = true;
            used(partnerIdx) = true;
            % Verify the pair product reconstructs real coefficients.
            pairPoly = conv([1, -r(i)], [1, -r(partnerIdx)]);
            if max(abs(imag(pairPoly))) > tol
                error('MDPP:InvalidInput', ...
                    'Complex-conjugate pair product has non-real coefficients.');
            end
            if cancelable(i)
                cancelRoots = [cancelRoots, r(i), r(partnerIdx)]; %#ok<AGROW>
            else
                nonCancelRoots = [nonCancelRoots, r(i), r(partnerIdx)]; %#ok<AGROW>
            end
        end
    end

    % Build polynomials from each root subset and force real coefficients.
    if ~isempty(cancelRoots)
        Bplus = real(poly(cancelRoots));
        if max(abs(imag(Bplus))) > tol
            error('MDPP:FactorizationMismatch', ...
                'Bplus reconstruction produced non-real coefficients (imag = %.2e).', ...
                max(abs(imag(Bplus))));
        end
    else
        Bplus = 1;
    end
    if ~isempty(nonCancelRoots)
        Bminus = real(poly(nonCancelRoots));
        if max(abs(imag(Bminus))) > tol
            error('MDPP:FactorizationMismatch', ...
                'Bminus reconstruction produced non-real coefficients (imag = %.2e).', ...
                max(abs(imag(Bminus))));
        end
    else
        Bminus = 1;
    end

    % Make Bplus monic.
    Bplus = Bplus / Bplus(1);

    % Scale Bminus so that conv(Bplus, Bminus) reproduces B exactly.
    reconstructed = conv(Bplus, Bminus);
    scale = B(1) / reconstructed(1);
    Bminus = Bminus * scale;

    % Verify factorization accuracy.
    if norm(conv(Bplus, Bminus) - B, Inf) > tol
        error('MDPP:FactorizationMismatch', ...
            'conv(Bplus, Bminus) does not reproduce B (residual = %.2e).', ...
            norm(conv(Bplus, Bminus) - B, Inf));
    end
end

function cancelable = isCancelableRoot(r, stabilityMargin, dampingMin, tol)
% isCancelableRoot  Classify a (set of) roots as cancelable.
%
%   A root is cancelable when it lies strictly inside the stability region
%   AND (optionally) meets a minimum damping-ratio requirement. Roots on or
%   outside the stability boundary are never cancelable.
%
%   The damping ratio is computed in the discrete-time stability sense used
%   by this toolbox (unit disk): map z to s = log(z) (sampling period T = 1)
%   and use zeta = -real(s)/abs(s) = -cos(angle(s)). For z = e^(sT) this is
%   the damping ratio of the corresponding continuous-time pole.

    cancelable = abs(r) < (stabilityMargin - tol);

    if ~isempty(dampingMin)
        s = log(r);
        zeta = zeros(size(r));
        for k = 1:numel(r)
            if abs(s(k)) < eps
                zeta(k) = 0;   % root at z = 1: undamped
            else
                zeta(k) = -real(s(k)) / abs(s(k));
            end
        end
        cancelable = cancelable & (zeta >= dampingMin - tol);
    end
end