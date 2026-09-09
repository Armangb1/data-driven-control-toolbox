function [Rprime, S, warnings] = solveDiophantine(A, Bminus, Ao, Am, tol, condWarnThreshold)
% solveDiophantine  Solve the Diophantine equation A*R' + Bminus*S = Ao*Am.
%
%   [Rprime, S, warnings] = ddc.str.mdpp.solveDiophantine(A, Bminus, Ao, Am, tol, condWarnThreshold)
%
%   Finds the minimal-degree polynomials R'(q) and S(q) with
%   deg(S) < deg(A) and deg(R') = deg(Ao*Am) - deg(A) such that
%   A*R' + Bminus*S = Ao*Am. This is the core algebraic step in
%   pole-placement design: it computes the controller numerator and
%   feedback polynomial from the plant and the desired closed-loop poles.
%
%   The Sylvester matrix of (A, Bminus) is built explicitly and solved with
%   backslash. Coprimeness of A and Bminus is verified (a singular Sylvester
%   matrix means no finite-degree causal solution exists, typically because a
%   non-cancelable plant zero coincides with a plant pole).
%
%   Optional warnings cell output reports ill-conditioning (near-common roots)
%   so the caller can surface it without failing the design.
%
%   Inputs:
%     A        - Plant denominator (monic, descending powers)
%     Bminus   - Non-cancelable part of plant numerator
%     Ao       - Observer polynomial
%     Am       - Desired closed-loop characteristic polynomial
%     tol      - Numerical tolerance (default 1e-9)
%     condWarnThreshold - Warn if cond(Sylvester) exceeds this (default 1e8)
%
%   Outputs:
%     Rprime   - Controller numerator ('=', deg(Ao*Am) - deg(A))
%     S        - Feedback polynomial (deg S = deg A - 1)
%     warnings - Cell array of strings (ill-conditioning, etc.)

    if nargin < 6 || isempty(condWarnThreshold), condWarnThreshold = 1e8; end
    if nargin < 5 || isempty(tol),                tol = 1e-9;             end

    A      = ddc.str.mdpp.trimLeadingZeros(A(:).');
    Bminus = ddc.str.mdpp.trimLeadingZeros(Bminus(:).');
    Ao     = ddc.str.mdpp.trimLeadingZeros(Ao(:).');
    Am     = ddc.str.mdpp.trimLeadingZeros(Am(:).');

    nA = length(A) - 1;       % deg(A)
    m  = length(Bminus) - 1;  % deg(Bminus)

    % Desired closed-loop characteristic polynomial.
    C = conv(Ao, Am);
    nC = length(C) - 1;

    % Minimal-degree solution: deg(R') = deg(C) - deg(A), deg(S) = deg(A) - 1.
    nRprime = nC - nA;
    if nRprime < 0
        error('MDPP:InvalidInput', ...
            'deg(Ao*Am) = %d < deg(A) = %d; no solution of nonnegative degree exists.', ...
            nC, nA);
    end
    nS = nA - 1;

    % The Sylvester matrix is square of size (deg C + 1).
    nRows = nC + 1;
    nCols = (nRprime + 1) + (nS + 1);
    if nRows ~= nCols
        error('MDPP:InvalidInput', ...
            'Sylvester matrix not square (%dx%d). Check degree bookkeeping.', ...
            nRows, nCols);
    end

    % Build the Sylvester matrix for (A, Bminus).
    % Row k matches the coefficient of q^k in A*R' + Bminus*S against C,
    % for k = deg(C) down to 0.
    % Columns are [r0, r1, ..., r_{nRprime}, s0, s1, ..., s_{nS}].
    % With A = [A0..A_{nA}] (A_i = coeff of q^{nA-i}) and
    % R' = [r0..r_{nRprime}] (r_i = coeff of q^{nRprime-i}),
    % the product term A_p * r_i has power nA - p + nRprime - i = k, so
    % p = nA + nRprime - k - i, and i ranges over (nRprime-k)..(nA+nRprime-k).
    % Similarly for Bminus = [B0..B_m] and S = [s0..s_{nS}], term B_p * s_j
    % has power m - p + nS - j = k, so p = m + nS - k - j, j in (nS-k)..(m+nS-k).

    Syl = zeros(nRows, nCols);

    for rowIdx = 1:nRows
        k = nC - (rowIdx - 1);

        % R' contributions.
        iLo = max(0, nRprime - k);
        iHi = min(nRprime, nA + nRprime - k);
        for i = iLo:iHi
            p = nA + nRprime - k - i;
            Syl(rowIdx, i + 1) = A(p + 1);
        end

        % S contributions.
        jLo = max(0, nS - k);
        jHi = min(nS, m + nS - k);
        for j = jLo:jHi
            p = m + nS - k - j;
            Syl(rowIdx, nRprime + 2 + j) = Bminus(p + 1);
        end
    end

    % Right-hand side.
    b = C(:);
    if length(b) ~= nRows
        error('MDPP:InvalidInput', 'RHS length mismatch (%d vs %d rows).', length(b), nRows);
    end

    % Coprimeness check: A and Bminus must not share a root (no common factor).
    rcondVal = rcond(Syl);
    if isnan(rcondVal) || ~isfinite(rcondVal) || rcondVal < tol
        commonRoots = findCommonRoots(A, Bminus, tol);
        error('MDPP:NotCoprime', ...
            ['A and Bminus are not coprime (rcond(Sylvester) = %.2e). ', ...
             'Approximate common root(s): %s. ', ...
             'No finite-degree causal controller exists for the requested closed-loop poles.'], ...
            rcondVal, formatRoots(commonRoots));
    end

    % Condition check: near-common roots make the design numerically fragile.
    warnings = {};
    condVal = cond(Syl);
    if condVal > condWarnThreshold
        warnings{end+1} = sprintf( ...
            'Sylvester matrix ill-conditioned (cond = %.2e > %.2e); design may be numerically fragile.', ...
            condVal, condWarnThreshold);
    end

    % Solve for the unknown coefficients.
    unknowns = Syl \ b;

    Rprime = unknowns(1:nRprime + 1).';
    S      = unknowns(nRprime + 2:end).';

    % Verify the residual. Pad with leading zeros so coefficients align by
    % highest power (descending-coefficient convention).
    lhsR = conv(A, Rprime);
    lhsB = conv(Bminus, S);
    lhsLen = max(length(lhsR), length(lhsB));
    lhs = [zeros(1, lhsLen - length(lhsR)), lhsR] + ...
          [zeros(1, lhsLen - length(lhsB)), lhsB];
    maxLen = max(length(lhs), length(C));
    lhsPad = [zeros(1, maxLen - length(lhs)), lhs];
    cPad   = [zeros(1, maxLen - length(C)),   C];
    residual = norm(lhsPad - cPad, Inf);

    if residual > tol
        error('MDPP:DiophantineResidual', ...
            'Diophantine equation residual = %.2e exceeds tolerance %.2e.', ...
            residual, tol);
    end
end

function commonRoots = findCommonRoots(A, B, tol)
% findCommonRoots  Approximate common roots of two polynomials.
    rA = roots(A(:).');
    rB = roots(B(:).');
    commonRoots = [];
    for i = 1:numel(rA)
        for j = 1:numel(rB)
            if abs(rA(i) - rB(j)) < tol
                commonRoots(end+1) = rA(i); %#ok<AGROW>
                break;
            end
        end
    end
end

function s = formatRoots(r)
% formatRoots  Format a list of (possibly complex) roots for display.
    if isempty(r)
        s = '(none found)';
        return;
    end
    parts = cell(1, numel(r));
    for k = 1:numel(r)
        if abs(imag(r(k))) < 1e-12
            parts{k} = sprintf('%.6g', real(r(k)));
        else
            parts{k} = sprintf('%.6g%+.6gi', real(r(k)), imag(r(k)));
        end
    end
    s = strjoin(parts, ', ');
end