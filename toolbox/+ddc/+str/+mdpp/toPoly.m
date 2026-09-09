function [num, den] = toPoly(P)
% TOPOLY  Convert a polynomial or transfer-function object to num/den row vectors.
%
%   [num, den] = ddc.str.mdpp.toPoly(P)
%
%   P may be any of:
%     * a numeric polynomial row vector  -> num = P, den = P (as-is)
%     * a SISO tf object                 -> numerator/denominator extracted
%     * a SISO zpk object                -> converted via tf
%     * a SISO ss object                 -> converted via tf
%
%   The returned vectors follow the toolbox convention: row vectors in
%   descending powers of q (z), with leading zeros trimmed. A continuous-time
%   model is accepted but produces a warning, because the MDPP stability
%   semantics assume a discrete-time (unit-disk) model — convert with c2d
%   first.
%
%   Requires the Control System Toolbox only when a tf/zpk/ss object is
%   actually passed; numeric inputs work in base MATLAB.
%
%   Throws 'MDPP:InvalidInput' for multi-output/multi-input models and for
%   unsupported input types.

    if isa(P, 'tf')
        if numel(P.Numerator) > 1 || numel(P.Denominator) > 1
            error('MDPP:InvalidInput', ...
                'toPoly requires SISO models; received a %d-output transfer function.', ...
                numel(P.Numerator));
        end
        num = P.Numerator{1};
        den = P.Denominator{1};
        warnIfContinuous(P.Ts);
    elseif isa(P, 'zpk') || isa(P, 'ss')
        % zpk/ss only exist if the Control System Toolbox is installed, so
        % converting them via tf() is guaranteed to be available here.
        t = tf(P);
        if numel(t.Numerator) > 1 || numel(t.Denominator) > 1
            error('MDPP:InvalidInput', ...
                'toPoly requires SISO models.');
        end
        num = t.Numerator{1};
        den = t.Denominator{1};
        warnIfContinuous(t.Ts);
    elseif isnumeric(P) || islogical(P)
        num = P;
        den = P;
    else
        error('MDPP:InvalidInput', ...
            'Unsupported input type ''%s''. Provide a polynomial row vector or a SISO tf/zpk/ss object.', ...
            class(P));
    end

    num = ddc.str.mdpp.trimLeadingZeros(num(:).');
    den = ddc.str.mdpp.trimLeadingZeros(den(:).');
end

function warnIfContinuous(Ts)
    if isscalar(Ts) && Ts == 0
        warning('MDPP:ContinuousModel', ...
            ['Continuous-time model provided. MDPP design assumes a discrete-time ', ...
             '(unit-disk) stability model; convert with c2d first for correct results.']);
    end
end