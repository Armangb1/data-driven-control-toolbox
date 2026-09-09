function p = trimLeadingZeros(p)
% trimLeadingZeros  Remove leading zero coefficients from a polynomial row vector.
%
%   p = ddc.str.mdpp.trimLeadingZeros(p)
%
%   Preserves at least one element (never returns []). A zero polynomial
%   [0 0 ... 0] is returned as [0].

    idx = find(p ~= 0, 1, 'first');
    if isempty(idx)
        p = 0;
    else
        p = p(idx:end);
    end
end
