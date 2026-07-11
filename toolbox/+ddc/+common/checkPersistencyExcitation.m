function [isPE, r, rRequired] = checkPersistencyExcitation(u, order, m)
%CHECKPERSISTENCYEXCITATION Verify persistency of excitation of order L.
%   [ISPE, R, RREQUIRED] = ddc.common.checkPersistencyExcitation(U, ORDER, M)
%   builds the Hankel matrix of U with ORDER block rows and checks whether
%   it has full row rank, i.e. whether U is persistently exciting of
%   order ORDER, as required by Willems' Fundamental Lemma for DeePC.
%
%   U      - m-by-T (or vector) input data sequence.
%   ORDER  - required persistency-of-excitation order, typically
%            ORDER = L + n + 1 where L is the prediction/Hankel depth
%            used by DeePC and n is an upper bound on the system order.
%   M      - (optional) input dimension, used only for the required-rank
%            calculation; inferred from U if omitted.
%
%   Outputs:
%   ISPE       - true if U is persistently exciting of order ORDER.
%   R          - rank of the Hankel matrix H_ORDER(U).
%   RREQUIRED  - required rank = M*ORDER (full row rank condition).
%
%   See also ddc.common.HankelBuilder, ddc.deepc.deepcDesign.

    if isvector(u)
        u = u(:).';
    end
    if nargin < 3 || isempty(m)
        m = size(u, 1);
    end

    H = ddc.common.HankelBuilder(u, order);
    r = rank(H);
    rRequired = m * order;
    isPE = (r >= rRequired);
end
