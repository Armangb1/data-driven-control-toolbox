function sys = parseBasisExpression(expression, Ts)
%PARSEBASISEXPRESSION Parse a discrete-time transfer function expression.
%   SYS = ddc.vrft.parseBasisExpression(EXPRESSION, TS) evaluates a
%   restricted arithmetic EXPRESSION written in terms of the discrete
%   shift operator z and the sample time Ts, and returns the resulting
%   discrete-time tf object at sample time TS. Used by the Controller
%   Structure and Prefilter panels of ddc.vrft.VRFTDesignApp to let users
%   type custom basis entries / prefilters such as "Ts/(z-1)" or
%   "(z-1)/(Ts*z)" without exposing a general-purpose eval of arbitrary
%   MATLAB code.
%
%   EXPRESSION - a character vector or scalar string containing only
%                digits, decimal points, the identifiers z and Ts,
%                whitespace, parentheses, and the operators
%                + - * / ^ . No other characters, function calls, or
%                identifiers are permitted; anything else raises an
%                error rather than being evaluated.
%   TS         - sample time in seconds (> 0).
%
%   SYS is a discrete-time tf object with SYS.Ts == TS.
%
%   See also ddc.vrft.controllerBasisPreset, ddc.vrft.vrftDesign,
%   ddc.vrft.prefilterDesign.

    arguments
        expression (1,1) string
        Ts (1,1) double {mustBePositive}
    end

    expression = strtrim(expression);
    if strlength(expression) == 0
        error('ddc:vrft:parseBasisExpression:EmptyExpression', ...
            'Basis/prefilter expression must not be empty.');
    end

    % Whitelist characters permitted in EXPRESSION: digits, '.', whitespace,
    % parentheses, the four arithmetic operators, '^', and the two
    % identifiers 'z' and 'Ts' (checked separately below). This blocks
    % arbitrary code execution while still allowing normal transfer
    % function algebra.
    sanitizedForIdentifierCheck = regexprep(expression, 'Ts', '');
    allowedPattern = '^[0-9zZ\.\+\-\*\/\^\(\)\s]*$';
    if ~isempty(regexp(sanitizedForIdentifierCheck, '[A-Za-y]', 'once')) ...
            || isempty(regexp(sanitizedForIdentifierCheck, allowedPattern, 'once'))
        error('ddc:vrft:parseBasisExpression:DisallowedCharacters', ...
            ['Expression "%s" contains characters or identifiers other than ', ...
             'digits, z, Ts, whitespace, parentheses, and + - * / ^.'], expression);
    end

    try
        f = str2func("@(z,Ts) " + expression);
        z = tf('z', Ts);
        sys = f(z, Ts);
    catch ME
        error('ddc:vrft:parseBasisExpression:InvalidExpression', ...
            'Could not evaluate expression "%s": %s', expression, ME.message);
    end

    if isnumeric(sys)
        sys = tf(sys, 1, Ts);
    elseif ~(isa(sys, 'DynamicSystem') || isa(sys, 'lti'))
        error('ddc:vrft:parseBasisExpression:InvalidExpression', ...
            'Expression "%s" did not evaluate to a scalar or LTI system.', expression);
    end
end
