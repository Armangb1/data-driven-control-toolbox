classdef IndirectSTRController < matlab.System
    %INDIRECTSTRCONTROLLER Indirect self-tuning regulator (pole placement).
    %   Generalized adaptive controller following Astrom & Wittenmark,
    %   "Adaptive Control", Algorithm 3.2: at each sampling period, identify
    %   a SISO ARX plant model online using recursive least squares (RLS),
    %   then compute an RST polynomial controller via minimum-degree pole
    %   placement (MDPP) to achieve the desired closed-loop behavior.
    %
    %   This is an INDIRECT STR: identification and controller design are
    %   separate steps.  The plant is first identified, and the estimated
    %   plant is then passed to the MDPP design engine.
    %
    %   Plant model (general SISO ARX):
    %       A(q^-1) y(k) = B(q^-1) u(k) + e(k)
    %
    %   with
    %       A(q^-1) = 1 + a1 q^-1 + ... + a_Na q^-Na
    %       B(q^-1) = b0 q^-1 + ... + b_{Nb-1} q^{-Nb}
    %
    %   Note: B starts at q^-1 (no direct feedthrough).  In the row-vector
    %   convention [b0 b1 ...], b0 is the coefficient of q^-1.
    %
    %   The ARX parameters [-a1;...;-a_Na; b0;...;b_{Nb-1}] are estimated
    %   online via RLS.  At every sampling period the estimated plant is
    %   passed to the MDPP design engine together with:
    %       Am  - desired closed-loop characteristic polynomial
    %       Bm  - desired reference model numerator
    %       ObserverPole - observer pole location (default: 0, deadbeat)
    %
    %   The observer polynomial Ao is computed dynamically at each step
    %   from the B factorization and the ObserverPole, ensuring degree
    %   compatibility regardless of transient plant estimates.
    %
    %   The resulting R, S, T implement:
    %       R(q^-1) u(k) = T(q^-1) u_c(k) - S(q^-1) y(k)
    %
    %   Startup behavior:
    %       Before RLS has produced a reliable estimate, the controller
    %       uses R=1, S=0, T=1 (i.e. u = u_c, pass-through).  If MDPP
    %       fails for a transient RLS estimate, the previous valid R/S/T
    %       is retained.
    %
    %   Usable as a plain MATLAB object or as a Simulink "MATLAB System"
    %   block.
    %
    %   Example (first-order plant):
    %       ctrl = ddc.str.IndirectSTRController( ...
    %           'Na', 1, 'Nb', 1, ...
    %           'Am', [1 -0.5], 'Bm', [0.5]);
    %       y = 0;
    %       for k = 1:200
    %           [u, theta] = ctrl.step(y, 1);
    %           y = 0.8*y + 0.5*u;
    %       end
    %
    %   See also ddc.str.DirectSTRController, ddc.common.RLSEstimator,
    %            ddc.str.mdpp.mdpp_design.

    properties (Nontunable)
        Na (1,1) double {mustBePositive, mustBeInteger} = 1  % order of A(q^-1) excluding leading 1
        Nb (1,1) double {mustBePositive, mustBeInteger} = 1  % order of B(q^-1) (b0 q^-1 + ... + b_{Nb-1} q^{-Nb})
    end

    properties
        Am (1,:) double {mustBeFinite} = [1 -0.5]  % desired closed-loop characteristic polynomial
        Bm (1,:) double {mustBeFinite} = [0.5]     % desired reference model numerator
        ObserverPole (1,1) double {mustBeFinite} = 0  % observer pole location (0 = deadbeat)
        ForgettingFactor (1,1) double {mustBeGreaterThan(ForgettingFactor,0), ...
                                        mustBeLessThanOrEqual(ForgettingFactor,1)} = 0.98
        MinB (1,1) double {mustBePositive} = 1e-3  % guard against near-zero B estimate
    end

    properties (Access = private)
        RLS_            % ddc.common.RLSEstimator
        R_              % latest valid controller R polynomial (row vector)
        S_              % latest valid controller S polynomial (row vector)
        T_              % latest valid controller T polynomial (row vector)
        Ahat_           % latest estimated A polynomial
        Bhat_           % latest estimated B polynomial
        Theta_          % latest RLS parameter vector
        YHistory_       % past outputs y(k-1), y(k-2), ...  (Na-by-1)
        UHistory_       % past inputs  u(k-1), u(k-2), ...  (max(Nb,Na-1)-by-1)
        RHistory_       % past references r(k-1), ...        (max(Na-1,0)-by-1)
        StepCount_      % number of step calls (for diagnostics)
    end

    methods
        function obj = IndirectSTRController(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            nTheta = obj.Na + obj.Nb;
            obj.RLS_ = ddc.common.RLSEstimator('NumParameters', nTheta, ...
                'ForgettingFactor', obj.ForgettingFactor);

            obj.R_ = [1];
            obj.S_ = [0];
            obj.T_ = [1];
            obj.Ahat_ = [1];
            obj.Bhat_ = [1];
            obj.Theta_ = zeros(nTheta, 1);
            obj.YHistory_ = zeros(obj.Na, 1);
            obj.UHistory_ = zeros(max(obj.Nb, obj.Na - 1), 1);
            obj.RHistory_ = zeros(max(obj.Na - 1, 0), 1);
            obj.StepCount_ = 0;
        end

        function resetImpl(obj)
            reset(obj.RLS_);
            nTheta = obj.Na + obj.Nb;
            obj.R_ = [1];
            obj.S_ = [0];
            obj.T_ = [1];
            obj.Ahat_ = [1];
            obj.Bhat_ = [1];
            obj.Theta_ = zeros(nTheta, 1);
            obj.YHistory_ = zeros(obj.Na, 1);
            obj.UHistory_ = zeros(max(obj.Nb, obj.Na - 1), 1);
            obj.RHistory_ = zeros(max(obj.Na - 1, 0), 1);
            obj.StepCount_ = 0;
        end

        function [u, theta] = stepImpl(obj, y, r)
            obj.StepCount_ = obj.StepCount_ + 1;

            % Step 1: Build regressor and update RLS.
            phi = obj.buildRegressor();
            theta = obj.RLS_.step(phi, y);
            obj.Theta_ = theta;

            % Step 2: Convert RLS parameters to ARX polynomials.
            [Ahat, Bhat] = obj.parametersToPolynomials(theta);
            obj.Ahat_ = Ahat;
            obj.Bhat_ = Bhat;

            % Step 3: Design controller via MDPP.
            obj.designController(Ahat, Bhat);

            % Step 4: Apply the RST control law.
            u = obj.applyRST(y, r);

            % Shift histories for the next sample.
            obj.YHistory_ = obj.shiftHistory(obj.YHistory_, y);
            obj.UHistory_ = obj.shiftHistory(obj.UHistory_, u);
            obj.RHistory_ = obj.shiftHistory(obj.RHistory_, r);
        end

        % ---- Private helpers ----

        function phi = buildRegressor(obj)
        %buildRegressor  Construct the ARX regressor vector.
        %   phi = [y(k-1); ...; y(k-Na); u(k-1); ...; u(k-Nb)]
            phi = [obj.YHistory_(1:obj.Na); obj.UHistory_(1:obj.Nb)];
        end

        function [A, B] = parametersToPolynomials(obj, theta)
        %parametersToPolynomials  Convert RLS parameter vector to A, B polynomials.
        %   theta = [-a1; ...; -a_Na; b0; b1; ...; b_{Nb-1}]
        %   A = [1, -theta(1), ..., -theta(Na)]    (descending powers)
        %   B = [theta(Na+1), ..., theta(Na+Nb)]    (descending powers)
            A = [1; -theta(1:obj.Na)];
            B = theta(obj.Na+1:end);
            A = A(:).';
            B = B(:).';
        end

        function designController(obj, Ahat, Bhat)
        %designController  Run MDPP to obtain R, S, T from the estimated plant.
        %   Computes Ao dynamically from the Bhat factorization and the
        %   ObserverPole property.  On any failure, the previous valid R/S/T
        %   are retained.
            if all(abs(Bhat) < obj.MinB)
                return;
            end

            % Factor Bhat to determine the required Ao degree.
            try
                [Bplus, ~] = ddc.str.mdpp.factorB(Bhat);
            catch
                return;
            end

            nA = length(Ahat) - 1;
            nBplus = length(Bplus) - 1;
            nAo = nA - nBplus - 1;
            if nAo < 0
                return;
            end

            % Build Ao from the observer pole specification.
            if nAo == 0
                Ao = [1];
            else
                Ao = poly(repmat(obj.ObserverPole, 1, nAo));
            end

            try
                result = ddc.str.mdpp.mdpp_design(Ahat, Bhat, obj.Am, obj.Bm, Ao);
                obj.R_ = result.R;
                obj.S_ = result.S;
                obj.T_ = result.T;
            catch
                % Retain previous valid controller.
            end
        end

        function u = applyRST(obj, y, r)
        %applyRST  Compute u(k) from the polynomial RST control law.
        %   R(q^-1) u(k) = T(q^-1) r(k) - S(q^-1) y(k)
        %
        %   R, S, T are row vectors in descending powers of q (i.e.
        %   R = [r0 r1 ... rr] represents r0 + r1*q^-1 + ... + rr*q^-rr).

            R = obj.R_;
            S = obj.S_;
            T = obj.T_;

            nR = length(R) - 1;
            nS = length(S) - 1;
            nT = length(T) - 1;

            % Past control input contribution: sum_{i=1}^{nR} R(i+1) * u(k-i).
            rPast = 0;
            for i = 1:nR
                rPast = rPast + R(i + 1) * obj.UHistory_(i);
            end

            % Past output contribution: sum_{i=1}^{nS} S(i+1) * y(k-i).
            sPast = 0;
            for i = 1:nS
                sPast = sPast + S(i + 1) * obj.YHistory_(i);
            end

            % Past reference contribution: sum_{i=1}^{nT} T(i+1) * r(k-i).
            tPast = 0;
            for i = 1:nT
                tPast = tPast + T(i + 1) * obj.RHistory_(i);
            end

            % Solve for u(k):
            %   R(1)*u(k) = T(1)*r(k) + tPast - S(1)*y(k) - sPast - rPast
            u = (T(1)*r + tPast - S(1)*y - sPast - rPast) / R(1);
        end

        function h = shiftHistory(~, hist, newVal)
        %shiftHistory  Shift a history buffer and insert the newest value.
            h = [newVal; hist(1:end-1)];
        end

        % ---- Simulink / System object queries ----

        function [sz1, sz2] = getOutputSizeImpl(obj)
            sz1 = [1 1];
            sz2 = [obj.Na + obj.Nb, 1];
        end

        function [dt1, dt2] = getOutputDataTypeImpl(~)
            dt1 = 'double'; dt2 = 'double';
        end

        function [cp1, cp2] = isOutputComplexImpl(~)
            cp1 = false; cp2 = false;
        end

        function [fz1, fz2] = isOutputFixedSizeImpl(~)
            fz1 = true; fz2 = true;
        end

        function num = getNumInputsImpl(~)
            num = 2;
        end

        function num = getNumOutputsImpl(~)
            num = 2;
        end

        function [inputNames, outputNames] = getInputOutputNamesImpl(~)
            inputNames  = {'y', 'r'};
            outputNames = {'u', 'theta'};
        end
    end

    % ---- Public getters for diagnostics ----

    methods
        function R = getR(obj)
        %getR  Latest controller R polynomial.
            R = obj.R_;
        end

        function S = getS(obj)
        %getS  Latest controller S polynomial.
            S = obj.S_;
        end

        function T = getT(obj)
        %getT  Latest controller T polynomial.
            T = obj.T_;
        end

        function Ahat = getEstimatedA(obj)
        %getEstimatedA  Latest estimated plant A polynomial.
            Ahat = obj.Ahat_;
        end

        function Bhat = getEstimatedB(obj)
        %getEstimatedB  Latest estimated plant B polynomial.
            Bhat = obj.Bhat_;
        end
    end
end
