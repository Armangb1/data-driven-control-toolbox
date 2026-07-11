classdef DirectSTRController < matlab.System
    %DIRECTSTRCONTROLLER Direct self-tuning regulator (pole placement).
    %   Baseline adaptive-control scheme following Astrom & Wittenmark,
    %   "Adaptive Control", Ch. 5: reparametrize the plant model so that
    %   the controller gains are estimated directly ("direct" = no
    %   intermediate plant-parameter identification step).
    %
    %   Plant assumption (first-order ARX, unit delay):
    %       y(k) = -a1*y(k-1) + b1*u(k-1) + e(k)
    %
    %   For the minimal-degree RST design (R=1, S=s0, T=t0) that places
    %   the closed loop at 1 + Ac1*q^-1 = 0, substituting the control law
    %   u(k-1) = t0*r(k-1) - s0*y(k-1) into the plant equation gives the
    %   directly-estimable regression model
    %
    %       y(k) + Ac1*y(k-1) = beta0*u(k-1) + beta1*y(k-1) + e(k)
    %
    %   with beta0 = b1, beta1 = b1*s0. RLS estimates [beta0; beta1]
    %   directly from data; the controller gains follow immediately:
    %       s0 = beta1/beta0,   t0 = (1+Ac1)/beta0.
    %
    %   Extending to higher-order plants requires a general Diophantine
    %   solver; this first-order case is the standard textbook baseline.
    %
    %   Usable as a plain MATLAB object or as a Simulink "MATLAB System"
    %   block.
    %
    %   See also ddc.str.IndirectSTRController, ddc.common.RLSEstimator.

    properties
        Ac1 (1,1) double {mustBeReal, mustBeGreaterThan(Ac1,-1), ...
                           mustBeLessThan(Ac1,1)} = -0.5  % desired closed-loop pole coeff
        ForgettingFactor (1,1) double {mustBeGreaterThan(ForgettingFactor,0), ...
                                        mustBeLessThanOrEqual(ForgettingFactor,1)} = 0.98
        MinBeta0 (1,1) double {mustBePositive} = 1e-3  % guard against near-zero beta0 estimate
        InitialS0 (1,1) double {mustBeReal} = 0  % initial controller gain (avoids u==0 deadlock)
        InitialT0 (1,1) double {mustBeReal} = 1  % initial feedforward gain (avoids u==0 deadlock)
    end

    properties (Access = private)
        RLS_
        PrevY_
        PrevU_
        S0_
        T0_
    end

    methods
        function obj = DirectSTRController(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.RLS_ = ddc.common.RLSEstimator('NumParameters', 2, ...
                'ForgettingFactor', obj.ForgettingFactor);
            obj.PrevY_ = 0;
            obj.PrevU_ = 0;
            obj.S0_ = obj.InitialS0;
            obj.T0_ = obj.InitialT0;
        end

        function resetImpl(obj)
            reset(obj.RLS_);
            obj.PrevY_ = 0;
            obj.PrevU_ = 0;
            obj.S0_ = obj.InitialS0;
            obj.T0_ = obj.InitialT0;
        end

        function [u, theta] = stepImpl(obj, y, r)
            phi = [obj.PrevU_; obj.PrevY_];
            z = y + obj.Ac1 * obj.PrevY_;
            theta = obj.RLS_.step(phi, z);
            beta0 = theta(1);
            beta1 = theta(2);

            if abs(beta0) < obj.MinBeta0
                s0 = obj.S0_;
                t0 = obj.T0_;
            else
                s0 = beta1 / beta0;
                t0 = (1 + obj.Ac1) / beta0;
            end

            u = t0*r - s0*y;

            obj.PrevY_ = y;
            obj.PrevU_ = u;
            obj.S0_ = s0;
            obj.T0_ = t0;
        end

        function [sz1, sz2] = getOutputSizeImpl(~)
            sz1 = [1 1];
            sz2 = [2 1];
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
    end
end
