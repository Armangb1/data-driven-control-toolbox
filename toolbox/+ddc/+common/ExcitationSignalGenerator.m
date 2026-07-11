classdef ExcitationSignalGenerator < matlab.System
    %EXCITATIONSIGNALGENERATOR Persistency-of-excitation-rich signal source.
    %   Generates an excitation signal suitable for collecting the
    %   offline data required by DeePC / VRFT, or for exciting a plant
    %   before switching on an adaptive controller. Supports pseudo-random
    %   binary sequence (PRBS) and band-limited random (multisine-like)
    %   signals.
    %
    %   Example (MATLAB):
    %       gen = ddc.common.ExcitationSignalGenerator('SignalType', 'prbs', ...
    %                                                   'Amplitude', 1);
    %       u = arrayfun(@(~) gen.step(), 1:200);
    %
    %   See also ddc.common.HankelBuilder, ddc.common.checkPersistencyExcitation.

    properties (Nontunable)
        SignalType (1,:) char {mustBeMember(SignalType, {'prbs', 'random'})} = 'prbs'
        Amplitude (1,1) double {mustBePositive} = 1
        Seed (1,1) double {mustBeInteger, mustBeNonnegative} = 0
        SwitchProbability (1,1) double {mustBeGreaterThan(SwitchProbability,0), ...
                                         mustBeLessThanOrEqual(SwitchProbability,1)} = 0.5
    end

    properties (Access = private)
        RngStream
        CurrentLevel
    end

    methods
        function obj = ExcitationSignalGenerator(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.RngStream = RandStream('mt19937ar', 'Seed', obj.Seed);
            obj.CurrentLevel = obj.Amplitude;
        end

        function resetImpl(obj)
            obj.RngStream.reset();
            obj.CurrentLevel = obj.Amplitude;
        end

        function u = stepImpl(obj)
            switch obj.SignalType
                case 'prbs'
                    if rand(obj.RngStream) < obj.SwitchProbability
                        obj.CurrentLevel = -obj.CurrentLevel;
                    end
                    u = obj.CurrentLevel;
                case 'random'
                    u = obj.Amplitude * (2*rand(obj.RngStream) - 1);
            end
        end

        function sz = getOutputSizeImpl(~)
            sz = [1 1];
        end

        function dt = getOutputDataTypeImpl(~)
            dt = 'double';
        end

        function cp = isOutputComplexImpl(~)
            cp = false;
        end

        function fz = isOutputFixedSizeImpl(~)
            fz = true;
        end

        function num = getNumInputsImpl(~)
            num = 0;
        end

        function num = getNumOutputsImpl(~)
            num = 1;
        end
    end
end
