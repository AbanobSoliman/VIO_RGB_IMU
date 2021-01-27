classdef HelperScrollingPlotter < matlab.System
%HelperScrollingPlotter - Streaming plotting
%
%   % EXAMPLE: Plot position and orientation error.
%
%   Fs = 1;
%   numInputs = 4;
%   yLimits = [-30 30;
%              -30 30;
%              -30 30;
%               0  180];
%   titles = {'Position Error (x-axis)', ...
%             'Position Error (y-axis)', ...
%             'Position Error (z-axis)', ...
%             'Orientation Error'};
%
%
%   plotter = HelperScrollingPlotter('SampleRate', Fs, ...
%                                    'NumInputs', numInputs, ...
%                                    'YLimits', yLimits, ...
%                                    'Title', titles);
%
%   plotter(0, 0, 0, 0);
%   plotter(10, 10, 10, 10);
%   plotter(20, 20, 20, 20);

%   Copyright 2017-2018 The MathWorks, Inc.

    properties
        YLimits = [-10 10];
        TimeSpan = 10;
        YLabel = '';
        Title = '';
    end

    properties (Nontunable)
        SampleRate = 1;
        NumInputs = 1;
    end

    properties (Hidden)
        AppWindow;
    end

    properties (Access = private, Nontunable)
        pAllChannels;
    end

    properties (Access = private)
        pAllAxes;
        pAllLines;
        pCurrMinTimeLim = 0;
        pCurrTime = 0;
    end

    properties (Access = private, Dependent)
        pTimeLim;
    end

    properties (Access = private, Constant)
        X_AXIS_LABEL = 'Time (s)';
        ALL_AXES_POSITION = [0 0 1 1];
    end

    methods % Set and Get methods
        function val = get.pTimeLim(obj)
            val = cumsum([obj.pCurrMinTimeLim obj.TimeSpan]);
        end

        function set.TimeSpan(obj, val)
            obj.TimeSpan = val;
            updateAxesXLim(obj);
        end

        function set.YLimits(obj, val)
            obj.YLimits = val;
            updateAxesYLim(obj);
        end

        function set.YLabel(obj, val)
            obj.YLabel = val;
            updateAxesYLabel(obj);
        end

        function set.Title(obj, val)
            obj.Title = val;
            updateAxesTitle(obj);
        end

        function set.NumInputs(obj, val)
            oldNumInputs = obj.NumInputs;
            obj.NumInputs = val;
            updateNumAxes(obj, oldNumInputs);
        end

        % Constructor
        function obj = HelperScrollingPlotter(varargin)
            setProperties(obj, nargin, varargin{:});
            initializeAppWindow(obj);
        end

        % Destructor
        function delete(obj)
            fig = obj.AppWindow;
            if (~isempty(fig) && ishghandle(fig))
                delete(fig);
            end
        end

        function show(obj)
            if ishghandle(obj.AppWindow)
                obj.AppWindow.Visible = 'on';
            end
        end
    end

    methods (Access = protected)
        function initializeAppWindow(obj)
            fig = obj.AppWindow;
            if (isempty(fig) || ~ishghandle(fig))
                fig = figure('Name', 'Scrolling Plotter', ...
                    'NumberTitle', 'off', ...
                    'ToolBar',     'auto', ...%'none', ...
                    'MenuBar',     'figure', ...
                    'DockControls','on', ...
                    'Units', 'normalized', ...
                    'OuterPosition',    [0 0.5 0.5 0.5], ...
                    'Visible','off', ...
                    'HandleVisibility', 'on', ...
                    'NextPlot', 'new', ...
                    'IntegerHandle', 'off', ...
                    'CloseRequestFcn', @(fig, ~) set(fig, 'Visible', 'off'));
                obj.AppWindow = fig;
            end
            updateNumAxes(obj, 0);
            updateAxesXLim(obj);
            updateAxesYLim(obj);
            updateAxesYLabel(obj);
            updateAxesTitle(obj);
        end

        function updateNumAxes(obj, oldNumInputs)
            fig = obj.AppWindow;
            if ishghandle(fig)
                newNumInputs = obj.NumInputs;
                allAxes = obj.pAllAxes;
                if newNumInputs < oldNumInputs
                    for i = oldNumInputs:-1:(newNumInputs+1)
                        delete(allAxes(i));
                        allAxes(i) = [];
                    end
                elseif newNumInputs > oldNumInputs
                    for i = (newNumInputs-oldNumInputs):-1:1
                        ax = axes(fig); %#ok<LAXES>
                        ax.XLabel.String = obj.X_AXIS_LABEL;
                        tmp{i} = ax;
                    end
                    allAxes = vertcat(allAxes(:), tmp{:});
                end
                allAxesPosition = obj.ALL_AXES_POSITION;
                axesHeight = allAxesPosition(4) / newNumInputs;
                currAxesPosition = [allAxesPosition(1:3), axesHeight];
                for i = newNumInputs:-1:1
                    set(allAxes(i), 'OuterPosition', currAxesPosition);
                    currAxesPosition(2) = currAxesPosition(2) + axesHeight;
                end

                obj.pAllAxes = allAxes;
            end
        end

        function updateAxesXLim(obj)
            if (~isempty(obj.pAllAxes) && all(ishghandle(obj.pAllAxes(:))))
                set(obj.pAllAxes, 'XLim', obj.pTimeLim);
            end
        end

        function updateAxesYLim(obj)
            if (~isempty(obj.pAllAxes) && all(ishghandle(obj.pAllAxes(:))))
                if (size(obj.YLimits, 1) == 1)
                    set(obj.pAllAxes, 'YLim', obj.YLimits);
                else
                    for i = 1:size(obj.YLimits, 1)
                        set(obj.pAllAxes(i), 'YLim', obj.YLimits(i,:));
                    end
                end
            end
        end

        function updateAxesYLabel(obj)
            if (~isempty(obj.pAllAxes) && all(ishghandle(obj.pAllAxes(:))))
                if ischar(obj.YLabel)
                    arrayfun(@(x) set(get(x, 'YLabel'), 'String', obj.YLabel), obj.pAllAxes);
                else
                    for i = 1:numel(obj.YLabel)
                        set(get(obj.pAllAxes(i), 'YLabel'), 'String', obj.YLabel{i});
                    end
                end
            end
        end

        function updateAxesTitle(obj)
            if (~isempty(obj.pAllAxes) && all(ishghandle(obj.pAllAxes(:))))
                if ischar(obj.Title)
                    arrayfun(@(x) set(get(x, 'Title'), 'String', obj.Title), obj.pAllAxes);
                else
                    for i = 1:numel(obj.Title)
                        set(get(obj.pAllAxes(i), 'Title'), 'String', obj.Title{i});
                    end
                end
            end
        end

        function val = getNumInputsImpl(obj)
            val = obj.NumInputs;
        end

        function setupImpl(obj, varargin)
            allChannels = cellfun(@(x) size(x, 2), varargin);
            allAxes = obj.pAllAxes;
            if all(ishghandle(allAxes(:)))
                bufferLength = obj.SampleRate * (obj.TimeSpan+1);
                for i = 1:obj.NumInputs
                    initVals = NaN(bufferLength, allChannels(i));
                    obj.pAllLines{i} = line(allAxes(i), initVals, initVals);
                end

                obj.pAllChannels = allChannels;
            end

            show(obj);
        end

        function stepImpl(obj, varargin)
            % Update all lines and scroll the plot
            allAxes = obj.pAllAxes;
            if any(~ishghandle(allAxes(:)))
                return;
            end
            allChannels = obj.pAllChannels;
            allLines = obj.pAllLines;

            dt = 1/obj.SampleRate;
            numSamples = size(varargin{1}, 1);
            time = obj.pCurrTime:dt:( obj.pCurrTime + (dt*(numSamples-1)) );
            timeLim = obj.pTimeLim;
            for currSample = 1:numSamples

                for currInput = 1:obj.NumInputs
                    allLinesCurrChannel = allLines{currInput};
                    numChans = allChannels(currInput);
                    u = varargin{currInput};
                    for currChannel = 1:numChans
                        currLine = allLinesCurrChannel(currChannel);
                        set(currLine, ...
                            'XData', [currLine.XData(2:end), time(currSample)], ...
                            'YData', [currLine.YData(2:end), u(currSample, currChannel)]);
                    end
                end
                timeDisplayDiff = max(time(currSample) - timeLim(2), 0);
                timeLim = timeLim + timeDisplayDiff;
                set(allAxes, 'XLim', timeLim);
                drawnow limitrate;
            end

            obj.pCurrTime = time(end) + dt;
            obj.pCurrMinTimeLim = timeLim(1);
        end
    end
end
