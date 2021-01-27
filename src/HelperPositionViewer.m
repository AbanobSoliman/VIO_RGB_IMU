classdef HelperPositionViewer < matlab.System
%HelperPositionViewer - Position visualization

%   Copyright 2017-2019 The MathWorks, Inc.


    properties
        HistoryLength = 1000;
        LegendLabels = {'Estimated', 'Ground Truth'};
    end

    properties (Nontunable) %(Access = private, Nontunable)
        NumInputs = 1;
    end

    properties (Access = private)
        pAxes;

        pLines;
        pMarkers;
        IsTopLevelUI = false;
    end

    properties (Access = private, Constant)
        INITIAL_VIEW_ANGLE = [1 1 1]; %[-90, 90];
        ViewTitle = 'Position (meters)';
    end

    properties (Hidden)
        AppWindow;
        AxesPosition = [0.1300 0.1100 0.7750 0.8150];
        XLimits = [-1, 1];
        YLimits = [-1, 1];
        ZLimits = [-1, 1];
        
        ReferenceFrame = 'NED';
    end


    methods
        % Constructor
        function obj = HelperPositionViewer(varargin)
            setProperties(obj,nargin,varargin{:});
            createUI(obj);
        end

        % Destructor
        function delete(obj)
            fig = obj.AppWindow;
            if (obj.IsTopLevelUI && ~isempty(fig) && ishghandle(fig))
                delete(fig);
            end
        end

        function show(obj)
            fig = obj.AppWindow;
            if (obj.IsTopLevelUI && ~isempty(fig) && ishghandle(fig))
                fig.Visible = 'on';
            end
        end
    end

    methods (Access = protected)
        function setupImpl(obj, varargin)
            numInputs = numel(varargin);

            ax = obj.pAxes;
            colors = ax.ColorOrder;
            numColors = size(colors, 1);
            colorIdx = 1;
            for i = 1:numInputs
                obj.pLines(i) = line(ax, NaN, NaN, NaN, ...
                                     'Color', colors(colorIdx,:), 'LineStyle', '--');
                obj.pMarkers(i) = line(ax, NaN, NaN, NaN, ...
                                       'Color', colors(colorIdx,:), ...
                                       'Marker', 'd', 'MarkerSize', 6, 'LineStyle', '--');

                colorIdx = colorIdx + 1;
                if colorIdx > numColors
                    colorIdx = 1;
                end
            end

            ax.XLimMode = 'manual';
            ax.XLim = obj.XLimits;
            ax.YLimMode = 'manual';
            ax.YLim = obj.YLimits;
            ax.ZLimMode = 'manual';
            ax.ZLim = obj.ZLimits;
            obj.NumInputs = numInputs;
            show(obj);
        end

        function val = getNumInputsImpl(obj)
            val = obj.NumInputs;
        end

        function stepImpl(obj, varargin)
            lines = obj.pLines;
            markers = obj.pMarkers;
            for i = 1:numel(varargin)
                pos = varargin{i};
                currLine = lines(i);
                currMarker = markers(i);

                xd = get(currLine, 'XData');
                yd = get(currLine, 'YData');
                zd = get(currLine, 'ZData');

                n = size(pos,1);
                datahist = max(obj.HistoryLength - n, n);
                datahist = min(datahist, numel(xd)-1);
                bufx = [xd(end - datahist:end) pos(:,1).'];
                bufy = [yd(end - datahist:end) pos(:,2).'];
                bufz = [zd(end - datahist:end) pos(:,3).'];
                set(currLine, ...
                    'XData', bufx, ...
                    'YData', bufy, ...
                    'ZData', bufz);

                set(currMarker, 'XData', pos(end,1));
                set(currMarker, 'YData', pos(end,2));
                set(currMarker, 'ZData', pos(end,3));
            end
            drawnow limitrate;
        end
    end

    methods (Access = private)
        function createUI(obj)
            createAppWindow(obj);
            createAxes(obj);
        end

        function createAppWindow(obj)
            fig = obj.AppWindow;
            if (isempty(fig) || ~ishghandle(fig))
                fig = figure('Name', 'Position Viewer', ...
                             'NumberTitle', 'off', ...
                             'DockControls','off', ...
                             'Units', 'normalized', ...
                             'OuterPosition', [0 0.5 0.25 0.5], ...
                             'Visible', 'off', ...
                             'IntegerHandle', 'off', ...
                             'HandleVisibility', 'on', ...
                             'NextPlot', 'new', ...
                             'CloseRequestFcn', @(x,~)set(x,'Visible', 'off'));
                obj.AppWindow = fig;
                obj.IsTopLevelUI = true;
            end
        end

        function createAxes(obj)
            fig = obj.AppWindow;

            ax = axes(fig, 'OuterPosition', obj.AxesPosition);

            ax.Title.String = obj.ViewTitle;

            view(ax,obj.INITIAL_VIEW_ANGLE);

            % axis equal;
            ax.DataAspectRatioMode = 'manual';
            ax.DataAspectRatio = [1 1 1];
            ax.PlotBoxAspectRatioMode = 'manual';
            ax.PlotBoxAspectRatio = [1.2 1 1];

            if strcmp(obj.ReferenceFrame, 'NED')
                % Reference frame is NED.
                ax.YDir = 'reverse';
                ax.ZDir = 'reverse';
                ax.XLabel.String = 'x (North)';
                ax.YLabel.String = 'y (East)';
                ax.ZLabel.String = 'z (Down)';
            else
                % Reference frame is ENU.
                ax.XLabel.String = 'x (East)';
                ax.YLabel.String = 'y (North)';
                ax.ZLabel.String = 'z (Up)';
            end

            ax.XGrid = 'on';
            ax.XMinorGrid = 'on';
            ax.YGrid = 'on';
            ax.YMinorGrid = 'on';
            ax.ZGrid = 'on';
            ax.ZMinorGrid = 'on';

            ax.XLimMode = 'manual';
            ax.XLim = obj.XLimits;
            ax.YLimMode = 'manual';
            ax.YLim = obj.YLimits;
            ax.ZLimMode = 'manual';
            ax.ZLim = obj.ZLimits;

            obj.pAxes = ax;
        end

        function hide(obj)
            set(obj.AppWindow,'Visible','off');
        end
    end
end
