% ousterlidar Creates an Ouster lidar(TM) sensor object.
%
%   ouster = ousterlidar(model, calibrationFilePath ) constructs an ousterlidar object,
%   ouster that can connect to an ouster lidar sensor of the specified device model.
%   model is a character vector or string specifying the name of the ouster
%   lidar sensor. Possible values for model are:
%       'OS0-64' for the OS0-64 sensor,
%       'OS1-64' for the OS1-64 sensor,
%       'OS2-64' for the OS2-64 sensor,
%   calibrationFilePath is an full file path of a JSON file containing
%   ouster lidar sensor calibration data.
%
%   osterLidarObj = ousterlidar(model, <CalibrationFile>,  Name=Value) specifies additional
%   name-value arguments described below:
%   'Port'             Sensor's Data Port. Can be seen in the Web
%                      Interface's Data Port field.
%                      Default: 7502
%
%   'Timeout'          Timeout specified for reading a point cloud from the
%                      sensor, in seconds.
%                      Default: 10 seconds
%   ousterlidar properties:
%       Model                   - A character vector or
%                                 string scalar specifiying the sensor device model(read only)
%       IPAddress               - A character vector or string specifying the IP address read from the incoming UDP packets (read only)
%       Port                    - Sensor's data port, needed when
%                                 creating the object if different than the
%                                 default (read only after object creation)
%       Timeout                 - Positive scalar used to Timeout when
%                                 reading a point cloud.
%       CalibrationFile         - Path of the Calibration file (read only)
%       Streaming               - Indicates whether the object is currently
%                                 streaming or not(read only)
%       NumPointCloudsAvailable - Count of the number of the point clouds
%                                 available in the buffer
%
%   ousterlidar methods:
%
%   start          - Starts streaming point clouds to the object buffer
%   stop           - Stops streaming point clouds to the object buffer
%   preview        - Opens a live point cloud preview window.
%   closePreview   - Closes the live point cloud preview window.
%   flush          - Flushes the object buffer.
%   read           - Reads point clouds from the input buffer.
%
% Notes
% -----
% - Providing an incorrect calibration file for the ouster lidar sensor will not
%   through an error, rather an improperly calibrated point cloud will be
%   generated.
%
%   Example: Read and visualize point clouds from an OS1-64 sensor
%   --------------------------------------------------------------                                           
%  osterLidarObj = ousterlidar('OS1-64', calibFileName);
%   preview(osterLidarObj); % opens a live preview window.
%   closePreview(osterLidarObj); % closes the preview window.
%   [pointCloud, timeStamps] = read(osterLidarObj, 'latest'); % Get the latest point cloud
%   pcshow(pointCloud);
% Copyright 2021 The MathWorks, Inc.

classdef ousterlidar < handle & matlab.mixin.SetGet

    properties(SetAccess = immutable) % These properties are set during construction
        % Ouster lidar sensor model
        Model
        % IP Address of the lidar sensor
        IPAddress
        % Data Port that the sensor is sending packets on.
        Port
    end

    properties (Dependent = true, SetAccess = private)
        % Number of point clouds available in the buffer.
        NumPointCloudsAvailable
    end

    properties (Dependent = true)
        % Timeout when reading point clouds from the buffer
        Timeout
    end

    properties(GetAccess = 'public', SetAccess ='private')
        % Indicates whether the object is currently streaming.
        Streaming = false;
    end

    properties(Access = 'public')
        % JSON file containing calibration data
        CalibrationFile
    end

    properties (Access = 'private')
        % Shared UDP object.
        UdpObj
        % ProductID is a numeric ID provided with most ouster lidar sensors.
        ProductID
        % Laser Calibration Data, can be user provided or defaults from
        % ouster will be used.
        LaserCalibrationData
        % Laser Corrections, for more info, look at ouster manuals.
        AltitudeAngles
        AzimuthAngles
        DataFormat
        LidarToSensorTransform
        %JSONCalibrationVersion
        LidarMode
        LidarOriginToBeamDistance
        PixelShiftByRow
        ColumnsPerPacket
        ColumnsPerFrame
        PixelsPerColumn
        LidarStatus
        DistanceResolution
        % Map to enforce only one connection to ouster lidar sensors from
        % MATLAB.
        ConnectionMap = containers.Map()
        % Timer object that updates the preview window.
        UpdatePreviewTimer
        % Stores the status of streaming before preview, so that it can be
        % restored after preview is closed.
        StreamingStatusBeforePreview = false;
        % Flag that indicates if the object is previewing or not.
        IsPreviewing = false;
        % pcplayer
        Player
         % Stores the name of the calibration file currently being used.
        CalibrationFileInternal;
    end

    methods
        function obj = ousterlidar(model, calibrationFile,  varargin)
            % ousterlidar constructor

            % Make sure the ousterlidar message catalog is pulled in when running in
            % deployed mode or otherwise.

            validateattributes(model, {'char', 'string'}, {'nonempty'});
            if isdeployed
                rootDir = ctfroot;
            else
                rootDir = matlabroot;
            end
            % Register resources/ousterlidar.
            matlab.internal.msgcat.setAdditionalResourceLocation(rootDir);

            % Only one ousterlidar object is supported currently,
            % error out if there is already a connection.
            if (obj.ConnectionMap.Count >= 1)
                throwAsCaller(MException('ousterlidar:ousterlidar:OnlyOneConnectionAllowed', message('ousterlidar:ousterlidar:OnlyOneConnectionAllowed').getString));
            end

            % Validate inputs .
            try
                paramsStruct = obj.parseAndValidateInputs(model, calibrationFile, varargin{:});
                obj.CalibrationFile =  paramsStruct.CalibrationFile ;
                obj.Model = paramsStruct.Model ;
            catch ME
                throwAsCaller(ME);
            end
            try
                obj.CalibrationFile = obj.validateCalibrationFile(calibrationFile);

            catch ME
                throwAsCaller(ME);

            end

            %% Connect to the sensor using shared UDP
            obj.UdpObj = matlabshared.network.internal.UDP();
            obj.UdpObj.LocalPort = paramsStruct.Port;

            %% Get one packet, and get the product ID
            try
                connect(obj.UdpObj);
                pause(0.5); % Pause to get some packets
                % Look for a packet with size 1206, skip over other packets
                % like position packets of size 512
                foundDataPacket = false;
                datagramSize = 12608 ;
                while ~foundDataPacket
                    [data, datagramAddress] = read(obj.UdpObj);
                    if isempty(data)
                        throwAsCaller(MException('ousterlidar:ousterlidar:CheckConnection', message('ousterlidar:ousterlidar:CheckConnection').getString));
                    end
                    % read can get multiple packets, find the first data
                    % packet.
                    for packetIdx = 1:numel(data)
                        packet = data{packetIdx};
                        if numel(packet) == datagramSize
                            foundDataPacket = true;
                            obj.ProductID = packet(end); % The last(1206th) byte has the productID information.
                            obj.IPAddress = datagramAddress{1};
                            break;
                        end
                    end
                end

                % Adjust product ID based on model name input by user. This
                % is required because the OS1-64 is the only ouster model
                % which does not provide a product ID byte in each packet.
                if (strcmp(obj.Model, 'OS1-64'))
                    obj.ProductID = '64';
                end

                % Check to see that the model number specified by the user and
                % the one returned by the packet is the same
                %numLasersForUserInputModel = 64 ; %lidar.supportpackages.ousterlidar.internal.Utility.getNumLasersFromModelName(obj.Model);
                %productIDForUserInputModel = 'OS1-64'; %lidar.supportpackages.ousterlidar.internal.Utility.getProductIDFromModelName(obj.Model);
                %numLasersInConnectedSensor = 64 ; %lidar.supportpackages.ousterlidar.internal.Utility.getNumLasersFromProductID(obj.ProductID);
                %modelOfConnectedSensor = 64; %lidar.supportpackages.ousterlidar.internal.Utility.getModelFromProductID(obj.ProductID);

                % Checks if model name specified matches what is actually
                % connected. There are two if conditions:
                % 1. Verifies the above using number of lasers. If there is
                % a mismatch, error out.
                % 2. Some sensors have the same number of lasers, but are
                % different (different product IDs)
                
                
                % Setup filter and converter
                obj.setupFilterAndConverter;

            catch ME
                % Fix for g1948784: If the error message from the shared UDP
                % interface mentions that only one application can connect
                % to the port, make the error message more helpful.
                % Messages might be different on different platforms, so account for both.
                %
                % Fix for g2128537: Account for impacts of PLT (Pseudo
                % Localization Testing). Use error ID rather than message.
                if strcmp(ME.identifier, 'network:udp:connectFailed')
                    ME = MException('ousterlidar:ousterlidar:ConnectFailed', message('ousterlidar:ousterlidar:ConnectFailed',obj.UdpObj.LocalPort).getString);
                end
                throwAsCaller(ME);
            end
            % Set properties on the object once connection is successful
            % and the filter and converter are set
            obj.Timeout = paramsStruct.Timeout;
            obj.Port = paramsStruct.Port;
            % To allow only one connection to an ousterlidar sensor, add
            % this to the connection map.
             obj.ConnectionMap(obj.Model) = obj.ProductID;
        end

        function start(obj)
            % START starts adding point clouds to the buffer. start also
            % deletes all existing point clouds in the buffer before adding
            % new point clouds.
            % Usage:
            % ousterLidar = ousterlidar('OS1-64');
            % start(ousterLidar) % starts adding point clouds to the buffer.
            % % Get the number of point clouds currently in the buffer.
            % numPointClouds = ousterLidar.NumPointCloudsAvailable

            % Start and preview cannot happen at the same time, so error out
            % if preview is on.
            if obj.IsPreviewing
                error(message('ousterlidar:ousterlidar:StartPreviewNotAllowed'));
            end

            if obj.Streaming
                warning(message('ousterlidar:ousterlidar:AlreadyStreaming'));
            end

            % Warn that start will flush the buffer.
            if obj.NumPointCloudsAvailable>0
                warning(message('ousterlidar:ousterlidar:StartFlushesBuffer'));
            end

            % Flush the buffer and start streaming.
            obj.flush;
            setStream(obj,true);
        end

        function stop(obj)
            % STOP stops adding point clouds to the buffer. All existing
            % point clouds will remain in the object buffer.
            % Usage:
            % ousterLidar = ousterlidar('OS1-64');
            % start(ousterLidar) % starts adding point clouds to the buffer.
            % stop(ousterLidar); % stops adding point clouds to the buffer.
            if obj.IsPreviewing
                error(message('ousterlidar:ousterlidar:PreviewStopNotAllowed'));
            end

            setStream(obj,false);
        end

        function out = get.NumPointCloudsAvailable(obj)
            % Returns the number of available point clouds in the object
            % buffer.
            try
                out = obj.UdpObj.NumDatagramsAvailable;
            catch readNumDatagramsException
                throwAsCaller(readNumDatagramsException);
            end
        end

        function out = get.Timeout(obj)
            % Getter for timeout, uses the UDP object timeout.
            try
                out = obj.UdpObj.Timeout;
            catch timeoutSetException
                throwAsCaller(timeoutSetException);
            end
        end
        function value = get.CalibrationFile(obj)
            value = obj.CalibrationFileInternal;
        end
        function set.Timeout(obj, value)
            % Setter for timeout, sets it on the UDP object.
            try
                obj.UdpObj.Timeout = value;
            catch timeoutSetException
                throwAsCaller(timeoutSetException);
            end
        end

        function set.CalibrationFile(obj, value)
            try
                setCalibrationFile(obj,value);
            catch ME
                throwAsCaller(ME);
            end
        end

        function flush(obj)
            % FLUSH deletes all available point clouds in the buffer.
            % Usage:
            % ousterLidar = ousterlidar('OS1-64');
            % start(ousterLidar) % starts adding point clouds to the buffer.
            % flush(ousterLidar); % deletes point clouds currently in the
            % object buffer.
            % Note:
            % The state of Streaming is maintained when flush is called.
            % i.e. if the object was streaming before flush was called,
            % flush will delete the existing point clouds in the buffer, and
            % continue streaming.
            try
                obj.UdpObj.flushInput;
            catch flushException
                throwAsCaller(flushException);
            end
        end

        function [pointClouds, timeStamps] = read(obj, varargin)
            % READ method reads point cloud(s) from the input buffer. Use the
            % Timeout property to set a new value for timeout.
            % Usage:
            % pointClouds = read(ousterLidar,'all') reads all available point clouds from the input
            % buffer, along with timestamps.
            % pointClouds = read(ousterLidar,NumPointClouds,mode) reads NumPointClouds point clouds in
            % specified mode.
            % pointClouds = read(ousterLidar,10,'latest') will read 10 latest point clouds
            % from the buffer. Older point clouds will be discarded.
            % pointClouds = read(ousterLidar,10,'oldest') will read 10 oldest point clouds
            % from the buffer. Newer point clouds will continue to exist in
            % the buffer.
            % pointClouds = read(ousterLidar,'latest') will read the latest point cloud
            % from the buffer, and older point clouds will be discarded.
            % [pointClouds,timeStamps] = read(ousterLidar,'oldest') will read the oldest point cloud from the
            % buffer. Newer point clouds will continue to exist in
            % the buffer.
            %
            % Notes 
            % -----
            % 1. If the object is not streaming, and there are leftover
            % point clouds in the buffer, read method will read those point clouds into MATLAB.
            % 2. If the object is not streaming, read method will start the 
            % streaming, then get the required point clouds, and stop streaming.

            % Read and preview cannot happen at the same time, so error out
            % if preview is on.
            if obj.IsPreviewing
                error(message('ousterlidar:ousterlidar:PreviewReadNotAllowed'));
            end

            narginchk(1,3);
            validModes = {'oldest','latest','all'};
            validModesFor3Arg = {'oldest','latest'};
            mode =  'oldest';
            NumPointClouds = 1;
            if nargin == 3
                NumPointClouds = varargin{1};
                validateattributes(NumPointClouds, {'numeric'}, {'scalar', 'integer', 'real', 'finite', 'positive'}, mfilename, 'NumPointClouds', 2);
                mode = validatestring(varargin{2}, validModesFor3Arg, 'read', 'mode');
            elseif nargin == 2
                input = varargin{1};
                if isnumeric(input)
                    validateattributes(input, {'numeric'}, {'scalar', 'integer', 'real', 'finite', 'positive'}, mfilename, 'NumPointClouds', 2);
                    NumPointClouds = input;
                elseif isstring(input) || ischar(input)
                    mode = validatestring(input, validModes, 'read', 'mode');
                else
                    error(message('ousterlidar:ousterlidar:WrongInputs'));
                end
            end

            % Read from the AsyncIO Input Buffer.
            switch(lower(mode))
                case 'all'
                    [pointClouds, timeStamps] = readAllPointClouds(obj);
                case 'oldest'
                    [pointClouds, timeStamps] = readOldestPointClouds(obj, NumPointClouds);
                case 'latest'
                    [pointClouds, timeStamps] = readLatestPointClouds(obj, NumPointClouds);
                otherwise
                    error(message('ousterlidar:ousterlidar:InvalidMode',mode));
            end
        end

        function preview(obj, varargin)
            % PREVIEW method opens a live preview window that displays point clouds
            % from the ousterlidar sensor.
            % Usage:
            % PREVIEW(ousterLidar) opens a live preview window that
            % displays point clouds from the ousterLidar sensor.
            % PREVIEW(ousterLidar, xlimits, ylimits, zlimits) opens a live
            % preview window with X,Y and Z axes with xlimits, ylimits and
            % zlimits. Each limit must be specified as 1-by-2 vectors, and
            % all 3 axes must be specified.
            %
            % Example 1:
            % ousterLidar = ousterlidar('OS1-64');
            % preview(ousterLidar); % opens a live preview window.
            % closePreview(ousterLidar); % closes the preview window.
            %
            % Example 2:
            % ousterLidar = ousterlidar('OS1-64');
            % preview(v, [-3 3],[-3 3],[-1 1]);
            % closePreview(ousterLidar);

            % preview can optionally take limits for pcplayer. pcplayer
            % needs 3 inputs xlimits, ylimits and zlimits.
            if (nargin ~=1 && nargin ~=4)
                error(message('ousterlidar:ousterlidar:PreviewArguments'));
            end

            if (nargin == 4) % Expect xlimits, ylimits and zlimits for pcplayer
                if isempty(obj.Player) || ~isvalid(obj.Player)
                    obj.Player = pcplayer(varargin{:});
                end
            end

            % If object is streaming or if the buffer has PointClouds,
            % warn that PointClouds in the buffer will be removed.
            if obj.Streaming || obj.NumPointCloudsAvailable>0
                warning(message('ousterlidar:ousterlidar:PreviewFlushesBuffer'));
            end

            % If object is already previewing, just make sure the window is
            % visible (if hidden)
            if obj.IsPreviewing
                if ~obj.Player.isOpen % If the player isn't open, show it.
                    obj.Player.show;
                end
            else
                % Store streaming status before preview is started, so it
                % can be restored after preview is closed.
                obj.StreamingStatusBeforePreview = obj.Streaming;

                try
                    % Set streaming to true, and flush the buffer.
                    setStream(obj,true);
                    flush(obj);

                    % Read one point cloud to calculate limits on pcplayer
                    pointCloudDatagrams = read(obj.UdpObj,1);
                    ptCloud = pointCloud(pointCloudDatagrams.LocationPoints(:,:,1:3),'Intensity',pointCloudDatagrams.Intensity(:,:,1));

                    % Create the pcplayer if it does not exist already.
                    if isempty(obj.Player) || ~isvalid(obj.Player)
                        maxLimit = max(abs([ptCloud.XLimits ptCloud.YLimits]));
                        xlimits = [-maxLimit maxLimit];
                        ylimits = [-maxLimit maxLimit];
                        zlimits = ptCloud.ZLimits;
                        % Create the pcplayer
                        obj.Player = pcplayer(xlimits, ylimits, zlimits);
                    end

                    % Get the pcplayer figure window and change the name.
                    pcplayerFigWindow = ancestor(obj.Player.Axes, 'figure');
                    pcplayerFigWindow.IntegerHandle = 'off';
                    pcplayerFigWindow.Name = ['Ouster lidar Preview: ' obj.Model];

                    % Setup preview timer
                    obj.UpdatePreviewTimer = internal.IntervalTimer(0.1);
                    % Add the listener, that runs at each interval.
                    % TODO: Take a custom refresh rate, currently at 10 Hz
                    addlistener(obj.UpdatePreviewTimer, 'Executing', @(src, event)obj.updatePreviewWindow);

                    % Flush object and start timer.
                    flush(obj);
                    obj.UpdatePreviewTimer.start;

                    % Display pcplayer window.
                    obj.Player.show;
                catch  ME % If there were errors in preview, restore streaming status.
                    obj.setStream(obj.StreamingStatusBeforePreview);
                    throwAsCaller(ME);
                end
                % Set isPreviewing flag to true.
                obj.IsPreviewing = true;
            end
        end

        function closePreview(obj,varargin)
            %  CLOSEPREVIEW closes the live preview window.
            %  CLOSEPREVIEW(ousterLidar) closes the live preview window.
            %  Example:
            %  ousterLidar = ousterlidar('OS1-64');
            %  preview(ousterLidar); % opens a live preview window.
            %  closePreview(ousterLidar); % closes the preview window.

            % If preview is not on, there is nothing to close
            if ~obj.IsPreviewing
                warning(message('ousterlidar:ousterlidar:PreviewNeedsToBeOn'));
            end

            % Stop the update timer.
            try
                if ~isempty(obj.UpdatePreviewTimer) && isvalid(obj.UpdatePreviewTimer)
                    stop(obj.UpdatePreviewTimer);
                    obj.UpdatePreviewTimer = [];
                end

                % Hide the pcplayer object and delete it.
                if ~isempty(obj.Player)
                    delete(obj.Player);
                end

                % Restore streaming status
                setStream(obj,obj.StreamingStatusBeforePreview);

                % Flush input
                if ~isempty(obj.UdpObj)
                    flush(obj);
                end
            catch ME
                obj.IsPreviewing = false;
                throwAsCaller(ME);
            end

            % Set IsPreviewing to false
            obj.IsPreviewing = false;
        end
    end

    methods (Access = public, Hidden)
        function delete(obj)
            try
                % Cleanup
                % Close preview window if it is open.
                if obj.IsPreviewing
                    obj.closePreview;
                end

                % Stop streaming and flush the input buffer
                if obj.Streaming
                    obj.setStream(false);
                end

                % Disconnect the UDP object, and remove entry from the map.
                if ~isempty(obj.UdpObj) && obj.UdpObj.Connected
                    obj.UdpObj.disconnect;
                    % Remove entry from the connection map.
                    if isKey(obj.ConnectionMap, obj.Model)
                        remove(obj.ConnectionMap, obj.Model);
                    end
                end
            catch
                % Do not throw errors/warnings on destruction
            end
        end
    end

    methods (Access = private)
        function setStream(obj, flag)
            % setStream toggles streaming
            try
                obj.UdpObj.tuneInputFilter(struct('EnableStream',flag));
            catch ME
                throwAsCaller(ME);
            end
            obj.Streaming = flag;
        end

        function [pointClouds, timeStamps] = convertDatagramsToPointClouds(~, dgs, startPos, endPos)
            pointClouds = [];
            timeStamps = [];

            % Compute the first absolute timestamp.
            % The converter returns 2 values:
            % 1. Absolute start time : the actual time when streaming was
            %    started (software timestamp)
            % 2. Current HW timestamp - first HW timestamp when streaming
            %     started
            % Note that the absolute time is the number of seconds since
            % 00:00:00 UTC Jan 1,1970 (epoch time).

            %Calculate first timestamp.
            % Convert to datetime, and set the TimeZone to local and
            % millisecond granularity display.
            firstAbsTimestamp = datetime(dgs(1).StartTimeAbs,'ConvertFrom','epochtime','Format','dd-MMM-uuuu HH:mm:ss:SSS', 'TimeZone', 'UTC');
            firstAbsTimestamp.TimeZone = 'local'; % All future timestamps will be converted to local.
            rolloverCount = 0;
            for i = startPos:endPos
                % Create point clouds with coordinates and intensity
                % Laser vertical angles are stored in 2nd column of
                % LaserCalibrationData.
                ptCloudObj = pointCloud(dgs(i).LocationPoints(:, :, 1:3), 'Intensity',...
                    dgs(i).Intensity(: , :, 1));
                % Extract range data (internal property on pointCloud obj)
                rangeData = dgs(i).RangeData(: , : , :);
                % Convert to radian for angles
                rangeData(:,:,2:3) = rangeData(:,:,2:3) * pi / 180;
                % range, pitch, yaw
                ptCloudObj.RangeData = (rangeData(:,:,[1 3 2]));
                pointClouds = [pointClouds; ptCloudObj];

                % Populate the first timestamp.
                curAbsTs = firstAbsTimestamp + seconds(dgs(i).RelativeTime*1e-6) + rolloverCount*hours(1);
                if isempty(timeStamps)
                    timeStamps = curAbsTs;
                else
                    prevAbsTs = timeStamps(end); % Need this to calculate the hour rollover.
                    % The HW timestamp has microsec granularity, convert that
                    % to seconds
                    if curAbsTs < prevAbsTs
                        rolloverCount = rolloverCount+1;
                    end
                    timeStamps = [timeStamps; curAbsTs];
                end
            end
        end

        function updatePreviewWindow (obj, ~, ~)
            % This function updates the preview window with the latest
            % point clouds from the sensor.
            if obj.Player.isOpen
                numPointClouds = obj.NumPointCloudsAvailable;
                if numPointClouds > 0
                    try
                        dgs = read(obj.UdpObj,numPointClouds);    
                        for i=1:length(dgs)
                            ptCloud = pointCloud(dgs(i).LocationPoints(:,:,1:3),'Intensity',dgs(i).Intensity(:,:,1));
                            view(obj.Player, ptCloud);
                        end
                        drawnow('limitrate')
                    catch ME
                        throw ME
                    end
                end
            else
                obj.closePreview;
            end
        end

        function setCalibrationFile (obj, value)

            % Check for input file type.
            [~, ~, fileExtension] = fileparts(value);
            if ~strcmpi(fileExtension, '.json')
                error(message('ousterlidar:ousterlidar:InvalidCalibrationFileType', fileExtension));
            end

            % Check if xml file exists
            fid = fopen(value, 'r');
            if fid == -1
                if ~isempty(dir(value))
                    error(message('ousterlidar:ousterlidar:FileReadPermission', value));
                else
                    error(message('ousterlidar:ousterlidar:FileDoesNotExist', value));
                end
            else
               obj.CalibrationFileInternal = fopen(fid);
               fclose(fid);
            end

            % Read Calibration data from JSON file.
            %laserCalibrationStructure = struct;
            laserCalibrationStructure  = lidar.supportpackages.ousterlidar.internal.Utility.getOusterCorrectionsFromJSON(value) ;
            % Number of lasers based on Ouster device model.
            numLasersOfDeviceModel = 64;
            if strcmpi('OS0-64', obj.Model) ||  strcmpi ('OS1-64', obj.Model) || strcmpi('OS2-64', obj.Model )
                numLasersOfDeviceModel = 64;
            elseif  strcmpi('OS0-128', obj.Model) ||  strcmpi ('OS1-128', obj.Model) || strcmpi('OS2-128', obj.Model )
                numLasersOfDeviceModel = 128;
            end
            % Validate laser count in calibration data with number of
            % lasers for given device model.
            % if size(obj.LaserCalibrationData, 1) ~= numLasersOfDeviceModel
            % error(message('ousterlidar:ousterlidar:InvalidCalibrationFileIncorrectEnabledLaserCountForDeviceModel', obj.Model, numLasersOfDeviceModel));
            % end
            % Assign corrections
            obj.AzimuthAngles = laserCalibrationStructure.beam_azimuth_angles;
            obj.AltitudeAngles = laserCalibrationStructure.beam_altitude_angles;
            obj.LidarToSensorTransform = laserCalibrationStructure.imu_to_sensor_transform;
            obj.LidarOriginToBeamDistance = laserCalibrationStructure.lidar_origin_to_beam_origin_mm;
            obj.PixelShiftByRow = laserCalibrationStructure.data_format.pixel_shift_by_row ;
            obj.ColumnsPerPacket = laserCalibrationStructure.data_format.columns_per_packet;
            obj.ColumnsPerFrame = laserCalibrationStructure.data_format.columns_per_frame;
            obj.PixelsPerColumn = laserCalibrationStructure.data_format.pixels_per_column;
            obj.LidarMode = laserCalibrationStructure.lidar_mode;
            %obj.JSONCalibrationVersion = laserCalibrationStructure.json_calibration_version;
            obj.LidarStatus = laserCalibrationStructure.status ;

            % Once you have the corrections, tune the input filter, if it
            % has been created already. At construction, this will not run,
            % and the addInputFilter function will add these corrections
            % instead. Setting the CalibrationFile property post-construction
            % should tune the filter.
            try
                if ~isempty(obj.UdpObj)
                    obj.UdpObj.tuneInputFilter(struct('AzimuthAngles',obj.AzimuthAngles,...
                        'AltitudeAngles',obj.AltitudeAngles,...
                        'PixelShiftByRow', obj.PixelShiftByRow,...
                        'LidarToSensorTransform',obj.LidarToSensorTransform,...
                        'LidarOriginToBeamDistance',obj.LidarOriginToBeamDistance,...
                        'LidarMode', obj.LidarMode,...
                        'ColumnsPerPacket', obj.ColumnsPerPacket,...
                        'ColumnsPerFrame', obj.ColumnsPerFrame,...
                        'PixelsPerColumn', obj.PixelsPerColumn ));
                end
            catch ME
                throwAsCaller(ME);
            end
        end

        function calibrationFile = validateCalibrationFile(~ , calibFile)
            
            validateattributes(calibFile, {'char', 'string'}, ...
                {'nonempty', 'scalartext'});

            % Check the file extension.
            [~,~,fileExtension] = fileparts(calibFile);

            if ~strcmpi(fileExtension, '.json')
                error(message('ousterlidar:ousterlidar:invalidFileType', ...
                    'Calibration JSON', expectedExtension, fileExtension));
            end

            % Check if the file exists.
            [fid, msg] = fopen(calibFile, 'r');
            if fid == -1
                error(message('ousterlidar:ousterlidar:fileOpenFailed', ...
                    'Calibration JSON', calibFile , msg));
            else
                % Get full path name of file.
                calibrationFile = fopen(fid);
                fclose(fid);
            end

        end

        function paramsStruct = parseAndValidateInputs(~, model, calibrationFile, varargin)
            % Parse and validate inputs.

            narginchk(2,7); % Possible optional arguments are Port, CalibrationFile and Timeout.

            p = inputParser;
            defaultCalibrationFile = '';
            validateattributes(model, {'char', 'string'}, {'nonempty'});

            defaultPort = 7502;
            defaultTimeout = 10;
            validPort = @(x) isnumeric(x) && isscalar(x) && (x > 0) && (x<=65535);
            validScalarPosNum = @(x) isnumeric(x) && isscalar(x) && (x > 0);

            addParameter(p, 'Port', defaultPort, validPort);
            addParameter(p, 'Timeout', defaultTimeout, validScalarPosNum);

            parse(p, varargin{:})
            paramsStruct = p.Results;
            try
                paramsStruct.Model = validatestring(model, lidar.supportpackages.ousterlidar.internal.OusterConstants.Sensors);
                paramsStruct.CalibrationFile = calibrationFile ;
            catch
                throwAsCaller(MException('ousterlidar:ousterlidar:InvalidOusterModel',message('ousterlidar:ousterlidar:InvalidOusterModel', model).getString));
            end
        end

        function setupFilterAndConverter(obj)
            %This function sets up the filter and the converter on the UDP
            %object.
            % Set up the filter
            fullPathtoUtility = which('lidar.supportpackages.ousterlidar.internal.Utility');
            pathStr = fileparts(fullPathtoUtility);
            pluginDir = fullfile(pathStr, '..', '..', '..','..','bin', computer('arch'));
            filter = fullfile(pluginDir, 'ousterfilter');
            try
                % Add filter, and set up laser corrections as filter parameters
                obj.UdpObj.addInputFilter(filter, struct('EnableStream',false,'ProductID', obj.ProductID,...
                    'AzimuthAngles',obj.AzimuthAngles,...
                    'AltitudeAngles',obj.AltitudeAngles,...
                    'PixelShiftByRow', obj.PixelShiftByRow,...
                    'LidarToSensorTransform',obj.LidarToSensorTransform,...
                    'LidarOriginToBeamDistance',obj.LidarOriginToBeamDistance,...
                    'LidarMode', obj.LidarMode ,...
                    'ColumnsPerPacket', uint32(obj.ColumnsPerPacket),...
                    'ColumnsPerFrame', uint32(obj.ColumnsPerFrame),...
                    'PixelsPerColumn', uint32(obj.PixelsPerColumn)));
                obj.UdpObj.NativeDataType = 'struct';
                obj.UdpObj.DataFieldName = '';
                % Set up the converter
                disconnect(obj.UdpObj);
                converter = fullfile(pluginDir, 'oustermlconverter');
                obj.UdpObj.CustomConverterPlugIn = converter;
                connect(obj.UdpObj);
            catch ME
                throwAsCaller(ME);
            end
        end

        function [pointClouds, timeStamps] = readAllPointClouds(obj)
            % Reads all point clouds in the buffer
            pointClouds = [];
            timeStamps  = [];
            % Do not && ~obj.Streaming. Assuming that streaming produces
            % pointclouds between now and the read attempt, results in error. g2277771.
            if obj.NumPointCloudsAvailable == 0
                warnStruct = warning('backtrace','off');
                warning(message('ousterlidar:ousterlidar:NoPointCloudsAvailable'));
                warning(warnStruct);
                return;
            end
            try
                dgs = read(obj.UdpObj,obj.NumPointCloudsAvailable);
            catch ME
                throwAsCaller(ME);
            end
            [pointClouds, timeStamps] = obj.convertDatagramsToPointClouds(dgs, 1, length(dgs));
        end

        function [pointClouds, timeStamps] = readLatestPointClouds(obj, NumPointClouds)
            % Read latest point clouds from the buffer.
            % If streaming, just read the latest NumPointClouds datagrams
            if obj.Streaming
                try
                    dgs = read(obj.UdpObj,obj.NumPointCloudsAvailable);
                catch ME
                    throwAsCaller(ME);
                end
                numdgs = length(dgs);
                if numdgs >= NumPointClouds % Older point clouds need to be discarded
                    % Warn that older point clouds will be discarded
                    warning(message('ousterlidar:ousterlidar:ReadLatestRemovesOlderPointClouds'));
                    [pointClouds, timeStamps] = obj.convertDatagramsToPointClouds(dgs, numdgs-NumPointClouds+1, numdgs);
                else
                    % If less than NumPointClouds were acquired, warn and read
                    % whatever is available
                    warning(message('ousterlidar:ousterlidar:NotEnoughPCsAvailable', NumPointClouds));
                    [pointClouds, timeStamps] = obj.convertDatagramsToPointClouds(dgs, 1, numdgs);
                end
            else
                % Start streaming, acquire latest NumPointClouds and stop streaming
                obj.setStream(true);
                alldgs = [];

                % If there are point clouds in the buffer, warn that they
                % will be discarded irrespective of the number requested.
                if obj.NumPointCloudsAvailable>0
                    warning(message('ousterlidar:ousterlidar:ReadLatestRemovesOlderPointClouds'));
                end

                % Keep streaming and collecting till you get NumPointClouds
                % datagrams
                elapsedTime = tic;
                try
                    while numel(alldgs) < NumPointClouds
                        % Time out if read does not return within the
                        % Timeout period.
                        if toc(elapsedTime) > obj.Timeout
                            ex = MException(message('ousterlidar:ousterlidar:Timeout'));
                            throw(ex);
                        end
                        dgs = read(obj.UdpObj, obj.NumPointCloudsAvailable);
                        alldgs = [alldgs dgs]; %#ok<*AGROW>
                    end

                    numdgs = numel(alldgs);
                    % Keep only latest, discard rest
                    [pointClouds, timeStamps] = obj.convertDatagramsToPointClouds(alldgs, numdgs-NumPointClouds+1, numdgs);
                    % Stop streaming and flush the buffer
                    obj.setStream(false);
                    flush(obj);
                catch ME
                    % If there was an error during reading, restore
                    % streaming status and error out.
                    obj.setStream(false);
                    flush(obj);
                    throwAsCaller(ME);
                end
            end
        end

        function [pointClouds, timeStamps] = readOldestPointClouds(obj, NumPointClouds)
            % Reads oldest point clouds in the buffer.
            % If streaming, get oldest point clouds from the buffer
            if obj.Streaming
                try
                    dgs = read(obj.UdpObj,NumPointClouds);
                catch ME
                    throwAsCaller(ME);
                end
                [pointClouds, timeStamps] = obj.convertDatagramsToPointClouds(dgs, 1, length(dgs));
            else
                % If not streaming, but there's some point clouds left in the
                % buffer, read those.
                if obj.NumPointCloudsAvailable > 0
                    % Read NumPointClouds or, if the buffer doesn't have as
                    % many, read what's available.
                    if NumPointClouds > obj.NumPointCloudsAvailable
                        warning(message('ousterlidar:ousterlidar:NotEnoughPCsAvailable',NumPointClouds));
                    end
                    NumPointCloudsToRead = min(obj.NumPointCloudsAvailable, NumPointClouds);
                    try
                        dgs = read(obj.UdpObj, NumPointCloudsToRead);
                    catch ME
                        throwAsCaller(ME);
                    end
                    [pointClouds, timeStamps] = obj.convertDatagramsToPointClouds(dgs, 1, length(dgs));
                else
                    % Start streaming, acquire NumPointClouds and stop streaming
                    obj.setStream(true);
                    alldgs = [];
                    elapsedTime = tic;
                    try
                        % Keep streaming and collecting till you get NumPointClouds
                        % datagrams.
                        while numel(alldgs) < NumPointClouds
                            % Timeout if read does not return within the
                            % Timeout period.
                            if toc(elapsedTime) > obj.Timeout
                                ex = MException(message('ousterlidar:ousterlidar:Timeout'));
                                throw(ex);
                            end
                            dgs = read(obj.UdpObj, obj.NumPointCloudsAvailable);
                            alldgs = [alldgs dgs];
                        end

                        [pointClouds, timeStamps] = obj.convertDatagramsToPointClouds(alldgs, 1, NumPointClouds);
                        % Stop streaming and flush the buffer.
                        obj.setStream(false);
                        flush(obj);

                    catch ME
                        % If there was an error during reading, restore
                        % streaming status and error out.
                        obj.setStream(false);
                        flush(obj);
                        throwAsCaller(ME);
                    end
                end
            end
        end

        % Disable save method
        function out = saveobj(~)
            warnStruct = warning('backtrace','off');
            warning(message('ousterlidar:ousterlidar:SaveNotSupported'));
            warning(warnStruct);
            out = [];
        end
    end

    methods(Static, Hidden)
        % Disable load method
        function out = loadobj(~)
            out = ousterlidar.empty();
            warnStruct = warning('backtrace','off');
            warning(message('ousterlidar:ousterlidar:LoadNotSupported'));
            warning(warnStruct); % Restore warning status
        end
    end
end
