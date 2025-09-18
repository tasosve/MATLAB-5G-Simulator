clear; clc; close all;

%% PARAMETERS

datasetFolder = 'dataset18';

if ~exist(datasetFolder, 'dir')
    mkdir(datasetFolder);
end

numRuns = 100;
simTime = 100; % seconds
dt = 0.3; % time step
numSteps = round(simTime / dt);
areaSize = 230; % Total area 230x230 m
roadWidth = 10; % 10 m roads/free space
buildingSize = 100; % 100 m building side length
buildingHeight = 20; % 20 m building height
gNB_height_above_building = 5; % 5 m antenna height above rooftop
UE_height = 1.5; % UE height
fixedUESpeed = 30; % km/h
num_gNB = 4;
numUE = 18;
carrierFreq = 6e9; % Hz
BW = 20e6; % 20 MHz
noiseFigure = 9; % dB
kT_dBmHz = -174; % dBm/Hz
noisePower_dBm = kT_dBmHz + 10*log10(BW) + noiseFigure;
noisePower_W = 10^((noisePower_dBm - 30)/10);
txPower_dBm = 49 * ones(num_gNB,1);
txPower_W = 10.^((txPower_dBm - 30)/10);
mapSINRtoCQI = @(sinr) min(max(round((sinr + 10)/2),1),15);
LOS_hold_time = 8.0; % seconds - minimum duration to hold LOS/NLOS state before change allowed
LOS_refresh_steps = ceil(LOS_hold_time / dt);


useLOS = true; % Set true to apply LOS blockage checks probabilistic, false = all LOS assumed
useShadowing = true; % Set to false to turn off shadowing effect in calculations
useFading = false; % Set to false to turn off fast fading effect in calculations


% Building placement (2x2 grid with 10m roads around and between)

startOffset = roadWidth; % 10 m margin from edges
gap = buildingSize + roadWidth; % 110 m between building starts

buildings = zeros(4,4);
buildings(1,:) = [startOffset, startOffset, buildingSize, buildingSize]; % Bottom-left
buildings(2,:) = [startOffset + gap, startOffset, buildingSize, buildingSize]; % Bottom-right
buildings(3,:) = [startOffset, startOffset + gap, buildingSize, buildingSize]; % Top-left
buildings(4,:) = [startOffset + gap, startOffset + gap, buildingSize, buildingSize]; % Top-right

% gNB positions only on building centers (no mid-block gNBs)

gNB_height = buildingHeight + gNB_height_above_building;

gNB_pos = [...
    buildings(1,1)+5, buildings(1,2)+5;
    buildings(2,1)+buildingSize-5, buildings(2,2)+5;
    buildings(3,1)+5, buildings(3,2)+buildingSize-5;
    buildings(4,1)+buildingSize-5, buildings(4,2)+buildingSize-5];

gNB_pos = [gNB_pos, gNB_height*ones(size(gNB_pos,1),1)];

% Roads centerlines (x fixed for vertical roads, y fixed for horizontal roads)

roadCenters_vertical = [...
    buildings(1,1)-roadWidth/2;
    buildings(1,1)+buildingSize+roadWidth/2;
    buildings(2,1)+buildingSize+roadWidth/2;
    buildings(1,1)+buildingSize/2];

roadCenters_horizontal = [...
    buildings(1,2)-roadWidth/2;
    buildings(1,2)+buildingSize+roadWidth/2;
    buildings(3,2)+buildingSize+roadWidth/2;
    buildings(1,2)+buildingSize/2];

% Define road centerlines explicitly (vertical and horizontal lines)

verticalRoadXs = [startOffset - roadWidth/2, ...
    startOffset + buildingSize + roadWidth/2, ...
    startOffset + 2*buildingSize + roadWidth/2];

verticalRoadXs = unique([verticalRoadXs, gNB_pos(:,1)']); % add gNB x (building centers)

horizontalRoadYs = [startOffset - roadWidth/2, ...
    startOffset + buildingSize + roadWidth/2, ...
    startOffset + 2*buildingSize + roadWidth/2];

horizontalRoadYs = unique([horizontalRoadYs, gNB_pos(:,2)']);

%% FILTER ROAD CENTERLINES TO EXCLUDE BUILDING INTERVALS

validVerticalRoadXs = [];

for x_val = verticalRoadXs
    insideBuilding = false;
    for b = 1:size(buildings,1)
        bx = buildings(b,1);
        bw = buildings(b,3);
        if x_val > bx && x_val < bx + bw
            insideBuilding = true;
            break;
        end
    end
    if ~insideBuilding
        validVerticalRoadXs(end+1) = x_val; %#ok
    end
end

validVerticalRoadXs = unique(validVerticalRoadXs);

validHorizontalRoadYs = [];

for y_val = horizontalRoadYs
    insideBuilding = false;
    for b = 1:size(buildings,1)
        by = buildings(b,2);
        bh = buildings(b,4);
        if y_val > by && y_val < by + bh
            insideBuilding = true;
            break;
        end
    end
    if ~insideBuilding
        validHorizontalRoadYs(end+1) = y_val; %#ok
    end
end

validHorizontalRoadYs = unique(validHorizontalRoadYs);

% Add boundary roads at the top and right edges explicitly if not already included
topEdgeRoad = areaSize - roadWidth/2;
rightEdgeRoad = areaSize - roadWidth/2;

if ~ismember(topEdgeRoad, validHorizontalRoadYs)
    validHorizontalRoadYs = sort([validHorizontalRoadYs, topEdgeRoad]);
end

if ~ismember(rightEdgeRoad, validVerticalRoadXs)
    validVerticalRoadXs = sort([validVerticalRoadXs, rightEdgeRoad]);
end

% Path loss configuration (requires 5G Toolbox)

plconf = nrPathLossConfig;
plconf.Scenario = "UMa";

% Event A3 handover parameters
A3_offset = 0; % dB offset for easier handover trigger
TTT_steps = 3; % 0.3 s trigger

minTurnInterval = 10; % Minimum steps between turns (1s)

%% Generate random start points for UEs on roads
fixedStartPoints = zeros(numUE, 3);
min_road_coord = roadWidth/2;
max_road_coord = areaSize - roadWidth/2;
rng('shuffle');

for u = 1:numUE
    found_valid_pos = false;
    while ~found_valid_pos
        if rand > 0.5
            % Vertical road
            if ~isempty(validVerticalRoadXs)
                x_chosen = validVerticalRoadXs(randi(length(validVerticalRoadXs)));
                y_chosen = min_road_coord + rand() * (max_road_coord - min_road_coord);
            else
                continue;
            end
        else
            % Horizontal road
            if ~isempty(validHorizontalRoadYs)
                y_chosen = validHorizontalRoadYs(randi(length(validHorizontalRoadYs)));
                x_chosen = min_road_coord + rand() * (max_road_coord - min_road_coord);
            else
                continue;
            end
        end
        
        % Check inside building
        is_in_building = false;
        for b_idx = 1:size(buildings,1)
            bx = buildings(b_idx,1);
            by = buildings(b_idx,2);
            bw = buildings(b_idx,3);
            bh = buildings(b_idx,4);
            if x_chosen >= bx && x_chosen <= bx + bw && y_chosen >= by && y_chosen <= by + bh
                is_in_building = true;
                break;
            end
        end
        
        % Min distance check (10 m)
        min_distance = 10;
        too_close_to_other_ue = false;
        for existing_u = 1:u-1
            dist = sqrt((fixedStartPoints(existing_u,1) - x_chosen)^2 + (fixedStartPoints(existing_u,2) - y_chosen)^2);
            if dist < min_distance
                too_close_to_other_ue = true;
                break;
            end
        end
        
        if ~is_in_building && ~too_close_to_other_ue
            fixedStartPoints(u,:) = [x_chosen, y_chosen, UE_height];
            found_valid_pos = true;
        end
    end
end

%% MAIN SIMULATION LOOP

for runIdx = 1:numRuns
    fprintf('Starting simulation run %d/%d...\n', runIdx, numRuns);
    
    % UE Initialization
    uePos = zeros(numSteps,3,numUE);
    ueDir = zeros(numUE,1); % 1=vertical, 2=horizontal
    ueSpeed = fixedUESpeed * ones(numUE,1); % km/h
    ueSpeed_mps = ueSpeed*1000/3600;
    
    for u=1:numUE
        uePos(1,:,u) = fixedStartPoints(u,:);
    end
    
    for u=1:numUE
        xStart = uePos(1,1,u);
        yStart = uePos(1,2,u);
        dist_to_v_road = min(abs(xStart - validVerticalRoadXs));
        dist_to_h_road = min(abs(yStart - validHorizontalRoadYs));
        if dist_to_v_road < dist_to_h_road
            ueDir(u) = 1;
        else
            ueDir(u) = 2;
        end
    end
    
    prevMove = zeros(numUE,2);
    
    for u=1:numUE
        if ueDir(u) == 1
            if uePos(1,2,u) < areaSize/2
                prevMove(u,:) = [0 1];
            else
                prevMove(u,:) = [0 -1];
            end
        else
            if uePos(1,1,u) < areaSize/2
                prevMove(u,:) = [1 0];
            else
                prevMove(u,:) = [-1 0];
            end
        end
    end
    
    stepsSinceLastTurn = minTurnInterval*ones(numUE,1);
    
    allMoves = [0 1; 0 -1; -1 0; 1 0];
    
    for t=2:numSteps
        for u=1:numUE
            prev = uePos(t-1,:,u);
            v = ueSpeed_mps(u)*dt;
            pmv = prevMove(u,:);
            stepsSinceLastTurn(u) = stepsSinceLastTurn(u)+1;
            
            % Check near intersection
            isAtIntersection = false;
            tol = roadWidth/2;
            for vx = validVerticalRoadXs
                for hy = validHorizontalRoadYs
                    if abs(prev(1)-vx)<=tol && abs(prev(2)-hy)<=tol
                        isAtIntersection = true;
                        break;
                    end
                end
                if isAtIntersection, break; end
            end
            
            turnAllowed = (isAtIntersection && stepsSinceLastTurn(u)>=minTurnInterval);
            
            if turnAllowed
                reverseMove = -pmv;
                validMoves = [];
                for m = 1:size(allMoves,1)
                    moveCandidate = allMoves(m,:);
                    if all(moveCandidate == reverseMove)
                        continue;
                    end
                    newX = prev(1) + moveCandidate(1)*v;
                    newY = prev(2) + moveCandidate(2)*v;
                    x_snap_candidate = newX;
                    y_snap_candidate = newY;
                    if abs(moveCandidate(1)) > 0 % Horizontal
                        [~, idxH] = min(abs(newY - validHorizontalRoadYs'));
                        if ~isempty(idxH)
                            y_snap_candidate = validHorizontalRoadYs(idxH);
                        else
                            continue;
                        end
                    else % Vertical
                        [~, idxV] = min(abs(newX - validVerticalRoadXs'));
                        if ~isempty(idxV)
                            x_snap_candidate = validVerticalRoadXs(idxV);
                        else
                            continue;
                        end
                    end
                    inBuildingCandidate = false;
                    tol_check = 1e-6;
                    for b_chk=1:size(buildings,1)
                        bx_chk = buildings(b_chk,1);
                        by_chk = buildings(b_chk,2);
                        bw_chk = buildings(b_chk,3);
                        bh_chk = buildings(b_chk,4);
                        if x_snap_candidate > bx_chk+tol_check && x_snap_candidate < bx_chk+bw_chk-tol_check && ...
                                y_snap_candidate > by_chk+tol_check && y_snap_candidate < by_chk+bh_chk-tol_check
                            inBuildingCandidate = true;
                            break;
                        end
                    end
                    if ~inBuildingCandidate
                        validMoves = [validMoves; moveCandidate];
                    end
                end
                if isempty(validMoves)
                    moveVec = pmv;
                else
                    idxPick = randi(size(validMoves,1));
                    moveVec = validMoves(idxPick,:);
                    stepsSinceLastTurn(u) = 0;
                end
            else
                moveVec = pmv;
            end
            
            x_new = prev(1) + moveVec(1)*v;
            y_new = prev(2) + moveVec(2)*v;
            
            % Snap final position
            if abs(moveVec(1)) > 0 % horizontal
                [~, idxH] = min(abs(y_new - validHorizontalRoadYs'));
                if ~isempty(idxH)
                    y_new = validHorizontalRoadYs(idxH);
                end
                x_new = max(min(x_new, areaSize - roadWidth/2), roadWidth/2);
            else % vertical
                [~, idxV] = min(abs(x_new - validVerticalRoadXs'));
                if ~isempty(idxV)
                    x_new = validVerticalRoadXs(idxV);
                end
                y_new = max(min(y_new, areaSize - roadWidth/2), roadWidth/2);
            end
            
            % Check inside building
            inBuilding = false;
            tol_check_pos = 1e-6;
            for b_chk=1:size(buildings,1)
                bx_chk = buildings(b_chk,1);
                by_chk = buildings(b_chk,2);
                bw_chk = buildings(b_chk,3);
                bh_chk = buildings(b_chk,4);
                if x_new > bx_chk+tol_check_pos && x_new < bx_chk+bw_chk-tol_check_pos && ...
                        y_new > by_chk+tol_check_pos && y_new < by_chk+bh_chk-tol_check_pos
                    inBuilding = true;
                    break;
                end
            end
            
            if inBuilding
                uePos(t,:,u) = uePos(t-1,:,u);
            else
                uePos(t,:,u) = [x_new, y_new, UE_height];
                prevMove(u,:) = moveVec;
                if abs(moveVec(2)) > abs(moveVec(1))
                    ueDir(u) = 1;
                else
                    ueDir(u) = 2;
                end
            end
        end
    end
    
    % Initialize logs - servingCellLog now a cell array for names
    servingCellLog = cell(numSteps,numUE);
    handoverEvents = zeros(numSteps,numUE);
    SINRLog = zeros(numSteps,numUE);
    throughputLog = zeros(numSteps,numUE);
    CQILog = zeros(numSteps, numUE);
    
    % New metrics: distances and received powers
    distances = zeros(numSteps, numUE, num_gNB);
    rxPowersAll = zeros(numSteps, numUE, num_gNB);
    
    % Initial serving cell selection
    for u=1:numUE
        rxPowers_W_init = zeros(num_gNB,1);
        for g=1:num_gNB
            % Use probabilistic LOS if enabled
            if useLOS
                dxy = norm(gNB_pos(g,1:2) - uePos(1,1:2,u));
                los = probabilisticLOS(dxy);
            else
                los = true; % ignore LOS checks, all assumed LOS
            end
            
            [pl, shadowing] = nrPathLoss(plconf, carrierFreq, los, gNB_pos(g,:)', uePos(1,:,u)');
            
            % Fast fading generation
            fading_dB = generateFastFading(los);

            % Antenna gain (omnidirectional simplified)
            antenna_gain_dB = 0;

            pl_total_dB = pl + shadowing;
            rxPower_dBm = txPower_dBm(g) + antenna_gain_dB - pl_total_dB + fading_dB;
            rxPowers_W_init(g) = 10.^((rxPower_dBm - 30)/10);
        end
        [~, index] = max(rxPowers_W_init);        
        servingCellLog{1, u} = sprintf('gNB%d', index);
    end
    
    LOSState = true(num_gNB, numUE); % start with all LOS=true
    LOSTimer = zeros(num_gNB, numUE);
    
    % Main time loop: SINR, handover
    A3_timer = zeros(numUE,1);
    A3_candidate = zeros(numUE,1);
    
    for t=1:numSteps
        for u=1:numUE
            currentUE = uePos(t,:,u)';
            rxPowers_W = zeros(num_gNB,1);
            
            for g=1:num_gNB
                % Update LOS state with temporal correlation only if useLOS enabled
                if useLOS
                    LOSTimer(g,u) = LOSTimer(g,u) + 1;
                    if LOSTimer(g,u) >= LOS_refresh_steps
                        dxy = norm(gNB_pos(g,1:2) - currentUE(1:2)');
                        LOSState(g,u) = probabilisticLOS(dxy);
                        LOSTimer(g,u) = 0;
                    end
                    los = LOSState(g,u);
                else
                    los = true; % default all LOS
                end
                
                % Compute path loss
                [pl, shadowing] = nrPathLoss(plconf, carrierFreq, los, gNB_pos(g,:)', currentUE);
                
                % Shadow fading
                if useShadowing
                    shadowing_dB = shadowing;
                else
                    shadowing_dB = 0;

                end
                
                % Fast fading
                if useFading
                    fading_dB = generateFastFading(los);
                else
                    fading_dB = 0;
                end
                
                antenna_gain_dB = 0; % omnidirectional antenna simplified
                
                pl_total_dB = pl + shadowing_dB;
                rxPower_dBm = txPower_dBm(g) + antenna_gain_dB - pl_total_dB + fading_dB;
                rxPowers_W(g) = 10.^((rxPower_dBm - 30)/10);
                
                % Update distances metric
                distances(t,u,g) = norm(uePos(t,1:2,u) - gNB_pos(g,1:2)');
            end
            
            % Store received powers for UE and gNBs
            rxPowersAll(t,u,:) = rxPowers_W;
            
            % Serving cell selection logic (Event A3 based handover)
            if t == 1
                servingCellName = servingCellLog{1,u};
                servingCell = sscanf(servingCellName, 'gNB%d');
            else
                servingCellName = servingCellLog{t-1,u};
                servingCell = sscanf(servingCellName, 'gNB%d');
            end
            
            servingPower_W = rxPowers_W(servingCell);
            servingPower_dB = 10*log10(servingPower_W);
            
            neighborPowers_dB = 10*log10(rxPowers_W);
            neighborPowers_dB(servingCell) = -Inf;
            
            [maxNeighborPower_dB, maxIdx] = max(neighborPowers_dB);
            
            hoTriggered = false;
            if maxNeighborPower_dB > servingPower_dB + A3_offset
                if A3_candidate(u) == maxIdx
                    A3_timer(u) = A3_timer(u) + 1;
                else
                    A3_candidate(u) = maxIdx;
                    A3_timer(u) = 1;
                end
                if A3_timer(u) >= TTT_steps
                    servingCell = maxIdx;
                    hoTriggered = true;
                    A3_timer(u) = 0;
                    A3_candidate(u) = 0;
                end
            else
                A3_timer(u) = 0;
                A3_candidate(u) = 0;
            end
            servingCellLog{t,u} = sprintf('gNB%d', servingCell);
            
            if hoTriggered
                handoverEvents(t,u) = 1;
            end
            
            % SINR calculation
            interferencePower_W = sum(rxPowers_W) - rxPowers_W(servingCell);
            sinrLinear = rxPowers_W(servingCell) / (noisePower_W + interferencePower_W);
            sinr_dB = 10*log10(sinrLinear);
            SINRLog(t,u) = sinr_dB;
            throughputLog(t,u) = BW * log2(1 + sinrLinear) / 1e6;
            
            cqiValue = mapSINRtoCQI(sinr_dB);
            CQILog(t, u) = cqiValue;
        end
    end
    
    % Save run data (removed dt, CQI_interval, carrierFreq, buildingHeight)
    filename = fullfile(datasetFolder, sprintf('simulation_data_run%d.mat', runIdx));
    save(filename, 'servingCellLog', 'handoverEvents', 'SINRLog', 'CQILog', ...
        'throughputLog', 'uePos', 'gNB_pos', 'num_gNB', 'numUE', 'numSteps', ...
        'simTime', 'distances', 'rxPowersAll');
    fprintf('Run %d data saved to %s\n', runIdx, filename);
    
    %% --- CSV Export of Metrics ---
    
    metricsStruct.handoverEvents = handoverEvents;
    metricsStruct.SINRLog = SINRLog;
    metricsStruct.throughputLog = throughputLog;
    metricsStruct.servingCellLog = servingCellLog; % cell, so handle separately below
    metricsStruct.CQILog = CQILog; % CQILog may be larger, truncated below
    metricsStruct.distances = distances;
    metricsStruct.rxPowersAll = rxPowersAll;
    
    allMetricNames = {};
    allMetricValues = [];
    
    fields = fieldnames(metricsStruct);
    
    for f = 1:numel(fields)
        metric = metricsStruct.(fields{f});
        
        % Handle servingCellLog separately since it's a cell array of strings
        if strcmp(fields{f}, 'servingCellLog')
            for ue = 1:size(metric,2)
                name = sprintf('servingCell_ue_%d', ue);
                % Convert cell array strings column to categorical and then numeric codes or save empty
                % For CSV numeric table, convert string to numeric codes:
                cellColumn = metric(:, ue);
                % Filling missing with NaN if empty
                numericCodes = nan(size(cellColumn));
                uniqueNames = unique(cellColumn);
                for k = 1:length(uniqueNames)
                    if isempty(uniqueNames{k})
                        continue;
                    end
                    idxs = strcmp(cellColumn, uniqueNames{k});
                    numericCodes(idxs) = k;
                end
                allMetricNames{end+1} = name;
                allMetricValues = [allMetricValues numericCodes]; %#ok
            end
            
            continue
        end
        
        if ndims(metric) == 2
            % 2D data (numSteps x numUE)
            [rows, cols] = size(metric);
            limitRows = min(rows,numSteps);
            for ue = 1:cols
                name = sprintf('%s_ue_%d', fields{f}, ue);
                allMetricNames{end+1} = name; %#ok
                columnVals = metric(1:limitRows, ue);
                if limitRows < numSteps
                    columnVals(end+1:numSteps,1) = NaN;
                end
                allMetricValues = [allMetricValues columnVals]; %#ok
            end
        elseif ndims(metric) == 3
            % 3D data (numSteps x numUE x num_gNB)
            [rows, cols, depth] = size(metric);
            limitRows = min(rows,numSteps);
            for ue = 1:cols
                for g = 1:depth
                    baseName = fields{f};
                    name = sprintf('%s_ue_%d_gNB_%d', baseName, ue, g);
                    allMetricNames{end+1} = name; %#ok
                    columnVals = squeeze(metric(1:limitRows, ue, g));
                    if limitRows < numSteps
                        columnVals(end+1:numSteps,1) = NaN;
                    end
                    allMetricValues = [allMetricValues columnVals]; %#ok
                end
            end
        end
    end
    
    T = array2table(allMetricValues, 'VariableNames', allMetricNames);
    csvFilename = fullfile(datasetFolder, sprintf('simulation_metrics_run%d.csv', runIdx));
    writetable(T, csvFilename);
    fprintf('CSV metrics saved to %s\n', csvFilename);
    
    % Enhanced Visualization for first run with interactive UE visibility toggles
    if runIdx == 1
        figure('Color','w','Position',[100 100 1000 700]); hold on;
        
        % Plot buildings with transparency
        for b=1:size(buildings,1)
            rectangle('Position', buildings(b,:), ...
                'FaceColor', [0.6 0.6 0.6 0.5], 'EdgeColor', 'k', 'LineWidth', 1.5);
            text(buildings(b,1) + buildingSize/2, buildings(b,2) + buildingSize/2, ...
                sprintf('Building %d', b), 'HorizontalAlignment', 'center', ...
                'FontWeight', 'bold', 'FontSize', 11, 'Color', 'k', 'BackgroundColor', 'w', 'Margin', 1);
        end
        
        % Roads (fill with lighter color)
        roadColor = [0.9 0.9 0.9];
        % Horizontal roads
        for y_road = validHorizontalRoadYs
            rectangle('Position', [-roadWidth, y_road - roadWidth/2, areaSize + 2*roadWidth, roadWidth], ...
                'FaceColor', roadColor, 'EdgeColor', 'none');
        end
        % Vertical roads
        for x_road = validVerticalRoadXs
            rectangle('Position', [x_road - roadWidth/2, -roadWidth, roadWidth, areaSize + 2*roadWidth], ...
                'FaceColor', roadColor, 'EdgeColor', 'none');
        end
        
        % Plot gNB positions
        scatter(gNB_pos(:,1), gNB_pos(:,2), 140, 'r^', 'filled', 'DisplayName', 'gNB');
        for g=1:size(gNB_pos,1)
            text(gNB_pos(g,1), gNB_pos(g,2) + 8, sprintf('gNB %d', g), ...
                'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 10, 'Color', 'r', ...
                'BackgroundColor', 'w', 'Margin', 1);
        end
        
        % Plot UE trajectories with distinct colors and store the plot handles
        colors = lines(numUE);
        uePlots = gobjects(numUE,1); % Preallocate graphic handles
        for u=1:numUE
            uePlots(u) = plot(uePos(:,1,u), uePos(:,2,u), '-', 'Color', colors(u,:), ...
                'LineWidth',1.7, 'DisplayName', sprintf('UE %d path', u));
            scatter(uePos(1,1,u), uePos(1,2,u), 100, colors(u,:), 's', 'filled', ...
                'DisplayName', sprintf('UE %d start', u));
            scatter(uePos(end,1,u), uePos(end,2,u), 100, colors(u,:), 'o', 'filled', ...
                'DisplayName', sprintf('UE %d end', u));
        end
        
        axis([-10 areaSize+10 -10 areaSize+10]); axis square;
        grid on; box on; set(gca,'GridAlpha',0.4, 'LineWidth',1.2);
        
        xticks(0:50:areaSize);
        yticks(0:50:areaSize);
        
        xlabel('X Position (meters)', 'FontSize', 12, 'FontWeight', 'bold');
        ylabel('Y Position (meters)', 'FontSize', 12, 'FontWeight', 'bold');
        
        title('Urban Grid Scenario: Buildings, Roads, gNBs, and UE Trajectories', ...
            'FontSize', 14, 'FontWeight', 'bold');
        
        % Legend
        h_gNB = findobj(gca,'DisplayName','gNB');
        h_ue_starts = findobj(gca,'-regexp','DisplayName','UE \\d+ start');
        h_ue_ends = findobj(gca,'-regexp','DisplayName','UE \\d+ end');
        h_ue_paths = findobj(gca,'-regexp','DisplayName','UE \\d+ path');
        
        legend([h_gNB(1); flipud(h_ue_paths); flipud(h_ue_starts); flipud(h_ue_ends)], ...
            {'gNBs', 'UE Trajectories', 'UE Start', 'UE End'}, 'Location', 'northeastoutside', ...
            'FontSize', 10);
        set(gca,'FontSize',10);
        
        % Add buttons for UE path visibility toggling
        baseX = areaSize + 15; % X coordinate for buttons outside plot
        baseY = areaSize - 10; % Starting Y coordinate
        buttonHeight = 25;
        spacing = 30;
        
        % Store toggle state for buttons (false means all UEs visible)
        toggleState = false(numUE,1);
        
        % Create a container for callback access
        handles.uePlots = uePlots;
        handles.toggleState = toggleState;
        handles.numUE = numUE;
        
        for u = 1:numUE
            uicontrol('Style', 'pushbutton', 'String', sprintf('Toggle UE %d', u), ...
                'Position', [baseX, baseY - (u-1)*spacing, 100, buttonHeight], ...
                'Callback', @(src,event) toggleUEPathVisibility(src, u, handles));
        end
        
    end
end

fprintf('All %d simulation runs completed and data saved.\n', numRuns);

% Print UE start and end positions
for u=1:numUE
    fprintf('UE %d start position: (%.2f, %.2f), end position: (%.2f, %.2f)\n', ...
        u, uePos(1,1,u), uePos(1,2,u), uePos(end,1,u), uePos(end,2,u));
end

%% --- Helper functions ---

% 3GPP TR 38.901 Probabilistic LOS determination for UMa scenario
function isLOS = probabilisticLOS(d2d)
    % d2d = 2D horizontal distance in meters between Tx and Rx
    if d2d <= 18
        pLOS = 1;
    else
        pLOS = (18/d2d) + exp(-d2d/63)*(1 - (18/d2d));
    end
    isLOS = rand() < pLOS;
end

% Generate fast fading dB gain (Rician for LOS, Rayleigh for NLOS)
function fading_dB = generateFastFading(los)
    if los
        K = 7.94; % Linear scale K-factor ~ 9 dB
        s = sqrt(K / (K + 1));
        sigma = sqrt(1 / (2*(K + 1)));
        fading_real = sigma*randn() + s;
        fading_imag = sigma*randn();
        fading_mag = sqrt(fading_real^2 + fading_imag^2);
    else
        fading_mag = sqrt(0.5 * (randn()^2 + randn()^2));
    end
    fading_dB = 20*log10(fading_mag + eps); % Convert magnitude to dB
end

function toggleUEPathVisibility(src, ueIdx, handles)
    persistent lastVisibleIdx
    if isempty(lastVisibleIdx)
        lastVisibleIdx = 0; % No UE isolated at start
    end
    
    uePlots = handles.uePlots;
    n = handles.numUE;
    
    if lastVisibleIdx == ueIdx
        % Restore all UE paths visible
        for k=1:n
            uePlots(k).Visible = 'on';
        end
        lastVisibleIdx = 0; % Reset state
        src.String = sprintf('Toggle UE %d', ueIdx);
    else
        % Hide all except selected UE path
        for k=1:n
            if k == ueIdx
                uePlots(k).Visible = 'on';
            else
                uePlots(k).Visible = 'off';
            end
        end
        lastVisibleIdx = ueIdx;
        src.String = 'Show all UEs';
    end
end
