%% 加载和浏览记录的数据
% 将各部分信息（如磁盘名、目录名等）合成完整的路径
% 源文件为excel
% dataFolder = fullfile('D:', 'Matlab storage', '生成轨迹线','数据整理.xlsx');
% insDataTable = readtable(dataFolder,'sheet',2);
% 源文件为txt
dataFolder = fullfile('D:', 'Matlab storage', '生成轨迹线','处理数据.txt');
insDataTable = readtable(dataFolder);

%% 创建空视窗并显示
vSet = pcviewset;
hFigBefore = figure('Name', 'View Set Display');
hAxBefore = axes(hFigBefore);


% 初始化转换参数
absTform   = rigid3d;  % 坐标系的绝对变换
relTform   = rigid3d;  % 连续扫描之间的相对变换

viewId = 1;
skipFrames  = 5;
numFrames   = height(insDataTable);
displayRate = 100;      % 每100帧更新一次显示


%% 计算各视图间的变换矩阵
for n = 1 : skipFrames : numFrames
        
    firstFrame = (n==1);
    if firstFrame
        vSet = addView(vSet, viewId, absTform);
        viewId = viewId + 1;
        continue;
    end
    
    % 使用INS估计初始变换以进行配准
    insData = insDataTable(n-skipFrames:n, :);
    % INS读数显示X指向前方，Y指向左侧，Z向上。
    % 下面的转换说明了如何转换为激光雷达框架。
    insToLidarOffset = [0.022 0.332 0.700]; % See DATAFORMAT.txt
    Tnow = [-insData.Y(end), insData.X(end), insData.Z(end)].' + insToLidarOffset';
    Tbef = [-insData.Y(1)  , insData.X(1)  , insData.Z(1)].' + insToLidarOffset';
    
    %（选）去除GPS的XYZ信息
%     insToLidarOffset = [0.022 0.332 0.700]; % See DATAFORMAT.txt
%     Tnow = [0, 0, 0].' + insToLidarOffset';
%     Tbef = [0, 0, 0].' + insToLidarOffset';

    % 由于车辆预计将沿地面移动，因此横摇和纵摇的变化最小。
    % 忽略横摇和俯仰的变化，仅使用航向。
    Rnow = rotmat(quaternion([insData.Heading(end) 0 0], 'euler', 'ZYX', 'point'), 'point');
    Rbef = rotmat(quaternion([insData.Heading(1)   0 0], 'euler', 'ZYX', 'point'), 'point');
    T = [Rbef Tbef;0 0 0 1] \ [Rnow Tnow;0 0 0 1];
    initTform = rigid3d(T.');

        
    % 将绝对变换更新到参照系（第一个点云）
    absTform = rigid3d( initTform.T * absTform.T );   
    % 将当前点云扫描作为视图添加到视图集中
    vSet = addView(vSet, viewId, absTform);
    
    % 添加从上一个视图到当前视图的连接，表示它们之间的相对转换
    vSet = addConnection(vSet, viewId-1, viewId, relTform);
    
    viewId = viewId + 1;      
    initTform = relTform;
    
    if n>1 && mod(n, displayRate) == 1
        plot(vSet, "Parent", hAxBefore);
        drawnow update
    end
end

%% 测试
return;

%% 源程序拷贝（可跑通）
%% 加载和浏览记录的数据
%将各部分信息（如磁盘名、目录名等）合成完整的路径
dataFolder = fullfile('D:', 'Matlab storage', 'BuildAMapFromLidarDataUsingSLAMExample',filesep);

% 从数据存储中提取文件列表
imuConfigFile = fullfile(dataFolder, 'scenario1', 'imu.cfg');
insDataTable = helperReadINSConfigFile(imuConfigFile);
% 删除不使用的列
insDataTable(1447, :) = [];
insDataTable = removevars(insDataTable, ...
    {'Num_Satellites', 'Latitude', 'Longitude', 'Altitude', 'Omega_Heading', ...
     'Omega_Pitch', 'Omega_Roll', 'V_X', 'V_Y', 'V_ZDown'});
% insDataTable = addvars(insDataTable, pointCloudFiles, 'Before', 1, ...
%     'NewVariableNames', "PointCloudFileName");
%读取imu数据，包括来自INS的GPS时间，航向、俯仰、横滚、X、Y和Z信息
insDataTable = insDataTable(:, 1:end);

%% 创建空视窗并显示
vSet = pcviewset;
hFigBefore = figure('Name', 'View Set Display');
hAxBefore = axes(hFigBefore);


% 初始化转换参数
absTform   = rigid3d;  % 坐标系的绝对变换
relTform   = rigid3d;  % 连续扫描之间的相对变换

viewId = 1;
skipFrames  = 5;
numFrames   = height(insDataTable);
displayRate = 100;      % 每100帧更新一次显示


%% 计算各视图间的变换矩阵
for n = 1 : skipFrames : numFrames
        
    firstFrame = (n==1);
    if firstFrame
        vSet = addView(vSet, viewId, absTform);
        viewId = viewId + 1;
        continue;
    end
    
    % 使用INS估计初始变换以进行注册
    insData = insDataTable(n-skipFrames:n, :);
    % INS读数显示X指向前方，Y指向左侧，Z向上。
    % 下面的转换说明了如何转换为激光雷达框架。
    insToLidarOffset = [0 -0.79 -1.73]; % See DATAFORMAT.txt
    Tnow = [-insData.Y(end), insData.X(end), insData.Z(end)].' + insToLidarOffset';
    Tbef = [-insData.Y(1)  , insData.X(1)  , insData.Z(1)].' + insToLidarOffset';

    % 由于车辆预计将沿地面移动，因此横摇和纵摇的变化最小。
    % 忽略横摇和俯仰的变化，仅使用航向。
    Rnow = rotmat(quaternion([insData.Heading(end) 0 0], 'euler', 'ZYX', 'point'), 'point');
    Rbef = rotmat(quaternion([insData.Heading(1)   0 0], 'euler', 'ZYX', 'point'), 'point');
    T = [Rbef Tbef;0 0 0 1] \ [Rnow Tnow;0 0 0 1];
    initTform = rigid3d(T.');

        
    % 将绝对变换更新到参照系（第一个点云）
    absTform = rigid3d( initTform.T * absTform.T );   
    % 将当前点云扫描作为视图添加到视图集中
    vSet = addView(vSet, viewId, absTform);
    
    % 添加从上一个视图到当前视图的连接，表示它们之间的相对转换
    vSet = addConnection(vSet, viewId-1, viewId, relTform);
    
    viewId = viewId + 1;      
    initTform = relTform;
    
    if n>1 && mod(n, displayRate) == 1
        plot(vSet, "Parent", hAxBefore);
        drawnow update
    end
end


