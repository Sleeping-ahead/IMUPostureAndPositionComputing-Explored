%% 加载和浏览记录的数据
% 将各部分信息（如磁盘名、目录名等）合成完整的路径
% 源文件为excel
% dataFolder = fullfile('D:', 'Matlab storage', '生成轨迹线','数据整理.xlsx');
% insDataTable = readtable(dataFolder,'sheet',2);
% 源文件为txt
dataFolder = fullfile('D:', 'Matlab storage', '生成轨迹线','处理数据.txt');
insDataTable = readtable(dataFolder);


%% 仅用XYZ数据生成图像
% 生成数据
data = [insDataTable.X - insDataTable.X(1), insDataTable.Y - insDataTable.Y(1), insDataTable.Z - insDataTable.Z(1)]; % 使用相对坐标
% data = [insDataTable.X, insDataTable.Y, insDataTable.Z]; % 使用绝对坐标

% 绘制轨迹线
figure;
% plot3(data(:,1), data(:,2), data(:,3), 'LineWidth', 2); % 连线形式加载数据
plot3(data(:,1), data(:,2), data(:,3), 'o','MarkerSize', 1.5); % 散点形式
% xlim([431767 431775]) % 强制XYZ显示范围
% ylim([3895270 3895276])
% zlim([76 84])
xlabel('X'); % XYZ轴标签
ylabel('Y');
zlabel('Z');
% grid on; %显示坐标区网格线
daspect([1 1 1]); %查看曲面图以使每个轴上的相对大小彼此相等
box on; %显示坐标区的边轮廓
view(45,30); %控制相机视角


%% 测试
return;