//#pragma optimize( "", off )
#include<Windows.h>//控制台显示
#include <io.h>//文件读取
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <Eigen/Dense>


///路径
std::string filePath = ".\\输入\\";
std::string outputPath = ".\\";
std::string fileName_out = "处理数据";
///读入文件参数
int totalRow = 0;			//总行数，只用于显示进度条
int rowStartRead = 1;		//表头的最后所在行，去除表头用
///四元数相关
//参与计算的加速度单位g 陀螺仪单位是弧度/s()【度*pi/180=弧度】
#define Kp 10.0f               
#define Ki 0.008f            
//#define pi 3.14159265f 
#define halfT 0.0025f		//采样时间的一半
#define gValue 9.80f		//世界坐标系下的重力值
///用于加速度增量滤除，去除区间内的值
//trick//零速修正?
#define correct_ax 0.05f
#define correct_ay 0.05f
#define correct_az 0.05f


//六轴IMU原始数据
struct ImuData
{
	float Timestamps;
	float accel_x;//加速度
	float accel_y;
	float accel_z;
	float gyro_x;//角速度//陀螺仪的英文简写为gyro
	float gyro_y;
	float gyro_z;
};
//六轴IMU处理数据
struct ImuState
{
	float roll;
	float pitch;
	float yaw;
	double x;
	double y;
	double z;
};
//三轴方向速度及加速度
struct ImuVelocity
{
	float vx;
	float vy;
	float vz;
};
//自定义向量及四元数
struct Vector3 {
	float x, y, z;
	Vector3() : x(0), y(0), z(0) {}
	Vector3(float ax, float ay, float az) : x(ax), y(ay), z(az) {}
};
struct Quaternion {
	float w, x, y, z;
	Quaternion() : w(1), x(0), y(0), z(0) {}
	Quaternion(float aw, float ax, float ay, float az) : w(aw), x(ax), y(ay), z(az) {}
};



/*函数 功能*/////////////////////////////////////
class File
{
public:


	/*检查路径下是否为空文件夹*/
	bool isEmptyDir(std::string _path)
	{
		_path = _path + "*.*";// 拼接路径字符串，以便在路径末尾添加通配符，以匹配所有文件
		const char* path = _path.c_str();// 转为const char* 型

		WIN32_FIND_DATA fd;//WIN32_FIND_DATA结构体，存储文件信息
		HANDLE hFind = FindFirstFile(path, &fd);//使用FindFirstFile函数返回文件句柄

		// 判断文件句柄是否有效，若无效，则表示文件夹为空
		if (hFind == INVALID_HANDLE_VALUE)
		{
			return false;
		}

		while (FindNextFile(hFind, &fd))
		{
			// 如果找到了非 . 和 .. 的文件，则关闭文件句柄，返回 false
			if (strcmp(fd.cFileName, ".") != 0 && strcmp(fd.cFileName, "..") != 0)
			{
				FindClose(hFind);
				return false;
			}
		}
		// 关闭文件句柄，返回 true，表示文件夹为空
		FindClose(hFind);
		return true;
	}

	/*查找某一目录下的文件名*/
	void GetFiles(std::string path, std::vector<std::string>& files)
	{
		//文件句柄
		intptr_t hFile = 0;//注意：有些文章代码此处是long类型，实测运行中会报错访问异常
		//文件信息
		struct _finddata_t fileinfo;
		std::string p;

		if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
		{
			do
			{
				//如果是目录,迭代之
				//如果不是,加入列表
				if ((fileinfo.attrib &  _A_SUBDIR))
				{
					if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
						GetFiles(p.assign(path).append("\\").append(fileinfo.name), files);
				}
				else
				{
					files.push_back(p.assign(path).append("\\").append(fileinfo.name));
					//if (isFeature)
					//	featureNameList.push_back(fileinfo.name);
				}
			} while (_findnext(hFile, &fileinfo) == 0);
			_findclose(hFile);
		}
	}

	/*显示进度条*/
	void ShowProgressBar(int i, int count)
	{
		/*控制台进度条显示*/
		if (i < count - 1)
			printf("\r运行中[%.2lf%%]:", i*100.0 / (count - 1));
		else
		{
			printf("\r运行完成[%.2lf%%]:", i*100.0 / (count - 1));
		}
		int show_num = i * 20 / count;
		for (int j = 1; j <= show_num; j++)
		{
			std::cout << "█";
			Sleep(0.000001);
		}
	}

	/*读取txt，存入imu*/
	void ReadTxt(std::string filePath, std::vector<ImuData> &imu)
	{
		std::cout << "imu读取并存储" << std::endl;

		/*1、读取txt文件*/
		std::ifstream ifs(filePath);
		std::string line;

		int i = 0;
		while (std::getline(ifs, line))//将ifs文件中的每一行字符读入到line中
		{
			//跳过表头
			if (i < rowStartRead)
			{
				i++;
				continue;
			}

			std::istringstream ss(line);//使用string初始化stringstream
			//将一行字符存入临时的string容器
			std::vector<std::string> temp_v;
			std::string temp;
			while (ss >> temp)
				temp_v.push_back(temp);
			//若读取到空行，跳过
			if (temp_v.size() == 0)
			{
				i++;
				continue;
			}
			//string容器存入p中
			ImuData p;
			p.Timestamps = stof(temp_v[0]);//stod//调换y和z
			p.gyro_x = stof(temp_v[1]);
			p.gyro_y = stof(temp_v[3]);
			p.gyro_z = stof(temp_v[2]);
			p.accel_x = stof(temp_v[4]);
			p.accel_y = stof(temp_v[6]);
			//p.accel_y = stof(temp_v[5]) - 9.8f;
			p.accel_z = stof(temp_v[5]);

			///输出测试_p
			//std::cout.precision(10);
			//std::cout
			//	<< p.Timestamps << "  "
			//	<< p.accel_x << "  "
			//	<< p.accel_y << "  "
			//	<< p.accel_z << "  "
			//	<< p.gyro_x << "  "
			//	<< p.gyro_y << "  "
			//	<< p.gyro_z << "  "
			//	<< std::endl;

			/*2、将p存入imuInf*/
			imu.push_back(p);


			//显示进度条
			//ShowProgressBar(i, totalRow);

			i++;
		}

		ifs.close();
	}

	/*存储数据为txt文件*/
	void SaveTxt(std::vector<ImuState> imu)
	{
		std::ofstream ofs;
		ofs.flags(std::ios::fixed);
		ofs.precision(10);//保留小数点后3位
		ofs.open(outputPath + fileName_out + ".txt", std::ios::out);
		std::cout << std::endl << "正在保存文件：" + outputPath + fileName_out + ".txt" << std::endl;

		//表头
		ofs << "Timestamps" << "      "
			<< "X" << "      "
			<< "Y" << "      "
			<< "Z" << "      "
			<< "Heading" << "      "
			<< "Pitch" << "      "
			<< "Roll" << "      " << std::endl;
		//imu数据	
		for (int i = 0; i < imu.size(); i++)
		{
			ofs << i << "      "
				<< imu[i].x << "      "
				<< imu[i].y << "      "
				<< imu[i].z << "      "
				<< imu[i].yaw << "      "
				<< imu[i].pitch << "      "
				<< imu[i].roll << std::endl;

			//显示进度条
			//ShowProgressBar(i, imu.size());
		}
		ofs.close();

	}

private:
};




/*函数 计算*/////////////////////////////////////
/*计算txt行数*/
int CountLines(std::string filename)
{
	std::cout << "正在计算txt总行数......";

	std::ifstream ReadFile;
	int n = 0;
	std::string tmp;
	ReadFile.open(filename, std::ios::in);//ios::in 表示以只读的方式读取文件
	if (ReadFile.fail())//文件打开失败:返回0
	{
		return 0;
	}
	else//文件存在
	{
		while (getline(ReadFile, tmp, '\n'))
		{
			n++;
		}
		ReadFile.close();
		std::cout << "计算完成，文件行数：" << n << std::endl;
		return n;
	}
}

/*抽稀，按一定间隔选取imu数据*/
void AdjustImu(std::vector<ImuState> &imu)
{
	std::cout << std::endl << "imu数据抽稀" << std::endl;

	int j = 0;
	std::vector<ImuState> temp;
	temp.push_back(imu[0]);
	for (unsigned int i = 0; i < imu.size(); i++)
	{
		if (j == 9)
		{
			temp.push_back(imu[i]);
			j = -1;
		}
		j++;
	}

	imu = temp;
}

	#pragma region 向量/四元数运算符操作

//重载运算符
Quaternion operator*(const Quaternion& q1, const Quaternion& q2) {//四元数相乘，q1右乘q2
	Quaternion q1q2;
	q1q2.w = q2.w*q1.w - q2.x*q1.x - q2.y*q1.y - q2.z*q1.z;
	q1q2.x = q2.x*q1.w + q2.w*q1.x + q2.z*q1.y - q2.y*q1.z;
	q1q2.y = q2.y*q1.w - q2.z*q1.x + q2.w*q1.y + q2.x*q1.z;
	q1q2.z = q2.z*q1.w + q2.y*q1.x - q2.x*q1.y + q2.w*q1.z;
	return q1q2;
}
Vector3 operator-(const Vector3& q1, const Vector3& q2) {
	Vector3 q(q1.x - q2.x, q1.y - q2.y, q1.z - q2.z);
	return q;
}
Vector3 operator+(const Vector3& q1, const Vector3& q2) {
	Vector3 q(q1.x + q2.x, q1.y + q2.y, q1.z + q2.z);
	return q;
}
//获取四元数的共轭
Quaternion getConjugate(const Quaternion& q) {
	Quaternion qConj(q.w, -q.x, -q.y, -q.z);
	return qConj;
}
Vector3 operator*(const Quaternion& q, const Vector3& v) {
	Quaternion qv(0, v.x, v.y, v.z);
	Quaternion qvq = q * qv * getConjugate(q);
	Vector3 res(qvq.x, qvq.y, qvq.z);
	return res;
}
Vector3 operator*(const Vector3& v, const float &t) {
	Vector3 res(v.x * t, v.y * t, v.z * t);
	return res;
}

	#pragma endregion



/*IMU解算模块*/////////////////////////
class ImuCalculate_Qua
{
public:
	ImuState previous_state_ = { 0, 0, 0, 0, 0, 0 };//位姿信息初始化
	ImuVelocity pre_v_ = { 0,0,0 };//累计速度初始化
	float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;//integral积分误差值
	float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;//传感器帧相对于辅助帧的四元数
	Quaternion q_initial = { q0, q1, q2, q3 };//初始时刻的姿态四元数
	Quaternion g_world = { 0, 0, 0, gValue };//世界坐标系下的重力加速度_四元数形式

	/* 魔法函数InvSqrt()相当于1.0/sqrt() *///快速反平方根算法,强制类型转换容易出bug
	static float invSqrt(float number)
	{
		volatile long i;
		volatile float x, y;
		volatile const float f = 1.5F;

		x = number * 0.5F;
		y = number;
		i = *((long *)&y);// 把浮点数类型的值的地址转成长整型的地址，通过 long 类型可以进行位运算
		i = 0x5f375a86 - (i >> 1);// 使用魔数0x5f375a86，通过右移一位加减操作得到 y 的平方根的倒数的近似值
		y = *((float *)&i);// 把长整型地址转换成浮点数的地址
		y = y * (f - (x * y * y));// 迭代计算得到更加精确的倒数值
		return y;
	}

	/*计算从q1的坐标系变换到q2的坐标系的四元数*/
	//q1当前，q2目标
	Quaternion RotateQua(Quaternion q1, Quaternion q2)
	{
		//Quaternion q_1To2 = q2 * (q1 * getConjugate(q2));
		Quaternion q_1To2 = q2 * getConjugate(q1);//普遍的坐标系变换写法，能保证两位有效数字是相同的

		return q_1To2;
	}

	/*计算本时刻加速度在大地坐标系下的值V2，根据姿态信息旋转*/
	Vector3 RotateAcceV2(Quaternion qua, Quaternion acce)
	{
		//旋转向量
		Quaternion qvq = qua * (acce * getConjugate(qua));
		Vector3 a_world(qvq.x, qvq.y, qvq.z);

		return a_world;
	}

	/*筛选加速度增量，在世界坐标系下*/
	void FilterAcceInWorld(Quaternion qua, Vector3 &a_local)
	{
		//将加速度增量旋转至世界坐标系下
		Quaternion a_local_(0, a_local.x, a_local.y, a_local.z);
		Quaternion q_toIni = RotateQua(qua, q_initial);
		Vector3 a_world = RotateAcceV2(q_toIni, a_local_);
		
		//将加速度增量小于区间范围的值滤除
		if (a_world.x > -correct_ax && a_world.x < correct_ax) a_local.x = 0;
		if (a_world.y > -correct_ay && a_world.y < correct_ay) a_local.y = 0;
		if (a_world.z > -correct_az && a_world.z < correct_az) a_local.z = 0;

		///测试
		//std::cout << a_world.x << "  " << a_world.y << "  " << a_world.z << std::endl;
	}


	/*对一组新的原始数据执行IMU预处理，返回新状态*/
	ImuState ProcessNewData(ImuData &data) 
	{
		///声明
		//四元数参数
		float recipNorm;//归一化系数
		float vx, vy, vz;//估算的三轴重力分量
		float ex, ey, ez;//估算重力与实测重力的三轴误差值
		//位置参数
		Quaternion v_prev(0, pre_v_.vx, pre_v_.vy, pre_v_.vz);//上一时刻速度
		Vector3 p_prev(previous_state_.x, previous_state_.y, previous_state_.z);//上一时刻位置
		Vector3 a_next(data.accel_x, data.accel_y, data.accel_z);//本时刻加速度
																 //空间中的三维向量可以用纯四元数的形式表示
		Quaternion q_prev(q0, q1, q2, q3);//上一时刻姿态四元数


		#pragma region 姿态解算

		/*四元数初始化 */
		//float q0q0 = q0 * q0;
		//float q0q1 = q0 * q1;
		//float q0q2 = q0 * q2;

		//float q1q1 = q1 * q1;
		//float q1q3 = q1 * q3;

		//float q2q2 = q2 * q2;
		//float q2q3 = q2 * q3;

		//float q3q3 = q3 * q3;

		if (data.accel_x * data.accel_y * data.accel_z == 0)
			return previous_state_;
		/* 对加速度数据进行归一化处理 */
		recipNorm = invSqrt(data.accel_x * data.accel_x + data.accel_y * data.accel_y + data.accel_z * data.accel_z);
		data.accel_x = data.accel_x * recipNorm;
		data.accel_y = data.accel_y * recipNorm;
		data.accel_z = data.accel_z * recipNorm;

		/* DCM矩阵旋转 */
		vx = 2 * (q1*q3 - q0*q2);
		vy = 2 * (q0*q1 + q2*q3);
		vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
		//另一种写法
		//vx = q1*q3 - q0*q2;
		//vy = q0*q1 + q2*q3;
		//vz = q0*q0 - 0.5f + q3*q3;

		/* 在机体坐标系下做向量叉积得到补偿数据 *///计算两个向量的叉积
		ex = data.accel_y * vz - data.accel_z * vy;
		ey = data.accel_z * vx - data.accel_x * vz;
		ez = data.accel_x * vy - data.accel_y * vx;

		/* 对误差进行PI计算，补偿角速度 */
		exInt = exInt + ex * Ki;
		eyInt = eyInt + ey * Ki;
		ezInt = ezInt + ez * Ki;

		data.gyro_x = data.gyro_x + Kp * ex + exInt;
		data.gyro_y = data.gyro_y + Kp * ey + eyInt;
		data.gyro_z = data.gyro_z + Kp * ez + ezInt;

		/* 按照四元数微分公式进行四元数更新 *///陀螺仪四元数的变化率
		q0 = q0 + (-q1 * data.gyro_x - q2 * data.gyro_y - q3 * data.gyro_z)*halfT;
		q1 = q1 + (q0 * data.gyro_x + q2 * data.gyro_z - q3 * data.gyro_y)*halfT;
		q2 = q2 + (q0 * data.gyro_y - q1 * data.gyro_z + q3 * data.gyro_x)*halfT;
		q3 = q3 + (q0 * data.gyro_z + q1 * data.gyro_y - q2 * data.gyro_x)*halfT;

		/* 四元数归一化 */
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 = q0 * recipNorm;
		q1 = q1 * recipNorm;
		q2 = q2 * recipNorm;
		q3 = q3 * recipNorm;

		/* 计算欧拉角 */
		float roll = atan2f(2 * q2*q3 + 2 * q0*q1, -2 * q1*q1 - 2 * q2*q2 + 1) * 57.3f;
		float pitch = asinf(2 * q1*q3 - 2 * q0*q2) * 57.3f;
		float yaw = -atan2f(2 * q1*q2 + 2 * q0*q3, -2 * q2*q2 - 2 * q3*q3 + 1) * 57.3f;

		///输出测试_欧拉角
		//printf("pitch:%.2f roll:%.2f yaw:%.2f\r\n", pitch, roll, yaw);
		//std::cout << " yaw: " << yaw << " pitch: " << pitch << " roll: " << roll << std::endl;
		//std::cout << q0 << "  " << q1 << "  " << q2 << "  " << q3 << std::endl;

		#pragma endregion

		#pragma region 位置解算

		/*1、局部坐标系_去重力影响，计算加速度增量*/
		Quaternion q_next(q0, q1, q2, q3);//姿态四元数_本时刻
		Quaternion q_iniToNext = RotateQua(q_initial, q_next);//姿态_世界坐标系->本时刻局部坐标系
		Vector3 g_next = RotateAcceV2(q_iniToNext, g_world);//重力加速度_本时刻局部坐标系
		g_next.z = -g_next.z;//依据未知。仅根据输出结果，修改g_next的Z值方向
		Vector3 a_local = a_next + g_next;
	
		FilterAcceInWorld(q_next, a_local);//滤除小于区间范围的加速度增量，强行减少扰动//trick

		/*2、局部坐标系_上一刻的速度变换至本时刻*/
		Quaternion q_prevToNext = RotateQua(q_prev, q_next);//姿态_上一时刻->本时刻
		Vector3 v_local = RotateAcceV2(q_prevToNext, v_prev);//转换至本时刻坐标系的速度

		/*3、局部坐标系_计算位移增量与本时刻速度量*/
		Vector3 p_local = v_local * 2 * halfT + a_local * 0.5 * (2*halfT) * (2*halfT);//视为匀变速运动，计算本时刻位移增量
		Vector3 v_next = v_local + a_local * 2 * halfT;//本时刻速度，传递给下次迭代用

		/*4、世界坐标系_位移增量旋转、计算坐标值累计*/
		Quaternion q_nextToIni = RotateQua(q_next, q_initial);//姿态_本时刻局部坐标系->世界坐标系
		Quaternion p_local_(0, p_local.x, p_local.y, p_local.z);
		Vector3 p_world = RotateAcceV2(q_nextToIni, p_local_);//世界坐标系下的位移增量
		Vector3 p_next = p_prev + p_world;//本时刻坐标值

		#pragma endregion

		//更新以前的状态（位置+姿态信息）,输出
		previous_state_ = { roll, pitch, yaw, p_next.x, p_next.y, p_next.z };
		pre_v_ = { v_next.x , v_next.y , v_next.z};
		return previous_state_;

		///测试_输出姿态信息+XXX
		//ImuState acceState = { roll, pitch, yaw, a_local.x, a_local.y, a_local.z };
		//return acceState;
	}

	/*对整个imu数据进行处理*/
	void ProcessAllData(std::vector<ImuData> &imu_original, std::vector<ImuState> &imu_state)
	{
		for (unsigned int i = 0; i < imu_original.size(); i++)
		{
			ImuState state = ProcessNewData(imu_original[i]);
			imu_state.push_back(state);

			//file.ShowProgressBar(i, imu_original.size());			
		}

	}

private:

};



/*滤波模块*/////////////////////////
class Filter
{
public:


	/*卡尔曼滤波，处理单轴加速度*/
	void KalmenFilterOne(std::vector<float> &acceList)
	{
		Eigen::Vector3f x(3, 1);
		x << 0, 0, 0;//1*3状态列向量，对应位置、速度、加速度

		Eigen::Matrix3f A(3, 3);
		A << 1, halfT * 2, halfT * halfT * 2,
			0, 1, halfT * 2,
			0, 0, 1;//离散化后的状态阵

		Eigen::RowVector3f H(1, 3);
		H << 0.0f, 0.0f, 1.0f;//观测向量

		Eigen::Matrix3f Q(3, 3);
		Q << 0, 0, 0,
			0, 0, 0,
			0, 0, 0.1f;//系统噪声,越大对测量数据的跟踪效果越好（越紧密），噪声越大
						//仅需改对角线的值，其中只有[3,3]有值是因为只对加速度赋予扰动

		float R = 0.1f;//量测（稳态）噪声
					   //Q / (Q + R)的值就是卡尔曼增益的收敛值

		Eigen::Matrix3f Pk(3, 3);
		Pk << 0.1f, 0, 0,
			0, 0.1f, 0,
			0, 0, 0.1f;//协方差矩阵初值//假设位移、速度、加速度值都为0.1f？

		std::vector<float> acceList_new;
		for (int i = 0; i < acceList.size(); i++)//卡尔曼更新公式
		{
			/*1、预测步*/
			Eigen::Vector3f x_(3, 1);
			x_ << A * x;

			Eigen::Matrix3f Pk_(3, 3);
			Pk_ << A * Pk * A.transpose() + Q;

			/*2、更新步*///实数矩阵直接用转置
			Eigen::Vector3f Kk(3, 1);
			Kk << (Pk_ * H.transpose()) / (H * Pk_ * H.transpose() + R);

			x << x_ + Kk * (acceList[i] - H * x_);

			Pk << (Eigen::MatrixXf::Identity(3, 3) - Kk * H) * Pk_;

			//position(i) = x(0);//位置信息
			//speed(i) = x(1);//速度信息
			//acc(i) = x(2);//加速度信息
			acceList_new.push_back(x(2));
		}

		acceList = acceList_new;

	}

	/*卡尔曼滤波，处理三轴加速度*/
	void KalmenFilterParent(std::vector<ImuData> &imu_original)
	{
		//取出三轴加速度
		std::vector<float> ax;
		std::vector<float> ay;
		std::vector<float> az;
		for (unsigned int i = 0; i < imu_original.size(); i++)
		{
			ax.push_back(imu_original[i].accel_x);
			ay.push_back(imu_original[i].accel_y);
			az.push_back(imu_original[i].accel_z);
		}

		//卡尔曼滤波处理
		KalmenFilterOne(ax);
		KalmenFilterOne(ay);
		KalmenFilterOne(az);
		
		//存入处理结果，值为位移量/速度变化量/加速度变化量，在KalmenFilterOne设置
		for (unsigned int i = 0; i < imu_original.size(); i++)
		{
			imu_original[i].accel_x = ax[i];
			imu_original[i].accel_y = ay[i];
			imu_original[i].accel_z = az[i];
		}

	}

	/*位置解算，且筛选三轴加速度*///可弃用，适用于对加速度增量进行滤波后的情况
	void CalculatePositionAndFilterAcce(std::vector<ImuState> &imu_state)
	{
		//位置参数
		Vector3 v_prev(0, 0, 0);//上一时刻速度
		Vector3 p_prev(0, 0, 0);//上一时刻位置

		for (unsigned i = 0; i < imu_state.size(); i++)
		{		
			//if (i<50)
			//{//trick，去掉初始数行的数据
			//	imu_state[i].x = 0;
			//	imu_state[i].y = 0;
			//	imu_state[i].z = 0;
			//	continue;
			//}

			/*1、筛选静止状态下的加速度*/
			if (imu_state[i].x > -correct_ax && imu_state[i].x < correct_ax) imu_state[i].x = 0;
			if (imu_state[i].y > -correct_ay && imu_state[i].y < correct_ay) imu_state[i].y = 0;
			if (imu_state[i].z > -correct_az && imu_state[i].z < correct_az) imu_state[i].z = 0;

			/*2、位置解算*/
			//视为匀变速运动，计算本时刻的位置
			Vector3 a_next(imu_state[i].x, imu_state[i].y, imu_state[i].z);//本时刻加速度
			Vector3 p_next = p_prev + v_prev * 2 * halfT + a_next * 0.5 * (2 * halfT) * (2 * halfT);
			Vector3 v_next = v_prev + a_next * 2 * halfT;//本时刻速度//传递给下次迭代用

			//更新状态
			imu_state[i].x = p_next.x;
			imu_state[i].y = p_next.y;
			imu_state[i].z = p_next.z;

			v_prev = { v_next.x, v_next.y, v_next.z };
			p_prev = { p_next.x, p_next.y, p_next.z };
		}

	}


private:
};



/****************************************
源自：https://www.ourfpv.com/chat/
这个程序使用加速度计和陀螺仪数据对IMU的姿态和位置进行估计。它的主要步骤如下：

1. 从原始IMU数据中计算加速度计角度。
2. 将加速度计角度与前一个状态结合，以获取估计的滚动角和俯仰角。
3. 从原始陀螺仪数据和前一个偏航角计算新的偏航角。
4. 将估计的姿态和前一个位置结合，以获取新的位置估计。
5. 更新前一个状态并返回新状态。
**************************************/
int main() 
{
	///声明
	std::vector<std::string> fileList;//文件名列表
	std::vector<ImuData> imu_original;//imu原始数据
	std::vector<ImuState> imu_state;//imu处理数据
	File file;//文件处理对象

	/*1、检查、读取imu文件*/
	if (file.isEmptyDir(filePath))
	{
		std::cout << std::endl << "未读取到imu文件，请检查是否有放入文件。" << std::endl;
		return 0;
	}
	file.GetFiles(filePath, fileList);
	filePath = fileList[0];
	std::cout << "已读取到imu文件，路径：" << filePath << std::endl;
	fileList.clear();
	totalRow = CountLines(filePath);//计算文件总行数，用于进度条
	file.ReadTxt(filePath, imu_original);//读取路径下的txt文件


	/*2、滤波处理*/
	Filter filter;//滤波处理对象
	std::cout << std::endl << "滤波处理中..." << std::endl;
	filter.KalmenFilterParent(imu_original);//对加速度进行卡尔曼滤波


	/*3、解算处理*/
	ImuCalculate_Qua imuCalculate_Qua;//imu预处理对象
	std::cout << std::endl << "imu解算中..." << std::endl;
	imuCalculate_Qua.ProcessAllData(imu_original, imu_state);

	///测试_打印六轴信息
	//std::cout << "X		Y		Z		Yaw		Roll		Pitch" << std::endl;
	//for (unsigned i = 0; i < imu_state.size(); i++)
	//	std::cout //<< std::cout.precision(8)
	//	<< imu_state[i].x << "  " << imu_state[i].y << "  " << imu_state[i].z
	//	<< imu_state[i].yaw << "	" << imu_state[i].roll << "	" << imu_state[i].pitch << std::endl;


	/*4、抽稀（选）*/
	//AdjustImu(imu_state);


	/*5、存储为txt文件*/
	file.SaveTxt(imu_state);


	std::cout << std::endl;
	system("PAUSE");
	return 0;
}


	#pragma region 弃用

/**************/
/**************/
//呃，你这么上心的嘛，还翻这里的垃圾场。如果确实有需要，可以加个QQ聊聊:459390464@qq.com。
//我可能只记得一二了，能帮尽帮，交个朋友XD
/*************/
/*************/

	#pragma region 不记得是啥了

// Simulate some raw IMU data and run it through the preprocessor//模拟一些原始IMU数据并通过预处理器运行
//ImuData data = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.1 };
//ImuState state = preprocessor.ProcessNewData(data);

/*
		// 预测下一个时刻的姿态
double norm = sqrt(data.gyro_x * data.gyro_x + data.gyro_y * data.gyro_y + data.gyro_z + data.gyro_z);
double theta = norm * halfT;
//
double v_x = data.gyro_x / norm * sin(theta);
double v_y = data.gyro_y / norm * sin(theta);
double v_z = data.gyro_z / norm * sin(theta);
if (norm == 0)
{
	v_x = 0;
	v_y = 0;
	v_z = 0;
}


double v_w = cos(theta);
//计算new
double new_w = pre_w + (v_w * pre_w - v_x * pre_x - v_y * pre_y - v_z * pre_z);
double new_x = pre_x + (v_w * pre_x + v_x * pre_w + v_y * pre_z - v_z * pre_y);
double new_y = pre_y + (v_w * pre_y - v_x * pre_z + v_y * pre_w + v_z * pre_x);
double new_z = pre_z + (v_w * pre_z + v_x * pre_y - v_y * pre_x + v_z * pre_w);
//new 归一化
double norm_new = sqrt(new_w * new_w + new_x * new_x + new_y + new_y + new_z + new_z);
new_w = new_w / norm_new;
new_x = new_x / norm_new;
new_y = new_y / norm_new;
new_z = new_z / norm_new;
if (norm_new == 0)
{
	new_w = 0;
	new_x = 0;
	new_y = 0;
	new_z = 0;
}

//
pre_w = new_w;
pre_x = new_x;
pre_y = new_y;
pre_z = new_z;

///测试_q
std::cout << new_w << "  " << new_x << "  " << new_y << "  " << new_z << std::endl;

*/

// 去重力影响类
//class RemoveGravity {
//public:
//	RemoveGravity(double gx, double gy, double gz) {
//		gravity = sqrt(gx * gx + gy * gy + gz * gz);
//		gx /= gravity;
//		gy /= gravity;
//		gz /= gravity;
//		this->g[0] = gx;
//		this->g[1] = gy;
//		this->g[2] = gz;
//	}
//
//	void remove(ImuData &data) {
//		double ax = data.accel_x - g[0] * gravity;
//		double ay = data.accel_y - g[1] * gravity;
//		double az = data.accel_z - g[2] * gravity;
//		data.accel_x = ax;
//		data.accel_y = ay;
//		data.accel_z = az;
//	}
//
//private:
//	double g[3];    // 三维重力向量
//	double gravity; // 重力加速度大小
//};

//预处理过程类
//class ImuPreprocessor
//{
//public:
//	ImuPreprocessor() : previous_state_({ 0.516, -2.347, 345.504, 432026.165, 3895254.285, 79.119 }) {}
//	//ImuPreprocessor() : previous_velocity_({ 0, 0, 0 }) {}
//
//	// Perform IMU preprocessing on a new set of raw data, returning the new state//对一组新的原始数据执行IMU预处理，返回新状态
//	ImuState ProcessNewData(const ImuData& data) {
//
//		///输出测试
//		//std::cout << pre_v_.vx << "  " << pre_v_.vy << "  " << pre_v_.vz << std::endl;
//		//std::cout << data.accel_x << "  " << data.accel_y << "  " << data.accel_z << std::endl;
//
//
//		// Compute accelerometer angles from raw accelerometer readings//根据原始加速计读数计算加速计角度
//		float accel_roll = atan2(data.accel_y, data.accel_z);
//		float accel_pitch = atan2(-data.accel_x, sqrt(data.accel_y * data.accel_y + data.accel_z * data.accel_z));
//
//		// Combine accelerometer angles with previous state to get estimated pitch and roll//将加速计角度与先前状态相结合，以获得估计的俯仰和侧倾//原始
//		//float new_roll = 0.5 * previous_state_.roll + 0.5 * (-data.gyro_y * dt_ + accel_roll);//0.5为比例系数
//		//float new_pitch = 0.5 * previous_state_.pitch + 0.5 * (data.gyro_x * dt_ + accel_pitch);
//		// Compute new yaw from raw gyroscope readings and previous yaw//根据原始陀螺仪读数和先前偏航计算新偏航
//		//float new_yaw = previous_state_.yaw + data.gyro_z * dt_;
//
//		//陀螺仪解算姿态角
//		float gyro_roll = (data.gyro_x +
//			(sin(accel_pitch) * sin(accel_roll)) / cos(accel_pitch) * data.gyro_y +
//			(cos(accel_roll) * sin(accel_pitch)) / cos(accel_pitch)*data.gyro_z) * dt_;
//		float gyro_pitch = (cos(accel_roll) * data.gyro_y - sin(accel_roll) * data.gyro_z) * dt_;
//		float gyro_yaw = (sin(accel_roll) / cos(accel_pitch) * data.gyro_y + cos(accel_roll) / cos(accel_pitch) * data.gyro_z) * dt_;
//
//		//姿态融合，加速度计算的姿态角与陀螺仪的姿态角融合
//		//float new_roll = previous_state_.roll + 0.4 * (accel_roll - gyro_roll);//0.4为比例系数
//		//float new_pitch = previous_state_.pitch + 0.4 * (accel_pitch - gyro_pitch);
//		//float new_yaw = previous_state_.yaw + 0.4 * gyro_yaw;
//		//float new_roll = accel_roll * 0.5 + (gyro_roll + previous_state_.roll) * 0.5;//使用互补滤波器
//		//float new_pitch = accel_pitch * 0.5 + (gyro_pitch + previous_state_.pitch) * 0.5;
//		//float new_yaw = gyro_yaw * 0.5 + previous_state_.yaw * 0.5;
//		float new_roll = gyro_roll + previous_state_.roll;//单纯使用陀螺仪的姿态角
//		float new_pitch = gyro_pitch + previous_state_.pitch;
//		float new_yaw = gyro_yaw + previous_state_.yaw;
//
//		///输出测试
//		//std::cout << " gyro_yaw: " << gyro_roll << " gyro_pitch: " << gyro_pitch << " gyro_roll: " << gyro_yaw << std::endl;
//		//std::cout << " accel_roll: " << accel_roll << " accel_pitch: " << accel_pitch << std::endl << std::endl;
//
//
//		//计算当前姿态下的重力向量
//		float g = sqrt(data.accel_x * data.accel_x + data.accel_y * data.accel_y + data.accel_z * data.accel_z);
//		//各方向加速度减去重力得到去重力加速度//转换为真实加速度（乘9.8）?
//		//float cur_ax = data.accel_x + g * sin(accel_pitch);
//		//float cur_ay = data.accel_y - g * cos(accel_pitch) * sin(accel_roll);
//		//float cur_az = data.accel_z - g * cos(accel_pitch) * cos(accel_roll);
//		float cur_ax = (-g * sin(new_pitch));
//		float cur_ay = (g * cos(new_pitch) * sin(new_roll));//
//		float cur_az = (g * cos(new_pitch) * cos(new_roll) - 9.8f);
//
//		///输出测试
//		//std::cout << cur_ax << "  " << cur_ay << "  " << cur_az << std::endl;
//
//
//		//更新三轴速度，根据加速度和当前速度计算下一时刻的速度
//		float new_vx = pre_v_.vx + (cur_ax - pre_v_.ax) * dt_;
//		float new_vy = pre_v_.vy + (cur_ay - pre_v_.ay) * dt_;
//		float new_vz = pre_v_.vz + (cur_az - pre_v_.az) * dt_;
//
//		//根据速度和当前位置计算下一时刻的位置
//		float new_position_x = previous_state_.position_x + new_vx * dt_;
//		float new_position_y = previous_state_.position_y + new_vy * dt_;
//		float new_position_z = previous_state_.position_z + new_vz * dt_;
//
//		// Combine estimated pose with previous position to get new position estimate//将估计的姿势与先前的位置组合以获得新的位置估计
//		//float new_position_x = previous_state_.position_x + cos(new_yaw) * sin(new_pitch) * cos(new_roll) + sin(new_yaw) * sin(new_roll);
//		//float new_position_y = previous_state_.position_y + cos(new_yaw) * sin(new_pitch) * sin(new_roll) - sin(new_yaw) * cos(new_roll);
//		//float new_position_z = previous_state_.position_z + cos(new_yaw) * cos(new_pitch);
//
//		// Update previous state and return new state//更新以前的状态并返回新状态
//		previous_state_ = { new_roll, new_pitch, new_yaw, new_position_x, new_position_y, new_position_z };
//		pre_v_ = { new_vx , new_vy , new_vz ,cur_ax , cur_ay , cur_az };
//		return previous_state_;
//	}
//
//private:
//	ImuState previous_state_;
//	ImuVelocity pre_v_ = { 0, 0, 0, 0, 0, 0 };
//	const float dt_ = 0.005f; // Sample time (in seconds)//采样时间（秒）
//};


////Vector3 a_imu = a_prev - g_prev;
////Vector3 a_world = getConjugate(q_prev) * (q_prev * a_imu);

//Quaternion a_imu_qv(0, a_initial.x, a_initial.y, a_initial.z);
//Quaternion a_imu_qvq = q_prev * a_imu_qv * getConjugate(q_prev);
//Vector3 a_world(a_imu_qvq.x, a_imu_qvq.y, a_imu_qvq.z);
//Vector3 a_world = q_prev * a_imu * getConjugate(q_prev);
//Vector3 v_next = v_prev + a_world * halfT * 2 + (g_prev + g_next) * halfT;


//Quaternion operator*(const Quaternion& q1, const Quaternion& q2) {
//	Quaternion q;
//	q.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
//	q.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
//	q.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
//	q.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
//	return q;
//}//ChatGPT坑人，q实际为q2q1

	#pragma endregion


///*计算重力向量在世界坐标系下的值*///可弃用
//Vector3 calculateGravity(const Quaternion& q) {
//
//	//Vector3 g_world(0, 0, -9.8f);
//	//Vector3 qg = q * g_world;
//	//Quaternion qv(0, qg.x, qg.y, qg.z);
//	//Quaternion qvq = qv * getConjugate(q);
//	//Vector3 g_imu(qvq.x, qvq.y, qvq.z);
//
//	Quaternion qv(0, 0, 0, -gValue);
//	Quaternion qvq = q * qv * getConjugate(q);
//	Vector3 g_imu(qvq.x, qvq.y, qvq.z);
//	///测试
//	//std::cout << g_imu.x << "  " << g_imu.y << "  " << g_imu.z << std::endl;
//
//	return g_imu;
//}
//
///*计算下一个时刻的速度*///可弃用
//Vector3 calculateVelocity(const Vector3& v_prev, const Vector3& a_prev, const Quaternion& q_prev, const Quaternion& q_next) {
//	Vector3 g_prev = calculateGravity(q_prev);
//	Vector3 g_next = calculateGravity(q_next);
//	Vector3 a_imu = a_prev - g_prev;
//	Vector3 a_world = getConjugate(q_prev) * (q_prev * a_imu);
//	Vector3 v_next = v_prev + a_world * halfT * 2 + (g_prev + g_next) * halfT;
//	return v_next;
//}
///	/*计算向量在大地坐标系下的值，并去重力*///弃用
//Vector3 RotateAcce(Quaternion qua, Quaternion acce)
//{
//	//坐标系转换
//	//Quaternion qvq = getConjugate(qua) * acce * qua;//大地转车载？
//	Quaternion qvq = qua * (acce * getConjugate(qua));//车载转大地？
//	Vector3 qv_world(qvq.x, qvq.y, qvq.z);
//	//去重力影响（在大地坐标系下）
//	qv_world.z -= gValue;
//
//	return qv_world;
//}
///	/*计算本时刻加速度在当前坐标系下的值V3，仅去除重力影响*/
//Vector3 RotateAcceV3(Quaternion acce)
//{
//	//向量acce不做姿态旋转，使用本身的局部坐标系
//	Vector3 a_prev_world(acce.x, acce.y, acce.z);
//	//去重力影响（在大地坐标系下）
//	a_prev_world.z -= gValue;
//
//	return a_prev_world;
//}


///四元数变化功能的测试
//Quaternion q_next(q0, q1, q2, q3);//姿态四元数_本时刻
//Quaternion q_iniTonext = RotateQua(q_initial, q_next);//四元数_世界坐标系->本时刻局部坐标系
//Vector3 g_next = RotateAcceV2(q_iniTonext, g_world);//重力加速度_本时刻局部坐标系
////Vector3 a_local = a_next - g_next;
//Vector3 a_local = a_next;
//
//
/////测试_四元数变化
//Quaternion a_local_(0, a_local.x, a_local.y, a_local.z);
//Vector3 a_world = RotateAcceV2(q_next, a_local_);
//Quaternion a_world_(0, a_world.x, a_world.y, a_world.z);
//Vector3 a_local2 = RotateAcceV2(q_iniTonext, a_world_);
//
//std::cout << a_world.x << "  " << a_world.y << "  " << a_world.z << std::endl;
//std::cout << a_local2.x << "  " << a_local2.y << "  " << a_local2.z << std::endl << std::endl;
///测试_四元数的变换问题
//Quaternion q1(1, 0, 0, 0);
//Quaternion q2(0.207f, 0, 0.207f, 0);
//Quaternion test1 = imuCalculate_Qua.RotateQua(q1, q2);
//std::cout << "test1:  " << test1.w << "  " << test1.x << "  " << test1.y << "  " << test1.z << std::endl;
//Quaternion test2 = imuCalculate_Qua.RotateQua(q2, q1);
//std::cout << "test2:  " << test2.w << "  " << test2.x << "  " << test2.y << "  " << test2.z << std::endl;
//Quaternion gQua = { 0,0,0,gValue };
//Vector3 test3 = imuCalculate_Qua.RotateAcceV2(test1, gQua);
//std::cout << "test3:  " << test3.x << "  " << test3.y << "  " << test3.z << std::endl;
///测试_四元数变化
//Quaternion a_local_(0, a_local.x, a_local.y, a_local.z);
//Vector3 a_world = RotateAcceV2(q_next, a_local_);
//Quaternion a_world_(0, a_world.x, a_world.y, a_world.z);
//Vector3 a_local2 = RotateAcceV2(q_iniTonext, a_world_);
//std::cout << a_local.x << "  " << a_local.y << "  " << a_local.z << std::endl;
//std::cout << a_local2.x << "  " << a_local2.y << "  " << a_local2.z << std::endl << std::endl;

///位置解算_不变更速度的坐标系，进行匀加速运动求位移
/*********************************/
//计算在大地坐标系下的上一时刻加速度//若存在-nan(ind)值溢出的情况，原因是变量未初始化
//Quaternion a_next_(0, a_next.x, a_next.y, a_next.z);
//Vector3 a_next_world = RotateAcceV2(q_next, a_next_);//将向量从当前坐标系变化到初始/世界坐标系
//a_next_world.z -= gValue;//去除重力影响（在世界坐标系下）
//视为匀变速运动，计算本时刻的位置
//Vector3 p_next = p_prev + v_prev * 2 * halfT + a_next_world * 0.5 * (2 * halfT) * (2 * halfT);
//Vector3 v_next = v_prev + a_next_world * 2 * halfT;//本时刻的速度//传递给下次迭代用
/*******************************/



	#pragma endregion
