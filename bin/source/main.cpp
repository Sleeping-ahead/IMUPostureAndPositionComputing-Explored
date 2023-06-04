//#pragma optimize( "", off )
#include<Windows.h>//����̨��ʾ
#include <io.h>//�ļ���ȡ
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <Eigen/Dense>


///·��
std::string filePath = ".\\����\\";
std::string outputPath = ".\\";
std::string fileName_out = "��������";
///�����ļ�����
int totalRow = 0;			//��������ֻ������ʾ������
int rowStartRead = 1;		//��ͷ����������У�ȥ����ͷ��
///��Ԫ�����
//�������ļ��ٶȵ�λg �����ǵ�λ�ǻ���/s()����*pi/180=���ȡ�
#define Kp 10.0f               
#define Ki 0.008f            
//#define pi 3.14159265f 
#define halfT 0.0025f		//����ʱ���һ��
#define gValue 9.80f		//��������ϵ�µ�����ֵ
///���ڼ��ٶ������˳���ȥ�������ڵ�ֵ
//trick//��������?
#define correct_ax 0.05f
#define correct_ay 0.05f
#define correct_az 0.05f


//����IMUԭʼ����
struct ImuData
{
	float Timestamps;
	float accel_x;//���ٶ�
	float accel_y;
	float accel_z;
	float gyro_x;//���ٶ�//�����ǵ�Ӣ�ļ�дΪgyro
	float gyro_y;
	float gyro_z;
};
//����IMU��������
struct ImuState
{
	float roll;
	float pitch;
	float yaw;
	double x;
	double y;
	double z;
};
//���᷽���ٶȼ����ٶ�
struct ImuVelocity
{
	float vx;
	float vy;
	float vz;
};
//�Զ�����������Ԫ��
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



/*���� ����*/////////////////////////////////////
class File
{
public:


	/*���·�����Ƿ�Ϊ���ļ���*/
	bool isEmptyDir(std::string _path)
	{
		_path = _path + "*.*";// ƴ��·���ַ������Ա���·��ĩβ���ͨ�������ƥ�������ļ�
		const char* path = _path.c_str();// תΪconst char* ��

		WIN32_FIND_DATA fd;//WIN32_FIND_DATA�ṹ�壬�洢�ļ���Ϣ
		HANDLE hFind = FindFirstFile(path, &fd);//ʹ��FindFirstFile���������ļ����

		// �ж��ļ�����Ƿ���Ч������Ч�����ʾ�ļ���Ϊ��
		if (hFind == INVALID_HANDLE_VALUE)
		{
			return false;
		}

		while (FindNextFile(hFind, &fd))
		{
			// ����ҵ��˷� . �� .. ���ļ�����ر��ļ���������� false
			if (strcmp(fd.cFileName, ".") != 0 && strcmp(fd.cFileName, "..") != 0)
			{
				FindClose(hFind);
				return false;
			}
		}
		// �ر��ļ���������� true����ʾ�ļ���Ϊ��
		FindClose(hFind);
		return true;
	}

	/*����ĳһĿ¼�µ��ļ���*/
	void GetFiles(std::string path, std::vector<std::string>& files)
	{
		//�ļ����
		intptr_t hFile = 0;//ע�⣺��Щ���´���˴���long���ͣ�ʵ�������лᱨ������쳣
		//�ļ���Ϣ
		struct _finddata_t fileinfo;
		std::string p;

		if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
		{
			do
			{
				//�����Ŀ¼,����֮
				//�������,�����б�
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

	/*��ʾ������*/
	void ShowProgressBar(int i, int count)
	{
		/*����̨��������ʾ*/
		if (i < count - 1)
			printf("\r������[%.2lf%%]:", i*100.0 / (count - 1));
		else
		{
			printf("\r�������[%.2lf%%]:", i*100.0 / (count - 1));
		}
		int show_num = i * 20 / count;
		for (int j = 1; j <= show_num; j++)
		{
			std::cout << "��";
			Sleep(0.000001);
		}
	}

	/*��ȡtxt������imu*/
	void ReadTxt(std::string filePath, std::vector<ImuData> &imu)
	{
		std::cout << "imu��ȡ���洢" << std::endl;

		/*1����ȡtxt�ļ�*/
		std::ifstream ifs(filePath);
		std::string line;

		int i = 0;
		while (std::getline(ifs, line))//��ifs�ļ��е�ÿһ���ַ����뵽line��
		{
			//������ͷ
			if (i < rowStartRead)
			{
				i++;
				continue;
			}

			std::istringstream ss(line);//ʹ��string��ʼ��stringstream
			//��һ���ַ�������ʱ��string����
			std::vector<std::string> temp_v;
			std::string temp;
			while (ss >> temp)
				temp_v.push_back(temp);
			//����ȡ�����У�����
			if (temp_v.size() == 0)
			{
				i++;
				continue;
			}
			//string��������p��
			ImuData p;
			p.Timestamps = stof(temp_v[0]);//stod//����y��z
			p.gyro_x = stof(temp_v[1]);
			p.gyro_y = stof(temp_v[3]);
			p.gyro_z = stof(temp_v[2]);
			p.accel_x = stof(temp_v[4]);
			p.accel_y = stof(temp_v[6]);
			//p.accel_y = stof(temp_v[5]) - 9.8f;
			p.accel_z = stof(temp_v[5]);

			///�������_p
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

			/*2����p����imuInf*/
			imu.push_back(p);


			//��ʾ������
			//ShowProgressBar(i, totalRow);

			i++;
		}

		ifs.close();
	}

	/*�洢����Ϊtxt�ļ�*/
	void SaveTxt(std::vector<ImuState> imu)
	{
		std::ofstream ofs;
		ofs.flags(std::ios::fixed);
		ofs.precision(10);//����С�����3λ
		ofs.open(outputPath + fileName_out + ".txt", std::ios::out);
		std::cout << std::endl << "���ڱ����ļ���" + outputPath + fileName_out + ".txt" << std::endl;

		//��ͷ
		ofs << "Timestamps" << "      "
			<< "X" << "      "
			<< "Y" << "      "
			<< "Z" << "      "
			<< "Heading" << "      "
			<< "Pitch" << "      "
			<< "Roll" << "      " << std::endl;
		//imu����	
		for (int i = 0; i < imu.size(); i++)
		{
			ofs << i << "      "
				<< imu[i].x << "      "
				<< imu[i].y << "      "
				<< imu[i].z << "      "
				<< imu[i].yaw << "      "
				<< imu[i].pitch << "      "
				<< imu[i].roll << std::endl;

			//��ʾ������
			//ShowProgressBar(i, imu.size());
		}
		ofs.close();

	}

private:
};




/*���� ����*/////////////////////////////////////
/*����txt����*/
int CountLines(std::string filename)
{
	std::cout << "���ڼ���txt������......";

	std::ifstream ReadFile;
	int n = 0;
	std::string tmp;
	ReadFile.open(filename, std::ios::in);//ios::in ��ʾ��ֻ���ķ�ʽ��ȡ�ļ�
	if (ReadFile.fail())//�ļ���ʧ��:����0
	{
		return 0;
	}
	else//�ļ�����
	{
		while (getline(ReadFile, tmp, '\n'))
		{
			n++;
		}
		ReadFile.close();
		std::cout << "������ɣ��ļ�������" << n << std::endl;
		return n;
	}
}

/*��ϡ����һ�����ѡȡimu����*/
void AdjustImu(std::vector<ImuState> &imu)
{
	std::cout << std::endl << "imu���ݳ�ϡ" << std::endl;

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

	#pragma region ����/��Ԫ�����������

//���������
Quaternion operator*(const Quaternion& q1, const Quaternion& q2) {//��Ԫ����ˣ�q1�ҳ�q2
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
//��ȡ��Ԫ���Ĺ���
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



/*IMU����ģ��*/////////////////////////
class ImuCalculate_Qua
{
public:
	ImuState previous_state_ = { 0, 0, 0, 0, 0, 0 };//λ����Ϣ��ʼ��
	ImuVelocity pre_v_ = { 0,0,0 };//�ۼ��ٶȳ�ʼ��
	float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;//integral�������ֵ
	float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;//������֡����ڸ���֡����Ԫ��
	Quaternion q_initial = { q0, q1, q2, q3 };//��ʼʱ�̵���̬��Ԫ��
	Quaternion g_world = { 0, 0, 0, gValue };//��������ϵ�µ��������ٶ�_��Ԫ����ʽ

	/* ħ������InvSqrt()�൱��1.0/sqrt() *///���ٷ�ƽ�����㷨,ǿ������ת�����׳�bug
	static float invSqrt(float number)
	{
		volatile long i;
		volatile float x, y;
		volatile const float f = 1.5F;

		x = number * 0.5F;
		y = number;
		i = *((long *)&y);// �Ѹ��������͵�ֵ�ĵ�ַת�ɳ����͵ĵ�ַ��ͨ�� long ���Ϳ��Խ���λ����
		i = 0x5f375a86 - (i >> 1);// ʹ��ħ��0x5f375a86��ͨ������һλ�Ӽ������õ� y ��ƽ�����ĵ����Ľ���ֵ
		y = *((float *)&i);// �ѳ����͵�ַת���ɸ������ĵ�ַ
		y = y * (f - (x * y * y));// ��������õ����Ӿ�ȷ�ĵ���ֵ
		return y;
	}

	/*�����q1������ϵ�任��q2������ϵ����Ԫ��*/
	//q1��ǰ��q2Ŀ��
	Quaternion RotateQua(Quaternion q1, Quaternion q2)
	{
		//Quaternion q_1To2 = q2 * (q1 * getConjugate(q2));
		Quaternion q_1To2 = q2 * getConjugate(q1);//�ձ������ϵ�任д�����ܱ�֤��λ��Ч��������ͬ��

		return q_1To2;
	}

	/*���㱾ʱ�̼��ٶ��ڴ������ϵ�µ�ֵV2��������̬��Ϣ��ת*/
	Vector3 RotateAcceV2(Quaternion qua, Quaternion acce)
	{
		//��ת����
		Quaternion qvq = qua * (acce * getConjugate(qua));
		Vector3 a_world(qvq.x, qvq.y, qvq.z);

		return a_world;
	}

	/*ɸѡ���ٶ�����������������ϵ��*/
	void FilterAcceInWorld(Quaternion qua, Vector3 &a_local)
	{
		//�����ٶ�������ת����������ϵ��
		Quaternion a_local_(0, a_local.x, a_local.y, a_local.z);
		Quaternion q_toIni = RotateQua(qua, q_initial);
		Vector3 a_world = RotateAcceV2(q_toIni, a_local_);
		
		//�����ٶ�����С�����䷶Χ��ֵ�˳�
		if (a_world.x > -correct_ax && a_world.x < correct_ax) a_local.x = 0;
		if (a_world.y > -correct_ay && a_world.y < correct_ay) a_local.y = 0;
		if (a_world.z > -correct_az && a_world.z < correct_az) a_local.z = 0;

		///����
		//std::cout << a_world.x << "  " << a_world.y << "  " << a_world.z << std::endl;
	}


	/*��һ���µ�ԭʼ����ִ��IMUԤ����������״̬*/
	ImuState ProcessNewData(ImuData &data) 
	{
		///����
		//��Ԫ������
		float recipNorm;//��һ��ϵ��
		float vx, vy, vz;//�����������������
		float ex, ey, ez;//����������ʵ���������������ֵ
		//λ�ò���
		Quaternion v_prev(0, pre_v_.vx, pre_v_.vy, pre_v_.vz);//��һʱ���ٶ�
		Vector3 p_prev(previous_state_.x, previous_state_.y, previous_state_.z);//��һʱ��λ��
		Vector3 a_next(data.accel_x, data.accel_y, data.accel_z);//��ʱ�̼��ٶ�
																 //�ռ��е���ά���������ô���Ԫ������ʽ��ʾ
		Quaternion q_prev(q0, q1, q2, q3);//��һʱ����̬��Ԫ��


		#pragma region ��̬����

		/*��Ԫ����ʼ�� */
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
		/* �Լ��ٶ����ݽ��й�һ������ */
		recipNorm = invSqrt(data.accel_x * data.accel_x + data.accel_y * data.accel_y + data.accel_z * data.accel_z);
		data.accel_x = data.accel_x * recipNorm;
		data.accel_y = data.accel_y * recipNorm;
		data.accel_z = data.accel_z * recipNorm;

		/* DCM������ת */
		vx = 2 * (q1*q3 - q0*q2);
		vy = 2 * (q0*q1 + q2*q3);
		vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
		//��һ��д��
		//vx = q1*q3 - q0*q2;
		//vy = q0*q1 + q2*q3;
		//vz = q0*q0 - 0.5f + q3*q3;

		/* �ڻ�������ϵ������������õ��������� *///�������������Ĳ��
		ex = data.accel_y * vz - data.accel_z * vy;
		ey = data.accel_z * vx - data.accel_x * vz;
		ez = data.accel_x * vy - data.accel_y * vx;

		/* ��������PI���㣬�������ٶ� */
		exInt = exInt + ex * Ki;
		eyInt = eyInt + ey * Ki;
		ezInt = ezInt + ez * Ki;

		data.gyro_x = data.gyro_x + Kp * ex + exInt;
		data.gyro_y = data.gyro_y + Kp * ey + eyInt;
		data.gyro_z = data.gyro_z + Kp * ez + ezInt;

		/* ������Ԫ��΢�ֹ�ʽ������Ԫ������ *///��������Ԫ���ı仯��
		q0 = q0 + (-q1 * data.gyro_x - q2 * data.gyro_y - q3 * data.gyro_z)*halfT;
		q1 = q1 + (q0 * data.gyro_x + q2 * data.gyro_z - q3 * data.gyro_y)*halfT;
		q2 = q2 + (q0 * data.gyro_y - q1 * data.gyro_z + q3 * data.gyro_x)*halfT;
		q3 = q3 + (q0 * data.gyro_z + q1 * data.gyro_y - q2 * data.gyro_x)*halfT;

		/* ��Ԫ����һ�� */
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 = q0 * recipNorm;
		q1 = q1 * recipNorm;
		q2 = q2 * recipNorm;
		q3 = q3 * recipNorm;

		/* ����ŷ���� */
		float roll = atan2f(2 * q2*q3 + 2 * q0*q1, -2 * q1*q1 - 2 * q2*q2 + 1) * 57.3f;
		float pitch = asinf(2 * q1*q3 - 2 * q0*q2) * 57.3f;
		float yaw = -atan2f(2 * q1*q2 + 2 * q0*q3, -2 * q2*q2 - 2 * q3*q3 + 1) * 57.3f;

		///�������_ŷ����
		//printf("pitch:%.2f roll:%.2f yaw:%.2f\r\n", pitch, roll, yaw);
		//std::cout << " yaw: " << yaw << " pitch: " << pitch << " roll: " << roll << std::endl;
		//std::cout << q0 << "  " << q1 << "  " << q2 << "  " << q3 << std::endl;

		#pragma endregion

		#pragma region λ�ý���

		/*1���ֲ�����ϵ_ȥ����Ӱ�죬������ٶ�����*/
		Quaternion q_next(q0, q1, q2, q3);//��̬��Ԫ��_��ʱ��
		Quaternion q_iniToNext = RotateQua(q_initial, q_next);//��̬_��������ϵ->��ʱ�ֲ̾�����ϵ
		Vector3 g_next = RotateAcceV2(q_iniToNext, g_world);//�������ٶ�_��ʱ�ֲ̾�����ϵ
		g_next.z = -g_next.z;//����δ֪�����������������޸�g_next��Zֵ����
		Vector3 a_local = a_next + g_next;
	
		FilterAcceInWorld(q_next, a_local);//�˳�С�����䷶Χ�ļ��ٶ�������ǿ�м����Ŷ�//trick

		/*2���ֲ�����ϵ_��һ�̵��ٶȱ任����ʱ��*/
		Quaternion q_prevToNext = RotateQua(q_prev, q_next);//��̬_��һʱ��->��ʱ��
		Vector3 v_local = RotateAcceV2(q_prevToNext, v_prev);//ת������ʱ������ϵ���ٶ�

		/*3���ֲ�����ϵ_����λ�������뱾ʱ���ٶ���*/
		Vector3 p_local = v_local * 2 * halfT + a_local * 0.5 * (2*halfT) * (2*halfT);//��Ϊ�ȱ����˶������㱾ʱ��λ������
		Vector3 v_next = v_local + a_local * 2 * halfT;//��ʱ���ٶȣ����ݸ��´ε�����

		/*4����������ϵ_λ��������ת����������ֵ�ۼ�*/
		Quaternion q_nextToIni = RotateQua(q_next, q_initial);//��̬_��ʱ�ֲ̾�����ϵ->��������ϵ
		Quaternion p_local_(0, p_local.x, p_local.y, p_local.z);
		Vector3 p_world = RotateAcceV2(q_nextToIni, p_local_);//��������ϵ�µ�λ������
		Vector3 p_next = p_prev + p_world;//��ʱ������ֵ

		#pragma endregion

		//������ǰ��״̬��λ��+��̬��Ϣ��,���
		previous_state_ = { roll, pitch, yaw, p_next.x, p_next.y, p_next.z };
		pre_v_ = { v_next.x , v_next.y , v_next.z};
		return previous_state_;

		///����_�����̬��Ϣ+XXX
		//ImuState acceState = { roll, pitch, yaw, a_local.x, a_local.y, a_local.z };
		//return acceState;
	}

	/*������imu���ݽ��д���*/
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



/*�˲�ģ��*/////////////////////////
class Filter
{
public:


	/*�������˲�����������ٶ�*/
	void KalmenFilterOne(std::vector<float> &acceList)
	{
		Eigen::Vector3f x(3, 1);
		x << 0, 0, 0;//1*3״̬����������Ӧλ�á��ٶȡ����ٶ�

		Eigen::Matrix3f A(3, 3);
		A << 1, halfT * 2, halfT * halfT * 2,
			0, 1, halfT * 2,
			0, 0, 1;//��ɢ�����״̬��

		Eigen::RowVector3f H(1, 3);
		H << 0.0f, 0.0f, 1.0f;//�۲�����

		Eigen::Matrix3f Q(3, 3);
		Q << 0, 0, 0,
			0, 0, 0,
			0, 0, 0.1f;//ϵͳ����,Խ��Բ������ݵĸ���Ч��Խ�ã�Խ���ܣ�������Խ��
						//����ĶԽ��ߵ�ֵ������ֻ��[3,3]��ֵ����Ϊֻ�Լ��ٶȸ����Ŷ�

		float R = 0.1f;//���⣨��̬������
					   //Q / (Q + R)��ֵ���ǿ��������������ֵ

		Eigen::Matrix3f Pk(3, 3);
		Pk << 0.1f, 0, 0,
			0, 0.1f, 0,
			0, 0, 0.1f;//Э��������ֵ//����λ�ơ��ٶȡ����ٶ�ֵ��Ϊ0.1f��

		std::vector<float> acceList_new;
		for (int i = 0; i < acceList.size(); i++)//���������¹�ʽ
		{
			/*1��Ԥ�ⲽ*/
			Eigen::Vector3f x_(3, 1);
			x_ << A * x;

			Eigen::Matrix3f Pk_(3, 3);
			Pk_ << A * Pk * A.transpose() + Q;

			/*2�����²�*///ʵ������ֱ����ת��
			Eigen::Vector3f Kk(3, 1);
			Kk << (Pk_ * H.transpose()) / (H * Pk_ * H.transpose() + R);

			x << x_ + Kk * (acceList[i] - H * x_);

			Pk << (Eigen::MatrixXf::Identity(3, 3) - Kk * H) * Pk_;

			//position(i) = x(0);//λ����Ϣ
			//speed(i) = x(1);//�ٶ���Ϣ
			//acc(i) = x(2);//���ٶ���Ϣ
			acceList_new.push_back(x(2));
		}

		acceList = acceList_new;

	}

	/*�������˲�������������ٶ�*/
	void KalmenFilterParent(std::vector<ImuData> &imu_original)
	{
		//ȡ��������ٶ�
		std::vector<float> ax;
		std::vector<float> ay;
		std::vector<float> az;
		for (unsigned int i = 0; i < imu_original.size(); i++)
		{
			ax.push_back(imu_original[i].accel_x);
			ay.push_back(imu_original[i].accel_y);
			az.push_back(imu_original[i].accel_z);
		}

		//�������˲�����
		KalmenFilterOne(ax);
		KalmenFilterOne(ay);
		KalmenFilterOne(az);
		
		//���봦������ֵΪλ����/�ٶȱ仯��/���ٶȱ仯������KalmenFilterOne����
		for (unsigned int i = 0; i < imu_original.size(); i++)
		{
			imu_original[i].accel_x = ax[i];
			imu_original[i].accel_y = ay[i];
			imu_original[i].accel_z = az[i];
		}

	}

	/*λ�ý��㣬��ɸѡ������ٶ�*///�����ã������ڶԼ��ٶ����������˲�������
	void CalculatePositionAndFilterAcce(std::vector<ImuState> &imu_state)
	{
		//λ�ò���
		Vector3 v_prev(0, 0, 0);//��һʱ���ٶ�
		Vector3 p_prev(0, 0, 0);//��һʱ��λ��

		for (unsigned i = 0; i < imu_state.size(); i++)
		{		
			//if (i<50)
			//{//trick��ȥ����ʼ���е�����
			//	imu_state[i].x = 0;
			//	imu_state[i].y = 0;
			//	imu_state[i].z = 0;
			//	continue;
			//}

			/*1��ɸѡ��ֹ״̬�µļ��ٶ�*/
			if (imu_state[i].x > -correct_ax && imu_state[i].x < correct_ax) imu_state[i].x = 0;
			if (imu_state[i].y > -correct_ay && imu_state[i].y < correct_ay) imu_state[i].y = 0;
			if (imu_state[i].z > -correct_az && imu_state[i].z < correct_az) imu_state[i].z = 0;

			/*2��λ�ý���*/
			//��Ϊ�ȱ����˶������㱾ʱ�̵�λ��
			Vector3 a_next(imu_state[i].x, imu_state[i].y, imu_state[i].z);//��ʱ�̼��ٶ�
			Vector3 p_next = p_prev + v_prev * 2 * halfT + a_next * 0.5 * (2 * halfT) * (2 * halfT);
			Vector3 v_next = v_prev + a_next * 2 * halfT;//��ʱ���ٶ�//���ݸ��´ε�����

			//����״̬
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
Դ�ԣ�https://www.ourfpv.com/chat/
�������ʹ�ü��ٶȼƺ����������ݶ�IMU����̬��λ�ý��й��ơ�������Ҫ�������£�

1. ��ԭʼIMU�����м�����ٶȼƽǶȡ�
2. �����ٶȼƽǶ���ǰһ��״̬��ϣ��Ի�ȡ���ƵĹ����Ǻ͸����ǡ�
3. ��ԭʼ���������ݺ�ǰһ��ƫ���Ǽ����µ�ƫ���ǡ�
4. �����Ƶ���̬��ǰһ��λ�ý�ϣ��Ի�ȡ�µ�λ�ù��ơ�
5. ����ǰһ��״̬��������״̬��
**************************************/
int main() 
{
	///����
	std::vector<std::string> fileList;//�ļ����б�
	std::vector<ImuData> imu_original;//imuԭʼ����
	std::vector<ImuState> imu_state;//imu��������
	File file;//�ļ��������

	/*1����顢��ȡimu�ļ�*/
	if (file.isEmptyDir(filePath))
	{
		std::cout << std::endl << "δ��ȡ��imu�ļ��������Ƿ��з����ļ���" << std::endl;
		return 0;
	}
	file.GetFiles(filePath, fileList);
	filePath = fileList[0];
	std::cout << "�Ѷ�ȡ��imu�ļ���·����" << filePath << std::endl;
	fileList.clear();
	totalRow = CountLines(filePath);//�����ļ������������ڽ�����
	file.ReadTxt(filePath, imu_original);//��ȡ·���µ�txt�ļ�


	/*2���˲�����*/
	Filter filter;//�˲��������
	std::cout << std::endl << "�˲�������..." << std::endl;
	filter.KalmenFilterParent(imu_original);//�Լ��ٶȽ��п������˲�


	/*3�����㴦��*/
	ImuCalculate_Qua imuCalculate_Qua;//imuԤ�������
	std::cout << std::endl << "imu������..." << std::endl;
	imuCalculate_Qua.ProcessAllData(imu_original, imu_state);

	///����_��ӡ������Ϣ
	//std::cout << "X		Y		Z		Yaw		Roll		Pitch" << std::endl;
	//for (unsigned i = 0; i < imu_state.size(); i++)
	//	std::cout //<< std::cout.precision(8)
	//	<< imu_state[i].x << "  " << imu_state[i].y << "  " << imu_state[i].z
	//	<< imu_state[i].yaw << "	" << imu_state[i].roll << "	" << imu_state[i].pitch << std::endl;


	/*4����ϡ��ѡ��*/
	//AdjustImu(imu_state);


	/*5���洢Ϊtxt�ļ�*/
	file.SaveTxt(imu_state);


	std::cout << std::endl;
	system("PAUSE");
	return 0;
}


	#pragma region ����

/**************/
/**************/
//��������ô���ĵ����������������������ȷʵ����Ҫ�����ԼӸ�QQ����:459390464@qq.com��
//�ҿ���ֻ�ǵ�һ���ˣ��ܰﾡ���������XD
/*************/
/*************/

	#pragma region ���ǵ���ɶ��

// Simulate some raw IMU data and run it through the preprocessor//ģ��һЩԭʼIMU���ݲ�ͨ��Ԥ����������
//ImuData data = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.1 };
//ImuState state = preprocessor.ProcessNewData(data);

/*
		// Ԥ����һ��ʱ�̵���̬
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
//����new
double new_w = pre_w + (v_w * pre_w - v_x * pre_x - v_y * pre_y - v_z * pre_z);
double new_x = pre_x + (v_w * pre_x + v_x * pre_w + v_y * pre_z - v_z * pre_y);
double new_y = pre_y + (v_w * pre_y - v_x * pre_z + v_y * pre_w + v_z * pre_x);
double new_z = pre_z + (v_w * pre_z + v_x * pre_y - v_y * pre_x + v_z * pre_w);
//new ��һ��
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

///����_q
std::cout << new_w << "  " << new_x << "  " << new_y << "  " << new_z << std::endl;

*/

// ȥ����Ӱ����
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
//	double g[3];    // ��ά��������
//	double gravity; // �������ٶȴ�С
//};

//Ԥ���������
//class ImuPreprocessor
//{
//public:
//	ImuPreprocessor() : previous_state_({ 0.516, -2.347, 345.504, 432026.165, 3895254.285, 79.119 }) {}
//	//ImuPreprocessor() : previous_velocity_({ 0, 0, 0 }) {}
//
//	// Perform IMU preprocessing on a new set of raw data, returning the new state//��һ���µ�ԭʼ����ִ��IMUԤ����������״̬
//	ImuState ProcessNewData(const ImuData& data) {
//
//		///�������
//		//std::cout << pre_v_.vx << "  " << pre_v_.vy << "  " << pre_v_.vz << std::endl;
//		//std::cout << data.accel_x << "  " << data.accel_y << "  " << data.accel_z << std::endl;
//
//
//		// Compute accelerometer angles from raw accelerometer readings//����ԭʼ���ټƶ���������ټƽǶ�
//		float accel_roll = atan2(data.accel_y, data.accel_z);
//		float accel_pitch = atan2(-data.accel_x, sqrt(data.accel_y * data.accel_y + data.accel_z * data.accel_z));
//
//		// Combine accelerometer angles with previous state to get estimated pitch and roll//�����ټƽǶ�����ǰ״̬���ϣ��Ի�ù��Ƶĸ����Ͳ���//ԭʼ
//		//float new_roll = 0.5 * previous_state_.roll + 0.5 * (-data.gyro_y * dt_ + accel_roll);//0.5Ϊ����ϵ��
//		//float new_pitch = 0.5 * previous_state_.pitch + 0.5 * (data.gyro_x * dt_ + accel_pitch);
//		// Compute new yaw from raw gyroscope readings and previous yaw//����ԭʼ�����Ƕ�������ǰƫ��������ƫ��
//		//float new_yaw = previous_state_.yaw + data.gyro_z * dt_;
//
//		//�����ǽ�����̬��
//		float gyro_roll = (data.gyro_x +
//			(sin(accel_pitch) * sin(accel_roll)) / cos(accel_pitch) * data.gyro_y +
//			(cos(accel_roll) * sin(accel_pitch)) / cos(accel_pitch)*data.gyro_z) * dt_;
//		float gyro_pitch = (cos(accel_roll) * data.gyro_y - sin(accel_roll) * data.gyro_z) * dt_;
//		float gyro_yaw = (sin(accel_roll) / cos(accel_pitch) * data.gyro_y + cos(accel_roll) / cos(accel_pitch) * data.gyro_z) * dt_;
//
//		//��̬�ںϣ����ٶȼ������̬���������ǵ���̬���ں�
//		//float new_roll = previous_state_.roll + 0.4 * (accel_roll - gyro_roll);//0.4Ϊ����ϵ��
//		//float new_pitch = previous_state_.pitch + 0.4 * (accel_pitch - gyro_pitch);
//		//float new_yaw = previous_state_.yaw + 0.4 * gyro_yaw;
//		//float new_roll = accel_roll * 0.5 + (gyro_roll + previous_state_.roll) * 0.5;//ʹ�û����˲���
//		//float new_pitch = accel_pitch * 0.5 + (gyro_pitch + previous_state_.pitch) * 0.5;
//		//float new_yaw = gyro_yaw * 0.5 + previous_state_.yaw * 0.5;
//		float new_roll = gyro_roll + previous_state_.roll;//����ʹ�������ǵ���̬��
//		float new_pitch = gyro_pitch + previous_state_.pitch;
//		float new_yaw = gyro_yaw + previous_state_.yaw;
//
//		///�������
//		//std::cout << " gyro_yaw: " << gyro_roll << " gyro_pitch: " << gyro_pitch << " gyro_roll: " << gyro_yaw << std::endl;
//		//std::cout << " accel_roll: " << accel_roll << " accel_pitch: " << accel_pitch << std::endl << std::endl;
//
//
//		//���㵱ǰ��̬�µ���������
//		float g = sqrt(data.accel_x * data.accel_x + data.accel_y * data.accel_y + data.accel_z * data.accel_z);
//		//��������ٶȼ�ȥ�����õ�ȥ�������ٶ�//ת��Ϊ��ʵ���ٶȣ���9.8��?
//		//float cur_ax = data.accel_x + g * sin(accel_pitch);
//		//float cur_ay = data.accel_y - g * cos(accel_pitch) * sin(accel_roll);
//		//float cur_az = data.accel_z - g * cos(accel_pitch) * cos(accel_roll);
//		float cur_ax = (-g * sin(new_pitch));
//		float cur_ay = (g * cos(new_pitch) * sin(new_roll));//
//		float cur_az = (g * cos(new_pitch) * cos(new_roll) - 9.8f);
//
//		///�������
//		//std::cout << cur_ax << "  " << cur_ay << "  " << cur_az << std::endl;
//
//
//		//���������ٶȣ����ݼ��ٶȺ͵�ǰ�ٶȼ�����һʱ�̵��ٶ�
//		float new_vx = pre_v_.vx + (cur_ax - pre_v_.ax) * dt_;
//		float new_vy = pre_v_.vy + (cur_ay - pre_v_.ay) * dt_;
//		float new_vz = pre_v_.vz + (cur_az - pre_v_.az) * dt_;
//
//		//�����ٶȺ͵�ǰλ�ü�����һʱ�̵�λ��
//		float new_position_x = previous_state_.position_x + new_vx * dt_;
//		float new_position_y = previous_state_.position_y + new_vy * dt_;
//		float new_position_z = previous_state_.position_z + new_vz * dt_;
//
//		// Combine estimated pose with previous position to get new position estimate//�����Ƶ���������ǰ��λ������Ի���µ�λ�ù���
//		//float new_position_x = previous_state_.position_x + cos(new_yaw) * sin(new_pitch) * cos(new_roll) + sin(new_yaw) * sin(new_roll);
//		//float new_position_y = previous_state_.position_y + cos(new_yaw) * sin(new_pitch) * sin(new_roll) - sin(new_yaw) * cos(new_roll);
//		//float new_position_z = previous_state_.position_z + cos(new_yaw) * cos(new_pitch);
//
//		// Update previous state and return new state//������ǰ��״̬��������״̬
//		previous_state_ = { new_roll, new_pitch, new_yaw, new_position_x, new_position_y, new_position_z };
//		pre_v_ = { new_vx , new_vy , new_vz ,cur_ax , cur_ay , cur_az };
//		return previous_state_;
//	}
//
//private:
//	ImuState previous_state_;
//	ImuVelocity pre_v_ = { 0, 0, 0, 0, 0, 0 };
//	const float dt_ = 0.005f; // Sample time (in seconds)//����ʱ�䣨�룩
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
//}//ChatGPT���ˣ�qʵ��Ϊq2q1

	#pragma endregion


///*����������������������ϵ�µ�ֵ*///������
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
//	///����
//	//std::cout << g_imu.x << "  " << g_imu.y << "  " << g_imu.z << std::endl;
//
//	return g_imu;
//}
//
///*������һ��ʱ�̵��ٶ�*///������
//Vector3 calculateVelocity(const Vector3& v_prev, const Vector3& a_prev, const Quaternion& q_prev, const Quaternion& q_next) {
//	Vector3 g_prev = calculateGravity(q_prev);
//	Vector3 g_next = calculateGravity(q_next);
//	Vector3 a_imu = a_prev - g_prev;
//	Vector3 a_world = getConjugate(q_prev) * (q_prev * a_imu);
//	Vector3 v_next = v_prev + a_world * halfT * 2 + (g_prev + g_next) * halfT;
//	return v_next;
//}
///	/*���������ڴ������ϵ�µ�ֵ����ȥ����*///����
//Vector3 RotateAcce(Quaternion qua, Quaternion acce)
//{
//	//����ϵת��
//	//Quaternion qvq = getConjugate(qua) * acce * qua;//���ת���أ�
//	Quaternion qvq = qua * (acce * getConjugate(qua));//����ת��أ�
//	Vector3 qv_world(qvq.x, qvq.y, qvq.z);
//	//ȥ����Ӱ�죨�ڴ������ϵ�£�
//	qv_world.z -= gValue;
//
//	return qv_world;
//}
///	/*���㱾ʱ�̼��ٶ��ڵ�ǰ����ϵ�µ�ֵV3����ȥ������Ӱ��*/
//Vector3 RotateAcceV3(Quaternion acce)
//{
//	//����acce������̬��ת��ʹ�ñ���ľֲ�����ϵ
//	Vector3 a_prev_world(acce.x, acce.y, acce.z);
//	//ȥ����Ӱ�죨�ڴ������ϵ�£�
//	a_prev_world.z -= gValue;
//
//	return a_prev_world;
//}


///��Ԫ���仯���ܵĲ���
//Quaternion q_next(q0, q1, q2, q3);//��̬��Ԫ��_��ʱ��
//Quaternion q_iniTonext = RotateQua(q_initial, q_next);//��Ԫ��_��������ϵ->��ʱ�ֲ̾�����ϵ
//Vector3 g_next = RotateAcceV2(q_iniTonext, g_world);//�������ٶ�_��ʱ�ֲ̾�����ϵ
////Vector3 a_local = a_next - g_next;
//Vector3 a_local = a_next;
//
//
/////����_��Ԫ���仯
//Quaternion a_local_(0, a_local.x, a_local.y, a_local.z);
//Vector3 a_world = RotateAcceV2(q_next, a_local_);
//Quaternion a_world_(0, a_world.x, a_world.y, a_world.z);
//Vector3 a_local2 = RotateAcceV2(q_iniTonext, a_world_);
//
//std::cout << a_world.x << "  " << a_world.y << "  " << a_world.z << std::endl;
//std::cout << a_local2.x << "  " << a_local2.y << "  " << a_local2.z << std::endl << std::endl;
///����_��Ԫ���ı任����
//Quaternion q1(1, 0, 0, 0);
//Quaternion q2(0.207f, 0, 0.207f, 0);
//Quaternion test1 = imuCalculate_Qua.RotateQua(q1, q2);
//std::cout << "test1:  " << test1.w << "  " << test1.x << "  " << test1.y << "  " << test1.z << std::endl;
//Quaternion test2 = imuCalculate_Qua.RotateQua(q2, q1);
//std::cout << "test2:  " << test2.w << "  " << test2.x << "  " << test2.y << "  " << test2.z << std::endl;
//Quaternion gQua = { 0,0,0,gValue };
//Vector3 test3 = imuCalculate_Qua.RotateAcceV2(test1, gQua);
//std::cout << "test3:  " << test3.x << "  " << test3.y << "  " << test3.z << std::endl;
///����_��Ԫ���仯
//Quaternion a_local_(0, a_local.x, a_local.y, a_local.z);
//Vector3 a_world = RotateAcceV2(q_next, a_local_);
//Quaternion a_world_(0, a_world.x, a_world.y, a_world.z);
//Vector3 a_local2 = RotateAcceV2(q_iniTonext, a_world_);
//std::cout << a_local.x << "  " << a_local.y << "  " << a_local.z << std::endl;
//std::cout << a_local2.x << "  " << a_local2.y << "  " << a_local2.z << std::endl << std::endl;

///λ�ý���_������ٶȵ�����ϵ�������ȼ����˶���λ��
/*********************************/
//�����ڴ������ϵ�µ���һʱ�̼��ٶ�//������-nan(ind)ֵ����������ԭ���Ǳ���δ��ʼ��
//Quaternion a_next_(0, a_next.x, a_next.y, a_next.z);
//Vector3 a_next_world = RotateAcceV2(q_next, a_next_);//�������ӵ�ǰ����ϵ�仯����ʼ/��������ϵ
//a_next_world.z -= gValue;//ȥ������Ӱ�죨����������ϵ�£�
//��Ϊ�ȱ����˶������㱾ʱ�̵�λ��
//Vector3 p_next = p_prev + v_prev * 2 * halfT + a_next_world * 0.5 * (2 * halfT) * (2 * halfT);
//Vector3 v_next = v_prev + a_next_world * 2 * halfT;//��ʱ�̵��ٶ�//���ݸ��´ε�����
/*******************************/



	#pragma endregion
