#pragma once
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include "robot_state.h"
//#include <opencv2/video/tracking.hpp>

#include "kal_filter.hpp"
#define PI 3.141592654

using namespace cv;

class KAL:public robot_state
{
private:
	double depth;
	
	Mat F_MAT;
	Eigen::Matrix3d F;
	
	Mat C_MAT;
	Eigen::Matrix<double, 1, 5>C;
	
	int measure_a;
	
	int state_a;
	
	float t = -1;
	
//	int stop_predict = 0;
	
	//double last_aim_yaw = 0;
	//double last_aim_pitch = 0;
	//cv::Point last_ct = {-1,-1};
	float last_yaw=0;
	
	float shoot_delay_init = 0.4519;
	float shoot_delay = shoot_delay_init;
	
	double filter = 0.05;

public:
	KAL() = default;
	
	void reset();
	void sp_reset(kal_filter &kf);
	kal_filter init();
	Eigen::Vector3d pnp_get_pc(const cv::Point2f p[4], const double& w, const double& h);
	float keep_pi(float angle);
	
	bool predict(RotatedRect& detection, kal_filter & kf, double t);
	
	inline Eigen::Vector3d pu_to_pc(Eigen::Vector3d& pu)
	{
		return F.inverse()*(pu * depth) ;//transpose�����ת��,inverse��������
	}
	
	inline Eigen::Vector3d pc_to_pu(Eigen::Vector3d& pc)
	{
		return F * pc / depth;
	}

	inline double get_gravity(Eigen::Vector3d &pos3)
	{
	    //printf("pos3.No3:%f\t%f\n",pos3[2]);
	    double del_ta = pow(SPEED, 4) + 2 * 9.8 * pos3(1, 0) * SPEED * SPEED - 9.8 * 9.8 * pos3(2, 0) * pos3(2, 0);

	    double t_2 = (9.8 * pos3(1, 0) + SPEED * SPEED - sqrt(del_ta)) / (0.5 * 9.8 * 9.8);

	    double height = 0.5 * 9.8 * t_2;
	    //printf("height:%f\n",height);

	    return height;
	}
	
	Mat _src;
	int type;
//	int send_flag = 0;

    inline void get_send(Eigen::Vector3d &pos, double height)
    {
        send.yaw = atan2(pos3(0, 0) + 0.106, pos3(2, 0))/PI * 180.0 - ab_yaw;
        double xishu = 5.02 * (3.10/pos3(2,0));

        send.pitch = atan2(pos3(1, 0) - 0.055 - height *  xishu, pos3(2, 0))/PI * 180.0 - ab_pitch;
    }
	
	struct information {
		float yaw = 0.0;
		float pitch = 0.0;
	};
	
	information send;
	
	float ab_pitch = 0.0;
	float ab_yaw = 0.0;
	float ab_roll = 0.0;
	float SPEED = 26.5;
	
	
};
