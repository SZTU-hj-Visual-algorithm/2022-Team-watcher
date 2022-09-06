#include "Thread.hpp"
#include <cstdio>
#include <opencv2/opencv.hpp>

using namespace cv;

bool is_continue = true;

typedef struct form
{
	RotatedRect ROT;
	int Armor_type;
	float a[4];
	int is_get;
	int mode;
	int da_is_get;
}form;

form send_data;

Mat ka_src_get;

SerialPort port("/dev/ttyUSB0");

void* Build_Src(void* PARAM)
{
	Mat get_src;
	auto camera_warper = new Camera;
	printf("camera_open\n");
	if (camera_warper->init())
	{
		printf("1\n");
		while (is_continue && !(waitKey(10) == 27))
		{
			if (camera_warper->read_frame_rgb())
			{
				//printf("1\n");
				get_src = cv::cvarrToMat(camera_warper->ipiimage).clone();
				//time_count = (double)getTickCount();
				pthread_mutex_lock(&mutex_new);
				{
					get_src.copyTo(src);
					is_start = true;
					pthread_cond_signal(&cond_new);
					pthread_mutex_unlock(&mutex_new);
					imshow("src",src);

					camera_warper->release_data();
				}
				//camera_warper->record_start();
				//camera_warper->camera_record();
			}
			else
			{
				src = cv::Mat();
			}
			
		}
		camera_warper->~Camera();
		is_continue = false;
	}
	else
	{
		printf("No camera!!\n");
		is_continue = false;
	}
}

void* Armor_Kal(void* PARAM)
{
	ArmorDetector shibie = ArmorDetector();
	RotatedRect mubiao;
	Mat src_copy;
	long int time_count = 0;

	port.initSerialPort();
	
	sleep(2);
	printf("Armor_open\n");
	while (is_continue)
	{
		pthread_mutex_lock(&mutex_new);

		while (!is_start) {

			pthread_cond_wait(&cond_new, &mutex_new);

		}

		is_start = false;

		src.copyTo(src_copy);

		//imshow("src_copy",src_copy);

		pthread_mutex_unlock(&mutex_new);
		float lin[4];
		int mode_temp/* = 0x22*/;
		//lin[0] = 0.0;
 		//lin[1] = 5.0;
		//lin[2] = 5.0;
		//lin[3] = 25.0;
		bool small_energy = false;
		int lin_is_get;
		//lin_is_get = true;
		lin_is_get = port.get_Mode1(mode_temp, lin[0], lin[1], lin[2], lin[3],shibie.enermy_color);
		//mode_temp = 0x22;
		shibie.enermy_color = RED;
		//printf("mode:%x\n",shibie.enemy_color);
		//printf("mode_temp:%x\n",mode_temp);
		//printf("speed:%lf\n",lin[3]);
		if (mode_temp == 0x21)
		{
		    aim_information aim_get = shibie.getTargetAera(src_copy, 0, 0);
		    RotatedRect mubiao_get = aim_get.final_rect;
		    if (aim_get.class_id == 0)
		    {
		        mubiao_get = RotatedRect();
		    }
			pthread_mutex_lock(&mutex_ka);
			send_data.ROT = mubiao_get;
			send_data.Armor_type = shibie.isSamllArmor();
			send_data.a[0] = lin[0];
			send_data.a[1] = lin[1];
			send_data.a[2] = lin[2];
			send_data.a[3] = lin[3];
			src_copy.copyTo(ka_src_get); 
			send_data.mode = mode_temp;
			send_data.is_get = lin_is_get;
			is_ka = true;
			pthread_cond_signal(&cond_ka);
			pthread_mutex_unlock(&mutex_ka);
		}
		

	}
}

void* Kal_predict(void* PARAM)
{
	VisionData vdata;
	int Armor_type = 1;
	KAL ka;
	kal_filter kf;
	int mode_temp;
	double time_count = 0;
	kf = ka.init();
	form get_data;
	sleep(3);
	printf("kal_open\n");
	int is_get;
	int mode;
	int is_send;
	float ji_pitch,ji_yaw;
	int pan_wu = 0;
	RotatedRect mubiao;

	while (is_continue)
	{
		pthread_mutex_lock(&mutex_ka);

		while (!is_ka) {

			pthread_cond_wait(&cond_ka, &mutex_ka);
		}

		is_ka = false;
	
		ka_src_get.copyTo(ka._src);
		ka.ab_pitch = send_data.a[0];
		ka.ab_yaw = send_data.a[1];
		ka.ab_roll = send_data.a[2];
		ka.SPEED = send_data.a[3];
		is_get=send_data.is_get;
		mode = send_data.mode;
		is_send = send_data.da_is_get;
		pthread_mutex_unlock(&mutex_ka);
		mubiao=send_data.ROT;
		ka.type = (send_data.Armor_type) ? 1 : 2;
		if(is_get)
		{
			if (mode == 0x21)
			{
				if (ka.predict(mubiao, kf, time_count))
				{
					time_count = (double)getTickCount();
					ji_pitch=ka.send.pitch;
					ji_yaw = ka.send.yaw;
					vdata = { -ji_pitch, -ji_yaw, 0x31 };
					printf("yaw:%f\npitch:%f\n", -ka.send.yaw, -ka.send.pitch);
					port.TransformData(vdata);
					port.send();
					pan_wu = 0;
				}
				else
				{
					if(pan_wu<=12)
					{
						
						vdata = { -ji_pitch, -ji_yaw, 0x31 };
						printf("yaw:%f\npitch:%f\npan_wu:%d\n",-ji_yaw, -ji_pitch,pan_wu);
						port.TransformData(vdata);
						port.send();
						pan_wu++;
					}else
					{
						ji_yaw = 0.0 - ka.ab_yaw;
						ji_pitch = 0.0 - ka.ab_pitch;
						ka.sp_reset(kf);
						vdata = { -ji_pitch, -ji_yaw, 0x32 };
						//printf("real none!!");
						//printf("chong\n");
						port.TransformData(vdata);
						port.send();
					}
				}
			}
		}
	}
}


