#pragma once
#include "camera.h"
#include "ArmorDetector.hpp"
#include "KAl.h"
#include <opencv2/core/cvstd.hpp>
#include "CRC_Check.h"
#include"serialport.h"
#include <thread>
#include <mutex>
#include<string>

extern pthread_mutex_t mutex_new; 
extern pthread_cond_t cond_new; 
extern pthread_mutex_t mutex_ka; 
extern pthread_cond_t cond_ka; 

extern bool is_ka;
extern bool is_start;         
extern cv::Mat src;
                     

void* Build_Src(void* PARAM);
void* Armor_Kal(void* PARAM);
void* Kal_predict(void* PARAM);
