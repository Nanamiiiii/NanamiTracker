#pragma once
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/serialize.h>
#include <vector>
#include <Windows.h>

// フィルタ用係数
#define LPF_VALUE_PRE 0.4
#define LPF_VALUE_CUR (1 - LPF_VALUE_PRE)
#define FACIAL_POINTS 68

// 顔角度の最大値
#define MAX_FACE_ANGLE 30

// 顔クラス
class Face {
public:
	float h_pos;
	float v_pos;
	float yaw;  // y_angle
	float pitch; // x_angle
	float roll;  // z_angle
};

// 目領域
typedef struct eye_region {
	cv::Point2d top;
	cv::Point2d bottom;
}EYE_REGION;

// 虹彩
typedef struct iris {
	cv::Point2f center;
	double radius;
}IRIS;

// 共有用パラメータ
typedef struct face_param {
	Face _face;
	IRIS left_iris;
	IRIS right_iris;
}FACE_PARAM;

// ローカル関数
void Trackingsystem(cv::VideoCapture cap, FACE_PARAM out_param, boolean window_gen);
static void DrawFaceBox(cv::Mat frame, std::vector<cv::Point2d> reprojectdst); // 顔枠生成
static void SetInitialPoints(std::vector<cv::Point3d>* in_BoxPoints, std::vector<cv::Point3d>* in_FaceLandmarkPoints); // 顔器官点の設定
static double calc_dst(cv::Point2d a, cv::Point2d b);
static IRIS detect_iris(cv::Mat eye_img);
static cv::Mat threshold_by_ptile(cv::Mat img_gs, double ratio);
static IRIS cp_iris(IRIS _iris);