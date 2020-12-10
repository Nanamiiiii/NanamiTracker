#include "FacetrackingSystem.h"

// カメラ用パラメータ
double K[9] = { 6.5308391993466671e+002, 0.0, 3.1950000000000000e+002, 0.0, 6.5308391993466671e+002, 2.3950000000000000e+002, 0.0, 0.0, 1.0 };
double D[5] = { 7.0834633684407095e-002, 6.9140193737175351e-002, 0.0, 0.0, -1.3073460323689292e+000 };

// main処理
void Trackingsystem(cv::VideoCapture cap, FACE_PARAM out_param, boolean window_gen) {

	/********** 検出器初期化 ************/
	// 顔検出器
	dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();

	// 顔器官点検出器
	// 機械学習データの読み込み
	dlib::shape_predictor predictor;
	//try {
		dlib::deserialize("shape_predictor_68_face_landmarks.dat") >> predictor;
	//}
	//catch (dlib::serialization_error e) {
	//	std::cout << e.info << std::endl;
	//	return 0;
	//}


	// カメラ組み込み関数・歪み係数
	cv::Mat cam_matrix = cv::Mat(3, 3, CV_64FC1, K);
	cv::Mat dist_coeffs = cv::Mat(5, 1, CV_64FC1, D);

	// 顔器官の3D座標初期値
	std::vector<cv::Point3d> object_pts;
	// 顔向きを示す矩形の3D
	std::vector<cv::Point3d> reprojectsrc;
	// 初期値を指定
	SetInitialPoints(&reprojectsrc, &object_pts);


	//---------- result ------------//
	cv::Mat rotation_vec;									// 回転ベクトル
	cv::Mat rotation_mat;									// 回転行列
	cv::Mat translation_vec;								// 変換ベクトル
	cv::Mat pose_mat = cv::Mat(3, 4, CV_64FC1);				// 姿勢 (3x4行列)
	cv::Mat euler_angle = cv::Mat(3, 1, CV_64FC1);			// Eular角(3x1行列)
	cv::Mat prev_euler_angle = cv::Mat(3, 1, CV_64FC1);		// Previous frame (Eular角)

	std::vector<cv::Point2d> reprojectdst;					// 2D座標の格納
	reprojectdst.resize(8);

	// 格納用顔クラス
	Face actor;

	// 出力用変数
	cv::Mat out_intrinsics = cv::Mat(3, 3, CV_64FC1);
	cv::Mat out_rotation = cv::Mat(3, 3, CV_64FC1);
	cv::Mat out_translation = cv::Mat(3, 1, CV_64FC1);

	// 画面出力
	std::ostringstream outtext;

	// メインのループ処理
	while (TRUE) {

		// カメラから画像フレームを取得
		cv::Mat temp;
		cap >> temp;

		// 変換
		dlib::cv_image<dlib::bgr_pixel> cimg(temp);

		// 顔検出
		std::vector<dlib::rectangle> faces = detector(cimg);

		// 顔が検出された場合
		if (faces.size() > 0) {

			// 顔器官点検出
			dlib::full_object_detection shape = predictor(cimg, faces[0]);
			// 顔領域取得
			auto rect = shape.get_rect();

			// 器官点描画
			for (unsigned int i = 0; i < FACIAL_POINTS; i++) {

				cv::circle(
					temp,						// 描画先データ
					cv::Point(					// 器官点の2D点obj
						shape.part(i).x(),
						shape.part(i).y()),
					2,
					cv::Scalar(0, 0, 255),
					-1);
			}

			// 2D座標の一部を格納
			std::vector<cv::Point2d> image_pts;
			image_pts.push_back(cv::Point2d(shape.part(17).x(), shape.part(17).y())); // #17 右眉右端
			image_pts.push_back(cv::Point2d(shape.part(21).x(), shape.part(21).y())); // #21 右眉左端
			image_pts.push_back(cv::Point2d(shape.part(22).x(), shape.part(22).y())); // #22 左眉右端
			image_pts.push_back(cv::Point2d(shape.part(26).x(), shape.part(26).y())); // #26 左眉左端

			image_pts.push_back(cv::Point2d(shape.part(36).x(), shape.part(36).y())); // #36 右目右端
			image_pts.push_back(cv::Point2d(shape.part(39).x(), shape.part(39).y())); // #39 右目左端
			image_pts.push_back(cv::Point2d(shape.part(42).x(), shape.part(42).y())); // #42 左目右端
			image_pts.push_back(cv::Point2d(shape.part(45).x(), shape.part(45).y())); // #45 左目左端
			image_pts.push_back(cv::Point2d(shape.part(31).x(), shape.part(31).y())); // #31 鼻右
			image_pts.push_back(cv::Point2d(shape.part(35).x(), shape.part(35).y())); // #35 鼻左
			image_pts.push_back(cv::Point2d(shape.part(48).x(), shape.part(48).y())); // #48 口右
			image_pts.push_back(cv::Point2d(shape.part(54).x(), shape.part(54).y())); // #54 口左
			image_pts.push_back(cv::Point2d(shape.part(57).x(), shape.part(57).y())); // #57 口中央下
			image_pts.push_back(cv::Point2d(shape.part(8).x(), shape.part(8).y()));   // #8  顎

			// 目関連のパラメータを別で格納
			std::vector<cv::Point2d> eye_left;
			std::vector<cv::Point2d> eye_right;

			for (int i = 42; i < 48; i++) {
				// 左目の各点座標 42~47
				eye_left.push_back(cv::Point2d(shape.part(i).x(), shape.part(i).y()));
			}

			for (int i = 36; i < 42; i++) {
				// 右目の各点座標 36~41
				eye_right.push_back(cv::Point2d(shape.part(i).x(), shape.part(i).y()));
			}

			// 顔器官点の3D座標現在値と姿勢情報の算出
			cv::solvePnP(object_pts, image_pts, cam_matrix, dist_coeffs, rotation_vec, translation_vec);

			// 顔向き矩形の3D座標現在値を算出
			cv::projectPoints(reprojectsrc, rotation_vec, translation_vec, cam_matrix, dist_coeffs, reprojectdst);

			// 顔向き矩形の描画
			DrawFaceBox(temp, reprojectdst);

			// 顔向き矩形の対角線交点の座標算出
			cv::Point2d P1(reprojectdst[8].x, reprojectdst[8].y);
			cv::Point2d P2(reprojectdst[9].x, reprojectdst[9].y);
			cv::Point2d P3(reprojectdst[10].x, reprojectdst[10].y);
			cv::Point2d P4(reprojectdst[11].x, reprojectdst[11].y);

			double S1 = ((P4.x - P2.x) * (P1.y - P2.y) - (P4.y - P2.y) * (P1.x - P2.x)) / 2;
			double S2 = ((P4.x - P2.x) * (P2.y - P3.y) - (P4.y - P2.y) * (P2.x - P3.x)) / 2;

			long Center_X = P1.x + (P3.x - P1.x) * S1 / (S1 + S2);
			long Center_Y = P1.y + (P3.y - P1.y) * S1 / (S1 + S2);

			// 顔の現在位置設定
			cv::Point2d CenterPoint(Center_X, Center_Y);
			actor.h_pos = ((float)Center_X / 2) / 640;
			actor.v_pos = ((float)Center_Y / 2) / 360;

			// 顔の姿勢情報
			cv::Rodrigues(rotation_vec, rotation_mat);
			cv::hconcat(rotation_mat, translation_vec, pose_mat);
			cv::decomposeProjectionMatrix(pose_mat, out_intrinsics, out_rotation, out_translation, cv::noArray(), cv::noArray(), cv::noArray(), euler_angle);

			// フィルタ処理で姿勢変化を滑らかに
			// 変動にしきい値を設定 & Smoothing
			// 顔の角度制限(Live2Dが30まで対応なのでそれに制限)
			for (int i = 0; i < 3; i++)
			{
				if (euler_angle.at<double>(i) > MAX_FACE_ANGLE) euler_angle.at<double>(i) = MAX_FACE_ANGLE;
				if (euler_angle.at<double>(i) < -MAX_FACE_ANGLE) euler_angle.at<double>(i) = -MAX_FACE_ANGLE;
				if ((std::abs(prev_euler_angle.at<double>(i) - euler_angle.at<double>(i)) > 1))
				{
					if (prev_euler_angle.at<double>(i) > euler_angle.at<double>(i))
					{
						euler_angle.at<double>(i) -= 0.005;
					}
					else
					{
						euler_angle.at<double>(i) += 0.005;
					}
					euler_angle.at<double>(i) = (LPF_VALUE_PRE * prev_euler_angle.at<double>(i)) + (LPF_VALUE_CUR * euler_angle.at<double>(i));
				}
				else
				{
					euler_angle.at<double>(i) = prev_euler_angle.at<double>(i);
				}
			}

			prev_euler_angle.at<double>(0) = euler_angle.at<double>(0);
			prev_euler_angle.at<double>(1) = euler_angle.at<double>(1);
			prev_euler_angle.at<double>(2) = euler_angle.at<double>(2);

			// 目の開閉の判定処理
			double ear_left = (calc_dst(eye_left[1], eye_left[5]) + calc_dst(eye_left[2], eye_left[4])) / (2.0 * calc_dst(eye_left[0], eye_left[3]));
			double ear_right = (calc_dst(eye_right[1], eye_right[5]) + calc_dst(eye_right[2], eye_right[4])) / (2.0 * calc_dst(eye_right[0], eye_right[3]));
			double close_val = 0.15; // 閉じている判定のしきい値
			double open_val = 0.3; // 開きの最大判定値 (後々初期値設定側で導入)

			// しきい値で値を0~1に収める
			ear_left /= open_val;
			ear_right /= open_val;
			if (ear_left <= close_val / open_val) ear_left = 0;
			if (ear_right <= close_val / open_val) ear_right = 0;


			// 眼球座標の算出
			// 目領域の設定
			EYE_REGION eye_left_region, eye_right_region;
			cv::Mat left_eye_img, right_eye_img;
			IRIS left_iris, right_iris;

			if (ear_left > 0) {
				eye_left_region.top = cv::Point2d(eye_left[0].x, eye_left[1].y < eye_left[2].y ? eye_left[1].y : eye_left[2].y);
				eye_left_region.bottom = cv::Point2d(eye_left[3].x, eye_left[4].y < eye_left[5].y ? eye_left[5].y : eye_left[4].y);
				left_eye_img = temp(cv::Rect(eye_left_region.top.x, eye_left_region.top.y, eye_left_region.bottom.x - eye_left_region.top.x, eye_left_region.bottom.y - eye_left_region.top.y));
				left_iris = detect_iris(left_eye_img);
			}
			else {
				left_iris.center = cv::Point2f(0, 0);
				left_iris.radius = 0;
			}

			if (ear_right > 0) {
				eye_right_region.top = cv::Point2d(eye_right[0].x, eye_right[1].y < eye_right[2].y ? eye_right[1].y : eye_right[2].y);
				eye_right_region.bottom = cv::Point2d(eye_right[3].x, eye_right[4].y < eye_right[5].y ? eye_right[5].y : eye_right[4].y);
				right_eye_img = temp(cv::Rect(eye_right_region.top.x, eye_right_region.top.y, eye_right_region.bottom.x - eye_right_region.top.x, eye_right_region.bottom.y - eye_right_region.top.y));
				right_iris = detect_iris(right_eye_img);
			}
			else {
				right_iris.center = cv::Point2f(0, 0);
				right_iris.radius = 0;
			}
			
			cv::Point2d left_center(left_iris.center.x + eye_left_region.top.x, left_iris.center.y + eye_left_region.top.y);
			cv::Point2d right_center(right_iris.center.x + eye_right_region.top.x, right_iris.center.y + eye_right_region.top.y);

			// 絶対座標を格納
			IRIS left_iris_gl = cp_iris(left_iris);
			IRIS right_iris_gl = cp_iris(right_iris);

			left_iris_gl.center = left_center;
			right_iris_gl.center = right_center;

			cv::circle(temp, left_iris_gl.center, left_iris_gl.radius, cv::Scalar(0, 255, 0), 1);
			cv::circle(temp, right_iris_gl.center, right_iris_gl.radius, cv::Scalar(0, 255, 0), 1);

			// 画面表示：顔角度
			outtext << "X: " << std::setprecision(3) << euler_angle.at<double>(0);
			cv::putText(temp, outtext.str(), cv::Point(50, 40), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255));
			outtext.str("");
			outtext << "Y: " << std::setprecision(3) << euler_angle.at<double>(1);
			cv::putText(temp, outtext.str(), cv::Point(50, 60), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255));
			outtext.str("");
			outtext << "Z: " << std::setprecision(3) << euler_angle.at<double>(2);
			cv::putText(temp, outtext.str(), cv::Point(50, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255));
			outtext.str("");

			// 画面表示：顔位置
			outtext << "Pos_X: " << std::setprecision(3) << actor.h_pos;
			cv::putText(temp, outtext.str(), cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255));
			outtext.str("");

			outtext << "Pos_Y: " << std::setprecision(3) << actor.v_pos;
			cv::putText(temp, outtext.str(), cv::Point(50, 120), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255));
			outtext.str("");

			actor.yaw = euler_angle.at<double>(1);
			actor.pitch = euler_angle.at<double>(0);
			actor.roll = euler_angle.at<double>(2);

			// 共有用
			out_param._face = actor;
			out_param.left_iris = left_iris;
			out_param.right_iris = right_iris;

			image_pts.clear();
			//press esc to end
			//escで終了
			
			auto key = cv::waitKey(2);

			// if (key == '\x1b')
			// {
			//	break;
			// }
		}
		Sleep(1 / 1000);

		// カメラ映像出力
		if(window_gen) cv::imshow("FaceTrack", temp);
		cv::waitKey(1);
	}
	return;
}

void DrawFaceBox(cv::Mat frame, std::vector<cv::Point2d> reprojectdst) {

	cv::line(frame, reprojectdst[8], reprojectdst[9], cv::Scalar(0, 0, 255));
	cv::line(frame, reprojectdst[9], reprojectdst[10], cv::Scalar(0, 0, 255));
	cv::line(frame, reprojectdst[10], reprojectdst[11], cv::Scalar(0, 0, 255));
	cv::line(frame, reprojectdst[11], reprojectdst[8], cv::Scalar(0, 0, 255));

	cv::line(frame, reprojectdst[8], reprojectdst[10], cv::Scalar(0, 0, 255));
	cv::line(frame, reprojectdst[9], reprojectdst[11], cv::Scalar(0, 0, 255));
}

void SetInitialPoints(std::vector<cv::Point3d>* in_BoxPoints, std::vector<cv::Point3d>* in_FaceLandmarkPoints) {

	std::vector<cv::Point3d> reprojectsrc = (std::vector<cv::Point3d>) * in_BoxPoints;
	std::vector<cv::Point3d> object_pts = (std::vector<cv::Point3d>) * in_FaceLandmarkPoints;

	reprojectsrc.push_back(cv::Point3d(10.0, 10.0, 10.0));
	reprojectsrc.push_back(cv::Point3d(10.0, 10.0, -10.0));
	reprojectsrc.push_back(cv::Point3d(10.0, -10.0, -10.0));
	reprojectsrc.push_back(cv::Point3d(10.0, -10.0, 10.0));
	reprojectsrc.push_back(cv::Point3d(-10.0, 10.0, 10.0));
	reprojectsrc.push_back(cv::Point3d(-10.0, 10.0, -10.0));
	reprojectsrc.push_back(cv::Point3d(-10.0, -10.0, -10.0));
	reprojectsrc.push_back(cv::Point3d(-10.0, -10.0, 10.0));

	reprojectsrc.push_back(cv::Point3d(-10.0, -10.0, 0));
	reprojectsrc.push_back(cv::Point3d(10.0, -10.0, 0));
	reprojectsrc.push_back(cv::Point3d(10.0, 10.0, 0));
	reprojectsrc.push_back(cv::Point3d(-10.0, 10.0, 0));

	//顔器官点の3D座標初期値,  この点を基準に、顔の姿勢を算出する 
	//参照元 http://aifi.isr.uc.pt/Downloads/OpenGL/glAnthropometric3DModel.cpp
	object_pts.push_back(cv::Point3d(6.825897, 6.760612, 4.402142));     //#33 右眉右端
	object_pts.push_back(cv::Point3d(1.330353, 7.122144, 6.903745));     //#29 右眉左端
	object_pts.push_back(cv::Point3d(-1.330353, 7.122144, 6.903745));    //#34 左眉右端
	object_pts.push_back(cv::Point3d(-6.825897, 6.760612, 4.402142));    //#38 左眉左端
	object_pts.push_back(cv::Point3d(5.311432, 5.485328, 3.987654));     //#13 右目右端
	object_pts.push_back(cv::Point3d(1.789930, 5.393625, 4.413414));     //#17 右目左端
	object_pts.push_back(cv::Point3d(-1.789930, 5.393625, 4.413414));    //#25 左目右端
	object_pts.push_back(cv::Point3d(-5.311432, 5.485328, 3.987654));    //#21 左目左端
	object_pts.push_back(cv::Point3d(2.005628, 1.409845, 6.165652));     //#55 鼻右
	object_pts.push_back(cv::Point3d(-2.005628, 1.409845, 6.165652));    //#49 鼻左
	object_pts.push_back(cv::Point3d(2.774015, -2.080775, 5.048531));    //#43 口右
	object_pts.push_back(cv::Point3d(-2.774015, -2.080775, 5.048531));   //#39 口左
	object_pts.push_back(cv::Point3d(0.000000, -3.116408, 6.097667));    //#45 口中央下
	object_pts.push_back(cv::Point3d(0.000000, -7.415691, 4.070434));    //#6 顎
	*in_BoxPoints = reprojectsrc;
	*in_FaceLandmarkPoints = object_pts;
}

double calc_dst(cv::Point2d a, cv::Point2d b) {
	return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

/// <summary>
/// 虹彩検出
/// </summary>
/// <param name="eye_img">cv::Mat 目の画像</param>
/// <returns>struct IRIS 検出した虹彩</returns>
IRIS detect_iris(cv::Mat eye_img) {

	cv::Mat eye_img_gs; 
	cv::cvtColor(eye_img, eye_img_gs, cv::COLOR_BGR2GRAY);
	cv::Mat eye_img_gau;
	cv::GaussianBlur(eye_img_gs, eye_img_gau, cv::Size(5, 5), 0);
	
	// 二値化
	// しきい値の60は最も安定したため用いた
	int bin_th = 60;
	cv::Mat eye_threshold;
	cv::threshold(eye_img_gau, eye_threshold, bin_th, 255, cv::THRESH_BINARY);
	cv::rectangle(eye_threshold, cv::Point(0, 0), cv::Point(eye_threshold.cols - 1, eye_threshold.rows - 1), cv::Scalar(255, 255, 255), 1);

	// 輪郭検出
	// 出力先変数の宣言
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	// 検出
	cv::findContours(eye_threshold, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

	// 虹彩の導出
	IRIS _iris = { cv::Point2d(0, 0), .0 };
	for (std::vector<cv::Point> v : contours) {
		cv::Point2f center;
		float radius;
		cv::minEnclosingCircle(v, center, radius);

		// 丸め込み
		center = cv::Point2f((int)center.x, (int)center.y);
		radius = (int)radius;

		// あまりにも大きい半径は除外
		if (eye_threshold.size[0] < radius * 0.8) continue;

		// そのうえで半径が最大のものを虹彩とする
		if (_iris.radius < radius) {
			_iris.radius = radius;
			_iris.center = center;
		}
	}
	return _iris;
}



// 上手くいかないのでpタイルは一旦廃止

/// <summary>
/// Pタイル法による二値化
/// </summary>
/// <param name="img_gs">グレースケールイメージ</param>
/// <param name="ratio">二値化の割合</param>
/// <returns>二値化処理後のイメージ</returns>
cv::Mat threshold_by_ptile(cv::Mat img_gs, double ratio) {
	
	// ヒストグラム生成
	cv::MatND img_hist;
	int histSize = { 256 };
	float range[] = { 0,256 };
	const float* histRange = { range };

	cv::calcHist(&img_gs, 1, 0, cv::Mat(), img_hist, 1, &histSize, &histRange);

	// 画素数
	int pixel = img_gs.rows * img_gs.cols;
	int threshold = pixel * ratio;

	double rat_sum = 0;
	int p_tile_th = 0;
	
	
	for (int i = 0; i < 256; i++) {
		double rat = img_hist.at<double>(i);
		rat_sum += rat;
		if (rat_sum > threshold) break;
		p_tile_th++;
	}
	
	
	cv::Mat img_th;
	cv::threshold(img_gs, img_th, p_tile_th, 255, cv::THRESH_BINARY);

	return img_th;
}

IRIS cp_iris(IRIS _iris) {

	IRIS ret;
	ret.center = cv::Point2f(_iris.center.x, _iris.center.y);
	ret.radius = _iris.radius;

	return ret;
}