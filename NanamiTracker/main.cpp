#include "FacetrackingSystem.h"
#include "CameraDetector.h"
#include "DxLib.h"
#include <iostream>
#include <thread>

// Live2D���f���̃p�����[�^
std::string Live2D_Params[] = { "ParamAngleX", "ParamAngleY", "ParamAngleZ", "ParamEyeLOpen", "ParamEyeROpen", "ParamEyeBallX", "ParamEyeBallY" };

/// <summary>
/// 
/// </summary>
void DispConsole() {

	// Create Console Window
	AllocConsole();

	// setup standard input and output
	FILE* fp = NULL;
	freopen_s(&fp, "CONOUT$", "w", stdout);
	freopen_s(&fp, "CONIN$", "r", stdin);

}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) {

	// run console
	DispConsole();

	// �J�����f�o�C�X�̎擾
	int dev_id;
	if(CameraDeviceDetect() != 0) return -1;

	// �I��
	std::cout << "Select device: ";
	std::cin >> dev_id;

	// �J�����I�[�v��
	cv::VideoCapture cap(dev_id);
	if (!cap.isOpened()) {

		std::cout << "Unable to connect." << std::endl;
		return -1;

	}

	// Set Live2D SDK DLL Path
#ifdef _WIN64
	Live2D_SetCubism4CoreDLLPath("dll/x64/Live2DCubismCore.dll");
#else
	Live2D_SetCubism4CoreDLLPath("dll/x86/Live2DCubismCore.dll");
#endif

	// DxLib Initialization
	SetMainWindowText("NanamiTracker");
	SetGraphMode(1920, 1080, 32);
	ChangeWindowMode(TRUE);
	if (DxLib_Init() != 0) return -1;
	SetDrawScreen(DX_SCREEN_BACK);

	// Load Live2D Model
	int ModelHandle = Live2D_LoadModel("D:/model_data/Hiyori/Hiyori.model3.json");

	// �e�p�����[�^���i�[����\����
	FACE_PARAM face_p;

	// �X���b�h�����iTracking�V�X�e���͗��œ��삳����j
	std::thread tracking_sys(Trackingsystem, cap, face_p, FALSE);
	tracking_sys.join();

	// 500ms�����ҋ@
	WaitTimer(500);

	while (ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0) {
		
		// �e��p�����[�^�̍X�V
		Live2D_Model_SetParameterValue(ModelHandle, Live2D_Params[0].c_str(), face_p._face.pitch);
		Live2D_Model_SetParameterValue(ModelHandle, Live2D_Params[1].c_str(), face_p._face.yaw);
		Live2D_Model_SetParameterValue(ModelHandle, Live2D_Params[2].c_str(), face_p._face.roll);


	}

	return 0;
}