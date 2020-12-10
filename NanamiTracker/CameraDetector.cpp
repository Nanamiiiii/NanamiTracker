#define _ITERATOR_DEBUG_LEVEL 0
#include "CameraDetector.h"
#include <iostream>
#include <dshow.h>
#include <winerror.h>

int CameraDeviceDetect(void) {

	// 各種変数の設定
	// Enum: 列挙  Moniker: あだ名
	ICreateDevEnum* pCreateDevEnum = NULL;
	IEnumMoniker* pEnumMoniker = NULL;
	IMoniker* pMoniker = NULL;
	ULONG nFetched = 0;

	// COM initialization
	if(FAILED(CoInitialize(NULL))) return -1;

	// デバイス列挙用のCreateDevEnum生成
	if(FAILED(CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, IID_ICreateDevEnum, (PVOID*)&pCreateDevEnum))) return -1;

	// 列挙用EnumMoniker生成
	pCreateDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEnumMoniker, 0);

	// デバイスがない場合
	if (pEnumMoniker == NULL) {
		std::cout << "No device detected." << std::endl;
		return -1;
	}

	// Enumシーケンスを先頭に戻す
	pEnumMoniker->Reset();

	// 数え上げ用
	int num = 0;

	// Enumシーケンスの最後尾まで繰り返す
	while (pEnumMoniker->Next(1, &pMoniker, &nFetched) == S_OK) {

		// バインド用のIPropertyBagを用意
		// IPropertyBag : 任意のキーとVARIANT型を結合したCOMインタフェース
		IPropertyBag* pPropertyBag;
		TCHAR devname[256];
		TCHAR description[256];
		TCHAR devpath[256];

		// バインド
		pMoniker->BindToStorage(0, 0, IID_IPropertyBag, (void**)&pPropertyBag);

		VARIANT var;

		// FriendlyName取得
		var.vt = VT_BSTR;

		pPropertyBag->Read(L"FriendlyName", &var, 0);
		WideCharToMultiByte(CP_ACP, 0, var.bstrVal, -1, devname, sizeof(devname), 0, 0);

		VariantClear(&var);

		// Description取得
		var.vt = VT_BSTR;

		pPropertyBag->Read(L"description", &var, 0);
		WideCharToMultiByte(CP_ACP, 0, var.bstrVal, -1, description, sizeof(description), 0, 0);

		VariantClear(&var);

		// DevicePath取得
		var.vt = VT_BSTR;

		pPropertyBag->Read(L"DevicePath", &var, 0);
		WideCharToMultiByte(CP_ACP, 0, var.bstrVal, -1, devpath, sizeof(devpath), 0, 0);

		VariantClear(&var);

		// 出力
		TCHAR result[1024];
		memset(result, 0, sizeof(result));
		sprintf_s(result, "%d : %s", num, devname);
		std::cout << result << std::endl;

		// リソースを開放
		pMoniker->Release();
		pPropertyBag->Release();

		num++;

	}

	// リソース開放
	pEnumMoniker->Release();
	pCreateDevEnum->Release();

	// COMの終了
	CoUninitialize();

	return 0;
}