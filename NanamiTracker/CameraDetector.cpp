#define _ITERATOR_DEBUG_LEVEL 0
#include "CameraDetector.h"
#include <iostream>
#include <dshow.h>
#include <winerror.h>

int CameraDeviceDetect(void) {

	// �e��ϐ��̐ݒ�
	// Enum: ��  Moniker: ������
	ICreateDevEnum* pCreateDevEnum = NULL;
	IEnumMoniker* pEnumMoniker = NULL;
	IMoniker* pMoniker = NULL;
	ULONG nFetched = 0;

	// COM initialization
	if(FAILED(CoInitialize(NULL))) return -1;

	// �f�o�C�X�񋓗p��CreateDevEnum����
	if(FAILED(CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, IID_ICreateDevEnum, (PVOID*)&pCreateDevEnum))) return -1;

	// �񋓗pEnumMoniker����
	pCreateDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEnumMoniker, 0);

	// �f�o�C�X���Ȃ��ꍇ
	if (pEnumMoniker == NULL) {
		std::cout << "No device detected." << std::endl;
		return -1;
	}

	// Enum�V�[�P���X��擪�ɖ߂�
	pEnumMoniker->Reset();

	// �����グ�p
	int num = 0;

	// Enum�V�[�P���X�̍Ō���܂ŌJ��Ԃ�
	while (pEnumMoniker->Next(1, &pMoniker, &nFetched) == S_OK) {

		// �o�C���h�p��IPropertyBag��p��
		// IPropertyBag : �C�ӂ̃L�[��VARIANT�^����������COM�C���^�t�F�[�X
		IPropertyBag* pPropertyBag;
		TCHAR devname[256];
		TCHAR description[256];
		TCHAR devpath[256];

		// �o�C���h
		pMoniker->BindToStorage(0, 0, IID_IPropertyBag, (void**)&pPropertyBag);

		VARIANT var;

		// FriendlyName�擾
		var.vt = VT_BSTR;

		pPropertyBag->Read(L"FriendlyName", &var, 0);
		WideCharToMultiByte(CP_ACP, 0, var.bstrVal, -1, devname, sizeof(devname), 0, 0);

		VariantClear(&var);

		// Description�擾
		var.vt = VT_BSTR;

		pPropertyBag->Read(L"description", &var, 0);
		WideCharToMultiByte(CP_ACP, 0, var.bstrVal, -1, description, sizeof(description), 0, 0);

		VariantClear(&var);

		// DevicePath�擾
		var.vt = VT_BSTR;

		pPropertyBag->Read(L"DevicePath", &var, 0);
		WideCharToMultiByte(CP_ACP, 0, var.bstrVal, -1, devpath, sizeof(devpath), 0, 0);

		VariantClear(&var);

		// �o��
		TCHAR result[1024];
		memset(result, 0, sizeof(result));
		sprintf_s(result, "%d : %s", num, devname);
		std::cout << result << std::endl;

		// ���\�[�X���J��
		pMoniker->Release();
		pPropertyBag->Release();

		num++;

	}

	// ���\�[�X�J��
	pEnumMoniker->Release();
	pCreateDevEnum->Release();

	// COM�̏I��
	CoUninitialize();

	return 0;
}