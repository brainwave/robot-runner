#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream>

#include <Python.h>

#define INDEX 0

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

CameraPtr cameraInit()
{
	// Display Greeting Message
	cout <<"Application build date: "<< __DATE__ << " "<< __TIME__ <<"\n\n";
	
	
	// Singleton reference to system object
	SystemPtr system = System::GetInstance();
	
	// Retrieve list of cameras from the system
	CameraList camList = system->GetCameras();

	// End execution if no cameras by returning null pCam
	if (camList.GetSize() == 0)
	{
		camList.Clear();
		system->ReleaseInstance();
		
		cout << "No camera detected, press Enter to exit...\n";
		
		CameraPtr pCam = NULL;
		return pCam;
	}
	
	// If cameras were found, return pointer to the camera pointed by Index
	CameraPtr pCam = camList.GetByIndex(INDEX);
	
	return pCam;
}

int main() {}
	
		
