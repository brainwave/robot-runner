//Spinnaker SDK
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"


//Standard C++ Header files
#include <iostream>
#include <sstream> 

//Ros header files
#include <ros/ros.h>
#include <image_transport/image_transport.h>

#define INDEX 0

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

// This function acquires and saves 10 images from a device.  
int AcquireImages(CameraPtr pCam, INodeMap & nodeMap, INodeMap & nodeMapTLDevice)
{
	int result = 0;
	
	cout << endl << endl << "*** IMAGE ACQUISITION ***" << endl << endl;
	
	try
	{
		CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
		if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
		{
			cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << endl << endl;
			return -1;
		}
		
		// Retrieve entry node from enumeration node
		CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
		if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
		{
			cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..." << endl << endl;
			return -1;
		}
		
		// Retrieve integer value from entry node
		int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
		
		// Set integer value from entry node as new value of enumeration node
		ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
		
		pCam->BeginAcquisition();

		cout << "Acquiring images..." << endl;
		
		gcstring deviceSerialNumber("");
		CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
		if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
		{
			deviceSerialNumber = ptrStringSerial->GetValue();

			cout << "Device serial number retrieved as " << deviceSerialNumber << "..." << endl;
		}
		cout << endl;
		
		// Retrieve, convert, and save images
		
		
		ImagePtr  pImage ;
		while(1)
		{
			try
			{
				pImage = pCam->GetNextImage();

				if (pImage->IsIncomplete())
				{
					
					cout << "Image incomplete with image status " << pImage->GetImageStatus() << "..." << endl << endl;
				}
				else
				{
					size_t width = pImage->GetWidth();
					size_t height = pImage->GetHeight();

					ImagePtr convertedImage = pImage->Convert(PixelFormat_BGR8, NEAREST_NEIGHBOR);

        				unsigned int XPadding = convertedImage->GetXPadding();
        				unsigned int YPadding = convertedImage->GetYPadding();
        				unsigned int rowsize = convertedImage->GetWidth();
        				unsigned int colsize = convertedImage->GetHeight();

        				//image data contains padding. When allocating Mat container size, you
        				Mat cvimg = cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC3, convertedImage->GetData(), convertedImage->GetStride());
        				namedWindow("current Image", CV_WINDOW_AUTOSIZE);
					imshow("Current Image",cvimg);
					resizeWindow("current Image",rowsize/2, colsize/2);
					waitKey(1);
					

				}

				pImage->Release();

			}
			catch (Spinnaker::Exception &e)
			{
				cout << "Error: " << e.what() << endl;
				result = -1;
			}
		}
		
		pCam->EndAcquisition();
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}
	
	return result;
}

// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int PrintDeviceInfo(INodeMap & nodeMap)
{
	int result = 0;
	
	cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;

	try
	{
		FeatureList_t features;
		CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
		if (IsAvailable(category) && IsReadable(category))
		{
			category->GetFeatures(features);

			FeatureList_t::const_iterator it;
			for (it = features.begin(); it != features.end(); ++it)
			{
				CNodePtr pfeatureNode = *it;
				cout << pfeatureNode->GetName() << " : ";
				CValuePtr pValue = (CValuePtr)pfeatureNode;
				cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
				cout << endl;
			}
		}
		else
		{
			cout << "Device control information not available." << endl;
		}
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}
	
	return result;
}

// This function acts as the body of the example; please see NodeMapInfo example 
// for more in-depth comments on setting up cameras.
int RunSingleCamera(CameraPtr pCam)
{
	int result = 0;

	try
	{
		// Retrieve TL device nodemap and print device information
		INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
		
		result = PrintDeviceInfo(nodeMapTLDevice);
		
		// Initialize camera
		pCam->Init();
		
		// Retrieve GenICam nodemap
		INodeMap & nodeMap = pCam->GetNodeMap();

		// Acquire images
		result = result | AcquireImages(pCam, nodeMap, nodeMapTLDevice);
		
		// Deinitialize camera
		pCam->DeInit();
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}

// Example entry point; please see Enumeration example for more in-depth 
// comments on preparing and cleaning up the system.
int main(int /*argc*/, char** /*argv*/)
{
	int result = 0;

	//initialize ros
	ros::init(argc, argv, "pointgrey_publisher");
	
	cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;
	
	// Retrieve singleton reference to system object
	SystemPtr system = System::GetInstance();

	// Retrieve list of cameras from the system
	CameraList camList = system->GetCameras();
	
	// Finish if there are no cameras
	if (camList.GetSize() == 0)
	{
		// Clear camera list before releasing system
		camList.Clear();

		// Release system
		system->ReleaseInstance();

		cout << "Not enough cameras!" << endl;
		cout << "Done! Press Enter to exit..." << endl;
		getchar();
		
		return -1;
	}

	CameraPtr pCam = NULL;

	// Select camera, default is one
	pCam = camList.GetByIndex(INDEX);

	cout << endl << "Running example for camera " << INDEX << "..." << endl;
		
		// Run example
		result = result | RunSingleCamera(pCam);
		
		cout << "Camera " << INDEX << " example complete..." << endl << endl;
	

	//
	// Release reference to the camera
	//
	// *** NOTES ***
	// Had the CameraPtr object been created within the for-loop, it would not
	// be necessary to manually break the reference because the shared pointer
	// would have automatically cleaned itself up upon exiting the loop.
	//
	pCam = NULL;

	// Clear camera list before releasing system
	camList.Clear();

	// Release system
	system->ReleaseInstance();

	cout << endl << "Done! Press Enter to exit..." << endl;
	getchar();

	return result;
}
