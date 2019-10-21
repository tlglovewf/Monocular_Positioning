#ifndef __SSDFunction_H__
#define __SSDFunction_H__
#include <vector>
#include <opencv2/opencv.hpp>
#include "SignDefine.h"
#include "SignAdapterContainer.h"
#include "ISignDetector.h"

namespace FunctionSpace
{
	struct ImageCacheInfo
	{
		cv::Mat img;
		int frameID;
		std::string imgFile;
	};

	class SSDDetect
	{
	public:
		SSDDetect(int argc, char** argv);
		~SSDDetect(){}

		void MonoSignDetectAndLocation();
	private:

		bool Init(int argc, char** argv);

		std::shared_ptr<SignSDK::SignAdapterContainer> m_ptrContainer;

		bool iniFlag;

		std::string m_input;
		std::string m_output;
		int m_display;
		int m_gpuID;
		int m_start;
		int m_end;
		std::string m_posfile;
	};


}

#endif // !__SSDFunction_H__



