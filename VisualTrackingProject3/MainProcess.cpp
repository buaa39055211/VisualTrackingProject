#include <iostream>
#include <string>
#include<vector>
#include<opencv2\opencv.hpp>
#include "Tracker.h"
//#include"SingleTemplateTracker.h"
#include"MultiTemplateTracker.h"
#include"datasets.h"




using namespace std;
using namespace cv;
using namespace mycv;


namespace global {
	bool paused = true;//单击鼠标右键，暂停标志
	Mat displayImg;//绘制选择目标时的鼠标的拖动轨迹
	bool selectObject = false;//selectObject初始值为false
	bool isRoiReady = 0;//ROI区域是否已经选择好
	Point origin;//ROI区域左上角起始位置
	Rect selectedRoi;//最终通过鼠标选择的ROI区域

	static void onMouse(int event, int x, int y, int, void*)
	{
		if (selectObject)//按下鼠标左键后，该段语句开始执行
		{//按住左键拖动鼠标时候，该鼠标响应函数
		 //会被不断触发，不断计算目标矩形的窗口
			selectedRoi.x = MIN(x, origin.x);
			selectedRoi.y = MIN(y, origin.y);
			selectedRoi.width = std::abs(x - origin.x);
			selectedRoi.height = std::abs(y - origin.y);
			selectedRoi &= Rect(0, 0, displayImg.cols, displayImg.rows);//不能越界
			rectangle(displayImg, selectedRoi, Scalar(0, 0, 255), 1);//画出鼠标的选择痕迹
		}
		switch (event)
		{
			//当在第一帧按下鼠标左键后，selectObject被设置为true,拖动鼠标开始选择目标的矩形区域
		case CV_EVENT_LBUTTONDOWN:
			origin = Point(x, y);
			selectedRoi = Rect(x, y, 0, 0);
			selectObject = true;
			isRoiReady = false;
			break;
			//直到左键抬起，标志目标区域选择完毕，selectObject被设置为false
		case CV_EVENT_LBUTTONUP:
			selectObject = false;
			if (selectedRoi.width > 0 && selectedRoi.height > 0)
				isRoiReady = true;
			cout << "选中的矩形区域为：" << selectedRoi << endl;
			break;
			//单击右键，暂停或开始
		case CV_EVENT_RBUTTONDOWN:
			paused = !paused;
		}
	}
}

/*namespace datsets {
	//const string datasets_dir = "\\";
	const string video1 = "123.mp4";//datasetsir + "123.mp4";
	int video1_strat_frame = 5;

	const string video = video1;
	int strat_frame = video1_strat_frame;

}*/

int main(int argc, char*argv[]) {
    //指定数据集,起始帧，起始目标
    mycv::DataSet dataset = mycv::dataset21;
	
	
	//声明一个读取视频流的类VideoCapture的对象
	VideoCapture capture;
	//调用Videocapture对象的open函数打开视频文件
	capture.open(dataset.video_name);
	//判断相机是否被打开
	CV_Assert(capture.isOpened());
	//获取视频信息
	const int FrameCount = (int)capture.get(VideoCaptureProperties::CAP_PROP_FRAME_COUNT);
	const int FrameWidth = (int)capture.get(VideoCaptureProperties::CAP_PROP_FRAME_WIDTH);
	const int FrameHeight = (int)capture.get(VideoCaptureProperties::CAP_PROP_FRAME_HEIGHT);
	const Rect FrameArea(0, 0, FrameWidth, FrameHeight);
	//设置从第几帧开始读取
	int frameIndex = dataset.strat_frame;
	capture.set(VideoCaptureProperties::CAP_PROP_POS_FRAMES,double(frameIndex ));

	//创建显示跟踪过程的窗口
	const string winName = "Tracking Window";
	namedWindow(winName, 1);
	setMouseCallback(winName, global::onMouse, 0);


	//读取指定的起始帧
	Mat CurrentFrame, WorkFrame;
	capture >> CurrentFrame;
	CV_Assert(!CurrentFrame.empty());
	cout << "当前帧索引：" << frameIndex << endl;
	frameIndex++;

	//在起始帧上选择目标
	while (!global::isRoiReady)
	{
		//将起始帧拷贝到displayImg中
		CurrentFrame.copyTo(global::displayImg);
		//在按下鼠标左键和抬起左键之间，selectObject为真
		//selectROI会随着鼠标的移动不断变化，直到抬起鼠标左键后
		//selectObject为假，selectedROI就是选中的目标矩形框
		if (global::selectObject&&global::selectedRoi.width > 0 && global::selectedRoi.height>0)
		{
			Mat roi_img(global::displayImg, global::selectedRoi);
			cv::bitwise_not(roi_img, roi_img);//把选中的区域图像反转显示
		}
		//显示鼠标选择过程
		imshow(winName, global::displayImg);
		waitKey(10);
	}

	//如果lock_roi==true,就表示鼠标的额选择区域无效
	if (dataset.lock_roi)
		global::selectedRoi = dataset.start_roi;
	cout << "初始帧上的目标位置" <<global::selectedRoi<< endl;


	cout << "声明跟踪器对象实例，初始化目标跟踪器。。。" << endl;

	//mycv::STTracker::Params params = mycv::STTracker::Params();//命名空间：：类：：结构体，赋值默认参数
	//params.alpha = 0.7;//修改默认参数
	//params.numPoints = 800;
	//Ptr<mycv::Tracker> tracker=new mycv::SingleTemplateTracker(params);//更改为指针类型mycv::Tracker tracker;子类的指针直接赋值给父类的指针，类型提升SingleTemplateTracker()
	
	
	//后面程序里都是中的父类的声明，所以改为多尺度目标模板的时候只需要把子类的声明改掉
	mycv::MTTracker::Params mtparams = mycv::MTTracker::Params();//命名空间：：类：：结构体，赋值默认参数
	mtparams.alpha = 0.7;//修改默认参数
	mtparams.numPoints = 800;
	mtparams.sigma = Point2d(0.4, 0.4);
	mtparams.expandWidth = 50;
	Ptr<mycv::Tracker> tracker=new mycv::MultiTemplateTracker(mtparams);//更改为指针类型mycv::Tracker tracker;子类的指针直接赋值给父类的指针，类型提升SingleTemplateTracker()
	

	cvtColor(CurrentFrame, WorkFrame, CV_BGR2GRAY);
	tracker->init(WorkFrame,global::selectedRoi);//更改为指针类型tracker.init();
	cout << "单击鼠标右键开启跟踪。。。" << endl;

	//进入循环，处理视频图像序列，跟踪目标
	for (; frameIndex < FrameCount;) {
		//如果没有信号，则继续读入下一帧图像
		if (!global::paused)
		{
			capture >> CurrentFrame;
			CV_Assert(!CurrentFrame.empty());
			cout << "当前索引帧：" << endl;
			frameIndex++;

			//键读入的图像拷贝到displayImg当中
			CurrentFrame.copyTo(global::displayImg);
			//转换为灰度图像
			cvtColor(CurrentFrame, WorkFrame, CV_BGR2GRAY);

			//开始跟踪
			Rect CurrentBondingBox;
			tracker->track(WorkFrame,CurrentBondingBox);//mycv::Tracker tracker类的指针，父类


			//更新目标模型
			Rect NextSearchBox;
			tracker->update(NextSearchBox);
			
			//显示当前帧跟踪结果图像
			rectangle(global::displayImg, CurrentBondingBox, Scalar(0, 0, 255), 2);
			rectangle(global::displayImg, NextSearchBox, Scalar(255, 0, 0), 2);
			imshow(winName, global::displayImg); 
			waitKey(30);
		}
		else
		{
			//显示当前帧跟踪结果图像
			imshow(winName, global::displayImg);
			waitKey(300);
		}



	}




}

