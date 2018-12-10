#pragma once
#include <string>
#include<vector>
#include<opencv2\opencv.hpp>
using namespace std;
using namespace cv;

namespace mycv {  //创建一个放置tracker的空间,避免与openCV空间内的定义重复
	class Tracker
	{
	public:
		Tracker();
		virtual ~Tracker();
		//初始化跟踪器
		virtual bool init(const Mat&initFrame,const Rect&initBoundingBox);//定义为虚函数，使得调用的时候只能调用子类的函数
		//跟踪目标
		virtual bool track(const Mat&cutrrentFrame,Rect &currentBondingbox);
		//更新目标模型
		virtual bool update(Rect&searchBox);//传回搜索区域
	};

}


