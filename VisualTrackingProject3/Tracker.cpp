#include "Tracker.h"
//父类

namespace mycv {
	//构造函数
	Tracker::Tracker()
	{
		cout << "运行Tracker::Tracker()" << endl;
	}
	//析构函数
	Tracker::~Tracker()
	{
	}
	//初始化跟踪器
	bool Tracker::init(const Mat&initFrame, const Rect&initBoundingBox)
	{
		cout << "运行Tracker::init()" << endl;
		return false;
	}
	//跟踪目标
	bool Tracker::track(const Mat&cutrrentFrame, Rect &currentBondingbox)
	{
		cout << "运行Tracker::track()" << endl;
		return false;
	}
	//跟新目标模型
	bool Tracker::update(Rect&searchBox)
	{
		cout << "运行Tracker::update()" << endl;
		return false;
	}

}
