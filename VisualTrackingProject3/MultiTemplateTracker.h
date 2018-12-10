#pragma once


#include <string>
#include<vector>
#include<opencv2\opencv.hpp>
#include "Tracker.h"
using namespace std;
using namespace cv;

namespace mycv {


	class MultiTemplateTracker :public mycv::Tracker
	{
	public:
		
		virtual ~MultiTemplateTracker();
	public:
		enum MatchMethod { SQDIFF = 0, SADIFF = 1 };
		enum MatchStrategy { UNIFORM = 0, NORMAL = 1 };
		struct Params {
			//搜索范围的扩展
			int expandWidth;
			//匹配方法
			MatchMethod matchMethod;
			//模板匹配策略
			MatchStrategy matchStrategy;
			//模板更新速度
			double alpha;
			//模板匹配时随机采样点的数量
			int numPoints;
			//随机采样点（截断正态分布）的标准差
			Point2d sigma;
			//模板内的采样步长
			Vec2i xyStep;
			//模板在图像内的滑动步长
			Vec2i xyStride;
			Params() {//设置默认值
				expandWidth = 20;
				matchMethod = MatchMethod::SADIFF;
				matchStrategy = MatchStrategy::NORMAL;
				alpha = 0.7;
				numPoints = 500;
				sigma = Point2d(0.5, 0.5);
				xyStep = Vec2i(2, 2);
				xyStride = Vec2i(1, 1);
			}
		};

		//初始化Params
		MultiTemplateTracker(Params _params);
		//public:
		//SingleTemplateTracker(int expandWidth=50);
		
	public:

		//初始化跟踪器
		bool init(const Mat&initFrame, const Rect&initBoundingBox);
		//跟踪目标
		bool track(const Mat&cutrrentFrame, Rect &currentBondingbox);
		//更新目标模型
		bool update(Rect&searchBox);

		float MatchTemplate(const Mat&src, const Mat&templ, Rect2i&match_location,
			MatchMethod match_method, Vec2i& xy_step, Vec2i& xy_stride);

		//估计下一帧搜索范围
		void EstimateSearchArea(const Rect&target_location, Rect& search_area,
			int expand_x, int expand_y);
		// 在指定的采样范围内产生阶段正态分布点
		void GeneraeRandomSamplePoints(vector<Point2d>&sample_points,
			int num_points = 1000, Point2d sigma = Point2d(0.3, 0.3));
		float MatchTemplate(const Mat&src, const Mat&templ, Rect2i&match_location,
			MatchMethod match_method, const vector<Point2d>&sample_points);
		// 产生多尺度目标模板
		void GenerateMultiScaleTargetTempletes(const Mat&origin_target, vector<Mat>&multiscale_targets);
		//显示多尺度目标模板
		void ShowMultiScaleTemplates(const vector<Mat>& multiscale_targets);
		float MatchMultiScaleTemplates(const Mat& src, const vector<Mat>&multiscale_templs, Rect2i&best_match_location,
			MatchMethod match_method, MatchStrategy match_strategy, const vector<Point2d>&sample_points, Vec2i& xy_step, Vec2i& xy_stride);
		void UpdateMultiScaleTargetTemplates(const Mat& currentTargetPatch);
	public:

		//目标模板
		Mat TargetTemplate0;
		Mat TargetTemplate;
		//多尺度目标模板
		vector<Mat>MultiScaleTargetTemplates;
		//当前帧上找到的目标框
		Rect CurrentBoundingBox;
		//当前帧上找到的目标图像块
		Mat CurrentTargetPatch;
		//下一帧的搜索范围
		Rect NextSearchArea;
		//搜索范围的扩展
		//int expandWidth;

		//视频帧的矩形区域
		Rect FrameArea;
		//声明一个标准截断正态分布采样点集
		vector<Point2d>SamplePoints;
		//模板匹配策略
		//int matchStrategy;
		//跟踪器的参数结构体
		Params params;
	};
	typedef MultiTemplateTracker MTTracker;
}