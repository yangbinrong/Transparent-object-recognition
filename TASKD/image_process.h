#pragma once
#define _CRT_SECURE_NO_WARNINGS
#include <opencv2/opencv.hpp>
#include <opencv2/flann/miniflann.hpp>
#include <iostream>
#include <armadillo>
#include <cmath>
#include "aubo_robot_driver/aubo_robot_driver.h"

using namespace cv;
using namespace std;

const Rect rect_on_table_taskA(200, 300, 1300, 650);
const Rect rect_on_hand_taskA(860, 320, 160, 160);

//const Rect rect_on_table_taskB(200, 300, 1300, 650);
const Rect rect_on_table_taskB(510, 470, 650, 510);
const Rect rect_on_hand_taskB_element(860, 320, 160, 160);
const Rect rect_on_hand_taskB_cube(820, 300, 150, 150);

const float the_center_of_img = 780;
const string save_string = "//home//cassemblyc2//0801_taskb//pic//";

class ImageProcess
{
private:

    const Mat tf_on_table_taskA_right = (cv::Mat_<double>(3, 3) << 0.000723880348365276, -8.01003524553892e-05, -0.511109274533355,
            -1.88624607876053e-05, -0.000740427784349975, 0.859995907918861,
            1.43982048506075e-16, -2.27248775352962e-16, 1.00000000000000);

    const Mat tf_on_table_taskA_left = (cv::Mat_<double>(3, 3) << 0.000711776772774457,	0.000115728709599675,	-0.646618062269930,
            -1.88075109759453e-05,	-0.000725105993795582,	0.851264206557018,
            5.87637577487143e-17,	-3.98986399474666e-17,	1.00000000000000);

    const Mat tf_on_table_taskB_right = (cv::Mat_<double>(3, 3) << 0.000723880348365276, -8.01003524553892e-05, -0.511109274533355,
            -1.88624607876053e-05, -0.000740427784349975, 0.859995907918861,
            1.43982048506075e-16, -2.27248775352962e-16, 1.00000000000000);

    const Mat tf_on_table_taskB_left = (cv::Mat_<double>(3, 3) << 0.000711776772774457,	0.000115728709599675,	-0.646618062269930,
            -1.88075109759453e-05,	-0.000725105993795582,	0.851264206557018,
            5.87637577487143e-17,	-3.98986399474666e-17,	1.00000000000000);

private:

    static void Cv_mat_to_arma_mat(const cv::Mat& cv_mat_in, arma::mat& arma_mat_out)
    {//convert unsigned int cv::Mat to arma::Mat<double>
        for (int r = 0; r < cv_mat_in.rows; r++) {
            for (int c = 0; c < cv_mat_in.cols; c++) {
                arma_mat_out(r, c) = cv_mat_in.data[r*cv_mat_in.cols + c];
            }
        }
    };

    template<typename T>
    static void Arma_mat_to_cv_mat(const arma::Mat<T>& arma_mat_in, cv::Mat_<T>& cv_mat_out)
    {
        cv::transpose(cv::Mat_<T>(static_cast<int>(arma_mat_in.n_cols),
            static_cast<int>(arma_mat_in.n_rows),
            const_cast<T*>(arma_mat_in.memptr())),
            cv_mat_out);
    };

public:

    //===========================================变量=====================================//
    std::shared_ptr<DualAuboDriver> dualAuboDriver;
    cv::Mat original_colorimg_, original_depthimg_;
    cobotsys::CameraFrame _captureImages;
    cv::Mat _captureShow_color;
    cv::Mat _captureShow_depth;
    //===================================================================================//

    //===========================================函数=====================================//

    ImageProcess();

    ImageProcess(std::shared_ptr<DualAuboDriver> _dualAuboDriver);

    void ImageCapture();

    //------------------------------------------任务A-------------------------------------//
    // 任务A方向获取程序
    double GetOrientation_taskA(RotatedRect &rRect);

    // 任务A坐标变换程序
    Point2f PixelToWorld_taskA(Point2f uv);

    // 任务A抓取程序
    void task_a_vision_process(Mat &ModelCenAng);

    // 任务A识别程序
    void hole_shaft_judge_taskA(uint &ModelName);

    // 任务A识别大立方体程序
    void get_cube_contriod_taskA(vector<cv::Point> g_vContours_i_, cv::flann::Index kdtree_, cv::Mat source_cell,\
        cv::Mat new_original_, cv::Point2f &point_contriod_low, cv::Point2f &point_orit_low);
    //-----------------------------------------------------------------------------------//

    //------------------------------------------任务B-------------------------------------//
    // 任务B方向获取程序
    double GetOrientation_taskB(RotatedRect &rRect);

    // 任务B坐标变换程序
    Point2f PixelToWorld_taskB(Point2f uv);

    // 任务B抓取主程序
    void task_b_vision_process(Mat &element_controid);

    // 任务B堆叠处理程序
    void heap_Process(cv::Mat srcImg, cv::Point2f bias, cv::Point2f(&heap_rect)[4]);

    // 任务B寻找另一边界程序
    void finding_second_bd2(int max_delta_rot_img_idx_, arma::mat rot_binanry_img_arm_max_, cv::Mat binanry_img_, \
		float &x11_, float &x12_, float &y11_, float &y12_, int &delta_of_size_);

    // 任务B反算旋转前的位置坐标程序
    void get_point_img(float center_img_x_, float center_img_y_, float origi_x_, float origi_y_, int rot_angel_, \
		float &rot_x_, float &rot_y_);

    // 任务B识别程序
    // 识别零件种类
    void element_judge_taskB(uint &ModelName);

    // 任务B识别程序
    // 判断立方体表面有没有孔
    void hole_judge_taskB(uint &HoleName);

    // 任务B识别程序
    // 识别立方体种类，判断方块表面有没有孔，识别方块表面孔的形状
    void cube_judge_taskB(uint &CubeName);
    //-----------------------------------------------------------------------------------//

    //===================================================================================//
};

