
#include "image_process.h"
#include "cobotsys_abstract_dual_arm_robot_link.h"

#define  PI 3.1415926534

//---------------------------------------任务AB公用程序-----------------------------------------//

ImageProcess::ImageProcess() {
    //judge_callback_count = 0;
}

ImageProcess::ImageProcess(std::shared_ptr<DualAuboDriver> _dualAuboDriver){
    dualAuboDriver = _dualAuboDriver;
}

void ImageProcess::ImageCapture() {
    sleep(1);
    cobotsys::ErrorInfo errorInfo;
    int exposureTime=500;

    std::cout<<"here7"<<std::endl;
    _captureImages.frames.clear();
    std::cout<<"here8"<<std::endl;
    _captureImages = dualAuboDriver->dual_arm_driver->CaptureImage(errorInfo, HeadCamera, exposureTime);
//        cv::Mat color = _captureImages.at(0).image.clone;//分别对应深度图和彩色图
//        cv::Mat depth = _captureImages.at(1).image.clone;
    std::cout<<"here9"<<std::endl;
    _captureShow_color = _captureImages.frames.at(0).data.clone();
    _captureShow_depth = _captureImages.frames.at(1).data.clone();
    std::cout<<"here10"<<std::endl;
    if (_captureImages.frames.size() != 0) {
        std::cout<<"成功捕获图片"<<std::endl;
    }
    else
    {
        std::cout<<"捕获图片失败"<<std::endl;
    }

//    //cv::imshow("捕获的图片",_captureShow);
//    cv::imwrite("//home//geds//ybr//color.png",_captureShow_color);
//    cv::imwrite("//home//geds//ybr///depth.png",_captureShow_depth);
//    //std::cout<<_captureShow.channels()<<std::endl;
//
//    original_colorimg_ = cv::imread("//home//geds//ybr//color.png");
//    original_depthimg_ = cv::imread("//home//geds//ybr//depth.png");
//

    //cv::imshow("捕获的图片",_captureShow);
    cv::imwrite("//home//ybr//photo//data5//color.png",_captureShow_color);
    cv::imwrite("//home//ybr//photo//data5//depth.png",_captureShow_depth);
    //std::cout<<_captureShow.channels()<<std::endl;

    original_colorimg_ = cv::imread("//home//ybr//photo//data5//color.png");
    original_depthimg_ = cv::imread("//home//ybr//photo//data5//depth.png");



}

//----------------------------------------任务A程序---------------------------------------------//

/*
任务A的视觉程序0； 像素点世界坐标系下的坐标
输入：Pix_coordinate: 像素坐标
返回：cv::Point2f: 像素点世界坐标系下的坐标；
*/
cv::Point2f ImageProcess::PixelToWorld_taskA(cv::Point2f Pix_coordinate)
{
    cv::Mat tp, point_coodinate, tt;

    //对应的映射矩阵
    if(Pix_coordinate.x < the_center_of_img)
    {// 在图像的左边, 使用左臂的标定矩阵
        tp = tf_on_table_taskA_left;
        cout<< Pix_coordinate.x << "\n" << "left" << "\n" << endl;
    }
    else{ // 右边, 使用右臂的标定矩阵
        tp = tf_on_table_taskA_right;
        cout<< Pix_coordinate.x << "\n" << "right" << "\n" << endl;
    }

    point_coodinate = (cv::Mat_<double>(3, 1) << Pix_coordinate.x, Pix_coordinate.y, 1);
    tt = tp * point_coodinate;
    cv::Point2f World_coordnate;
    World_coordnate.x = tt.at<double>(0, 0);
    World_coordnate.y = tt.at<double>(1, 0);
    return World_coordnate;
}

/*
任务A的视觉程序0； 获取方向程序
输入：rRect:  矩形框
返回：double: 角度值；
*/
double ImageProcess::GetOrientation_taskA(cv::RotatedRect &rRect)
{
    cv::Point2f Vertices[4];
    rRect.points(Vertices);
    double orientation = rRect.angle;
    double orientation_rads = orientation * PI / 180;

    double arr[3];
    for (int n = 0; n < 3; n++)
    {
        double a = abs(Vertices[0].x - Vertices[n + 1].x) + abs(Vertices[0].y - Vertices[n + 1].y);
        arr[n] = a;
    }

    int min = 0;
    for (int i = 1; i < 3; i++)
    {
        if (arr[i] < arr[min])
        {
            min = i;
        }
    }
    min += 1;
    double x = abs(Vertices[min].x - Vertices[0].x);
    double y = abs(Vertices[min].y - Vertices[0].y);
    double z = sqrt(x * x + y * y);
    double Angle_rad;
    if ((Vertices[0].x - Vertices[min].x) < 0)
        Angle_rad = acos(y / z);
    else
        Angle_rad = asin(y / z) + PI / 2;
    double Angle = Angle_rad * 180 / PI;

    return Angle_rad;
}

/*
任务A的视觉程序1； 获取桌面物体的抓取位置姿态
输入：srcImg: 原始彩色图片
输出：ModelCenAng: 是一个8X3的矩阵，在机器人坐标系下的模型中心位置与方向；
*/
void ImageProcess::task_a_vision_process(Mat &element_controid)
{
    Mat srcImg_, depthImg_;
    ImageCapture();
    srcImg_ = original_colorimg_;
    depthImg_ = original_depthimg_;
//    //---------------------------------------深度图处理--------------------------------------------//
//    // 建立二叉树
////     cv::Mat source_cell_;
////     cv::flann::Index kdtree = get_the_kdtree_depth_taskA(depthImg_, source_cell_);
//    cv::Mat original_depthimg, depthimg_cut, depth_gray_img, depth_binary_img;
//    // 图片读取
//    original_depthimg = depthImg_.clone(); // cv::IMREAD_UNCHANGED
//                                          // 图片裁剪
//    depthimg_cut = original_depthimg(rect_on_table_taskA);
//    cv::cvtColor(depthimg_cut, depth_gray_img, CV_BGR2GRAY);
//    // 开操作
//    cv::Mat rectKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7), cv::Point(-1, -1));
//    cv::Mat opened_depth;
//    cv::morphologyEx(depth_gray_img, opened_depth, cv::MORPH_OPEN, rectKernel);
//
////     depth_gray_img = cv::imread("depthimg_cut1.png", IMREAD_UNCHANGED);
////     cv::Mat xgrad, ygrad, canny2;
////     cv::Sobel(depthimg_cut, xgrad, CV_16SC1, 1, 0);
////     cv::Sobel(depthimg_cut, ygrad, CV_16SC1, 0, 1);
////
////     cv::Canny(xgrad, ygrad, canny2, 170, 200);
////     cv::imwrite("drawing1_d.png", canny2);
//    // 深度图边缘提取
//    //     cout << depth_gray_img.channels() << endl;
//    //     cv::Mat canny1;
//    //     cv::Canny(depth_gray_img, canny1, 0.0063, 0.0156, 3);
//    //     cv::imshow("drawing1_d", canny1); cv::waitKey(1);
//    //     cv::imwrite("drawing1_d.png", canny1);
//
//    vector<vector<cv::Point> > g_vContours_d;
//    vector<cv::Vec4i> g_vHierarchy_d;
//    cv::findContours(opened_depth, g_vContours_d, g_vHierarchy_d, cv::RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));//RETR_EXTERNAL RETR_LIST RETR_CCOMP RETR_TREE
//                                                                                                                   // 深度图边缘绘制                                                                                                                       // 深度图边缘绘制                                                                                                                               // 绘制图片
//    cv::Mat drawing1_d = cv::Mat::zeros(opened_depth.size(), CV_8U);
//    drawContours(drawing1_d, g_vContours_d, -1, cv::Scalar(255, 255, 255));
////     cv::imshow("drawing1_d", drawing1_d); cv::waitKey(1);
//     cv::imwrite("drawing1_d.png", drawing1_d);
//    // 深度图二值化
//    cv::threshold(drawing1_d, depth_binary_img, 0, 1, cv::THRESH_BINARY);
//    // 矩阵转换
//    arma::mat depth_binary_img_arma(depth_binary_img.rows, depth_binary_img.cols);;
//    Cv_mat_to_arma_mat(depth_binary_img, depth_binary_img_arma);
//    int sum_of_depth = arma::sum(arma::sum(depth_binary_img_arma));
//
//    // 建立深度图二叉树
//    vector<cv::Point2f> source;
//    for (int y = 0; y < depth_binary_img.rows; y++)
//    {
//        for (int x = 0; x < depth_binary_img.cols; x++)
//        {
//            if (depth_binary_img.at<uchar>(y, x) == 1)
//            {
//                cv::Point2i p1(x, y);
//                source.push_back(p1); // 列，行
//            }
//        }
//    }
//    cv::Mat source_cell = cv::Mat(source).reshape(1);
//    cv::flann::KDTreeIndexParams indexParams(2);
//    cv::flann::Index kdtree(source_cell, indexParams);
    
    //-------------------------------------------彩色图处理--------------------------------------------//
    cv::Mat original_colorimg, img_cut, shifted_img, gray_img, binary_img;
    // 图片读取
    original_colorimg = srcImg_.clone();

    // 图片裁剪
    int img_cut_x = rect_on_table_taskA.x;
    int img_cut_y = rect_on_table_taskA.y;    
    img_cut = original_colorimg(rect_on_table_taskA);
    cv::Mat new_original;
    original_colorimg.copyTo(new_original);

    // 彩色滤波
    cv::pyrMeanShiftFiltering(img_cut, shifted_img, 21, 51);

    // 彩色图转灰度图
    cv::cvtColor(shifted_img, gray_img, CV_BGR2GRAY);

    // 二值化
    threshold(gray_img, binary_img, 95, 255, cv::THRESH_BINARY);

    // 形态学操作
    cv::Mat k = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
    cv::morphologyEx(binary_img, binary_img, cv::MORPH_ERODE, k);//可以实现多种形态学操作类型
//    cv::imshow("腐蚀后", binary_img);

    // 定义轮廓存储
    vector<vector<cv::Point> > g_vContours;
    // 轮廓索引层次
    vector<cv::Vec4i> g_vHierarchy;
    // 寻找轮廓
    findContours(binary_img, g_vContours, g_vHierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));//RETR_EXTERNAL RETR_LIST RETR_CCOMP RETR_TREE

    // 图形矩
    vector<cv::Moments> mu(g_vContours.size());
    // 形心位置  定义转化后的形心位置
    vector<cv::Point2f> mc(g_vContours.size());
    vector<cv::Point2f> global_mc2(g_vContours.size());
    vector<cv::Point2f> global_mc1(g_vContours.size());
    // 对每一个轮廓，计算对应的图形矩以及相应的形心位置
    for (unsigned int j = 0; j < g_vContours.size(); j++)
    {
        mu[j] = moments(g_vContours[j], false);
        mc[j] = cv::Point2f(static_cast<float>(mu[j].m10 / mu[j].m00), static_cast<float>(mu[j].m01 / mu[j].m00));
    }

    // 绘制图片
    cv::Mat drawing1;
    drawing1 = cv::Mat::zeros(original_colorimg.size(), CV_8UC3);

    // 存储各个轮廓对应的面积
    vector<double> area_vec;
    vector<int> g_vContours_vec;
    // 对每一个轮廓进行单独处理
    for (unsigned int i = 0; i < g_vContours.size(); i++)
    {
        // 轮廓绘制
        drawContours(drawing1, g_vContours, i, cv::Scalar(255, 255, 255), 1, 8, g_vHierarchy, 0, cv::Point());

        // 计算面积
        double AreaContour = contourArea(g_vContours[i], true);

        // 面积判断，画出形心位置以及方向并显示
        if (abs(AreaContour) > 1000)
        {
            // 计算每个轮廓的最小包络矩形
            cv::RotatedRect min_rect = minAreaRect(g_vContours.at(i));
            //对计算得到的包络矩形进行平移
            cv::Point2f global_rect_center;
            global_rect_center = min_rect.center + cv::Point2f(img_cut_x, img_cut_y);
            //cout << min_rect.center << endl;
            // 绘制最小包络矩形的中心
            circle(new_original, global_rect_center, 4, cv::Scalar(255, 0, 0), -1, 8, 0);
            //对轮廓中心进行平移
            cv::Point2f contour_center;
            contour_center = mc[i] + cv::Point2f(img_cut_x, img_cut_y);
            global_mc1[i] = mc[i] + cv::Point2f(img_cut_x, img_cut_y);

            // 绘制轮廓的中心
            if(global_mc1[i].x < the_center_of_img)
            {  // 在图像的左边为蓝色
                circle(new_original, global_mc1[i], 4, cv::Scalar(255, 0, 0), -1, 8, 0);
                cout<< global_mc1[i].x << "\n" << "left" << "\n" << endl;
            }
            else
            {  // 在图像的右边为红色
                circle(new_original, global_mc1[i], 4, cv::Scalar(0, 0, 255), -1, 8, 0);
                cout<< global_mc1[i].x << "\n" << "right" << "\n" << endl;
            }

            // 计算出方向
            double orientation;
            orientation = GetOrientation_taskA(min_rect);
            global_mc2[i].x = global_mc1[i].x + 30 * cos(orientation);
            global_mc2[i].y = global_mc1[i].y + 30 * sin(orientation);
            // 绘制轮廓
            cv::Point2f vertices[4];
            min_rect.points(vertices);
            // 完成对包络矩形的平移
            cv::Point2f new_vertices[4];
            for (int i = 0; i < 4; i++)
                new_vertices[i] = vertices[i] + cv::Point2f(img_cut_x, img_cut_y);
            for (int k = 0; k < 4; k++)
                line(new_original, new_vertices[k], new_vertices[(k + 1) % 4], cv::Scalar(0, 255, 0), 2);
            // 打印输出
            //char tam0[100];
            //sprintf(tam0, "Centroid:(%.1f,%.1f)", contour_center.x, contour_center.y);
            //cv::putText(new_original, tam0, cv::Point2f(contour_center.x, contour_center.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(20, 20, 250), 0.8);
            //cv::Point2f word_coordinate;
            //word_coordinate = PixToWorld(contour_center);
            //char tam1[100];
            //sprintf(tam1, "Worldcoordinate:(%.1f,%.1f)", word_coordinate.x, word_coordinate.y);
            //cv::putText(new_original, tam1, cv::Point2f(contour_center.x, contour_center.y - 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(20, 20, 250), 0.8);

            // 面积存储
            area_vec.push_back(abs(AreaContour));
            g_vContours_vec.push_back(i);
        }
    }
    // 零件位置与方向
    element_controid = cv::Mat::zeros(8, 3, CV_32FC1);
    cout << element_controid << endl;
    int m = 1;
    for (int i = 0; i < int(area_vec.size()); i++)
    {
        float area_vec_i = area_vec[i];
        int area_idx = g_vContours_vec[i];
        cv::RotatedRect min_rect = minAreaRect(g_vContours.at(area_idx));
        cv::Point2f vertices[4];
        min_rect.points(vertices);
        cv::Point2f new_vertices[4];
        for (int i = 0; i < 4; i++)
            new_vertices[i] = vertices[i] + cv::Point2f(img_cut_x, img_cut_y);
        char tam0[100]; char tam1[100]; char tam2[100];
        if (area_vec_i > 19000)
        {
            sprintf(tam0, "Cube");
            cv::putText(new_original, tam0, cv::Point2f(new_vertices[1].x, new_vertices[1].y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(20, 20, 250), 0.8);
            cout << "任务A立方体不抓取" << "\n" << endl;
//            vector<cv::Point> cube_Contours = g_vContours[area_idx];
//            // 找到立方体底面形心
//            cv::Point2f point_contriod_low_, point_orit_low_;
//            get_cube_contriod_taskA(cube_Contours, kdtree, source_cell, img_cut, point_contriod_low_, point_orit_low_);
//            // 转换到未裁剪图片中
//            cv::Point2f global_point_contriod_low = point_contriod_low_ + cv::Point2f(img_cut_x, img_cut_y);
//            circle(new_original, global_point_contriod_low, 4, cv::Scalar(0, 255, 0), -1, 8, 0);
//
//            cv::Point2f global_point_orit_low = point_orit_low_ + cv::Point2f(img_cut_x, img_cut_y);
//            line(new_original, global_point_contriod_low, global_point_orit_low, cvScalar(0, 0, 200), 2);
//            // 质心坐标转换到机器人坐标系
//            cv::Point2f cube_cont_world_coordinate = PixelToWorld_taskA(global_point_contriod_low);
//            // 方向转换到机器人坐标系
//            cv::Point2f cube_orit_world_coordinate = PixelToWorld_taskA(global_point_orit_low);
//            // 角度计算
//            float angle_grasp_cube = atan((global_point_orit_low.y - global_point_contriod_low.y) / (global_point_orit_low.x - global_point_contriod_low.x));
//            if (angle_grasp_cube > 0)
//            {
//                angle_grasp_cube = PI - angle_grasp_cube;
//            }
//            else if (angle_grasp_cube < 0)
//            {
//                angle_grasp_cube = -angle_grasp_cube;
//            }
//            // 输出
//            cout << "立方体形心坐标：" << "x: " << cube_cont_world_coordinate.x << " y: " \
//                << cube_cont_world_coordinate.y << "\n" << endl;
//            // 角度
//            cout << "立方体抓取方向与x轴的夹角：" << angle_grasp_cube << "\n" << endl;
//            // 存储
//            element_controid.at<float>(0, 0) = cube_cont_world_coordinate.x;
//            element_controid.at<float>(0, 1) = cube_cont_world_coordinate.y;
//            element_controid.at<float>(0, 2) = angle_grasp_cube - PI / 2;
        }
        else if ((area_vec_i > 9000) && (area_vec_i < 19000))
        {
            sprintf(tam0, "Sphere");
            cv::putText(new_original, tam0, cv::Point2f(new_vertices[1].x, new_vertices[1].y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(20, 20, 250), 0.8);
            vector<cv::Point> ball_Contours = g_vContours[area_idx];
            
            cv::Point2f ball_pos = cv::Point2f(ball_Contours[1].x, ball_Contours[1].y) + cv::Point2f(img_cut_x, img_cut_y);

            cv::Point2f Ball_world_coordinate = PixelToWorld_taskA(global_mc1[area_idx]);
            sprintf(tam2, "World coordinate:(%.1f,%.1f)", Ball_world_coordinate.x, Ball_world_coordinate.y);
            cv::putText(new_original, tam2, cv::Point2f(ball_pos.x - 100, ball_pos.y + 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cvScalar(20, 20, 250), 0.8);
            cout << "球的位置：" << "x: " << Ball_world_coordinate.x << " y: " \
                << Ball_world_coordinate.y << "\n" << endl;
            element_controid.at<float>(1, 0) = Ball_world_coordinate.x;
            element_controid.at<float>(1, 1) = Ball_world_coordinate.y;
        }
        else
        {
            line(new_original, global_mc1[area_idx], global_mc2[area_idx], cvScalar(0, 0, 200), 2);
            line(new_original, global_mc1[area_idx], global_mc2[area_idx], cvScalar(0, 0, 200), 2);
            cv::Point2f word_coordinate;
            word_coordinate = PixelToWorld_taskA(global_mc1[area_idx]);
            cout << "零件" << area_idx << "位置：" << "x: " << word_coordinate.x << " y: " \
                << word_coordinate.y << "\n" << endl;
            cv::Point2f orit_point;
            orit_point = PixelToWorld_taskA(global_mc2[area_idx]);
            float angle_grasp = atan((global_mc2[area_idx].y - global_mc1[area_idx].y) / (global_mc2[area_idx].x - global_mc1[area_idx].x));
            cout << "零件" << area_idx << "与x轴的夹角：" << angle_grasp << "\n" << endl;
            if (angle_grasp > 0)
            {
                angle_grasp = PI - angle_grasp;
            }
            else if (angle_grasp < 0)
            {
                angle_grasp = -angle_grasp;
            }
            element_controid.at<float>(1 + m, 0) = word_coordinate.x;
            element_controid.at<float>(1 + m, 1) = word_coordinate.y;
            element_controid.at<float>(1 + m, 2) = angle_grasp - PI / 2;
            m++;
        }
    }

    cout << element_controid << endl;
    cv::imwrite(save_string+"recognition_results_taskA.png", new_original);
//    cv::waitKey(1);

}

/*
任务A的视觉程序2； 获取手上物体的模型名称
输入：srcImg: 原始彩色图片
输出：ModelName: 返回无符号整型 0-不存在孔 1-圆形 2-三角形 3-矩形
*/
void ImageProcess::hole_shaft_judge_taskA(uint &ModelName_)
{
    cv::Mat srcImg;
    ImageCapture();
    srcImg = original_colorimg_;

    cv::Mat original_colorimg, img_cut, shifted_img, gray_img, binary_img;

    // 图片裁剪
    srcImg(rect_on_hand_taskA).copyTo(img_cut);

    cv::imwrite(save_string+"test_roi_hand_taskA.png",img_cut);

    // 彩色滤波
    cv::pyrMeanShiftFiltering(img_cut, shifted_img, 21, 51);

    // 彩色图转灰度图
    cv::cvtColor(shifted_img, gray_img, CV_BGR2GRAY);

    // 二值化
    threshold(gray_img, binary_img, 125, 255, cv::THRESH_BINARY);

    // 形态学操作
    cv::Mat k = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
    cv::morphologyEx(binary_img, binary_img, cv::MORPH_ERODE, k);//可以实现多种形态学操作类型
    cv::imwrite(save_string+"test_roi_binary_hand_taskA.png",binary_img);
    // 定义轮廓存储
    vector<vector<cv::Point> > g_vContours;
    // 轮廓索引层次
    vector<cv::Vec4i> g_vHierarchy;
    // 寻找轮廓
    findContours(binary_img, g_vContours, g_vHierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));//RETR_EXTERNAL RETR_LIST RETR_CCOMP RETR_TREE  RETR_EXTERNAL只寻找最外部轮廓不行
    double maxarea = 1000;
    int maxAreaIdx = -1;
    for (int index = g_vContours.size() - 1; index >= 0; index--)
    {
        // 若立方体的表面没有孔，则所截取的区域将完全是白色，面积必然会大于10000
        // 若立方体表面有孔，或者夹取的是零件，则面积会小于10000而大于1000
        // 如果不进入if，则说明没有零件或者立方体表面没有孔，可能存在问题
        double tmparea = fabs(contourArea(g_vContours[index]));
        if ((tmparea > maxarea) && (tmparea < 10000))
        {
            maxarea = tmparea;
            maxAreaIdx = index;
        }
    }

    if (maxAreaIdx == -1)
    {
        ModelName_ = 3;//对应的就是不存在物体
    cout << "任务A：不存在零件" << endl;
    }

    else
    {
        cv::Moments mu;
        cv::Point2f mc, mc1, mc2;
        mu = cv::moments(g_vContours[maxAreaIdx], false);

        mc = cv::Point2f(static_cast<float>(mu.m10 / mu.m00) + rect_on_hand_taskA.x, static_cast<float>(mu.m01 / mu.m00) + rect_on_hand_taskA.y);
        mc1 = cv::Point2f(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
        cv::RotatedRect rect = minAreaRect(g_vContours.at(maxAreaIdx));
        cv::Point2f vertices[4];
        rect.points(vertices);
        double orientation = rect.angle;
        double orientation_rads = orientation*3.1416 / 180;
        double area = mu.m00;//contourArea(g_vContours[i]),
        double perimeter = arcLength(g_vContours.at(maxAreaIdx), true);
        double roundness = (4 * 3.1416*area) / (pow(perimeter, 2));
        double rectarea = (rect.size.height)*(rect.size.width);
        double squareness = area / rectarea;
        cout << "roundness:" << roundness << endl;
        cout << "squareness:" << squareness << endl;
        char type_hole[100];
        if (squareness > 0.90)
        {
            ModelName_ = 1;//"square";
//            strcpy_s(type_hole, "square");
            cout << "任务A：方形" << endl;
        }
        else {

            if (roundness < 0.66)
            {
                ModelName_ = 0;//"triangle";
                              //strcpy_s(type_hole, "triangle");
                              cout << "任务A：三角形" << endl;
            }
            else if (roundness > 0.66 &&roundness < 0.84)
            {

                ModelName_ = 1;//"square";
                              //strcpy_s(type_hole, "square");
                              cout << "任务A：方形" << endl;

            }
            else if (roundness > 0.84)
            {
                ModelName_ = 2;//"circle";
                              //strcpy_s(type_hole, "circle");
                              cout << "任务A：圆形" << endl;
            }
        }
    }
//    cv::imwrite("//home//geds//tmp_test//test_out.png",img_cut);
}

/*
任务A的视觉程序3； 获取立方体底面的形心位置
输入：srcImg: 原始彩色图片
输出：ModelName: 返回无符号整型 0-不存在孔 1-圆形 2-三角形 3-矩形
*/
void ImageProcess::get_cube_contriod_taskA(vector<cv::Point> g_vContours_i, cv::flann::Index kdtree, cv::Mat source_cell,\
    cv::Mat img_cut, cv::Point2f &point_contriod_low, cv::Point2f &point_orit_low)
{
    // 返回底面形心位置和方向 g_vContours_i drawing1
    int k = 1;
    vector<int> indices(k);//找到点的索引
    vector<float> dists(k);
    cv::flann::SearchParams params(128);

    // 找到最高点与最低点
    arma::mat px_color(g_vContours_i.size(), 1), py_color(g_vContours_i.size(), 1);

    arma::mat x_vec_depth(999, 1);
    arma::mat y_vec_depth(999, 1);
    int ii = 0;
    for (int i = 0; i < g_vContours_i.size(); i++)
    {
        px_color(i, 0) = g_vContours_i[i].x;
        py_color(i, 0) = g_vContours_i[i].y;

        vector<float> vecQuery_color(2);
        vecQuery_color[0] = g_vContours_i[i].x;// 列
        vecQuery_color[1] = g_vContours_i[i].y;// 行
        kdtree.knnSearch(vecQuery_color, indices, dists, k, params);

        if (dists[0] < 20)
        {
            int xx_depth = source_cell.at<float>(indices[0], 0);// 列
            int yy_depth = source_cell.at<float>(indices[0], 1);// 行

            x_vec_depth(ii, 0) = xx_depth;
            y_vec_depth(ii, 0) = yy_depth;
            ii++;
        }

    }

    // 找到彩色图的上下顶点
    arma::uvec max_py_color = arma::find(py_color.col(0) == arma::max(py_color.col(0)));
    arma::uvec min_py_color = arma::find(py_color.col(0) == arma::min(py_color.col(0)));
    cv::Point2f maxy_p_color((px_color(max_py_color[0], 0)), py_color(max_py_color[0], 0));
    cv::Point2f miny_p_color((px_color(min_py_color[0], 0)), py_color(min_py_color[0], 0));
    circle(img_cut, maxy_p_color, 4, cv::Scalar(255, 0, 0), -1, 8, 0);
    circle(img_cut, miny_p_color, 4, cv::Scalar(255, 0, 0), -1, 8, 0);
    // 找到深度图的上下限
    int depth_maxy = maxy_p_color.y - 50;
    int depth_miny = miny_p_color.y - 10;

    // 深度图处理
    arma::mat x_cell_depth(ii, 1), y_cell_depth(ii, 1);
    x_cell_depth = x_vec_depth.rows(0, ii - 1);
    y_cell_depth = y_vec_depth.rows(0, ii - 1);
    // 优化深度图的取值
    arma::uvec y_idx_depth = arma::find((y_cell_depth > depth_miny) && (y_cell_depth < depth_maxy));

    arma::mat x_cell_depth_refine(y_idx_depth.size(), 1), y_cell_depth_refine(y_idx_depth.size(), 1);
    x_cell_depth_refine = x_cell_depth.rows(y_idx_depth);
    y_cell_depth_refine = y_cell_depth.rows(y_idx_depth);

    // 找到深度图的左右界
    arma::uvec x_max_depth = arma::find(x_cell_depth_refine.col(0) == arma::max(x_cell_depth_refine.col(0)));
    arma::uvec x_min_depth = arma::find(x_cell_depth_refine.col(0) == arma::min(x_cell_depth_refine.col(0)));                                       

    arma::mat point_maxx_y_depth_ = y_cell_depth_refine.rows(x_max_depth);
    int point_maxx_y_depth = arma::min(point_maxx_y_depth_.col(0));
    int point_maxx_x_depth = arma::max(x_cell_depth_refine.col(0));
    cv::Point2f point_maxx_depth(point_maxx_x_depth, point_maxx_y_depth);

    arma::mat point_minx_y_depth_ = y_cell_depth_refine.rows(x_min_depth);
    int point_minx_y_depth = arma::min(point_minx_y_depth_.col(0));
    int point_minx_x_depth = arma::min(x_cell_depth_refine.col(0));
    cv::Point2f point_minx_depth(point_minx_x_depth, point_minx_y_depth);
    circle(img_cut, point_maxx_depth, 4, cv::Scalar(0, 0, 255), -1, 8, 0);
    circle(img_cut, point_minx_depth, 4, cv::Scalar(0, 0, 255), -1, 8, 0);

    // 绘制顶面形心位置
    cv::Point2f point_contriod_upp;
    point_contriod_upp.x = (point_maxx_depth.x + point_minx_depth.x) / 2.0;
    point_contriod_upp.y = (point_maxx_depth.y + point_minx_depth.y) / 2.0;

    circle(img_cut, point_contriod_upp, 4, cv::Scalar(0, 255, 255), -1, 8, 0);
    // 顶面方向
    cv::Point2f point_orit_upp;
    point_orit_upp.x  = (point_maxx_depth.x + miny_p_color.x) / 2.0;
    point_orit_upp.y  = (point_maxx_depth.y + miny_p_color.y) / 2.0;

    // 绘制底面形心位置
    point_contriod_low = miny_p_color - point_contriod_upp + maxy_p_color;
    circle(img_cut, point_contriod_low, 4, cv::Scalar(0, 255, 0), -1, 8, 0);
    // 底面方向
    point_orit_low = point_orit_upp - point_contriod_upp + point_contriod_low;
    line(img_cut, point_contriod_low, point_orit_low, cvScalar(0, 0, 200), 2);

//    cv::imshow("img_cut", img_cut);
}

//--------------------------------------------------------------------------------------------//

//----------------------------------------任务B程序---------------------------------------------//
/*
任务B的视觉程序0； 像素点世界坐标系下的坐标
输入：Pix_coordinate: 像素坐标
返回：cv::Point2f: 像素点世界坐标系下的坐标；
*/
cv::Point2f ImageProcess::PixelToWorld_taskB(cv::Point2f Pix_coordinate)
{
    cv::Mat tp, point_coodinate, tt;

    //对应的映射矩阵
    if(Pix_coordinate.x < the_center_of_img)
    {// 在图像的左边, 使用左臂的标定矩阵
        tp = tf_on_table_taskB_left;
        cout<< Pix_coordinate.x << "\n" << "left" << "\n" << endl;
    }
    else{ // 右边, 使用右臂的标定矩阵
        tp = tf_on_table_taskB_right;
        cout<< Pix_coordinate.x << "\n" << "right" << "\n" << endl;
    }

    point_coodinate = (cv::Mat_<double>(3, 1) << Pix_coordinate.x, Pix_coordinate.y, 1);
    tt = tp * point_coodinate;
    cv::Point2f World_coordnate;
    World_coordnate.x = tt.at<double>(0, 0);
    World_coordnate.y = tt.at<double>(1, 0);
    return World_coordnate;
}



/*
任务B的堆叠处理程序；
输入： srcImg: 原彩色图像
      bias 裁剪偏移量
输出：heap_rect: 输出的是下方物体的矩形位置点；
*/
void ImageProcess::heap_Process(cv::Mat srcImg, cv::Point2f bias, cv::Point2f(&heap_rect)[4])
{
    // 原图片，彩色裁剪，灰度图，二值化图
    cv::Mat original_colorimg, colorimg_cut, imgGray, binanry_img;

    colorimg_cut = srcImg;
    /*    imshow("彩色图", colorimg_cut);*/
    // 彩色图转灰度图
    cvtColor(colorimg_cut, imgGray, CV_RGB2GRAY);
    //imshow("灰度图", imgGray);
    // 灰度图二值化
    threshold(imgGray, binanry_img, 101, 1, cv::THRESH_BINARY);
    //threshold(imgGray, binanry_img, 1, 1, CV_THRESH_BINARY| CV_THRESH_OTSU);
    //int plus_a = binanry_img.rows;
    //int plus_b = binanry_img.cols;
    //imshow("二值化", binanry_img);
    cv::copyMakeBorder(binanry_img, binanry_img, 50, 50, 50, 50, cv::BORDER_CONSTANT, 0);
    //imshow("填充边界", binanry_img);
    // 定义轮廓存储
    vector<vector<cv::Point> > g_vContours;
    // 轮廓索引层次
    vector<cv::Vec4i> g_vHierarchy;
    // 寻找轮廓
    findContours(binanry_img, g_vContours, g_vHierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));//RETR_EXTERNAL RETR_LIST RETR_CCOMP RETR_TREE
    for (unsigned int i = 0; i < g_vContours.size(); i++)
    {
        // 计算面积
        double AreaContour = contourArea(g_vContours[i], true);

        // 面积判断
        if (abs(AreaContour) < 2000)
            drawContours(binanry_img, g_vContours, i, cv::Scalar(0), CV_FILLED);

    }
    //imshow("消除小面积", binanry_img);

    // 旋转图片，注意是逆时针旋转
    // 图片中心
    cv::Point2f center_img(binanry_img.cols / 2, binanry_img.rows / 2);
    // 存储每次旋转的最大突变值
    arma::vec max_delta_num_cell(180);
    // 存储每次旋转的最大突变值对应的横坐标
    arma::vec max_delta_idx_cell(180);
    // 旋转图片，存储每次旋转的最大值
    for (int rot_angle = 0; rot_angle < 180; rot_angle++)
    {
        // 定义旋转后的图片
        cv::Mat rot_binanry_img;
        // 设置旋转变换的仿射矩阵
        cv::Mat rot_mat = getRotationMatrix2D(center_img, rot_angle, 1);
        // 图片旋转
        warpAffine(binanry_img, rot_binanry_img, rot_mat, cv::Size(binanry_img.cols, binanry_img.rows));
        arma::mat rot_binanry_img_arm(rot_binanry_img.rows, rot_binanry_img.cols);
        Cv_mat_to_arma_mat(rot_binanry_img, rot_binanry_img_arm);

        arma::mat cols_sum;
        // 矩阵中每一列的点数之和
        cols_sum = arma::sum(rot_binanry_img_arm);
        // 错位相减做差（求取一阶导数）
        // 建立行向量
        arma::rowvec cols_sum1(cols_sum.size() + 1);
        cols_sum1.col(0) = 0;
        cols_sum1.cols(1, cols_sum.size()) = cols_sum;
        arma::rowvec cols_sum2(cols_sum.size() + 1);
        cols_sum2.col(cols_sum.size()) = 0;
        cols_sum2.cols(0, cols_sum.size() - 1) = cols_sum;
        arma::rowvec delta_cols_sum(cols_sum.size() + 1);
        delta_cols_sum = abs(cols_sum1 - cols_sum2);
        int max_delta_cols_roti = arma::max(delta_cols_sum);
        arma::uvec max_delta_idx_img_roti1 = arma::find(delta_cols_sum == max_delta_cols_roti);
        int max_delta_idx_img_roti = max_delta_idx_img_roti1(0);
        max_delta_num_cell(rot_angle) = max_delta_cols_roti;//分别对应数值和量
        max_delta_idx_cell(rot_angle) = max_delta_idx_img_roti;

    }
    //-------------------------------------- 寻找最大突变数 ---------------------------------------//
    int max_delta_num = arma::max(max_delta_num_cell);
    //std::cout << max_delta_num << endl;
    // 最大突变数对应的旋转角度
    arma::uvec max_delta_rot_idx_vec = arma::find(max_delta_num_cell == max_delta_num);
    int max_delta_rot_angel = max_delta_rot_idx_vec(0);
    //std::cout << max_delta_rot_angel << endl;
    // 最大突变数对应的在当次旋转中的X索引值
    int max_delta_rot_img_idx = max_delta_idx_cell(max_delta_rot_angel);
    //std::cout << max_delta_rot_img_idx << endl;
    // 给出该旋转角度下的二值化图片
    // 定义旋转后的图片
    cv::Mat rot_binanry_img_max;
    // 设置旋转变换的仿射矩阵
    cv::Mat rot_mat_max = getRotationMatrix2D(center_img, max_delta_rot_angel, 1);
    // 图片旋转
    cv::Mat binanry_img1 = binanry_img.clone();
    warpAffine(binanry_img1, rot_binanry_img_max, rot_mat_max, cv::Size(binanry_img1.cols, binanry_img1.rows));
    arma::mat rot_binanry_img_arm_max(rot_binanry_img_max.rows, rot_binanry_img_max.cols);
    //imshow("旋转后的图像", rot_binanry_img_max);
    Cv_mat_to_arma_mat(rot_binanry_img_max, rot_binanry_img_arm_max);
    // 最大突变数的边界角点坐标 x11, x12, y11, y12;
    float x11 = 0, x12 = 0, y11 = 0, y12 = 0;

    // 寻找另一条边界
    int delta_of_size1 = 0;
    finding_second_bd2(max_delta_rot_img_idx, rot_binanry_img_arm_max, rot_binanry_img_max, \
		x11, x12, y11, y12, delta_of_size1);

    //-------------------------------- 寻找第二大突变数 ----------------------------------------//
    int secd_max_delta_num_idx1 = max_delta_rot_angel - 45; if (secd_max_delta_num_idx1 < 0) secd_max_delta_num_idx1 = 0;
    int secd_max_delta_num_idx2 = max_delta_rot_angel + 45; if (secd_max_delta_num_idx2 > 179) secd_max_delta_num_idx2 = 179;

    // 去掉原曲线中的最大峰值
    arma::vec max_delta_num_cell2;
    max_delta_num_cell2 = max_delta_num_cell;
    max_delta_num_cell2.rows(secd_max_delta_num_idx1, secd_max_delta_num_idx2) = \
		arma::zeros(secd_max_delta_num_idx2 - secd_max_delta_num_idx1 + 1, 1);
    int secd_max_delta_num = arma::max(max_delta_num_cell2.col(0));
    //cout << secd_max_delta_num << endl;
    // 第二大突变数对应的旋转角度
    arma::uvec secd_max_delta_num_idx_vec = arma::find(max_delta_num_cell == secd_max_delta_num);
    int secd_max_delta_rot_angel = secd_max_delta_num_idx_vec(0);
    //cout << "旋转角度" << secd_max_delta_rot_angel << endl;
    // 第二大突变数对应的在当次旋转中的X索引值
    int secd_max_delta_rot_img_idx = max_delta_idx_cell(secd_max_delta_rot_angel);
    //std::cout << secd_max_delta_rot_img_idx << endl;
    // 给出该旋转角度下的二值化图片
    // 定义旋转后的图片
    cv::Mat secd_rot_binanry_img_max;
    // 设置旋转变换的仿射矩阵
    cv::Mat secd_rot_mat_max = getRotationMatrix2D(center_img, secd_max_delta_rot_angel, 1);
    // 图片旋转
    cv::Mat binanry_img2 = binanry_img.clone();
    warpAffine(binanry_img2, secd_rot_binanry_img_max, secd_rot_mat_max, cv::Size(binanry_img2.cols, binanry_img2.rows));
    arma::mat secd_rot_binanry_img_arm_max(secd_rot_binanry_img_max.rows, secd_rot_binanry_img_max.cols);
    Cv_mat_to_arma_mat(secd_rot_binanry_img_max, secd_rot_binanry_img_arm_max);
    // 最大突变数的边界角点坐标 x21, x22, y21, y22;
    float x21 = 0, x22 = 0, y21 = 0, y22 = 0;
    // 寻找另一条边界
    int delta_of_size2 = 0;
    finding_second_bd2(secd_max_delta_rot_img_idx, secd_rot_binanry_img_arm_max, secd_rot_binanry_img_max, \
		x21, x22, y21, y22, delta_of_size2);
    // 图片中心点
    float center_img_x = binanry_img.cols / 2;
    float center_img_y = binanry_img.rows / 2;
    //------------------------------- 求取各个角点在原图像中的位置 ----------------------------------//
    // 第一个边界
    float p1_x = 0, p1_y = 0, p2_x = 0, p2_y = 0, p3_x = 0, p3_y = 0, p4_x = 0, p4_y = 0;
    get_point_img(center_img_x, center_img_y, x11, y11, max_delta_rot_angel, p1_x, p1_y);
    get_point_img(center_img_x, center_img_y, x11, y12, max_delta_rot_angel, p2_x, p2_y);
    get_point_img(center_img_x, center_img_y, x12, y11, max_delta_rot_angel, p3_x, p3_y);
    get_point_img(center_img_x, center_img_y, x12, y12, max_delta_rot_angel, p4_x, p4_y);
    // 第二个边界
    float q1_x = 0, q1_y = 0, q2_x = 0, q2_y = 0, q3_x = 0, q3_y = 0, q4_x = 0, q4_y = 0;
    get_point_img(center_img_x, center_img_y, x21, y21, secd_max_delta_rot_angel, q1_x, q1_y);
    get_point_img(center_img_x, center_img_y, x21, y22, secd_max_delta_rot_angel, q2_x, q2_y);
    get_point_img(center_img_x, center_img_y, x22, y21, secd_max_delta_rot_angel, q3_x, q3_y);
    get_point_img(center_img_x, center_img_y, x22, y22, secd_max_delta_rot_angel, q4_x, q4_y);
    //cout << max_delta_rot_angel << endl;
    //cout << secd_max_delta_rot_angel <<endl;
    // 在 colorimg_cut 中显示出来

    cv::Point2f aa1, bb1, cc1, dd1;

    aa1 = cv::Point2f(p1_x - 50, p1_y - 50);
    bb1 = cv::Point2f(p2_x - 50, p2_y - 50);
    cc1 = cv::Point2f(p3_x - 50, p3_y - 50);
    dd1 = cv::Point2f(p4_x - 50, p4_y - 50);

    cv::Point2f aa2, bb2, cc2, dd2;

    aa2 = cv::Point2f(q1_x - 50, q1_y - 50);
    bb2 = cv::Point2f(q2_x - 50, q2_y - 50);
    cc2 = cv::Point2f(q3_x - 50, q3_y - 50);
    dd2 = cv::Point2f(q4_x - 50, q4_y - 50);
    if (delta_of_size1 > delta_of_size2)
    {
        //         cv::line(colorimg_cut, aa1, bb1, cvScalar(0, 0, 200), 2);
        //         cv::line(colorimg_cut, aa1, cc1, cvScalar(0, 0, 200), 2);
        //         cv::line(colorimg_cut, bb1, dd1, cvScalar(0, 0, 200), 2);
        //         cv::line(colorimg_cut, cc1, dd1, cvScalar(0, 0, 200), 2);

        heap_rect[0] = aa1 + bias;
        heap_rect[1] = bb1 + bias;
        heap_rect[2] = cc1 + bias;
        heap_rect[3] = dd1 + bias;

    }
    else
    {
        //         cv::line(colorimg_cut, aa2, bb2, cvScalar(200, 0, 0), 2);
        //         cv::line(colorimg_cut, aa2, cc2, cvScalar(200, 0, 0), 2);
        //         cv::line(colorimg_cut, bb2, dd2, cvScalar(200, 0, 0), 2);
        //         cv::line(colorimg_cut, cc2, dd2, cvScalar(200, 0, 0), 2);

        heap_rect[0] = aa2 + bias;
        heap_rect[1] = bb2 + bias;
        heap_rect[2] = cc2 + bias;
        heap_rect[3] = dd2 + bias;

    }

    //imshow("colorimg_cut", colorimg_cut);

}




/*
任务B的识别程序； 获取手上物体的零件名称
输入：srcImg: 原始彩色图片
输出：ModelName: 返回无符号整型 0-不存在孔 1-圆柱 2-三棱柱 3-四棱柱
*/
void ImageProcess::element_judge_taskB(uint &ModelName_)
{
    cv::Mat srcImg;
    ImageCapture();
    srcImg = original_colorimg_;

    cv::Mat original_colorimg, img_cut, shifted_img, gray_img, binary_img;

    // 图片裁剪
    srcImg(rect_on_hand_taskB_element).copyTo(img_cut);

    cv::imwrite(save_string+"test_roi_hand_taskB.png",img_cut);

    // 彩色滤波
    cv::pyrMeanShiftFiltering(img_cut, shifted_img, 21, 51);

    // 彩色图转灰度图
    cv::cvtColor(shifted_img, gray_img, CV_BGR2GRAY);

    // 二值化
    threshold(gray_img, binary_img, 125, 255, cv::THRESH_BINARY);

    // 形态学操作
    cv::Mat k = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
    cv::morphologyEx(binary_img, binary_img, cv::MORPH_ERODE, k);//可以实现多种形态学操作类型
    cv::imwrite(save_string+"test_roi_binary_hand_taskB.png",binary_img);
    // 定义轮廓存储
    vector<vector<cv::Point> > g_vContours;
    // 轮廓索引层次
    vector<cv::Vec4i> g_vHierarchy;
    // 寻找轮廓
    findContours(binary_img, g_vContours, g_vHierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));//RETR_EXTERNAL RETR_LIST RETR_CCOMP RETR_TREE  RETR_EXTERNAL只寻找最外部轮廓不行
    double maxarea = 1000;
    int maxAreaIdx = -1;
    for (int index = g_vContours.size() - 1; index >= 0; index--)
    {
        // 若立方体的表面没有孔，则所截取的区域将完全是白色，面积必然会大于10000
        // 若立方体表面有孔，或者夹取的是零件，则面积会小于10000而大于1000
        // 如果不进入if，则说明没有零件或者立方体表面没有孔，可能存在问题
        double tmparea = fabs(contourArea(g_vContours[index]));
        if ((tmparea > maxarea) && (tmparea < 10000))
        {
            maxarea = tmparea;
            maxAreaIdx = index;
        }
    }

    if (maxAreaIdx == -1)
    {
        ModelName_ = 3;//对应的就是不存在物体
        cout << "任务B：该立方体这个面没有孔" << endl;
    }

    else
    {
        cv::Moments mu;
        cv::Point2f mc, mc1, mc2;
        mu = cv::moments(g_vContours[maxAreaIdx], false);

        mc = cv::Point2f(static_cast<float>(mu.m10 / mu.m00) + rect_on_hand_taskA.x, static_cast<float>(mu.m01 / mu.m00) + rect_on_hand_taskA.y);
        mc1 = cv::Point2f(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
        cv::RotatedRect rect = minAreaRect(g_vContours.at(maxAreaIdx));
        cv::Point2f vertices[4];
        rect.points(vertices);
        double orientation = rect.angle;
        double orientation_rads = orientation*3.1416 / 180;
        double area = mu.m00;//contourArea(g_vContours[i]),
        double perimeter = arcLength(g_vContours.at(maxAreaIdx), true);
        double roundness = (4 * 3.1416*area) / (pow(perimeter, 2));
        double rectarea = (rect.size.height)*(rect.size.width);
        double squareness = area / rectarea;
        cout << "roundness:" << roundness << endl;
        cout << "squareness:" << squareness << endl;
        char type_hole[100];
        if (squareness > 0.90)
        {
            ModelName_ = 1;//"square";
//            strcpy_s(type_hole, "square");
            cout << "任务B：方形" << endl;
        }
        else {

            if (roundness < 0.66)
            {
                ModelName_ = 0;//"triangle";
                //strcpy_s(type_hole, "triangle");
                cout << "任务B：三角形" << endl;
            }
            else if (roundness > 0.66 &&roundness < 0.84)
            {

                ModelName_ = 1;//"square";
                //strcpy_s(type_hole, "square");
                cout << "任务B：正方形" << endl;

            }
            else if (roundness > 0.84)
            {
                ModelName_ = 2;//"circle";
                //strcpy_s(type_hole, "circle");
                cout << "任务B：圆形" << endl;
            }
        }
    }
}


/*
任务B的识别程序； 获取手上物体的方块名称
输入：srcImg: 原始彩色图片
输出：CubeName: 返回无符号整型 0-不存在孔 1-圆孔 2-三角孔 3-方形孔
*/
void ImageProcess::cube_judge_taskB(uint &CubeName_)
{
    cv::Mat srcImg;
    ImageCapture();
    srcImg = original_colorimg_;

    cv::Mat original_colorimg, img_cut, shifted_img, gray_img, binary_img;

    // 图片裁剪
    srcImg(rect_on_hand_taskB_cube).copyTo(img_cut);

    cv::imwrite(save_string+"test_roi_hand_taskB.png",img_cut);

    // 彩色滤波
    cv::pyrMeanShiftFiltering(img_cut, shifted_img, 21, 51);

    // 彩色图转灰度图
    cv::cvtColor(shifted_img, gray_img, CV_BGR2GRAY);

    // 二值化
    threshold(gray_img, binary_img, 125, 255, cv::THRESH_BINARY);

    // 形态学操作
    cv::Mat k = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
    cv::morphologyEx(binary_img, binary_img, cv::MORPH_ERODE, k);//可以实现多种形态学操作类型
    cv::imwrite(save_string+"test_roi_binary_hand_taskB.png",binary_img);
    // 定义轮廓存储
    vector<vector<cv::Point> > g_vContours;
    // 轮廓索引层次
    vector<cv::Vec4i> g_vHierarchy;
    // 寻找轮廓
    findContours(binary_img, g_vContours, g_vHierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));//RETR_EXTERNAL RETR_LIST RETR_CCOMP RETR_TREE  RETR_EXTERNAL只寻找最外部轮廓不行
    double maxarea = 1000;
    int maxAreaIdx = -1;
    for (int index = g_vContours.size() - 1; index >= 0; index--)
    {
        // 若立方体的表面没有孔，则所截取的区域将完全是白色，面积必然会大于10000
        // 若立方体表面有孔，或者夹取的是零件，则面积会小于10000而大于1000
        // 如果不进入if，则说明没有零件或者立方体表面没有孔，可能存在问题
        double tmparea = fabs(contourArea(g_vContours[index]));
        if ((tmparea > maxarea) && (tmparea < 10000))
        {
            maxarea = tmparea;
            maxAreaIdx = index;
        }
    }

    if (maxAreaIdx == -1)
    {
        CubeName_ = 3;//对应的就是不存在物体
        cout << "任务B：该立方体这个面没有孔" << endl;
    }

    else
    {
        cv::Moments mu;
        cv::Point2f mc, mc1, mc2;
        mu = cv::moments(g_vContours[maxAreaIdx], false);

        mc = cv::Point2f(static_cast<float>(mu.m10 / mu.m00) + rect_on_hand_taskA.x, static_cast<float>(mu.m01 / mu.m00) + rect_on_hand_taskA.y);
        mc1 = cv::Point2f(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
        cv::RotatedRect rect = minAreaRect(g_vContours.at(maxAreaIdx));
        cv::Point2f vertices[4];
        rect.points(vertices);
        double orientation = rect.angle;
        double orientation_rads = orientation*3.1416 / 180;
        double area = mu.m00;//contourArea(g_vContours[i]),
        double perimeter = arcLength(g_vContours.at(maxAreaIdx), true);
        double roundness = (4 * 3.1416*area) / (pow(perimeter, 2));
        double rectarea = (rect.size.height)*(rect.size.width);
        double squareness = area / rectarea;
        cout << "roundness:" << roundness << endl;
        cout << "squareness:" << squareness << endl;
        char type_hole[100];
        if (squareness > 0.90)
        {
            CubeName_ = 1;//"square";
//            strcpy_s(type_hole, "square");
            cout << "任务B：方形" << endl;
        }
        else {

            if (roundness < 0.66)
            {
                CubeName_ = 0;//"triangle";
                //strcpy_s(type_hole, "triangle");
                cout << "任务B：三角形" << endl;
            }
            else if (roundness > 0.66 &&roundness < 0.84)
            {

                CubeName_ = 1;//"square";
                //strcpy_s(type_hole, "square");
                cout << "任务B：正方形" << endl;

            }
            else if (roundness > 0.84)
            {
                CubeName_ = 2;//"circle";
                //strcpy_s(type_hole, "circle");
                cout << "任务B：圆形" << endl;
            }
        }
    }

}
//--------------------------------------------------------------------------------------------//