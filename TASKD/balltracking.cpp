/*
 * File:   BallTracking.cpp
 * Author: haikalpribadi
 *
 * Created on July 14, 2012, 12:24 PM
 */
#include "image_process.h"
#include "balltracking.h"
#include <cobotsys.h>

#include <cobotsys_abstract_dual_arm_robot_link.h>


using namespace std;
using namespace cv;
double xxx1;
double yyy1;
double xxx;
double yyy;

double aaa=10;
int flag2=0;
cv::RNG g_rng(12345);
//画轮廓找椭圆的定义结束




//
void BallTracking::ImageCapture() {
    sleep(1);

    //cobotsys::init_library(1,a);
    cobotsys::ErrorInfo errorInfo;
    int exposureTime=500;
    _captureImages.frames.clear();
    _captureImages = dualAuboDriver->dual_arm_driver->CaptureImage(errorInfo, HeadCamera, exposureTime);
//        cv::Mat color = _captureImages.at(0).image.clone;//分别对应深度图和彩色图
//        cv::Mat depth = _captureImages.at(1).image.clone;
    _captureShow_color = _captureImages.frames.at(0).data.clone();
    _captureShow_depth = _captureImages.frames.at(1).data.clone();
    if (_captureImages.frames.size() != 0) {
        std::cout<<"成功捕获图片"<<std::endl;
    }
    else
    {
        std::cout<<"捕获图片失败"<<std::endl;
    }

    //cv::imshow("捕获的图片",_captureShow);
    cv::imwrite("//home//geds//ybr//color.jpg",_captureShow_color);
    cv::imwrite("//home//geds//ybr//depth.jpg",_captureShow_depth);
    //std::cout<<_captureShow.channels()<<std::endl;

    original_colorimg_ = cv::imread("//home//geds//ybr//color.jpg", 1);
    original_depthimg_ = cv::imread("//home//geds//ybr//depth.jpg",1);
   // return original_colorimg_;
}

//
//
BallTracking::BallTracking()
{
    initCaliMatrix();
}


BallTracking::BallTracking(std::shared_ptr<DualAuboDriver> _dualAuboDriver){
    dualAuboDriver = _dualAuboDriver;
    initCaliMatrix();
}

void BallTracking::initCaliMatrix() {
//    const Mat tf_on_table_taskA_right = (cv::Mat_<double>(3, 3) << 0.000723880348365276, -8.01003524553892e-05, -0.511109274533355,
//            -1.88624607876053e-05, -0.000740427784349975, 0.859995907918861,
//            1.43982048506075e-16, -2.27248775352962e-16, 1.00000000000000);
//
//    const Mat tf_on_table_taskA_left = (cv::Mat_<double>(3, 3) << 0.000711776772774457,	0.000115728709599675,	-0.646618062269930,
//            -1.88075109759453e-05,	-0.000725105993795582,	0.851264206557018,
//            5.87637577487143e-17,	-3.98986399474666e-17,	1.00000000000000);
//
//    const Mat tf_on_table_taskB_right = (cv::Mat_<double>(3, 3) << 0.000723880348365276, -8.01003524553892e-05, -0.511109274533355,
//            -1.88624607876053e-05, -0.000740427784349975, 0.859995907918861,
//            1.43982048506075e-16, -2.27248775352962e-16, 1.00000000000000);
//
//    const Mat tf_on_table_taskB_left = (cv::Mat_<double>(3, 3) << 0.000711776772774457,	0.000115728709599675,	-0.646618062269930,
//            -1.88075109759453e-05,	-0.000725105993795582,	0.851264206557018,
//            5.87637577487143e-17,	-3.98986399474666e-17,	1.00000000000000);
}

//检测椭圆是否合格
bool checkEllipseShape(cv::Mat src,cv::vector<cv::Point> contour,cv::RotatedRect ellipse,double ratio=0.01)
{
    //ellipse为对“contour【i】用”fiteclipse拟合的输出：RotatedRect 类型的矩形，是拟合出椭圆的最小外接矩形。
    //contour为找出的轮廓之一
    //get all the point on the ellipse point
    cv::vector<cv::Point> ellipse_point;

    //get the parameter of the ellipse
    cv::Point2f center = ellipse.center;
    double a_2 = pow(ellipse.size.width*0.5,2);//长轴半径
    double b_2 = pow(ellipse.size.height*0.5,2);//短轴半径
    double ellipse_angle = (ellipse.angle*3.1415926)/180;//角度改单位

//std::cout<<"ellipse_angle="<<ellipse_angle<<std::endl;
    //the uppart
    for(int i=0;i<ellipse.size.width;i++)
    {
        double x = -ellipse.size.width*0.5+i;
        double y_left = sqrt( (1 - (x*x/a_2))*b_2 );

        //rotate
        //[ cos(seta) sin(seta)]
        //[-sin(seta) cos(seta)]
        cv::Point2f rotate_point_left;
        rotate_point_left.x =  cos(ellipse_angle)*x - sin(ellipse_angle)*y_left;
        rotate_point_left.y = +sin(ellipse_angle)*x + cos(ellipse_angle)*y_left;

        //trans
        rotate_point_left += center;

        //store

        cv::Point2i rotate_point_left2 = rotate_point_left;
        ellipse_point.push_back(cv::Point(rotate_point_left2));
        //push_back是编程语言里面的一个函数名。如c++中的vector头文件里面就有这个push_back函数，在vector类中作用为在vector尾部加入一个数据。
        //string str1("abc");str1.push_back('d');则str1变为abcd
    }
    //the downpart
    for(int i=0;i<ellipse.size.width;i++)
    {
        double x = ellipse.size.width*0.5-i;
        double y_right = -sqrt( (1 - (x*x/a_2))*b_2 );

        //rotate
        //[ cos(seta) sin(seta)]
        //[-sin(seta) cos(seta)]
        cv::Point2f rotate_point_right;
        rotate_point_right.x =  cos(ellipse_angle)*x - sin(ellipse_angle)*y_right;
        rotate_point_right.y = +sin(ellipse_angle)*x + cos(ellipse_angle)*y_right;

        //trans
        rotate_point_right += center;

        cv::Point2i rotate_point_right2 = rotate_point_right;

        //store
        ellipse_point.push_back(cv::Point(rotate_point_right2));

    }


    cv::vector<cv::vector<cv::Point> > contours1;
    contours1.push_back(ellipse_point);
    //drawContours(src,contours1,-1,Scalar(255,0,0),2);

    //match shape
    double a0 = matchShapes(ellipse_point,contour,CV_CONTOURS_MATCH_I1,0);
    //ellipse_point为拟合后矩形轮廓上的点取样改变为椭圆轮廓，contour为原来的轮廓
    //MatchShapes
    //比较两个形状
    //double cvMatchShapes( const void* object1, const void* object2,

    //object1      第一个轮廓或灰度图像
    //object2      第二个轮廓或灰度图像
    //method     比较方法，其中之一 CV_CONTOUR_MATCH_I1, CV_CONTOURS_MATCH_I2 or CV_CONTOURS_MATCH_I3.
    //parameter       比较方法的参数 (目前不用).


//	if (a0>0.01)
//	{
//		return true;
//	}
    aaa=a0;
    //  std::cout<<"椭圆符合度a0="<<aaa<<std::endl;
    if (a0>10)
    {
        return true;
    }

    return false;
}



/** @brief *****
 *  @param[in]
 *  @param[out]
 * */
cv::Point2f BallTracking::PixelToWorld_dish(cv::Point2f Pix_coordinate)
{
   // cout<<"I am in function: PixelToWorld_taskB."<<endl;
    cv::Mat tp, point_coodinate, tt;
    double the_center_of_img=850;

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



geometry_msgs::PoseStamped current_pos8;
double zz;



//像素坐标系到世界坐标系的转换
cv::Point2f BallTracking::PixelToWorldBall(double x, double y)
{
    cv::Mat tt,tp;
//	tf = (Mat_<double>(2,3) <<2.27101175219679e-05, -0.00058656830709881, 0.6947129217938169,
//            -0.0005845671222829157, 4.680040017836091e-06, 0.2422717208519501);

    cv::Mat tfUp = (cv::Mat_<double>(2,3) <<2.27101175219679e-05, -0.00058656830709881, 0.6947129217938169,
            -0.0005845671222829157, 4.680040017836091e-06, 0.2422717208519501);
    tp = (cv::Mat_<double>(3, 1) << x, y, 1);
    tt = tfUp * tp;
    cv::Point2f point_World;
    point_World.x = tt.at<double>(0, 0);
    point_World.y = tt.at<double>(0, 1);
//    cout<<"标定函数:"<<point_World<<endl;
    return point_World;
}


//像素坐标系到世界坐标系的转换2
cv::Point2f BallTracking::PixelToWorldBall2(double x, double y)
{
    cv::Mat tt,tp;
    double X,Y,Z,zz;
    zz=1.2;

    X=1.05279194*zz+0.0000185763138*x*zz-0.00064771098*y*zz+0.0806523429;
    Y=0.915364912*zz-0.000926989692*x*zz-0.000039349283*y*zz- 0.102063492;
    Z=0.000036792144*x*zz-0.369886228*zz-0.000664352781*y*zz+0.000664352781;

    cv::Point2f point_World;
    point_World.x = X;
    point_World.y = Y;
//    cout<<"标定函数:"<<point_World<<endl;
    return point_World;
}




std::vector<double> BallTracking::GetBall()
{



// img_sub_ = img_transport_.subscribe("/kinect2/hd/image_color", 1, &BallTracking::imageCallback, this);

    //img_sub_ = img_transport_.subscribe("camera/image", 1, &BallTracking::imageCallback, this);
    //sub = node_handle_.subscribe("xfspeech", 1, &BallTracking::chatterCallback, this);
   // sub = node_handle_.subscribe("xfspeech", 1, &BallTracking::callBackFun, this);
    //sub = node_handle_.subscribe("xfspeech", 1, &BallTracking::chatterCallback2, this);

//    dishpub = node_handle_.advertise<geometry_msgs::PoseStamped>("dish", 1);
//    ballpub = node_handle_.advertise<geometry_msgs::PoseStamped>("ball", 1);

    cv::namedWindow(camerawindow);
    cv::namedWindow(hsvwindow);
    cv::namedWindow(filteredwindow);

    cv::namedWindow(camerawindow2);
    cv::namedWindow(hsvwindow2);
    cv::namedWindow(filteredwindow2);

    cv::namedWindow(filteredwindow9);
    cv::namedWindow(filteredwindow8);

//44，，，12，，无法识别,47,48,51,53差别大,参数160,210,data9
//16差大,32，46，51参数180,210
 //  Mat g_srcImage = imread("/home/ybr/photo/data5/kkdd.jpg", 1);
 //13差别大,25偏离


 //   Mat g_srcImage = imread("/home/ybr/photo/data10/0008_color.jpg", 1);
    Mat g_srcImage = imread("/home/ybr/photo/data12/0001_color.jpg", 1);

    int a=16;
    int b=-14;

//  std::cout<<"here1"<<std::endl;
//
//    ImageCapture();
//    sleep(2);
//    std::cout<<"here2"<<std::endl;
//    Mat g_srcImage = original_colorimg_;
//    std::cout<<"here3"<<std::endl;


    //cv::Mat img_hsv(g_srcImage.size(), CV_8U);
    cv::Mat img_hsv, img_filtered;
    cv::cvtColor(g_srcImage, img_hsv, CV_BGR2HSV);



//话轮廓找椭圆
// cv::Mat src2(g_srcImage, cv::Rect(300,690,1100,390));//ur
    /// cv::Mat g_srcImage(g_srcImage, cv::Rect(100,260,1600,810));//切割出奥博机器人的视觉区域


   // cv::Mat g_srcImage2(g_srcImage, cv::Rect(50,260,1650,810));//切割出奥博机器人的视觉区域

   int left_1=50;
   int up_1=260;
   int length_1=1450;
   int high_1=810;

    cv::Mat g_srcImage2(g_srcImage, cv::Rect(left_1,up_1,length_1,high_1));//切割出奥博机器人的视觉区域
   // cv::Mat g_srcImage2(g_srcImage, cv::Rect(50,260,1450,810));//切割出奥博机器人的视觉区域

    // cv::Mat src2(g_srcImage, cv::Rect(800,420,600,500));//切割出奥博机器人的盘子所在视觉区域

     int mmm=1;
    int nnn=1;


  //  g_srcImage2.at<uchar>(1, 2) = 150;
   // <Vec3b>
 //   g_srcImage2.at<Vec3b>(1, 2)[1] = 150;


    //找盘子方法一
/*
//找椭圆以减少畸变的影响
   // cv::Mat src2(cv_img_ptr->image, cv::Rect(300,690,1100,390));//ur
    cv::Mat src2=g_srcImage2;//切割出奥博机器人的视觉区域
   // cv::Mat src2(cv_img_ptr->image, cv::Rect(800,420,600,500));//切割出奥博机器人的盘子所在视觉区域

    cv::Mat src_gray;

    // convert into gray
    cvtColor( src2, src_gray, CV_BGR2GRAY );

    //ybr加上用hsv分割，不用那个灰度了
    cv::cvtColor(src2, img_hsv, CV_BGR2HSV);
   cv::cvtColor(src2, img_filtered, CV_BGR2GRAY);//不分割颜色

    cv::Mat threshold_output;
    cv::Mat threshold_output2;
    cv::vector<cv::vector<cv::Point> > contours;

   // cv::inRange(img_hsv, cv::Scalar(0, 0, 221), cv::Scalar(180, 30, 255), img_filtered);//ybr：分隔白色

    //ybr add
   // cv::GaussianBlur(src_gray, threshold_output2, cv::Size(3, 3), 0, 0);//高斯滤波
   cv::GaussianBlur(img_filtered, threshold_output2, cv::Size(3, 3), 0, 0);//高斯滤波
//    cv::GaussianBlur(img_filtered, threshold_output, cv::Size(3, 3), 0, 0);//高斯滤波


    // find contours
//	int threshold_value = threshold( src_gray, threshold_output, 0, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
    //  int threshold_value = threshold( src_gray, threshold_output, 230, 255, CV_THRESH_BINARY);//适应bb，jpg和cc.jpg、dd、yy.jpg,xx要阈值210
    int threshold_value = threshold( threshold_output2, threshold_output, 113, 255,  CV_THRESH_BINARY);//实验

    // int threshold_value = threshold( src_gray, threshold_output, 50, 254, CV_THRESH_TOZERO);
//    函数 cvThreshold 对单通道数组应用固定阈值操作。该函数的典型应用是对灰度图像进行阈值操作得到二值图像。
//像素值是原稿图像被数字化时由计算机赋予的值，它代表了原稿某一小方块的平均亮度信息0-255
//(cvCmpS 也可以达到此目的) 或者是去掉噪声，例如过滤很小或很大像素值的图像点。本函数支持的对图像取阈值的方法由 threshold_type 确定。
//void cvThreshold//( const CvArr* src,//CvArr* dst,//double threshold,//double max_value,//int threshold_type );
//src：原始数组 (单通道 , 8-bit of 32-bit 浮点数)。
//dst：输出数组，必须与 src 的类型一致，或者为 8-bit。
//threshold：阈值
//max_value：使用 CV_THRESH_BINARY 和 CV_THRESH_BINARY_INV 的最大值。
//threshold_type：阈值类型
//threshold_type=CV_THRESH_BINARY:如果 src(x,y)>threshold ,dst(x,y) = max_value; 否则,dst（x,y）=0;
//threshold_type=CV_THRESH_BINARY_INV:如果 src(x,y)>threshold,dst(x,y) = 0; 否则,dst(x,y) = max_value.
//threshold_type=CV_THRESH_TRUNC:如果 src(x,y)>threshold，dst(x,y) = max_value; 否则dst(x,y) = src(x,y).
//threshold_type=CV_THRESH_TOZERO:如果src(x,y)>threshold，dst(x,y) = src(x,y) ; 否则 dst(x,y) = 0。
//threshold_type=CV_THRESH_TOZERO_INV:如果 src(x,y)>threshold，dst(x,y) = 0 ; 否则dst(x,y) = src(x,y).
//词条标签：

    std::vector<cv::Vec3f> circles;

    cv::HoughCircles(img_filtered, circles, CV_HOUGH_GRADIENT, 2, 100, 100, 50, 210, 250);//找圆形函数，适合不太高的情况


*/
//end  shiyan


















    cvtColor(g_srcImage2, g_grayImage, COLOR_BGR2GRAY);
    blur(g_grayImage, g_grayImage, Size(3, 3));

    //创建窗口
//    namedWindow(WINDOW_NAME1, WINDOW_AUTOSIZE);
//    imshow(WINDOW_NAME1, g_srcImage);

    //创建滚动条并初始化
//    createTrackbar("canny阈值", WINDOW_NAME1, &g_nThresh, g_nThresh_max, on_ThreshChange);
//    on_ThreshChange(0, 0);
    //   waitKey(0);


//    g_nThresh = 240;
//    g_nThresh_max = 245;
//


    g_nThresh = 180;
    g_nThresh_max = 360;



    //用Canny算子检测边缘
    Canny(g_grayImage, g_cannyMat_output, g_nThresh, g_nThresh_max, 3);

//    void cvCanny(
//
//            const CvArr* image,              //第一个参数表示输入图像，必须为单通道灰度图
//            CvArr* edges,                      //第二个参数表示输出的边缘图像，为单通道黑白图
//            double threshold1,
//            double threshold2,               //第三个参数和第四个参数表示阈值，这二个阈值中当中的小阈值用来控制边缘连接，
//    大的阈值用来控制强边缘的初始分割即如果一个像素的梯度大与上限值，则被认为
//    是边缘像素，如果小于下限阈值，则被抛弃。如果该点的梯度在两者之间则当这个
//    点与高于上限值的像素点连接时我们才保留，否则删除。
//    int aperture_size=3              //第五个参数表示Sobel 算子大小，默认为3即表示一个3*3的矩阵。Sobel 算子与
//    高斯拉普拉斯算子都是常用的边缘算子
//    );

    //寻找轮廓
    findContours(g_cannyMat_output, g_vContours, g_vHierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<vector<Point>> contours_poly(g_vContours.size());

    //绘出轮廓
    //Mat drawing = Mat::zeros(g_cannyMat_output.size(), CV_8UC3);
    drawing = Mat::zeros(g_cannyMat_output.size(), CV_8UC3);
    drawing_ball = Mat::zeros(g_cannyMat_output.size(), CV_8UC3);
    for (int i = 0; i < g_vContours.size(); i++) {
        Scalar color = Scalar(g_rng.uniform(0, 255), g_rng.uniform(0, 255), g_rng.uniform(0, 255));//不同轮廓不同颜色
        cv::Scalar color2 = cv::Scalar( 255, 255, 255);//所有轮廓一个颜色

        approxPolyDP(Mat(g_vContours[i]),contours_poly[i],3,true);//这个和上面的True都是表示是闭曲线，轮廓近似处理


        //线宽为4最稳定，为2最灵敏
        drawContours(drawing, contours_poly, i, color2, 2, 8, g_vHierarchy, 0, Point());//线宽由2改为32，方便找椭圆，
        // drawContours(drawing, g_vContours, i, color2, 2, 8, g_vHierarchy, 0, Point());//不画近似，方便找椭圆
    }
    // imshow(WINDOW_NAME2, drawing);




    //ybr加

    //先闭运算，减少黑色小洞洞
    int morph_elem =2;//0-2
    int morph_size = 8;//0-21
    Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
  //  morphologyEx( drawing, drawing, MORPH_CLOSE, element );//此句为使用闭运算。注释即不使用闭运算
   // morphologyEx( drawing, drawing, MORPH_OPEN, element );//此句为使用闭运算。注释即不使用闭运算

//    Mat element3 = getStructuringElement(MORPH_RECT, Size(3, 3));
//    morphologyEx(drawing,drawing, MORPH_OPEN, element3);

//解释该函数    Opening: MORPH_OPEN : 2
//    Closing: MORPH_CLOSE: 3
//    Gradient: MORPH_GRADIENT: 4
//    Top Hat: MORPH_TOPHAT: 5
//    Black Hat: MORPH_BLACKHAT: 6



    cvtColor(drawing, g_grayImage2, COLOR_BGR2GRAY);
    blur(g_grayImage2, g_grayImage2, Size(3, 3));


    //  int threshold_value = threshold( g_grayImage2, g_grayImage2, 210, 255,  CV_THRESH_BINARY);//实验二值化


//    g_nThresh = 240;
//    g_nThresh_max = 245;
//

    g_nThresh = 180;
    g_nThresh_max = 360;


    Canny(g_grayImage2, g_cannyMat_output2, g_nThresh, g_nThresh_max, 3);


    findContours(g_cannyMat_output2, g_vContours2, g_vHierarchy2, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    cv::vector<cv::RotatedRect> minEllipse(g_vContours2.size());
    cv::vector<cv::RotatedRect> minEllipse2(g_vContours2.size());
    minEllipse2[0].size.height=0;
    minEllipse2[0].size.width=0;
    minEllipse2[0].center.x=0;
    minEllipse2[0].center.y=0;
    flag2=0;

    // Mat drawing2 = Mat::zeros(g_cannyMat_output2.size(), CV_8UC3);
    drawing2 = Mat::zeros(g_cannyMat_output2.size(), CV_8UC3);


    drawing2=g_srcImage2;
    //ybr
    int ybr_glag=0;
    for( int i = 0; i < g_vContours2.size(); i++ ) {
        //point size check
        if (g_vContours2[i].size() < 10) {
            continue;
        }

        //point area//面积
        if (contourArea(g_vContours2[i]) < 10) {
            continue;
        }

        minEllipse[i] = fitEllipse(cv::Mat(g_vContours2[i]));

        //输入：二维点集，要求拟合的点至少为6个点。存储在std::vector<> or Mat
        //处理：该函数使用的是最小二乘的方法进行拟合的。
        //输出：RotatedRect 类型的矩形，是拟合出椭圆的最小外接矩形。

        cv::Scalar color = cv::Scalar(0, 0, 255);

//找球
//                if(minEllipse[i].size.width>35&&minEllipse[i].size.height>35) {
//
//
//            if (minEllipse[i].size.width < 110 && minEllipse[i].size.height < 110) {
//                if (abs(minEllipse[i].size.width - minEllipse[i].size.height) < 30) {
//                    if (abs(minEllipse[i].center.x - minEllipse2[0].center.x) < 300&&abs(minEllipse[i].center.y - minEllipse2[0].center.y) < 300) {
//                        ellipse(drawing_ball2, minEllipse[i], cv::Scalar(0, 0, 255), 2);
//                        std::cout << "球半径" << "=" << minEllipse[i].size.height << std::endl;
//                    }
//                }
//            }
//        }



//限定圆度


        if (abs(minEllipse[i].size.width / minEllipse[i].size.height) > 1.4 ||
            abs(minEllipse[i].size.height / minEllipse[i].size.width) > 1.4) {
            continue;
        }

//限定长轴与短轴的差
        if (abs(minEllipse[i].size.width-minEllipse[i].size.height) > 150) {
            continue;
        }// img_sub_ = img_transport_.subscribe("/kinect2/hd/image_color", 1, &BallTracking::imageCallback, this);








//限定面积
//        if(minEllipse[i].size.width*minEllipse[i].size.height>190000)
//        {
//
//            continue;
//        }
//
//

        if(minEllipse[i].size.width*minEllipse[i].size.height<90000||minEllipse[i].size.width*minEllipse[i].size.height>250000)
        {

            continue;
        }

        //限定圆度


//限定面积结束


        //     查看所有的大圆

        if(minEllipse[i].size.width>300&&minEllipse[i].size.height>300)
        {

//最大514,403,除以后1.273
            if(minEllipse[i].size.width<550&&minEllipse[i].size.height<550) {
//                std::cout << "椭圆短轴" << i << "=" << minEllipse[i].size.width << std::endl;
//                std::cout << "椭圆长轴" << i << "=" << minEllipse[i].size.height << std::endl;


   //        ellipse(drawing2, minEllipse[i], color, 2);
//
//                cv::Point2f center2 = minEllipse[i].center;
//                cv::circle(drawing2, cv::Point(center2.x, center2.y), 2, cv::Scalar(0, 0, 255), 6, CV_AA);//画圆函数


//                if( checkEllipseShape(drawing,g_vContours2[i],minEllipse[i]))
//                    continue;



                double s_size=minEllipse[i].size.width*minEllipse[i].size.height;
                double s2_size=243.11*minEllipse[i].center.y+95548;
                if(s_size>s2_size+6000)
                    continue;



                if(0&&minEllipse[i].center.y<150){

                    if(flag2<1){
                        minEllipse2[0].size.width=800;
                        minEllipse2[0].size.height=800;
                        flag2++;}

                    //寻找最小椭圆形
                    if((minEllipse[i].size.width*minEllipse[i].size.height<minEllipse2[0].size.width*minEllipse2[0].size.height))
                        minEllipse2[0]=minEllipse[i];

                }
                else{


                    //寻找最大椭圆形
                    if((minEllipse[i].size.width*minEllipse[i].size.height>minEllipse2[0].size.width*minEllipse2[0].size.height)){
                        //使得椭圆不超界限,minEllipse[i].center.y)>300使得椭圆中心大于在桌子中心附近
                        if((minEllipse[i].center.y)>150&&(minEllipse[i].center.y)<high_1) {
                            if ((minEllipse[i].center.x ) >200&&(minEllipse[i].center.x ) < length_1) {
                                minEllipse2[0] = minEllipse[i];
                              }
                            }
                        }
                }


                continue;
            }
        }



    }


    if(minEllipse2[0].center.x>0&&minEllipse2[0].center.y>0){
        ellipse(drawing2, minEllipse2[0], cv::Scalar(0, 0, 255), 2);
        cv::Point2f center2 = minEllipse2[0].center;
        cv::circle(drawing2, cv::Point(center2.x, center2.y), 2, cv::Scalar(0, 0, 255), 6, CV_AA);//画圆函数

        //补偿椭圆畸变
//        double xxx=0.0337*center2.x;
//        double yyy=0;
//        if(xxx<0) yyy=-14;
//        else yyy=-10;

//补偿椭圆畸变
         xxx=0.9692*center2.x+28.092;
         yyy=0.9652*center2.y+2.2022;
       // cv::circle(drawing2, cv::Point(center2.x+a, center2.y+b), 2, cv::Scalar(0, 255, 0), 6, CV_AA);//修正圆心
        cv::circle(drawing2, cv::Point(xxx,yyy), 2, cv::Scalar(0, 255, 0), 6, CV_AA);//修正圆心


        //补偿标定高度差别
         xxx1=0.9*xxx+89.822;
         yyy1=0.9136*yyy+76.796;

        // cv::circle(drawing2, cv::Point(center2.x+a, center2.y+b), 2, cv::Scalar(0, 255, 0), 6, CV_AA);//修正圆心
        cv::circle(drawing2, cv::Point(xxx1,yyy1), 2, cv::Scalar(255,0, 0), 6, CV_AA);//修正圆心



        //修正圆心左边x减，y加，x+17，y-10

        std::cout<<"椭圆短轴"<<"="<<minEllipse2[0].size.width<<std::endl;
        std::cout<<"椭圆长轴"<<"="<<minEllipse2[0].size.height<<std::endl;
        std::cout<<"椭圆圆心"<<"=("<<center2.x<<","<<center2.y<<")"<<std::endl;
        std::cout<<"椭圆符合度"<<"="<<aaa<<std::endl;
        //imshow(WINDOW_NAME3, drawing2);
    }


    else {
        std::cout<<"未找到盘子"<<std::endl;


        //先闭运算，减少黑色小洞洞
        int morph_elem4 =2;//0-2
        int morph_size4 = 15;//0-21
        Mat element4 = getStructuringElement( morph_elem4, Size( 2*morph_size4 + 1, 2*morph_size4+1 ), Point( morph_size4, morph_size4 ) );
         morphologyEx( drawing, drawing, MORPH_CLOSE, element4 );//此句为使用闭运算。注释即不使用闭运算
        // morphologyEx( drawing, drawing, MORPH_OPEN, element );//此句为使用闭运算。注释即不使用闭运算

//    Mat element3 = getStructuringElement(MORPH_RECT, Size(3, 3));
//    morphologyEx(drawing,drawing, MORPH_OPEN, element3);

//解释该函数    Opening: MORPH_OPEN : 2
//    Closing: MORPH_CLOSE: 3
//    Gradient: MORPH_GRADIENT: 4
//    Top Hat: MORPH_TOPHAT: 5
//    Black Hat: MORPH_BLACKHAT: 6



        cvtColor(drawing, g_grayImage2, COLOR_BGR2GRAY);
        blur(g_grayImage2, g_grayImage2, Size(3, 3));


        //  int threshold_value = threshold( g_grayImage2, g_grayImage2, 210, 255,  CV_THRESH_BINARY);//实验二值化


        g_nThresh = 190;
        g_nThresh_max = 230;
        Canny(g_grayImage2, g_cannyMat_output2, g_nThresh, g_nThresh * 2, 3);


        findContours(g_cannyMat_output2, g_vContours2, g_vHierarchy2, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

        cv::vector<cv::RotatedRect> minEllipse(g_vContours2.size());
        cv::vector<cv::RotatedRect> minEllipse2(g_vContours2.size());
        minEllipse2[0].size.height=0;
        minEllipse2[0].size.width=0;
        minEllipse2[0].center.x=0;
        minEllipse2[0].center.y=0;
        flag2=0;

        // Mat drawing2 = Mat::zeros(g_cannyMat_output2.size(), CV_8UC3);
//        drawing2 = Mat::zeros(g_cannyMat_output2.size(), CV_8UC3);
//
//
//        drawing2=g_srcImage2;
        //ybr
        int ybr_glag=0;
        for( int i = 0; i < g_vContours2.size(); i++ ) {
            //point size check
            if (g_vContours2[i].size() < 10) {
                continue;
            }

            //point area//面积
            if (contourArea(g_vContours2[i]) < 10) {
                continue;
            }

            minEllipse[i] = fitEllipse(cv::Mat(g_vContours2[i]));

            //输入：二维点集，要求拟合的点至少为6个点。存储在std::vector<> or Mat
            //处理：该函数使用的是最小二乘的方法进行拟合的。
            //输出：RotatedRect 类型的矩形，是拟合出椭圆的最小外接矩形。

            cv::Scalar color = cv::Scalar(0, 0, 255);

//找球
//                if(minEllipse[i].size.width>35&&minEllipse[i].size.height>35) {
//
//
//            if (minEllipse[i].size.width < 110 && minEllipse[i].size.height < 110) {
//                if (abs(minEllipse[i].size.width - minEllipse[i].size.height) < 30) {
//                    if (abs(minEllipse[i].center.x - minEllipse2[0].center.x) < 300&&abs(minEllipse[i].center.y - minEllipse2[0].center.y) < 300) {
//                        ellipse(drawing_ball2, minEllipse[i], cv::Scalar(0, 0, 255), 2);
//                        std::cout << "球半径" << "=" << minEllipse[i].size.height << std::endl;
//                    }
//                }
//            }
//        }



//限定圆度


            if (abs(minEllipse[i].size.width / minEllipse[i].size.height) > 1.4 ||
                abs(minEllipse[i].size.height / minEllipse[i].size.width) > 1.4) {
                continue;
            }

//限定长轴与短轴的差
            if (abs(minEllipse[i].size.width-minEllipse[i].size.height) > 150) {
                continue;
            }// img_sub_ = img_transport_.subscribe("/kinect2/hd/image_color", 1, &BallTracking::imageCallback, this);








//限定面积
//        if(minEllipse[i].size.width*minEllipse[i].size.height>190000)
//        {
//
//            continue;
//        }
//
//

            if(minEllipse[i].size.width*minEllipse[i].size.height<90000||minEllipse[i].size.width*minEllipse[i].size.height>250000)
            {

                continue;
            }

            //限定圆度


//限定面积结束


            //     查看所有的大圆

            if(minEllipse[i].size.width>300&&minEllipse[i].size.height>300)
            {

//最大514,403,除以后1.273
                if(minEllipse[i].size.width<550&&minEllipse[i].size.height<550) {
//                std::cout << "椭圆短轴" << i << "=" << minEllipse[i].size.width << std::endl;
//                std::cout << "椭圆长轴" << i << "=" << minEllipse[i].size.height << std::endl;


//             ellipse(drawing2, minEllipse[i], color, 2);
//
//                cv::Point2f center2 = minEllipse[i].center;
//                cv::circle(drawing2, cv::Point(center2.x, center2.y), 2, cv::Scalar(0, 0, 255), 6, CV_AA);//画圆函数


//                if( checkEllipseShape(drawing,g_vContours2[i],minEllipse[i]))
//                    continue;


                    if(0&&minEllipse[i].center.y<150){

                        if(flag2<1){
                            minEllipse2[0].size.width=800;
                            minEllipse2[0].size.height=800;
                            flag2++;}

                        //寻找最小椭圆形
                        if((minEllipse[i].size.width*minEllipse[i].size.height<minEllipse2[0].size.width*minEllipse2[0].size.height))
                            minEllipse2[0]=minEllipse[i];

                    }
                    else{


                        //寻找最大椭圆形
                        if((minEllipse[i].size.width*minEllipse[i].size.height>minEllipse2[0].size.width*minEllipse2[0].size.height)){
                            //使得椭圆不超界限,minEllipse[i].center.y)>300使得椭圆中心大于在桌子中心附近
                            if((minEllipse[i].center.y)>150&&(minEllipse[i].center.y)<high_1) {
                                if ((minEllipse[i].center.x ) >200&&(minEllipse[i].center.x ) < length_1) {
                                    minEllipse2[0] = minEllipse[i];
                                }
                            }
                        }
                    }


                    continue;
                }
            }



        }





        ellipse(drawing2, minEllipse2[0], cv::Scalar(0, 0, 255), 2);
        cv::Point2f center2 = minEllipse2[0].center;
        cv::circle(drawing2, cv::Point(center2.x, center2.y), 2, cv::Scalar(0, 0, 255), 6, CV_AA);//画圆函数

        //补偿椭圆畸变
//        double xxx=0.0337*center2.x;
//        double yyy=0;
//        if(xxx<0) yyy=-14;
//        else yyy=-10;

//补偿椭圆畸变
        xxx=0.9692*center2.x+28.092;
        yyy=0.9652*center2.y+2.2022;
        // cv::circle(drawing2, cv::Point(center2.x+a, center2.y+b), 2, cv::Scalar(0, 255, 0), 6, CV_AA);//修正圆心
        cv::circle(drawing2, cv::Point(xxx,yyy), 2, cv::Scalar(0, 255, 0), 6, CV_AA);//修正圆心


        //补偿标定高度差别
        xxx1=0.9*xxx+89.822;
        yyy1=0.9136*yyy+76.796;

        // cv::circle(drawing2, cv::Point(center2.x+a, center2.y+b), 2, cv::Scalar(0, 255, 0), 6, CV_AA);//修正圆心
        cv::circle(drawing2, cv::Point(xxx1,yyy1), 2, cv::Scalar(255,0, 0), 6, CV_AA);//修正圆心



        //修正圆心左边x减，y加，x+17，y-10

        std::cout<<"椭圆短轴"<<"="<<minEllipse2[0].size.width<<std::endl;
        std::cout<<"椭圆长轴"<<"="<<minEllipse2[0].size.height<<std::endl;
        std::cout<<"椭圆圆心"<<"=("<<center2.x<<","<<center2.y<<")"<<std::endl;
        std::cout<<"椭圆符合度"<<"="<<aaa<<std::endl;
        //imshow(WINDOW_NAME3, drawing2);

//对着else

    }




    cv::Point2f dish_point;
//    dish_point.x=minEllipse2[0].center.x+50;
//    dish_point.y=minEllipse2[0].center.y+260;

    dish_point.x=(minEllipse2[0].center.x+xxx1)/2+50;
    dish_point.y=(minEllipse2[0].center.y+yyy1)/2+260;


//画轮廓找盘子方法结束




//找障碍物——画轮廓方法
//ybr加




    drawing_ball2 = Mat::zeros(g_cannyMat_output3.size(), CV_8UC3);
    drawing_ball2=g_srcImage2;
    vector<vector<Point>> contours_poly2(g_vContours.size());


    for (int i = 0; i < g_vContours.size(); i++) {
        // Scalar color8 = Scalar(g_rng.uniform(0, 255), g_rng.uniform(0, 255), g_rng.uniform(0, 255));//不同轮廓不同颜色
        cv::Scalar color4 = cv::Scalar( 255, 255, 255);//所有轮廓一个颜色
        // approxPolyDP(Mat(g_vContours[i]),contours_poly[i],3,true);//这个和上面的True都是表示是闭曲线，轮廓近似处理
        //drawContours(drawing_ball, g_vContours, i, color4, 8, 8, g_vHierarchy, 0, Point());//线宽由2改为32，方便找椭圆
        drawContours(drawing_ball, contours_poly, i, color4, 4, 8, g_vHierarchy, 0, Point());//画出近似
    }

    cvtColor(drawing_ball, g_grayImage3, COLOR_BGR2GRAY);
    blur(g_grayImage3, g_grayImage3, Size(3, 3));


//    std::vector<cv::Vec3f> circles3;
//    cv::HoughCircles(g_grayImage3, circles3, CV_HOUGH_GRADIENT, 2, 100, 100, 50, 20, 60);//找圆形函数，适合大部分情况
//    cv::Vec3i c3;
//    for (size_t i = 0; i < circles3.size(); i++)
//    {
//        c3 = circles3[0];
//       cv::circle(drawing_ball2, cv::Point(c3[0], c3[1]), c3[2], cv::Scalar(0, 0, 255), 3, CV_AA);
//       cv::circle(drawing_ball2, cv::Point(c3[0], c3[1]), 2, cv::Scalar(0, 255, 0), 3, CV_AA);//画圆心函数
//
//    }


    Canny(g_grayImage3, g_cannyMat_output3, g_nThresh, g_nThresh * 2, 3);

    findContours( g_cannyMat_output3, g_vContours3, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);//实验用
    // findContours(g_cannyMat_output3, g_vContours3, g_vHierarchy2, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    cv::vector<cv::RotatedRect> minEllipse_ball(g_vContours3.size());
    cv::vector<cv::RotatedRect> minEllipse3(g_vContours3.size());
    minEllipse3[0].size.height=0;
    minEllipse3[0].size.width=0;
    minEllipse3[0].center.x=0;
    minEllipse3[0].center.y=0;
    // Mat drawing2 = Mat::zeros(g_cannyMat_output2.size(), CV_8UC3);
//    drawing_ball2 = Mat::zeros(g_cannyMat_output3.size(), CV_8UC3);
//
//
//    drawing_ball2=g_srcImage;
    //ybr
    int ybr_glag2=0;
    for( int i = 0; i < g_vContours3.size(); i++ )
    {

        //  approxPolyDP(Mat(g_vContours3[i]),contours_poly2[i],3,true);//轮廓近似处理为多边形

        //point size check
        if(g_vContours3[i].size()<10)
        {
            continue;
        }

        //限定面积
        if(contourArea(g_vContours3[i])<4000||contourArea(g_vContours3[i])>12000)
        {
            continue;
        }



        minEllipse_ball[i] = fitEllipse(cv::Mat(g_vContours3[i]));


        cv::Scalar color3 = cv::Scalar( 0, 0, 255);


        if(minEllipse_ball[i].center.y>150&&minEllipse_ball[i].size.width>60&&minEllipse_ball[i].size.height>60) {


            if (minEllipse_ball[i].size.width < 150 && minEllipse_ball[i].size.height < 150) {
                if (abs(minEllipse_ball[i].size.width/minEllipse_ball[i].size.height) < 1.5&&abs(minEllipse_ball[i].size.height/minEllipse_ball[i].size.width)<1.5) {
                    //   if (abs(minEllipse_ball[i].center.x - minEllipse2[0].center.x) < 300&&abs(minEllipse_ball[i].center.y - minEllipse2[0].center.y) < 300) {



                    if(minEllipse2[0].center.x>0&&minEllipse2[0].center.y>0){

                        double xxx=minEllipse2[0].center.x-minEllipse_ball[i].center.x;
                        double yyy=minEllipse2[0].center.y-minEllipse_ball[i].center.y;

                        //使得障碍物在盘子外面
                        if(sqrt(pow(xxx,2)+pow(yyy,2))>220) {
                            if(minEllipse_ball[i].center.x>200&&minEllipse_ball[i].center.x<1300&&minEllipse_ball[i].center.y>200&&minEllipse_ball[i].center.y<650){


                                if(minEllipse_ball[i].size.height*minEllipse_ball[i].size.width>minEllipse3[0].size.height*minEllipse3[0].size.width)
                                    minEllipse3[0]=minEllipse_ball[i];
                               // std::cout << "障碍物面积" << "=" << contourArea(g_vContours3[i]) << std::endl;
//
//                         ellipse(drawing_ball2, minEllipse_ball[i], cv::Scalar(255, 0, 0), 2);
//                     std::cout << "障碍物半径" << "=" << minEllipse_ball[i].size.height << std::endl;
                            }
                        }
                    }
                    else std::cout << "未找到障碍物"  << std::endl;
                    // }
                }
            }
        }


    }
    ellipse(drawing_ball2, minEllipse3[0], cv::Scalar(255, 0, 0), 2);
    //cv::circle(drawing_ball2, cv::Point(minEllipse3[0].center.x,minEllipse3[0].center.y), 2, cv::Scalar(255, 0, 0), 6, CV_AA);//画圆函数
    cv::circle(drawing_ball2, cv::Point(200,150), 22, cv::Scalar(0,255, 0), 6, CV_AA);//画圆函数,测试桌子边界
    cv::circle(drawing_ball2, cv::Point(1300,150), 22, cv::Scalar(0,255, 0), 6, CV_AA);//画圆函数,测试桌子边界
    cv::circle(drawing_ball2, cv::Point(1300,650), 22, cv::Scalar(0,255, 0), 6, CV_AA);//画圆函数,测试桌子边界
    std::cout << "障碍物坐标" << "=" << minEllipse3[0].center.x<<","<< minEllipse3[0].center.y<< std::endl;
    std::cout << "障碍物半径" << "=" << minEllipse3[0].size.height<<","<< minEllipse3[0].size.width<< std::endl;

    cv::Point2f zhizhang_point;
    zhizhang_point.x=minEllipse3[0].center.x+50;
    zhizhang_point.y=minEllipse3[0].center.y+260;

//画出轮廓找球结束



//找小球

    //cv::Mat img_hsv(g_srcImage.size(), CV_8U);
    cv::Mat img_filtered2;


    //相片分割
//   cv::Mat  image2(g_srcImage, cv::Rect(cc3-230,cc4-230,460,460));
//    cv::Mat  img_hsv2(img_hsv, cv::Rect(cc3-230,cc4-230,460,460));

    // std::cout<<"分割相片参数x,y,z"<<dish1_x-dish1_r+300<<","<<dish1_y-dish1_r+690<<","<<2*dish1_r<<std::endl;
    //   cv::Mat  image2(g_srcImage, cv::Rect(dish1_x-dish1_r/2+300,dish1_y-dish1_r/2+690,dish1_r,dish1_r));

    //cv::Mat image2(g_srcImage, cv::Rect(300,690,1100,390));

    // cv::Mat image2(g_srcImage, cv::Rect(0,0,1900,1080));
    //  cv::Mat image2(g_srcImage, cv::Rect(100,260,1600,810));//切割出奥博机器人的视觉区域
    cv::Mat image2=g_srcImage2;//切割出奥博机器人的视觉区域,用上面一个
    //cv::Mat image2(g_srcImage, cv::Rect(100,300,1600,740));//切割出奥博机器人的更小视觉区域


    //切割盘子所在区域
    /*
      cv::Mat image4;
     if(minEllipse2[0].center.x-250>0&&minEllipse2[0].center.y-250>0&&minEllipse2[0].center.x+250<1500&&minEllipse2[0].center.y+250<550)
     { cv::Mat image3(g_srcImage, cv::Rect(minEllipse2[0].center.x-250,minEllipse2[0].center.y-250,500,500));//切割出盘子所在的视觉区域
         image4=image3;
     }
     else {
         cv::Mat image3(g_srcImage, cv::Rect(100, 260, 1600, 810));//切割出奥博机器人的视觉区域
         image4=image3;
     }
       cv::Mat image2=image4;*/
    //切割盘子所在区域结束


    //cv::Mat image2(g_srcImage, cv::Rect(300,690,1100,390));

    //cv::Mat  img_hsv2(img_hsv, cv::Rect(cc3-c[2],cc3-c[2],2*c[2],c[2]+1));
    cv::Mat  img_hsv2;
    cv::cvtColor(image2, img_hsv2, CV_BGR2HSV);
    //cv::cvtColor()用于将图像从一个颜色空间转换到另一个颜色空间的转换（目前常见的颜色空间均支持），并且在转换的过程中能够保证数据的类型不变，即转换后的图像的数据类型和位深与源图像一致。


    //   cv::inRange(img_hsv, cv::Scalar(150, 180, 100), cv::Scalar(255, 255, 255), img_filtered);
    //cv::inRange(img_hsv2, cv::Scalar(0, 0, 221), cv::Scalar(180, 30, 255), img_filtered2);//ybr：分隔白色
    // cv::inRange(img_hsv2, cv::Scalar(125, 43, 46), cv::Scalar(155, 255, 255), img_filtered2);//ybr：分割紫色色
    // cv::inRange(img_hsv2, cv::Scalar(0, 0, 46), cv::Scalar(180, 43, 220, 220),img_filtered2);//ybr：分割灰色
//    cv::inRange(img_hsv2, cv::Scalar(100, 43, 46), cv::Scalar(124, 255, 255),img_filtered2);//ybr：分割蓝色
//     cv::inRange(img_hsv2, cv::Scalar(35, 43, 46), cv::Scalar(77, 255, 255),img_filtered2);//ybr：分割绿色色，绿色球不行
    cv::inRange(img_hsv2, cv::Scalar(24, 43, 46), cv::Scalar(50, 255, 255),img_filtered2);//ybr：修改分割绿色色，绿色球行
//  cv::inRange(img_hsv2, cv::Scalar(24, 43, 46), cv::Scalar(34, 255, 255),img_filtered2);//ybr：分割黄色色
//    cv::inRange(img_hsv2, cv::Scalar(11, 43, 46), cv::Scalar(25, 255, 255),imBallWorldPoi.yg_filtered2);//ybr：分割橙色色
//    cv::inRange(img_hsv2, cv::Scalar(0, 43, 46), cv::Scalar(10, 255, 255),img_filtered2);//ybr：分割红色色
    // cv::inRange(img_hsv2, cv::Scalar(156, 43, 46), cv::Scalar(180, 255, 255),img_filtered2);//ybr：分割红色色，可以用
//    cv::inRange(img_hsv2, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 46),img_filtered2);//ybr：分割黑色色


    //   cv::cvtColor(img_hsv2, img_filtered2, CV_BGR2GRAY);//不分割颜色

//void cvInRange(//提取图像中在阈值中间的部分  const CvArr* src,//目标图像const CvArr* lower,//阈值下限  const CvArr* upper,//阈值上限  CvArr* dst//结果图像  )
//例如#  下阈值        lower_hsv = np.array([37,43,46])        #  上阈值        upper_hsv = np.array([77,255,255])此阈值可以取出摄像头中绿色区域

    // cv::GaussianBlur(img_filtered2, img_filtered2, cv::Size(9, 9), 0, 0);//高斯滤波
    // cv::GaussianBlur(img_filtered2, img_filtered2, cv::Size(5, 5), 1, 1);//高斯滤波效果最好
    cv::GaussianBlur(img_filtered2, img_filtered2, cv::Size(3, 3), 0, 0);//高斯滤波
    //void GaussianBlur(InputArray src, OutputArray dst, Size ksize, double sigmaX, double sigmaY=0, int borderType=BORDER_DEFAULT ) ;
    //参数：src和dst当然分别是输入图像和输出图像。Ksize为高斯滤波器模板大小，sigmaX和sigmaY分别为高斯滤波在横线和竖向的滤波系数（有点晦涩，等下解释）。borderType为边缘点插值类型。

    //   int threshold_value_1 = threshold( img_filtered2, img_filtered2, 10, 255,  CV_THRESH_BINARY);//二值化

//闭运算减小黑色点
    int morph_elem2 = 1;//0-2
    int morph_size2 = 15;//0-21
    Mat element2 = getStructuringElement( morph_elem2, Size( 2*morph_size2 + 1, 2*morph_size2+1 ), Point( morph_size2, morph_size2 ) );
   morphologyEx( img_filtered2, img_filtered2, MORPH_CLOSE, element2 );//闭运算
    //morphologyEx( img_filtered2, img_filtered2, MORPH_OPEN, element2 );//开运算
//闭运算结束


    std::vector<cv::Vec3f> circles2;
    // cv::HoughCircles(img_filtered2, circles2, CV_HOUGH_GRADIENT, 2, 100, 100, 50, 27, 30);//找圆形函数，小木头球的半径3-4cm，地面
    //cv::HoughCircles(img_filtered2, circles2, CV_HOUGH_GRADIENT, 2, 100, 100, 50, 47, 50);//找圆形函数，小木头球的半径3-4cm，离地面的距离25-33cm，变化20像素
    cv::HoughCircles(img_filtered2, circles2, CV_HOUGH_GRADIENT, 2, 100, 100, 50, 15, 65);//找圆形函数，适合大部分情况
    //  cv::HoughCircles(img_filtered2, circles2, CV_HOUGH_GRADIENT, 2, 100, 100, 50, 70, 80);//找盘中小圆


    cv::Vec3i c2;
    cv::Vec3i c3;

    c2[2]=170;
    c2[0]=0;
    c2[1]=0;
    //  c2=circles2[0];
    for (size_t i = 0; i < circles2.size(); i++)
    {

        if(minEllipse2[0].center.x>0&&minEllipse2[0].center.y>0){


            c3 = circles2[i];
            double xxxx=minEllipse2[0].center.x- c3[0];
            double yyyy=minEllipse2[0].center.y- c3[1];

            if(sqrt(xxxx*xxxx+yyyy*yyyy)<240){
                if(c3[2]<c2[2]&&c3[2]>25) c2 =c3;
            }
        }



    }


    cv::Point2f kk;
    double r;

    if(c2[0]>0&&c2[1]>0){
        cv::circle(image2, cv::Point(c2[0], c2[1]), c2[2], cv::Scalar(0, 0, 255), 3, CV_AA);
        cv::circle(image2, cv::Point(c2[0], c2[1]), 2, cv::Scalar(0, 255, 0), 3, CV_AA);//画圆心函数
        kk.x=c2[0];
        kk.y=c2[1];
        r=c2[2];
       // std::cout<<"小球半径="<<c2[2]<<std::endl;

    }
    else    std::cout<<"未找到小球"<<std::endl;


    cv::Point2f ball_point;
    ball_point.x=kk.x+50;
    ball_point.y=kk.y+260;


    std::cout<<"小球半径="<<r<<std::endl;


    std::cout<<"球x="<<ball_point.x<<std::endl;
   std::cout<<"球y="<<ball_point.y<<std::endl;
    std::cout<<"相片行="<<g_srcImage.rows<<std::endl;
    std::cout<<"相片列（长度）="<<g_srcImage.cols<<std::endl;







//找小球结束






  //  BallTracking tracker;

    std::vector<double> Ball(6);
    Ball[0] = BallTracking::PixelToWorld_dish(dish_point).x;
    Ball[1] = BallTracking::PixelToWorld_dish(dish_point).y;


  //补偿盘子高度所造成的误差
//    double H=0.735;
//    double h=0.90;
//    double x1=Ball[0]-0.0925;
//    double y1=Ball[1]-0.115;
//
//    double l=sqrt(x1*x1+y1*y1);
//    double x=l/(H-h);
//    double sin1=x1/l;
//    double cos1=y1/l;
//
//    Ball[0]=Ball[0]+x*sin1;
//    Ball[1]=Ball[1]+x*cos1;


//    cv::Point2f kkybr;
//    kkybr=BallTracking::PixelToWorld_dish(dish_point);
//    std::cout<<"dish_word="<<kkybr.x<<","<<kkybr.y<<std::endl;


    Ball[2] = BallTracking::PixelToWorld_dish(zhizhang_point).x;
    Ball[3] = BallTracking::PixelToWorld_dish(zhizhang_point).y;
    Ball[4] = BallTracking::PixelToWorld_dish(ball_point).x;
    Ball[5] = BallTracking::PixelToWorld_dish(ball_point).y;

    std::cout << "First BALL1: " << Ball[0] << ", " << Ball[1] << ", " << Ball[2] << ", "
              << Ball[3] << ", " << Ball[4] << ", " << Ball[5] << std::endl;





//球
  //  cv::imshow(camerawindow, g_srcImage2);
   cv::imshow(camerawindow, g_cannyMat_output2);
    cv::imshow(hsvwindow, drawing_ball);
    cv::imshow(filteredwindow, drawing_ball2);

    cv::imshow(camerawindow2, image2);
    cv::imshow(hsvwindow2, img_hsv2);
    cv::imshow(filteredwindow2, img_filtered2);

//盘子
    cv::imshow(filteredwindow8, drawing);
    cv::imshow(filteredwindow9, drawing2);

    cv::waitKey(0);


    sleep(5);

    cv::destroyWindow(camerawindow);
    cv::destroyWindow(hsvwindow);
    cv::destroyWindow(filteredwindow);

    cv::destroyWindow(camerawindow2);
    cv::destroyWindow(hsvwindow2);
    cv::destroyWindow(filteredwindow2);

    cv::destroyWindow(filteredwindow9);
    cv::destroyWindow(filteredwindow8);










    return Ball;
   // ros::spin();
  //  return 0;
}


