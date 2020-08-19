//
// Created by cobot on 19-8-6.
//
#include "balltracking.h"
#include "aubo_robot_driver/aubo_robot_driver.h"
std::shared_ptr<DualAuboDriver> dualAuboDriverPtr;
const bool use_cobotsys_sim = false;

int main(int argc, char** argv){
   // ros::init(argc, argv, "balltracking_test");


     BallTracking ballTracking;
    //BallTracking initCaliMatrix;


  //  ballTracking.ImageCapture();
    //initCaliMatrix.ImageCapture();

    std::vector<double> Wrench(6);
    std::vector<double> WrenchInit(6);

    for(int i=0; i<100; i++){
        WrenchInit = ballTracking.GetBall();
       // WrenchInit = initCaliMatrix.GetBall();
        std::cout << "First BALL: " << WrenchInit[0] << ", " << WrenchInit[1] << ", " << WrenchInit[2] << ", "
                  << WrenchInit[3] << ", " << WrenchInit[4] << ", " << WrenchInit[5] << std::endl;

        sleep(5);
    }







//    QApplication a(argc, argv);
//    dualAuboDriverPtr = std::make_shared<DualAuboDriver>(false);//new DualAuboDriver(false)
//    bool connectFlag = dualAuboDriverPtr->ConnectToRobot(argc, argv);
//    if (dualAuboDriverPtr->pFactory) {
//        if (connectFlag) {
//
//           // BallTracking ballTracking(dualAuboDriverPtr);
//            BallTracking initCaliMatrix;
//
//
//           // ballTracking.ImageCapture();
//            initCaliMatrix.ImageCapture();
//
//            std::vector<double> Wrench(6);
//            std::vector<double> WrenchInit(6);
//
//            for(int i=0; i<100; i++){
//               // WrenchInit = ballTracking.GetBall();
//                WrenchInit = initCaliMatrix.GetBall();
//                std::cout << "First BALL: " << WrenchInit[0] << ", " << WrenchInit[1] << ", " << WrenchInit[2] << ", "
//                          << WrenchInit[3] << ", " << WrenchInit[4] << ", " << WrenchInit[5] << std::endl;
//
//                sleep(5);
//            }
//
////    while(1)
////    {
////        Wrench = ballTracking.GetBall();
////        std::cout << "Current BALL: " << Wrench[0]- WrenchInit[0] << ", " << Wrench[1]- WrenchInit[1] << ", " << Wrench[2]- WrenchInit[2] << ", "
////                                        << Wrench[3]- WrenchInit[3] << ", " << Wrench[4]- WrenchInit[4] << ", " << Wrench[5]- WrenchInit[5] << std::endl;
////        usleep(8000);
////    }
//
//        }
//    }
//    a.exec();
    return 0;
}
