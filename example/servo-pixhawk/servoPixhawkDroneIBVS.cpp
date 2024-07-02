/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Example that shows how to do visual servoing with a drone equipped with a Pixhawk.
 *
*****************************************************************************/

#include <iostream>

#include <visp3/core/vpConfig.h>

// Check if std:c++17 or higher
#if defined(VISP_HAVE_MAVSDK) && ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L))) && \
  defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_PUGIXML)

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpMomentAreaNormalized.h>
#include <visp3/core/vpMomentBasic.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentGravityCenterNormalized.h>
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpTime.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/robot/vpRobotMavsdk.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureMomentAreaNormalized.h>
#include <visp3/visual_features/vpFeatureMomentGravityCenterNormalized.h>
#include <visp3/visual_features/vpFeatureVanishingPoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

//rtsp server 20240618
#include <opencv2/opencv.hpp>

// Comment next line to disable sending commands to the robot
#define CONTROL_UAV
#define HAVE_DISP_DEV

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

bool compareImagePoint(std::pair<size_t, vpImagePoint> p1, std::pair<size_t, vpImagePoint> p2)
{
  return (p1.second.get_v() < p2.second.get_v());
};

/*!
 * \example servoPixhawkDroneIBVS.cpp
 *
 * Example that shows how to how to achieve an image-based visual servo a drone
 * equipped with a Pixhawk connected to a Jetson TX2. An Intel Realsense camera
 * is also attached to the drone and connected to the Jetson. The drone is localized
 * thanks to Qualisys Mocap. Communication between the Jetson and the Pixhawk
 * is based on Mavlink using MAVSDK 3rd party.
 *
 * This program makes the drone detect and follow an AprilTag from the 36h11 family.
 *
 * \warning this program does no sensing or avoiding of obstacles, the drone
 * will collide with any objects in the way! Make sure the drone has about
 * 3-4 meters of free space around it before starting the program.
 *
 */
int main(int argc, char **argv)
{
  std::cout << "argc: " <<argc<< std::endl;
  std::cout << "argv: " <<argv<< std::endl;

  //rtsp part 1/3
  bool rtsp_enable =false;
  std::string rtsp_server_url = "rtsp://127.0.0.1:554/live/0";
  std::stringstream command;
  command << "ffmpeg ";
  // inputfile options
  command << "-y "  // overwrite output files
  << "-an " // disable audio
  << "-f rawvideo " // force format to rawvideo
  << "-vcodec rawvideo "  // force video rawvideo ('copy' to copy stream)
  << "-pix_fmt gray "  // set pixel format to bgr24 or gray //灰度流和彩色流这个参数要变一变不然会显示异常！！！！！！！！！
  << "-s 640x480 "  // set frame size (WxH or abbreviation)
  << "-r 10 "; // set frame rate (Hz value, fraction or abbreviation)
  command << "-i - ";
  // outputfile options
  command 
  << "-c:v libx264 "  // Hyper fast Audio and Video encoder
  << "-pix_fmt yuv420p "  // set pixel format to yuv420p
  << "-tune:v zerolatency "
  << "-preset ultrafast " // set the libx264 encoding preset to ultrafast
  << "-f rtsp " // force format to flv for rtmp, rtsp for rtsp
  << rtsp_server_url;
  FILE *fp = nullptr;
  bool rtsp_flag = true;
  try {
    std::string opt_connecting_info = "udp://:14550";
    double tagSize = -1;//tag黑色方框的尺寸，单位m
    double opt_distance_to_tag = -1;//相机中心到tag的期望距离,单位m
    bool opt_has_distance_to_tag = false;
    int opt_display_fps = 10;//显示结果图的频率，决定condition
    bool opt_verbose = false;//是否打印详细信息

    int acq_fps = 30;//图像的获取频率

    if (argc >= 3 && std::string(argv[1]) == "--tag-size") {
      tagSize = std::atof(argv[2]); // Tag size option is required
      if (tagSize <= 0) {
        std::cout << "Error : invalid tag size." << std::endl << "See " << argv[0] << " --help" << std::endl;
        return EXIT_FAILURE;
      }
      for (int i = 3; i < argc; i++) {
        if (std::string(argv[i]) == "--co" && i + 1 < argc) {
          opt_connecting_info = std::string(argv[i + 1]);
          i++;
        }
        else if (std::string(argv[i]) == "--distance-to-tag" && i + 1 < argc) {
          opt_distance_to_tag = std::atof(argv[i + 1]);
          if (opt_distance_to_tag <= 0) {
            std::cout << "Error : invalid distance to tag." << std::endl << "See " << argv[0] << " --help" << std::endl;
            return EXIT_FAILURE;
          }
          opt_has_distance_to_tag = true;
          i++;

        }
        else if (std::string(argv[i]) == "--display-fps" && i + 1 < argc) {
          opt_display_fps = std::stoi(std::string(argv[i + 1]));
          i++;
        }
        else if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
          opt_verbose = true;
        }
        else {
          std::cout << "Error : unknown parameter " << argv[i] << std::endl
            << "See " << argv[0] << " --help" << std::endl;
          return EXIT_FAILURE;
        }
      }
    }
    else if (argc >= 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
      std::cout << "\nUsage:\n"
        << "  " << argv[0]
        << " [--tag-size <tag size [m]>] [--co <connection information>] [--distance-to-tag <distance>]"
        << " [--display-fps <display fps>] [--verbose] [-v] [--help] [-h]\n"
        << std::endl
        << "Description:\n"
        << "  --tag-size <size>\n"
        << "      The size of the tag to detect in meters, required.\n\n"
        << "  --co <connection information>\n"
        << "      - UDP: udp://[host][:port]\n"
        << "      - TCP: tcp://[host][:port]\n"
        << "      - serial: serial://[path][:baudrate]\n"
        << "      - Default: udp://192.168.30.111:14552).\n\n"
        << "  --distance-to-tag <distance>\n"
        << "      The desired distance to the tag in meters (default: 1 meter).\n\n"
        << "  --display-fps <display_fps>\n"
        << "      The desired fps rate for the video display (default: 10 fps).\n\n"
        << "  --verbose, -v\n"
        << "      Enables verbosity (drone information messages and velocity commands\n"
        << "      are then displayed).\n\n"
        << "  --help, -h\n"
        << "      Print help message.\n"
        << std::endl;
      return EXIT_SUCCESS;

    }
    else {
      std::cout << "Error : tag size parameter required." << std::endl << "See " << argv[0] << " --help" << std::endl;
      return EXIT_FAILURE;
    }

    std::cout << std::endl
      << "WARNING:" << std::endl
      << " - This program does no sensing or avoiding of obstacles, " << std::endl//这里例程没有避障功能请确保安全飞行！
      << "   the drone WILL collide with any objects in the way! Make sure the " << std::endl
      << "   drone has approximately 3 meters of free space on all sides." << std::endl
      << " - The drone uses a forward-facing camera for Apriltag detection," << std::endl
      << "   make sure the drone flies  above a non-uniform flooring," << std::endl
      << "   or its movement will be inacurate and dangerous !" << std::endl
      << std::endl;

    // Connect to the drone
    vpRobotMavsdk drone(opt_connecting_info);//新建一个udp,tcp连接

    if (drone.isRunning()) {//连接成功
      vpRealSense2 rs;

      std::string product_line = rs.getProductLine();//获取相机信息
      if (opt_verbose) {
        std::cout << "Product line: " << product_line << std::endl;
      }

      if (product_line == "T200") {
        std::cout << "This example doesn't support T200 product line family !" << std::endl;
        return EXIT_SUCCESS;
      }
      rs2::config config;

      config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, acq_fps);//配置相机
      config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, acq_fps);
      config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, acq_fps);

      rs.open(config);//启动相机
      vpCameraParameters cam = rs.getCameraParameters(RS2_STREAM_COLOR);//获取相机 内参

      if (opt_verbose) {
        cam.printParameters();
      }

#ifdef CONTROL_UAV//这个宏定义是本例程中所有控制飞机指令的 使能
      drone.doFlatTrim(); // Flat trim calibration  //自动校正加速度计和陀螺仪，应该禁用掉！！！！！！！！！
      drone.takeOff(false,15,true); // Take off  //起飞，takeOff包含了：guided -> 解锁 -> 起飞 -> 位置保持！！！！！！！！！
#endif

      vpImage<unsigned char> I(rs.getIntrinsics(RS2_STREAM_COLOR).height, rs.getIntrinsics(RS2_STREAM_COLOR).width);//定义用于二维码检测的 灰度图

#if defined(VISP_HAVE_X11)
      vpDisplayX display;
#elif defined(VISP_HAVE_GTK)
      vpDisplayGTK display;
#elif defined(VISP_HAVE_GDI)
      vpDisplayGDI display;
#elif defined(HAVE_OPENCV_HIGHGUI)
      vpDisplayOpenCV display;//定义用于在屏幕显示结果图像的方式：GTK、X11、opencv
#endif

#if defined(HAVE_DISP_DEV)        
      int orig_displayX = 100;//结果图像的【尺寸】
      int orig_displayY = 100;
      display.init(I, orig_displayX, orig_displayY, "DRONE VIEW");
      vpDisplay::display(I);
      vpDisplay::flush(I);
      vpPlot plotter(1, 700, 700, orig_displayX + static_cast<int>(I.getWidth()) + 20, orig_displayY,"Visual servoing tasks");//定义一个绘画工具
#endif
      double time_since_last_display = vpTime::measureTimeMs();//上次显示结果图的 时间
      unsigned int iter = 0;

      //! [DJI-F450 apriltag family]
      vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;//tag图案类型
      vpDetectorAprilTag detector(tagFamily); // The detector used to detect Apritags  //定义一个检测器
      //! [DJI-F450 apriltag family]
      detector.setAprilTagQuadDecimate(2.0);//这是tag检测算法中与【4边形识别】相关的一个参数，增加这个参数可以提高检测速率，但是准确率会下降！！！！！！！！！！！！！！！4
      detector.setAprilTagNbThreads(4);//为检测器设置 线程数
      detector.setDisplayTag(true);//是否显示检测结果

      vpServo task; // Visual servoing task  //定义一个【视觉伺服】任务

      // double lambda = 0.5;
      vpAdaptiveGain lambda = vpAdaptiveGain(1.5, 0.7, 30);//xyz维度各自的lambda值，是IBVS理论中的一个参数！！！！！！！！！
      task.setServo(vpServo::EYEINHAND_L_cVe_eJe);//设置视觉伺服系统为 eye in hand 模式，这个模式需要提前知道cVe，eJe矩阵的值！
      task.setInteractionMatrixType(vpServo::CURRENT);//设置IBVS中雅可比矩阵J的计算方式为:CURRENT，矩阵J的计算方式可以参考我的博客
      task.setLambda(lambda);

      //! [DJI-F450 cMe]
      /*
       * In the following section, (c1) is an intermediate frame attached to the camera
       * that has axis aligned with the FLU body frame. The real camera frame is denoted (c).
       * The FLU body-frame of the drone denoted (e) is the one in which we need to convert
       * every velocity command computed by visual servoing.
       *
       * We can easily estimate the homogeneous matrix between (c1) and (c) where
       * in our case we have -10 degrees around X camera axis.
       * Then for the transformation between (e) and (c1) frames we can consider only translations.
       *
       * Using those matrices, we can in the end obtain the homogeneous matrix between (c) and (e) frames.
       * This homogeneous matrix is then used to compute the velocity twist matrix cVe.
       */
      vpRxyzVector c1_rxyz_c(vpMath::rad(-10.0), vpMath::rad(0), 0);
      vpRotationMatrix c1Rc(c1_rxyz_c); // Rotation between (c1) and (c)  //c->c1的3d空间坐标旋转矩阵
      vpHomogeneousMatrix c1Mc(vpTranslationVector(), c1Rc); // Homogeneous matrix between (c1) and (c) //c->c1的2d图像(平面)坐标旋转矩阵，即单应矩阵

      vpRotationMatrix c1Re { 1, 0, 0, 0, 0, 1, 0, -1, 0 };     // Rotation between (c1) and (e)  //c1和e的旋转关系
      vpTranslationVector c1te(0, -0.03, -0.07);             // Translation between (c1) and (e)  //c1和e的平移关系，
      vpHomogeneousMatrix c1Me(c1te, c1Re);                  // Homogeneous matrix between (c1) and (e)  //c1与e的单应矩阵

      vpHomogeneousMatrix cMe = c1Mc.inverse() * c1Me;       // Homogeneous matrix between (c) and (e)  //c与e的单应矩阵

      vpVelocityTwistMatrix cVe(cMe);  //伺服系统控制量V的坐标转换矩阵：c -> e，从相机系到FLU系！！而为什么通过视觉伺服计算得到的ve却是飞机机体系FRD下的量？？why？？？？？
      //! [DJI-F450 cMe]
      task.set_cVe(cVe);

      vpMatrix eJe(6, 4, 0);//哪些速度自由度是可控的！！！！！！！！！！！！

      eJe[0][0] = 1;//vx
      eJe[1][1] = 1;//vy
      eJe[2][2] = 1;//vz
      eJe[5][3] = 1;//wz
      
      // 打印cVe，eJe
      /*
        cVe: 
        1  0  0  0  0.03  0.07
        0  0.1736481777  0.984807753  -0.07414598804  0  0
        0  -0.984807753  0.1736481777  0.01738886015  0  0
        0  0  0  1  0  0
        0  0  0  0  0.1736481777  0.984807753
        0  0  0  0  -0.984807753  0.1736481777
        eJe: 
        1  0  0  0
        0  1  0  0
        0  0  1  0
        0  0  0  0
        0  0  0  0
        0  0  0  1
       */

      // Desired distance to the target
      double Z_d = (opt_has_distance_to_tag ? opt_distance_to_tag : 1.); //相机到二维码的 期望控制距离！！！！！！！！！！

      // Define the desired polygon corresponding the the AprilTag CLOCKWISE  //定义一个以 4个点表示的顺时针包裹 AprilTag 的多边形
      double X[4] = { tagSize / 2., tagSize / 2., -tagSize / 2., -tagSize / 2. }; //定义4个【相机坐标系下的视觉特征期望值】，Z轴的值无需！！！！！！！！！！！
      double Y[4] = { tagSize / 2., -tagSize / 2., -tagSize / 2., tagSize / 2. };
      std::vector<vpPoint> vec_P, vec_P_d;
      vpHomogeneousMatrix cdMo(0, 0, Z_d, 0, 0, 0);//定义相机坐标系与检测目标(AprilTag)机体系的期望位置关系（坐标转换矩阵、单应矩阵）！！！！
      for (int i = 0; i < 4; i++) {
        vpPoint P_d(X[i], Y[i], 0);//相机坐标系下的视觉特征期望值！！！！！！
        P_d.track(cdMo); //根据【相机坐标系下的视觉特征期望值】和 cdMo 计算得到【图像坐标系下的视觉特征期望值】，结果保存在 P_d 的祖父类 pTracker 的 cP 中！！！！！！！！！
        vec_P_d.push_back(P_d);//视觉特征期望值向量！！！！！！！！！！
      }
      // 关于visp中矩的使用描述请参考类vpMomentObject中的注释和例子
      vpMomentObject m_obj(3), m_obj_d(3);//定义一个用于计算图像矩的对象
      vpMomentDatabase mdb, mdb_d;//vpMomentDatabase的作用：Sometimes, a moment needs to have access to other moment's values to be computed. vpMomentDatabase offer ways to accessed other moments.
      vpMomentBasic mb_d; // Here only to get the desired area m00  //Basic矩即m00，目标区域的面积，对于连续区域则为surface，对于离散区域则为点的个数(number of discrete points)
      vpMomentGravityCenter mg, mg_d;//用于保存通过图像矩计算得到的【重心】
      vpMomentCentered mc, mc_d;//用于保存通过图像矩计算得到的【几何中心】
      vpMomentAreaNormalized man(0, Z_d), man_d(0, Z_d); // Declare normalized area updated below with m00  //目标区域的【归一化面积】
      vpMomentGravityCenterNormalized mgn, mgn_d;        // Declare normalized gravity center  //目标区域的【归一化重心】

      // Desired moments
      m_obj_d.setType(vpMomentObject::DENSE_POLYGON); // Consider the AprilTag as a polygon  //初始化vpMomentObject的方式设置为DENSE_POLYGON
      m_obj_d.fromVector(vec_P_d);                    // Initialize the object with the points coordinates   //使用vec_P_d中的点坐标来初始化vpMomentObject

      mb_d.linkTo(mdb_d);       // Add basic moments to database  //将m00分享到database。add到database的这些矩用于计算其他矩
      mg_d.linkTo(mdb_d);       // Add gravity center to database  
      mc_d.linkTo(mdb_d);       // Add centered moments to database  
      man_d.linkTo(mdb_d);      // Add area normalized to database  
      mgn_d.linkTo(mdb_d);      // Add gravity center normalized to database
      mdb_d.updateAll(m_obj_d); // All of the moments must be updated, not just an_d  //计算得到【期望视觉特征向量】的【m00】：以4个期望视觉特征点组成的图像计算得到的m00，【期望m00】
      mg_d.compute();           // Compute gravity center moment  //计算得到【期望视觉特征向量】的【重心矩】：以4个期望视觉特征点组成的图像计算得到的重心，【期望重心】
      mc_d.compute();           // Compute centered moments AFTER gravity center  //计算得到【期望视觉特征向量】的【几何中心矩】：以4个期望视觉特征点组成的图像计算得到的几何中心，【期望几何中心】

      double area = 0;//【期望面积】
      if (m_obj_d.getType() == vpMomentObject::DISCRETE)
        area = mb_d.get(2, 0) + mb_d.get(0, 2);
      else
        area = mb_d.get(0, 0);
      // Update moment with the desired area
      man_d.setDesiredArea(area);

      man_d.compute(); // Compute area normalized moment AFTER area //【期望归一化面积】
      mgn_d.compute(); // Compute gravity center normalized moment AFTER area normalized  //【期望归一化重心】
                       // moment

      // Desired plane  //在相机坐标系下，定义一个期望平面，用于构造【图像矩视觉特征】
      double A = 0.0; //空间平面方程Ax+By+C=1/z的3个系数ABC
      double B = 0.0;
      double C = 1.0 / Z_d;//相机坐标系下的期望平面！！！！！！！！！

      // Construct area normalized features  //基于图像矩的视觉伺服？【图像矩视觉特征】！！！！！！！！
      vpFeatureMomentGravityCenterNormalized s_mgn(mdb, A, B, C), s_mgn_d(mdb_d, A, B, C);//构造 图像归一化重心矩视觉特征！！！！！！！
      vpFeatureMomentAreaNormalized s_man(mdb, A, B, C), s_man_d(mdb_d, A, B, C);//构造 图像归一化目标区域面积矩视觉特征！！！！！！！！
      vpFeatureVanishingPoint s_vp, s_vp_d;//定义【灭点】视觉特征，赋值请看下面的vpFeatureBuilder::create()，Vanishing Point即灭点，是透视投影中中的一个概念！自行百度。

      // Add the features  //添加视觉特征到视觉伺服任务
      task.addFeature(s_mgn, s_mgn_d);//重心矩视觉特征
      task.addFeature(s_man, s_man_d);//面积矩视觉特征
      task.addFeature(s_vp, s_vp_d, vpFeatureVanishingPoint::selectAtanOneOverRho());//vanishing点视觉特征
#if defined(HAVE_DISP_DEV)  
      plotter.initGraph(0, 4);
      plotter.setLegend(0, 0, "Xn");          // Distance from center on X axis feature  //重心矩视觉特征的x分量误差！！！！！
      plotter.setLegend(0, 1, "Yn");          // Distance from center on Y axis feature  //重心矩视觉特征的y分量误差！！！！！
      plotter.setLegend(0, 2, "an");          // Tag area feature  //目标区域面积视觉特征误差！！！！！
      plotter.setLegend(0, 3, "atan(1/rho)"); // Vanishing point feature //灭点视觉特征误差！！！！！
#endif
      // Update desired gravity center normalized feature
      s_mgn_d.update(A, B, C);
      s_mgn_d.compute_interaction();
      // Update desired area normalized feature
      s_man_d.update(A, B, C);
      s_man_d.compute_interaction();

      // Update desired vanishing point feature for the horizontal line  //期望直线视觉特征！！！！！！！！！！！！
      s_vp_d.setAtanOneOverRho(0);//期望灭点的参数，可看作期望直线的偏置为0
      s_vp_d.setAlpha(0);//期望灭点的参数，可看作期望直线的水平角度为0

      bool condition;
      bool runLoop = true;
      bool vec_ip_has_been_sorted = false;
      bool send_velocities = false;
      std::vector<std::pair<size_t, vpImagePoint> > vec_ip_sorted;

      //** Visual servoing loop **//
      while (drone.isRunning() && runLoop) {

        double startTime = vpTime::measureTimeMs();

        // drone.getGrayscaleImage(I);
        rs.acquire(I);//获取当前图像
        condition = (startTime - time_since_last_display) > 1000. / opt_display_fps ? true : false;
        if (condition) {
#if defined(HAVE_DISP_DEV)  
          vpDisplay::display(I);
#endif
          time_since_last_display = vpTime::measureTimeMs();
        }
        std::vector<vpHomogeneousMatrix> cMo_vec;
        detector.detect(I, tagSize, cam, cMo_vec); // Detect AprilTags in current image  //使用当前帧检测AprilTags的位姿，结果保存在cMo_vec！！！！！！！！！！！！
#if defined(HAVE_DISP_DEV)  
        if (condition) {
          double t = vpTime::measureTimeMs() - startTime;
          std::stringstream ss;
          ss << "Detection time: " << t << " ms";
          vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
        }
#endif
        //主要循环逻辑
        if (detector.getNbObjects() != 0) {//检测到AprilTags！！！！！！！！！！！

          // Update current points used to compute the moments
          std::vector<vpImagePoint> vec_ip = detector.getPolygon(0);
          vec_P.clear();
          for (size_t i = 0; i < vec_ip.size(); i++) { // size = 4
            double x = 0, y = 0;
            vpPixelMeterConversion::convertPoint(cam, vec_ip[i], x, y);
            vpPoint P;
            P.set_x(x);
            P.set_y(y);
            vec_P.push_back(P);//当前帧的目标轮廓点，用于计算当前祯图像矩！！！！！！！！！！
          }

          // Current moments  //计算当前帧的图像矩，用于更新图像矩视觉特征！！！！！！！！！
            m_obj.setType(vpMomentObject::DENSE_POLYGON); // Consider the AprilTag as a polygon
            m_obj.fromVector(vec_P);                      // Initialize the object with the points coordinates

            mg.linkTo(mdb);           // Add gravity center to database
            mc.linkTo(mdb);           // Add centered moments to database
            man.linkTo(mdb);          // Add area normalized to database
            mgn.linkTo(mdb);          // Add gravity center normalized to database
            mdb.updateAll(m_obj);     // All of the moments must be updated, not just an_d
            mg.compute();             // Compute gravity center moment
            mc.compute();             // Compute centered moments AFTER gravity center
            man.setDesiredArea(area); // Desired area was init at 0 (unknow at construction),
                                      // need to be updated here
            man.compute();            // Compute area normalized moment AFTER centered moment
            mgn.compute();            // Compute gravity center normalized moment AFTER area normalized
                                      // moment

            s_mgn.update(A, B, C);
            s_mgn.compute_interaction();
            s_man.update(A, B, C);
            s_man.compute_interaction();

          /* Sort points from their height in the image, and keep original indexes.  //根据点在图像坐标系中的高度(height)将二维码轮廓点vec_ip重新排序，保存在到 vec_ip_sorted
          This is done once, in order to be independent from the orientation of the tag
          when detecting vanishing points. */
          if (!vec_ip_has_been_sorted) {
            for (size_t i = 0; i < vec_ip.size(); i++) {

              // Add the points and their corresponding index
              std::pair<size_t, vpImagePoint> index_pair = std::pair<size_t, vpImagePoint>(i, vec_ip[i]);
              vec_ip_sorted.push_back(index_pair);
            }

            // Sort the points and indexes from the v value of the points
            std::sort(vec_ip_sorted.begin(), vec_ip_sorted.end(), compareImagePoint);

            vec_ip_has_been_sorted = true;//仅进行一次，后续都使用第一祯(第一次)的排序结果！
          }

          // Use the two highest points for the first line, and the two others for the second line.  //构建灭点视觉特征，当前祯的【灭点视觉特征】！！！！！！！！！！
          vpFeatureBuilder::create(s_vp, cam, vec_ip[vec_ip_sorted[0].first], vec_ip[vec_ip_sorted[1].first],
                                   vec_ip[vec_ip_sorted[2].first], vec_ip[vec_ip_sorted[3].first],
                                   vpFeatureVanishingPoint::selectAtanOneOverRho());

          task.set_cVe(cVe);//相机坐标系到飞机body系(FRD)的控制量转换矩阵
          task.set_eJe(eJe);//对于eye-in-hand，eJe是单位阵

          // Compute the control law. Velocities are computed in the mobile robot reference frame
          vpColVector ve = task.computeControlLaw();//根据IBVS理论计算得到飞机的6D速度控制量[u,v,w,p,q,r]
          if (!send_velocities) {
            ve = 0;
          }

          // Sending the control law to the drone
          if (opt_verbose) {
            std::cout << "ve: " << ve.t() << std::endl;
          }

#ifdef CONTROL_UAV
          drone.setVelocity(ve);//发送控制飞机速度的指令，ve是飞机body坐标系FRD下的速度。4dim，vx,vy,vz,wz. ！！！！！！！！！！！！！！
#endif
#if defined(HAVE_DISP_DEV)  
          if (condition) {// 显示目标轮廓多边形、轮廓点、灭点直线、垂直和水平线(期望重心)
            for (size_t i = 0; i < 4; i++) {
              vpDisplay::displayCross(I, vec_ip[i], 15, vpColor::red, 1);
              std::stringstream ss;
              ss << i;
              vpDisplay::displayText(I, vec_ip[i] + vpImagePoint(15, 15), ss.str(), vpColor::green);
            }

            // Display visual features
            vpDisplay::displayPolygon(I, vec_ip, vpColor::green,3); // Current polygon used to compute an moment  //显示轮廓多边形
            vpDisplay::displayCross(I, detector.getCog(0), 15, vpColor::green,3); // Current polygon used to compute a moment  //显示轮廓点，十字
            vpDisplay::displayLine(I, 0, static_cast<int>(cam.get_u0()), static_cast<int>(I.getHeight()) - 1,static_cast<int>(cam.get_u0()), vpColor::red,3); // Vertical line as desired x position
            vpDisplay::displayLine(I, static_cast<int>(cam.get_v0()), 0, static_cast<int>(cam.get_v0()),static_cast<int>(I.getWidth()) - 1, vpColor::red,3); // Horizontal line as desired y position

            // Display lines corresponding to the vanishing point for the horizontal lines  //显示与灭点对应的直线！！
            vpDisplay::displayLine(I, vec_ip[vec_ip_sorted[0].first], vec_ip[vec_ip_sorted[1].first], vpColor::red, 1,false);
            vpDisplay::displayLine(I, vec_ip[vec_ip_sorted[2].first], vec_ip[vec_ip_sorted[3].first], vpColor::red, 1,false);
          }
#endif
        }
        else {//没有检测到任何AprilTags
          std::stringstream sserr;
          sserr << "Failed to detect an Apriltag, or detected multiple ones";
#if defined(HAVE_DISP_DEV)  
          if (condition) {
            vpDisplay::displayText(I, 120, 20, sserr.str(), vpColor::red);
            vpDisplay::flush(I);
          }
#endif
          std::cout << sserr.str() << std::endl;
          
#ifdef CONTROL_UAV
          drone.stopMoving(); // In this case, we stop the drone  //没有检测到AprilTags则停止飞行防止时空！！
#endif
        }
        //主要循环逻辑 end
#if defined(HAVE_DISP_DEV)  
        if (condition) {//绘制误差曲线！！！！！！！！！！！！！！
          {
            std::stringstream ss;
            ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot")
              << ", right click to quit.";
            vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);
          }
          vpDisplay::flush(I);
          plotter.plot(0, iter, task.getError());//绘制误差曲线！！！！！
        }
#endif
        vpMouseButton::vpMouseButtonType button;
        if (vpDisplay::getClick(I, button, false)) {//鼠标事件！！！！！！！！！！！
          switch (button) {
          case vpMouseButton::button1:
            send_velocities = !send_velocities;
            break;

          case vpMouseButton::button3:
            drone.land();
            runLoop = false;
            break;

          default:
            break;
          }
        }

        double totalTime = vpTime::measureTimeMs() - startTime;
        std::stringstream sstime;
        sstime << "Total time: " << totalTime << " ms";
        if (condition) {//显示用时、rtsp推流
#if defined(HAVE_DISP_DEV)  
          vpDisplay::displayText(I, 80, 20, sstime.str(), vpColor::red);
          vpDisplay::flush(I);
#endif
          //rtsp part 2/3
          if(rtsp_enable)
          {
            if(rtsp_flag)//只执行一次
            {
              // 在子进程中调用 ffmpeg 进行推流
              fp = popen(command.str().c_str(), "w");
              rtsp_flag=false;//让popen只执行第一次
              if (fp != nullptr) 
                std::cout << "fp open success!" << std::endl;
              else 
              {
                std::cout << "fp open fail!" << std::endl;
                pclose(fp);
              }
            }
            else
            {
              if (fp != nullptr)
              {
	              cv::Mat frame;
                vpImageConvert::convert(I, frame);
                if(frame.empty()) continue;
                fwrite(frame.data, sizeof(char), frame.total() * frame.elemSize(), fp);
              }
              else
              {
                std::cout << "fp open fail..." << std::endl;
                pclose(fp);
              }
            }
          }
        }
        iter++;
        vpTime::wait(startTime, 1000. / acq_fps);
      }

      return EXIT_SUCCESS;
    }
    else {
      std::cout << "ERROR : failed to setup drone control." << std::endl;
      return EXIT_FAILURE;
    }
  }
  catch (const vpException &e) {
    pclose(fp);//rtsp part 3/3
    std::cout << "Caught an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}

#else

int main()
{
#ifndef VISP_HAVE_MAVSDK
  std::cout << "\nThis example requires mavsdk library. You should install it, configure and rebuid ViSP.\n" << std::endl;
#endif
#ifndef VISP_HAVE_REALSENSE2
  std::cout << "\nThis example requires librealsense2 library. You should install it, configure and rebuid ViSP.\n" << std::endl;
#endif
#if !defined(VISP_HAVE_PUGIXML)
  std::cout << "\nThis example requires pugixml built-in 3rdparty." << std::endl;
#endif
#if !((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))
  std::cout
    << "\nThis example requires at least cxx17. You should enable cxx17 during ViSP configuration with cmake and "
    "rebuild ViSP.\n"
    << std::endl;
#endif
  return EXIT_SUCCESS;
}

#endif // #if defined(VISP_HAVE_MAVSDK)
