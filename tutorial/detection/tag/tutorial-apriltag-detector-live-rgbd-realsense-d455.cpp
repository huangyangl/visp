//! \example tutorial-apriltag-detector-live-rgbd-realsense.cpp
#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vpRealSense2.h>
#endif
//! [Include]
#include <visp3/detection/vpDetectorAprilTag.h>
//! [Include]
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/vision/vpPose.h>

int main(int argc, const char **argv)
{
//! [Macro defined]
#if defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_REALSENSE2)//最外层的宏定义，判断是否安装了 REALSENSE SDK
  //! [Macro defined]
//应用程序参数定义和初始化
  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  double tagSize = 0.053;
  float quad_decimate = 1.0;
  int nThreads = 1;
  bool display_tag = false;
  int color_id = -1;
  unsigned int thickness = 2;
  bool align_frame = false;
  bool opt_verbose = false;

#if !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  bool display_off = true;
  std::cout << "Warning: There is no 3rd party (X11, GDI or openCV) to dislay images..." << std::endl;
#else
  bool display_off = false;
#endif
//应用程序参数定义和初始化end

//输入参数解析
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--pose_method" && i + 1 < argc) {
      poseEstimationMethod = (vpDetectorAprilTag::vpPoseEstimationMethod)atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--tag_size" && i + 1 < argc) {
      tagSize = atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc) {
      quad_decimate = (float)atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--nthreads" && i + 1 < argc) {
      nThreads = atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--display_tag") {
      display_tag = true;
    }
    else if (std::string(argv[i]) == "--display_off") {
      display_off = true;
    }
    else if (std::string(argv[i]) == "--color" && i + 1 < argc) {
      color_id = atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--thickness" && i + 1 < argc) {
      thickness = (unsigned int)atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--tag_family" && i + 1 < argc) {
      tagFamily = (vpDetectorAprilTag::vpAprilTagFamily)atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--z_aligned") {
      align_frame = true;
    }
    else if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
      opt_verbose = true;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
        << " [--tag_size <tag_size in m> (default: 0.053)]"
        " [--quad_decimate <quad_decimate> (default: 1)]"
        " [--nthreads <nb> (default: 1)]"
        " [--pose_method <method> (0: HOMOGRAPHY, 1: HOMOGRAPHY_VIRTUAL_VS, "
        " 2: DEMENTHON_VIRTUAL_VS, 3: LAGRANGE_VIRTUAL_VS, "
        " 4: BEST_RESIDUAL_VIRTUAL_VS, 5: HOMOGRAPHY_ORTHOGONAL_ITERATION) (default: 0)]"
        " [--tag_family <family> (0: TAG_36h11, 1: TAG_36h10 (DEPRECATED), 2: TAG_36ARTOOLKIT (DEPRECATED),"
        " 3: TAG_25h9, 4: TAG_25h7 (DEPRECATED), 5: TAG_16h5, 6: TAG_CIRCLE21h7, 7: TAG_CIRCLE49h12,"
        " 8: TAG_CUSTOM48h12, 9: TAG_STANDARD41h12, 10: TAG_STANDARD52h13) (default: 0)]"
        " [--display_tag] [--z_aligned]";
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
      std::cout << " [--display_off] [--color <color id>] [--thickness <line thickness>]";
#endif
      std::cout << " [--verbose,-v] [--help,-h]" << std::endl;
      return EXIT_SUCCESS;
    }
  }
//输入参数解析end

//主要逻辑代码
  try {
    //! [Construct grabber]
    std::cout << "Use Realsense 2 grabber" << std::endl;
    vpRealSense2 g;
    rs2::config config;
    unsigned int width = 640, height = 480;//设置图片像素？？？？？？
    config.enable_stream(RS2_STREAM_COLOR, static_cast<int>(width), static_cast<int>(height), RS2_FORMAT_RGBA8, 30);
    config.enable_stream(RS2_STREAM_DEPTH, static_cast<int>(width), static_cast<int>(height), RS2_FORMAT_Z16, 30);
    config.enable_stream(RS2_STREAM_INFRARED, static_cast<int>(width), static_cast<int>(height), RS2_FORMAT_Y8, 30);

    vpImage<unsigned char> I;//灰度图像！！！
    vpImage<vpRGBa> I_color(height, width);//rgba图像！！！a表示透明度，0为完全透明255为不透明！
    vpImage<uint16_t> I_depth_raw(height, width);
    vpImage<vpRGBa> I_depth;//深度rgb图！！！为什么要定义为vpRGBa类型？？为了：用一个rgb图像来反映深度的大小，例如距离相机近的点呈现红色，远的点则呈现蓝色，看起来更直观！！

    g.open(config);
    const float depth_scale = g.getDepthScale();
    std::cout << "I_color: " << I_color.getWidth() << " " << I_color.getHeight() << std::endl;
    std::cout << "I_depth_raw: " << I_depth_raw.getWidth() << " " << I_depth_raw.getHeight() << std::endl;

    //sdk 获取图像保存在I_color，I_depth_raw
    rs2::align align_to_color = RS2_STREAM_COLOR;
    g.acquire(reinterpret_cast<unsigned char *>(I_color.bitmap), reinterpret_cast<unsigned char *>(I_depth_raw.bitmap),
              nullptr, nullptr, &align_to_color);

    std::cout << "Read camera parameters from Realsense device" << std::endl;
    vpCameraParameters cam;
    cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);//获取相机参数、包括内参
    //! [Construct grabber]

    std::cout << cam << std::endl;
    std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl;
    std::cout << "tagFamily: " << tagFamily << std::endl;
    std::cout << "nThreads : " << nThreads << std::endl;
    std::cout << "Z aligned: " << align_frame << std::endl;

    vpImage<vpRGBa> I_color2 = I_color;
    vpImage<float> depthMap;//注意与I_depth_raw的区别，一个是float类型，一个是uint16 !!
    vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);//生成深度的rgb图I_depth，远近判断更直观！近点呈红色，远点呈蓝色！

    vpDisplay *d1 = nullptr;
    vpDisplay *d2 = nullptr;
    vpDisplay *d3 = nullptr;
    if (!display_off) {
#ifdef VISP_HAVE_X11
      d1 = new vpDisplayX(I_color, 100, 30, "Pose from Homography");
      d2 = new vpDisplayX(I_color2, I_color.getWidth() + 120, 30, "Pose from RGBD fusion");
      d3 = new vpDisplayX(I_depth, 100, I_color.getHeight() + 70, "Depth");
#elif defined(VISP_HAVE_GDI)
      d1 = new vpDisplayGDI(I_color, 100, 30, "Pose from Homography");
      d2 = new vpDisplayGDI(I_color2, I_color.getWidth() + 120, 30, "Pose from RGBD fusion");
      d3 = new vpDisplayGDI(I_depth, 100, I_color.getHeight() + 70, "Depth");
#elif defined(HAVE_OPENCV_HIGHGUI)
      d1 = new vpDisplayOpenCV(I_color, 100, 30, "Pose from Homography");
      d2 = new vpDisplayOpenCV(I_color2, I_color.getWidth() + 120, 30, "Pose from RGBD fusion");
      d3 = new vpDisplayOpenCV(I_depth, 100, I_color.getHeight() + 70, "Depth");
#endif
    }

    //! [Create AprilTag detector]
    vpDetectorAprilTag detector(tagFamily);
    //! [Create AprilTag detector]

    //! [AprilTag detector settings]  //检测器的参数设置！！！！！
    detector.setAprilTagQuadDecimate(quad_decimate);
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setAprilTagNbThreads(nThreads);
    detector.setDisplayTag(display_tag, color_id < 0 ? vpColor::none : vpColor::getColor(color_id), thickness);//显示？？？？
    detector.setZAlignedWithCameraAxis(align_frame);
    //! [AprilTag detector settings]
    std::vector<double> time_vec;

    //核心死循环：持续检测
    for (;;) {
      double t = vpTime::measureTimeMs();

      //! [Acquisition]
      g.acquire(reinterpret_cast<unsigned char *>(I_color.bitmap),
                reinterpret_cast<unsigned char *>(I_depth_raw.bitmap), nullptr, nullptr, &align_to_color);
      //! [Acquisition]

      I_color2 = I_color;
      vpImageConvert::convert(I_color, I);
      vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

      depthMap.resize(I_depth_raw.getHeight(), I_depth_raw.getWidth());

//并行计算
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
//并行计算end

      //计算深度图: 得到每个像素点的深度值
      for (int i = 0; i < static_cast<int>(I_depth_raw.getHeight()); i++) {
        for (int j = 0; j < static_cast<int>(I_depth_raw.getWidth()); j++) {
          if (I_depth_raw[i][j]) {
            float Z = I_depth_raw[i][j] * depth_scale;//相机原始深度值的缩放系数
            depthMap[i][j] = Z;  //depthMap是真正的深度图，单位为m，I_depth_raw中保存的深度值是缩放后的，单位不是m
          }
          else {
            depthMap[i][j] = 0;
          }
        }
      }
      //计算深度图end

      //显示
      vpDisplay::display(I_color);
      vpDisplay::display(I_color2);
      vpDisplay::display(I_depth);

      //检测
      std::vector<vpHomogeneousMatrix> cMo_vec;//二维码位姿保存在cMo_vec中
      detector.detect(I, tagSize, cam, cMo_vec);//可知检测的输入是灰度图I，rgb图只用于显示！！

      //显示基于灰度图的检测结果：相机视角下 被检测对象的机体坐标系
      // Display camera pose for each tag
      for (size_t i = 0; i < cMo_vec.size(); i++) {
        vpDisplay::displayFrame(I_color, cMo_vec[i], cam, tagSize / 2, vpColor::none, 3);//显示 相机视角下的 被检测对象的 机体坐标系（轴）!!!!!
      }
      //显示基于灰度图的检测结果：相机视角下 被检测对象的机体坐标系end

      //显示基于深度图的检测结果：相机视角下 被检测对象的机体坐标系
      //! [Pose from depth map]
      std::vector<std::vector<vpImagePoint> > tags_corners = detector.getPolygon();//获取检测结果的4个角点
      std::vector<int> tags_id = detector.getTagsId();//获取检测结果的TagId
      std::map<int, double> tags_size;
      tags_size[-1] = tagSize; // Default tag size
      std::vector<std::vector<vpPoint> > tags_points3d = detector.getTagsPoints3D(tags_id, tags_size);
      for (size_t i = 0; i < tags_corners.size(); i++) {
        vpHomogeneousMatrix cMo;
        double confidence_index;
        if (vpPose::computePlanarObjectPoseFromRGBD(depthMap, tags_corners[i], cam, tags_points3d[i], cMo,
                                                    &confidence_index)) {
          if (confidence_index > 0.5) {//不同区间的可信度设置不同的坐标轴显示颜色
            vpDisplay::displayFrame(I_color2, cMo, cam, tagSize / 2, vpColor::none, 3);
          }
          else if (confidence_index > 0.25) {
            vpDisplay::displayFrame(I_color2, cMo, cam, tagSize / 2, vpColor::orange, 3);
          }
          else {
            vpDisplay::displayFrame(I_color2, cMo, cam, tagSize / 2, vpColor::red, 3);
          }
          std::stringstream ss;
          ss << "Tag id " << tags_id[i] << " confidence: " << confidence_index;
          vpDisplay::displayText(I_color2, 35 + i * 15, 20, ss.str(), vpColor::red);

          if (opt_verbose) {
            std::cout << "cMo[" << i << "]: \n" << cMo_vec[i] << std::endl;
            std::cout << "cMo[" << i << "] using depth: \n" << cMo << std::endl;
          }
        }
      }
      //! [Pose from depth map]
      //显示基于深度图的检测结果：相机视角下 被检测对象的机体坐标系end

      vpDisplay::displayText(I_color, 20, 20, "Pose from homography + VVS", vpColor::red);
      vpDisplay::displayText(I_color2, 20, 20, "Pose from RGBD fusion", vpColor::red);
      vpDisplay::displayText(I_color, 35, 20, "Click to quit.", vpColor::red);
      t = vpTime::measureTimeMs() - t;
      time_vec.push_back(t);

      std::stringstream ss;
      ss << "Detection time: " << t << " ms for " << detector.getNbObjects() << " tags";
      vpDisplay::displayText(I_color, 50, 20, ss.str(), vpColor::red);

      if (vpDisplay::getClick(I_color, false))//检测鼠标左击事件！！！
        break;

      vpDisplay::flush(I_color);
      vpDisplay::flush(I_color2);
      vpDisplay::flush(I_depth);
    }
    //核心死循环：持续检测end 

    std::cout << "Benchmark loop processing time" << std::endl;
    std::cout << "Mean / Median / Std: " << vpMath::getMean(time_vec) << " ms"
      << " ; " << vpMath::getMedian(time_vec) << " ms"
      << " ; " << vpMath::getStdev(time_vec) << " ms" << std::endl;

    if (!display_off) {
      delete d1;
      delete d2;
      delete d3;
    }

  }
  //主要逻辑代码end
  catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
  }

  return EXIT_SUCCESS;
#else
  (void)argc;
  (void)argv;
#ifndef VISP_HAVE_APRILTAG
  std::cout << "Enable Apriltag support, configure and build ViSP to run this tutorial" << std::endl;
#else
  std::cout << "Install librealsense 3rd party, configure and build ViSP again to use this example" << std::endl;
#endif
#endif
  return EXIT_SUCCESS;
}
