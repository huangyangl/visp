/*! \example tutorial-klt-tracker-live-v4l2.cpp */
#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vpV4l2Grabber.h>
#endif
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/klt/vpKltOpencv.h>

#if defined(VISP_HAVE_REALSENSE) || defined(VISP_HAVE_REALSENSE2)
#include <visp3/core/vpImage.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/sensor/vpRealSense.h>
#include <visp3/sensor/vpRealSense2.h>
#endif

int main(int argc, const char *argv [])
{
#if (defined(HAVE_OPENCV_HIGHGUI) && (defined(VISP_HAVE_REALSENSE) || defined(VISP_HAVE_REALSENSE2))) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
  try {
    bool opt_init_by_click = false;

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--init-by-click")
        opt_init_by_click = true;
      else if (std::string(argv[i]) == "--help") {
        std::cout << "Usage: " << argv[0] << " [--init-by-click] [--device <camera device>] [--help]" << std::endl;
        return EXIT_SUCCESS;
      }
    }

    //! [Acquire]
    int opt_fps = 30;
    unsigned int opt_width = 640;
    unsigned int opt_height = 480;
    vpImage<vpRGBa> Irgba;
    vpImage<unsigned char> I; // Create a gray level image container
  #ifdef VISP_HAVE_REALSENSE2
    std::cout << "SDK        : Realsense 2" << std::endl;
    vpRealSense2 g;
    rs2::config config;
    config.disable_stream(RS2_STREAM_DEPTH);
    config.disable_stream(RS2_STREAM_INFRARED);
    config.enable_stream(RS2_STREAM_COLOR, opt_width, opt_height, RS2_FORMAT_RGBA8, opt_fps);
    g.open(config);
  #else
    std::cout << "SDK        : Realsense 1" << std::endl;
    vpRealSense g;
    g.setStreamSettings(rs::stream::color, vpRealSense::vpRsStreamParams(opt_width, opt_height, rs::format::rgba8, 60));
    g.open();
  #endif
    g.acquire(Irgba);
    vpImageConvert::convert(Irgba, I);
    //! [Acquire]


    cv::Mat cvI;
    vpImageConvert::convert(I, cvI);

    // Display initialisation
    vpDisplayOpenCV d(I, 0, 0, "Klt tracking");
    vpDisplay::display(I);
    vpDisplay::flush(I);

    vpKltOpencv tracker;
    // Set tracker parameters
    tracker.setMaxFeatures(200);
    tracker.setWindowSize(10);
    tracker.setQuality(0.01);
    tracker.setMinDistance(15);
    tracker.setHarrisFreeParameter(0.04);
    tracker.setBlockSize(9);
    tracker.setUseHarris(1);
    tracker.setPyramidLevels(3);

    // Initialise the tracking
    if (opt_init_by_click) {
      vpMouseButton::vpMouseButtonType button;
      std::vector<cv::Point2f> guess;
      vpImagePoint ip;
      do {
        vpDisplay::displayText(I, 10, 10, "Left click to select a point, right to start tracking", vpColor::red);
        if (vpDisplay::getClick(I, ip, button, false)) {
          if (button == vpMouseButton::button1) {
            guess.push_back(cv::Point2f((float)ip.get_u(), (float)ip.get_v()));
            vpDisplay::displayText(I, 10, 10, "Left click to select a point, right to start tracking", vpColor::red);
            vpDisplay::displayCross(I, ip, 12, vpColor::green);
          }
        }
        vpDisplay::flush(I);
        vpTime::wait(20);
      } while (button != vpMouseButton::button3);
      tracker.initTracking(cvI, guess);
    }
    else {
      tracker.initTracking(cvI);
    }

    while (1) {
      g.acquire(Irgba);
      vpImageConvert::convert(Irgba, I);

      vpDisplay::display(I);

      vpImageConvert::convert(I, cvI);
      tracker.track(cvI);

      tracker.display(I, vpColor::red);
      vpDisplay::displayText(I, 10, 10, "Click to quit", vpColor::red);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
    }
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
#else
  (void)argc;
  (void)argv;
#endif
  return EXIT_SUCCESS;
}
