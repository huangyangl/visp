//! \example tutorial-me-line-tracker.cpp
#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
//! [camera headers]
#if defined(VISP_HAVE_REALSENSE) || defined(VISP_HAVE_REALSENSE2)
#include <visp3/core/vpImage.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/sensor/vpRealSense.h>
#include <visp3/sensor/vpRealSense2.h>
#endif
//! [camera headers]
#endif
//! [display headers]
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
//! [display headers]
//! [me line headers]
#include <visp3/me/vpMeLine.h>
//! [me line headers]

#if defined(HAVE_OPENCV_VIDEOIO)
#include <opencv2/videoio.hpp>
#endif

int main()
{
#if (defined(VISP_HAVE_REALSENSE) || defined(VISP_HAVE_REALSENSE2)||defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || defined(VISP_HAVE_V4L2) || defined(HAVE_OPENCV_VIDEOIO))
  try {

    //! [1st image Acquire]
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
    //! [1st image Acquire]

    //! [display container]
#if defined(VISP_HAVE_X11)
    vpDisplayX d(I, 0, 0, "Camera view");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I, 0, 0, "Camera view");
#elif defined(HAVE_OPENCV_HIGHGUI)
    vpDisplayOpenCV d(I, 0, 0, "Camera view");
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif
    //! [display container]
    //! [display image]
    vpDisplay::display(I);
    vpDisplay::flush(I);
    //! [display image]

    //! [me container]
    vpMe me;
    me.setRange(25);
    me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
    me.setThreshold(20);
    me.setSampleStep(10);
    //! [me container]

    //! [me line container]
    vpMeLine line;
    line.setMe(&me);
    line.setDisplay(vpMeSite::RANGE_RESULT);
    line.initTracking(I);
    //! [me line container]

    //! [loop]
    while (1) {
      g.acquire(Irgba);
      vpImageConvert::convert(Irgba, I);
      vpDisplay::display(I);
      line.track(I);
      line.display(I, vpColor::red);
      vpDisplay::flush(I);
    }
    //! [loop]
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
