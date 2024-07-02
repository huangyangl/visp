//! \example tutorial-blob-tracker-live-firewire.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/blob/vpDot2.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/sensor/vpRealSense.h>
#include <visp3/sensor/vpRealSense2.h>

int main()
{
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


#if defined(VISP_HAVE_X11)
  vpDisplayX d(I, 0, 0, "Camera view");
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d(I, 0, 0, "Camera view");
#elif defined(HAVE_OPENCV_HIGHGUI)
  vpDisplayOpenCV d(I, 0, 0, "Camera view");
#endif

  //! [Construction]
  vpDot2 blob;
  //! [Construction]
  //! [Setting]
  blob.setGraphics(true);
  blob.setGraphicsThickness(2);
  //! [Setting]

  vpImagePoint germ;
  bool init_done = false;

  while (1) {
    try {
      g.acquire(Irgba);
      vpImageConvert::convert(Irgba, I);

      vpDisplay::display(I);

      if (!init_done) {
        vpDisplay::displayText(I, vpImagePoint(10, 10), "Click in the blob to initialize the tracker", vpColor::red);
        if (vpDisplay::getClick(I, germ, false)) {
          //! [Init]
          blob.initTracking(I, germ);
          //! [Init]
          init_done = true;
        }
      }
      else {
        //! [Track]
        blob.track(I);
        //! [Track]
      }

      vpDisplay::flush(I);
    }
    catch (...) {
      init_done = false;
    }
  }
}
