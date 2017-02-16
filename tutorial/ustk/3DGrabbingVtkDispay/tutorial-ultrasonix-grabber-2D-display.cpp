//! \example tutorial-ultrasonix-grabber-vtk-display.cpp

#include <visp3/core/vpConfig.h>

#include <visp3/ustk_gui/usGuiConfig.h>

#if defined(VISP_HAVE_V4L2) && defined(VISP_HAVE_PTHREAD) && defined(USTK_HAVE_VTK_QT)

#include <QtCore>

#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usScanConverter3D.h>

#include <visp3/ustk_grabber/usGrabberUltrasonix.h>
#include <visp3/ustk_grabber/usGrabberFrame.h>

#include <visp3/ustk_grabber_volume/usGrabberThreadPreScan2D.h>

#include <visp3/ustk_gui/us3DSceneSlicing.h>


int main(int argc, char** argv)
{
  (void) argc;
  (void) argv;

  // QT application
  QApplication app( argc, argv );

  usImagePreScan2D<unsigned char> frame;
  QMutex frameMutex;

  std::cout << "1" << std::endl;
  us3DSceneSlicing viewer(&frame, &frameMutex);

  std::cout << "1" << std::endl;

  std::cout << "1" << std::endl;
  viewer.show();

  std::cout << "1" << std::endl;


  //Start capture thread
  usGrabberThreadPreScan2D grabberThread2D(&frame, &frameMutex);
  grabberThread2D.start();


  std::cout << "2" << std::endl;

  //connect grabber signal to viewer slot
  QObject::connect(&grabberThread2D,SIGNAL(newFrameGrabbed(int)),&viewer,SLOT(updateFrame(int)));

  std::cout << "3" << std::endl;



  return app.exec();
}

#else
int main()
{
#  ifndef VISP_HAVE_V4L2
  std::cout << "You should enable V4L2 to make this example working..." << std::endl;
#  elif !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  std::cout << "You should enable pthread usage and rebuild ViSP..." << std::endl;
#  else
  std::cout << "Multi-threading seems not supported on this platform" << std::endl;
#  endif
}

#endif
