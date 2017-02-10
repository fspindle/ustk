//! \example tutorial-ultrasonix-grabber-vtk-display.cpp
//! [capture-multi-threaded declaration]
#include <iostream>

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpMutex.h>
#include <visp3/core/vpThread.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpV4l2Grabber.h>

#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImagePostScan2D.h>

#include <visp3/ustk_grabber/usGrabberUltrasonix.h>
#include <visp3/ustk_grabber/usGrabberFrame.h>

#include <visp3/ustk_gui/usResliceMatrixViewer.h>

#include <visp3/ustk_core/usScanConverter3D.h>

#if defined(VISP_HAVE_V4L2) && defined(VISP_HAVE_PTHREAD) && defined(USTK_HAVE_VTK_QT)

// Shared vars
typedef enum {
  capture_waiting,
  capture_started,
  capture_stopped
} t_CaptureState;
t_CaptureState s_capture_state = capture_waiting;
usImagePreScan2D<unsigned char> s_frame_prescan;
vpMutex s_mutex_capture;

int s_frameIndex;
int s_volumeIndex;
//! [capture-multi-threaded declaration]

//! [capture-multi-threaded captureFunction]
vpThread::Return captureFunction(vpThread::Args args)
{
  usGrabberUltrasonix grabber = *((usGrabberUltrasonix *) args);
  usImagePreScan2D<unsigned char> m_frame_prescan;

  //resizing images and saving image type in s_imageType
 if(grabber.getImageType() == us::PRESCAN_2D) {
    s_imageType = us::PRESCAN_2D;
    m_frame_prescan.resize(grabber.getCommunicationsInformations()->m_header.height,
                           grabber.getCommunicationsInformations()->m_header.width);
  }
  else
    throw(vpException(vpException::badValue,"You should provide pre-scan images"));

  s_mutex_imageType.unlock();

  bool stop_capture_ = false;

  int frameIncrement = -1;
  int frameIndex = 0;
  int volumeIndex = 0;

  //init grabbers (only one used but they have to be defined at global scope)
  usGrabberFrame<usImagePreScan2D<unsigned char> > grabberFramePreScan;
  grabberFramePreScan.setCommunicationInformation(grabber.getCommunicationsInformations());
  grabberFramePreScan.setTransducerSettings(grabber.getTransducerSettings());


  while (! stop_capture_) {
    // Capture in progress
    if(grabber.getImageType() == us::PRESCAN_2D) {
      grabberFramePreScan.grabFrame(&m_frame_prescan);

      //update frame increment if we reach last frame in a direction
      if(grabber.getCommunicationsInformations()->m_totFrmIdx % (grabber.getCommunicationsInformations()->m_header.framesPerVolume - 1) == 0)
        frameIncrement = -1 * frameIncrement;

      volumeIndex = grabber.getCommunicationsInformations()->m_totFrmIdx / grabber.getCommunicationsInformations()->m_header.framesPerVolume;
      frameIndex = grabber.getCommunicationsInformations()->m_totFrmIdx % grabber.getCommunicationsInformations()->m_header.framesPerVolume;

      if (volumeIndex % 2)
        frameIndex = grabber.getCommunicationsInformations()->m_header.framesPerVolume - frameIndex - 1;

      //frameNumber += frameIncrement;
      std::cout << "total frame index = " << grabber.getCommunicationsInformations()->m_totFrmIdx << std::endl;
      std::cout << "frame index = " << frameIndex << std::endl;
    }

    // Update shared data
    {
      vpMutex::vpScopedLock lock(s_mutex_capture);
      if (s_capture_state == capture_stopped)
        stop_capture_ = true;
      else
        s_capture_state = capture_started;

      if(grabber.getImageType() == us::PRESCAN_2D) {
        s_frame_prescan = m_frame_prescan;
      }
      else {
        throw(vpException(vpException::badValue,"ultrasonix sending non pre-scan frame !"));
      }
      s_frameIndex = frameIndex;
      s_volumeIndex = volumeIndex;
    }
  }

  {
    vpMutex::vpScopedLock lock(s_mutex_capture);
    s_capture_state = capture_stopped;
  }
  return 0;
}
//! [capture-multi-threaded captureFunction]

//! [capture-multi-threaded displayFunction]
vpThread::Return displayFunction(vpThread::Args args)
{
  (void)args; // Avoid warning: unused parameter args
  int m_imageType;
  usImageRF2D<unsigned char> rf_;
  usImagePreScan2D<unsigned char> preScan_;
  usImagePostScan2D<unsigned char> postScan_;

  t_CaptureState capture_state_;
  bool display_initialized_ = false;
#if defined(VISP_HAVE_X11)
  vpDisplayX *d_ = NULL;
#endif

  do {
    s_mutex_capture.lock();
    capture_state_ = s_capture_state;
    s_mutex_capture.unlock();

    // Check if a frame is available
    if (capture_state_ == capture_started) {
      {
        vpMutex::vpScopedLock lock(s_mutex_imageType);
        m_imageType = s_imageType;
      }
      //capture started
      // Create a copy of the captured frame
      {
        vpMutex::vpScopedLock lock(s_mutex_capture);
       if(m_imageType == us::RF_2D) {
          rf_ = s_frame_rf;
       }
       else if(m_imageType == us::PRESCAN_2D) {
         preScan_ = s_frame_prescan;
       }
       else if(m_imageType == us::POSTSCAN_2D) {
         postScan_ = s_frame_postscan;
       }
      }


      // Check if we need to initialize the display with the first frame
      if (! display_initialized_) {
        // Initialize the display
#if defined(VISP_HAVE_X11)
        if(m_imageType == us::RF_2D) {
          d_ = new vpDisplayX(rf_);
          display_initialized_ = true;
        }
        else if(m_imageType == us::PRESCAN_2D) {
          d_ = new vpDisplayX(preScan_);
          display_initialized_ = true;
        }
        else if(m_imageType == us::POSTSCAN_2D) {
          d_ = new vpDisplayX(postScan_);
          display_initialized_ = true;
        }
#endif
      }
      if(m_imageType == us::RF_2D) {
        // Display the image
        vpDisplay::display(rf_);
        // Trigger end of acquisition with a mouse click
        vpDisplay::displayText(rf_, 10, 10, "Click to exit...", vpColor::red);
        if (vpDisplay::getClick(rf_, false)) {
          vpMutex::vpScopedLock lock(s_mutex_capture);
          s_capture_state = capture_stopped;
        }
        // Update the display
        vpDisplay::flush(rf_);
      }
      else if(m_imageType == us::PRESCAN_2D) {
        vpDisplay::display(preScan_);
        // Trigger end of acquisition with a mouse click
        vpDisplay::displayText(preScan_, 10, 10, "Click to exit...", vpColor::red);
        if (vpDisplay::getClick(preScan_, false)) {
          vpMutex::vpScopedLock lock(s_mutex_capture);
          s_capture_state = capture_stopped;
        }
        // Update the display
        vpDisplay::flush(preScan_);
      }
      else if(m_imageType == us::POSTSCAN_2D) {
        vpDisplay::display(postScan_);
        // Trigger end of acquisition with a mouse click
        vpDisplay::displayText(postScan_, 10, 10, "Click to exit...", vpColor::red);
        if (vpDisplay::getClick(postScan_, false)) {
          vpMutex::vpScopedLock lock(s_mutex_capture);
          s_capture_state = capture_stopped;
        }
        // Update the display
        vpDisplay::flush(postScan_);
      }


    }
    else {
      vpTime::wait(2); // Sleep 2ms
    }
  } while(capture_state_ != capture_stopped);

#if defined(VISP_HAVE_X11)
  delete d_;
#endif

  std::cout << "End of display thread" << std::endl;
  return 0;
}
//! [capture-multi-threaded displayFunction]

//! [capture-multi-threaded mainFunction]
int main(int argc, const char* argv[])
{
  (void) argc;
  (void) argv;

  // QT application
  QApplication app( argc, argv );

  //Double buffer system
  vtkSmartPointer<vtkImageData> image1 = vtkSmartPointer<vtkImageData>::New();
  vtkSmartPointer<vtkImageData> image2 = vtkSmartPointer<vtkImageData>::New();

  usImagePreScan3D<unsigned char> preScan1;
  usImagePreScan3D<unsigned char> preScan2;

  usImagePostScan3D<unsigned char> postScan1;
  usImagePostScan3D<unsigned char> postScan2;

  //For now, we set manually the transducer/motor settings because otherwise they are only know when the station starts sending frames,
  //and we have no time to initialize the 3D scan converter
  preScan1.resize(128,480,20);
  preScan1.setScanLinePitch(0.010625);
  preScan1.setTransducerRadius(0.0398);
  preScan1.setTransducerConvexity(true);
  preScan1.setAxialResolution(0.000308);
  preScan1.setMotorRadius(0.02725);
  preScan1.setMotorType(usMotorSettings::TiltingMotor);

  preScan2.resize(128,480,20);
  preScan2.setScanLinePitch(0.010625);
  preScan2.setTransducerRadius(0.0398);
  preScan2.setTransducerConvexity(true);
  preScan2.setAxialResolution(0.000308);
  preScan2.setMotorRadius(0.02725);
  preScan2.setMotorType(usMotorSettings::TiltingMotor);

  usScanConverter3D converter;
  converter.init(preScan1);

  //allocate memory for post-scanImages
  converter.convert(postScan1);
  converter.convert(postScan2);









  usResliceMatrixViewer viewer();
  viewer.show();

  return app.exec();
  // Instantiate the grabber
  usGrabberUltrasonix grabber;

  grabber.start();

  // Start the threads
  vpThread thread_capture((vpThread::Fn)captureFunction, (vpThread::Args)&grabber);

  //Qt application end
  app.exec();

  //end capture thread
  s_mutex_capture.lock();
  s_capture_state = capture_stopped;
  s_mutex_capture.unlock();

  // Wait until thread ends up
  thread_capture.join();

  return 0;
}
//! [capture-multi-threaded mainFunction]

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
