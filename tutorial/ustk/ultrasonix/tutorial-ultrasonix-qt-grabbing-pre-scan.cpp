//! \example tutorial-ultrasonix-qt-grabbing-pre-scan.cpp

#include <iostream>
#include <visp3/ustk_grabber/usGrabberConfig.h>

#if defined(USTK_GRABBER_HAVE_QT5) && defined(USTK_GRABBER_HAVE_QT5_WIDGETS) && defined(VISP_HAVE_X11)

#include <QThread>
#include <QApplication>

#include <visp3/ustk_grabber/usNetworkGrabberPreScan.h>

#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlot.h>

int main(int argc, char** argv)
{
  // QT application
  QApplication app( argc, argv );

  QThread * grabbingThread = new QThread();

  usNetworkGrabberPreScan * qtGrabber = new usNetworkGrabberPreScan();
  qtGrabber->setConnection(true);

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 15; // 4DC7 id = 15
  header.slotId = 0; //top slot id = 0
  header.transmitFrequency = 4000000;
  header.samplingFrequency = 2500000;
  header.imagingMode = 0; //B-mode = 0
  header.postScanMode = false;
  header.imageDepth = 140; //in mm
  header.sector = 100; //in %

  // 2D acquisition
  header.activateMotor = false; //to sweep the motor permanently
  header.motorPosition = 40; // motor in the middle

  /*IF 3D
  header.activateMotor = true;
  header.framesPerVolume = 10;
  header.degreesPerFrame = 3;*/

  //prepare image;
  usDataGrabbed<usImagePreScan2D<unsigned char> >* grabbedFrame;

  //Prepare display
  vpDisplayX * displayX = NULL;
  bool displayInit = false;

  bool captureRunning = true;

  //qtGrabber->setVerbose(true);
  // sending acquisition parameters
  qtGrabber->initAcquisition(header);

  // Move the grabber object to another thread, and run it
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  std::cout << "waiting ultrasound initialisation..." << std::endl;
  vpPlot plot(1);
  plot.initGraph(0, 1);
  plot.setTitle(0, "time measure");
  plot.setUnitY(0, "datarate");
  plot.setLegend(0, 0, "frame number");
  //our local grabbing loop
  do {
    if(qtGrabber->isFirstFrameAvailable()) {
      grabbedFrame = qtGrabber->acquire();
      std::cout <<"MAIN THREAD received frame No : " << grabbedFrame->getFrameCount() << std::endl;
      std::cout <<"DataRate : " << grabbedFrame->getDataRate() << std::endl;
      if(grabbedFrame->getDataRate() != std::numeric_limits<double>::infinity())
        plot.plot(0,0,(int)grabbedFrame->getFrameCount(),grabbedFrame->getDataRate());
      else
        plot.plot(0,0,(int)grabbedFrame->getFrameCount(),100);


      //init display
      if(!displayInit && grabbedFrame->getHeight() !=0 && grabbedFrame->getHeight() !=0) {
        displayX = new vpDisplayX(*grabbedFrame);
        displayInit = true;
      }

      // processing display
      if(displayInit) {
        vpDisplay::display(*grabbedFrame);
        vpDisplay::flush(*grabbedFrame);
      }
    }
    else {
      vpTime::wait(10);
    }
  }while(captureRunning);


  return app.exec();
}

#else
int main()
{
  std::cout << "You should intall Qt5 (with wigdets and network modules), and display X to run this tutorial" << std::endl;
  return 0;
}

#endif
