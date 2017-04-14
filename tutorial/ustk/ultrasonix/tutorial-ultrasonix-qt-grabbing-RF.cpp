//! \example tutorial-ultrasonix-qt-grabbing-pre-scan.cpp

#include <iostream>
#include <visp3/ustk_grabber/usGrabberConfig.h>

#if defined(USTK_GRABBER_HAVE_QT5) & defined(USTK_GRABBER_HAVE_QT5_WIDGETS)

#include <QThread>
#include <QApplication>

#include <visp3/ustk_grabber/usNetworkGrabberRF.h>

int main(int argc, char** argv)
{
  // QT application
  QApplication app( argc, argv );

  QThread * grabbingThread = new QThread();

  usNetworkGrabberRF * qtGrabber = new usNetworkGrabberRF();
  qtGrabber->setConnection(true);

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 15; // 4DC7 id = 15
  header.slotId = 0; //top slot id = 0
  header.transmitFrequency = 4000000;
  header.samplingFrequency = 2500000;
  header.imagingMode = 12; //B-mode = 0, RF = 12
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
  usDataGrabbed<usImageRF2D<unsigned char> >* grabbedFrame;
  usDataGrabbed<usImageRF2D<unsigned char> > localFrame;

  bool captureRunning = true;

  qtGrabber->setVerbose(true);
  // sending acquisition parameters
  qtGrabber->initAcquisition(header);

  // Move the grabber object to another thread
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  //our local grabbing loop
  do {
    if(qtGrabber->isFirstFrameAvailable()) {
      grabbedFrame = qtGrabber->acquire();

      //local copy for vpDisplay
      localFrame = *grabbedFrame;

      std::cout <<"MAIN THREAD received RF frame No : " << localFrame.getFrameCount() << std::endl;

      //TO DO : convert RF to pre-scan to display something ...


      vpTime::wait(20);
    }
    else {
      std::cout << "waiting ultrasound initialisation..." << std::endl;
      vpTime::wait(1000);
    }
  }while(captureRunning);
  return app.exec();
}

#else
int main()
{
  std::cout << "You should intall Qt5 (with wigdets and network modules) to run this tutorial" << std::endl;
  return 0;
}

#endif
