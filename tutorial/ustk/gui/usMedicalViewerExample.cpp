#include <QtGui/QMainWindow>
#include <QtGui/QApplication>
#include <visp3/ustk_gui/usMedicalImageViewer.h>

#ifdef USTK_HAVE_VTK

int main( int argc, char** argv )
{
  // QT application
  QApplication app( argc, argv );

  std::string fileName = "/home/mpouliqu/Documents/usData/prescan/3D/USpreScan_volume-0000/volume.mhd";
  usMedicalImageViewer medicalImageViewer(fileName);
  medicalImageViewer.show();

  return app.exec();
}
#else
int main( int argc, char** argv )
{
  std::cout << "You need VTK to run this example" << std::endl;
  return EXIT_SUCCESS;
}
#endif
