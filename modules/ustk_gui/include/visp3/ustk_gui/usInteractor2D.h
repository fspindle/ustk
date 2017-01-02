#ifndef US_INTERACTOR_2D_H
#define US_INTERACTOR_2D_H

#include <visp3/ustk_gui/usGuiConfig.h>

#ifdef USTK_HAVE_VTK

//ViSP includes
#include <visp3/core/vpConfig.h>
#include <visp3/ustk_gui/usViewer2D.h>
#include <visp3/ustk_gui/usViewer3D.h>

//VTK includes
#include <vtkInteractorStyleImage.h>
#include <vtkSmartPointer.h>
#include <vtkObjectFactory.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkActor.h>
#include <vtkImageViewer2.h>
#include <vtkDICOMImageReader.h>
#include <vtkInteractorStyleImage.h>
#include <vtkActor2D.h>
#include <vtkTextProperty.h>
#include <vtkTextMapper.h>


// Define own interaction style
class VISP_EXPORT usInteractor2D : public vtkInteractorStyleImage
{
public:
   static usInteractor2D* New();
   vtkTypeMacro(usInteractor2D, vtkInteractorStyleImage);

protected:
   usViewer3D* m_viewer3D;
   vtkSmartPointer<vtkImageReslice> m_reslice2D;
   vtkSmartPointer<vtkRenderWindow> m_window2D;
   //usViewer2D::Orientation m_orientation;

   vtkTextMapper* _StatusMapper;

   int _Slice;
   int _MinSlice;
   int _MaxSlice;

public:
   void SetImageViewer(vtkImageReslice* imageReslice, int sliceMin, int sliceMax, int initSlice) {
      m_reslice2D = imageReslice;

      _MinSlice = sliceMin;
      _MaxSlice = sliceMax;
      _Slice = initSlice;
      cout << "Slicer: Min = " << _MinSlice << ", Max = " << _MaxSlice << std::endl;
   }

   void SetStatusMapper(vtkTextMapper* statusMapper) {
      _StatusMapper = statusMapper;
   }

   void SetViewer3D(usViewer3D* viewer3D) {
      m_viewer3D = viewer3D;
   }

   void SetRenderWindow2D(vtkRenderWindow* window) {
      m_window2D = window;
   }


protected:
   void MoveSliceForward() {
      if(_Slice < _MaxSlice) {
         _Slice += 1;
         cout << "MoveSliceForward::Slice = " << _Slice << std::endl;
         m_viewer3D->sliceX(_Slice);
         std::stringstream tmp;
         tmp << "Slice " << _Slice + 1 << "/" << _MaxSlice + 1;
         std::string msg = tmp.str();
         //_StatusMapper->SetInput(msg.c_str());
         m_reslice2D->SetResliceAxesOrigin(0,0,_Slice);
         m_reslice2D->Update();
         m_window2D->Render();
      }
   }

   void MoveSliceBackward() {
      if(_Slice > _MinSlice) {
         _Slice -= 1;
         cout << "MoveSliceBackward::Slice = " << _Slice << std::endl;
         m_viewer3D->sliceX(_Slice);
         std::stringstream tmp;
         tmp << "Slice " << _Slice + 1 << "/" << _MaxSlice + 1;
         std::string msg = tmp.str();
         //_StatusMapper->SetInput(msg.c_str());
         m_reslice2D->SetResliceAxesOrigin(0,0,_Slice);
         m_reslice2D->Update();
         m_window2D->Render();
      }
   }


   virtual void OnKeyDown() {
      std::string key = this->GetInteractor()->GetKeySym();
      if(key.compare("Up") == 0) {
         //cout << "Up arrow key was pressed." << endl;
         MoveSliceForward();
      }
      else if(key.compare("Down") == 0) {
         //cout << "Down arrow key was pressed." << endl;
         MoveSliceBackward();
      }
      // forward event
      vtkInteractorStyleImage::OnKeyDown();
   }


   virtual void OnMouseWheelForward() {
      //std::cout << "Scrolled mouse wheel forward." << std::endl;
      MoveSliceForward();
      // don't forward events, otherwise the image will be zoomed
      // in case another interactorstyle is used (e.g. trackballstyle, ...)
      // vtkInteractorStyleImage::OnMouseWheelForward();
   }


   virtual void OnMouseWheelBackward() {
      //std::cout << "Scrolled mouse wheel backward." << std::endl;
      if(_Slice > _MinSlice) {
         MoveSliceBackward();
      }
      // don't forward events, otherwise the image will be zoomed
      // in case another interactorstyle is used (e.g. trackballstyle, ...)
      // vtkInteractorStyleImage::OnMouseWheelBackward();
   }
};

vtkStandardNewMacro(usInteractor2D);

#endif
#endif //US_INTERACTOR_2D_H
