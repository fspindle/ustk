#ifndef US_SLICING_CALLBACK_H
#define US_SLICING_CALLBACK_H

#include <visp3/ustk_gui/usGuiConfig.h>

#ifdef USTK_HAVE_VTK

//UsTK includes
#include <visp3/ustk_core/us.h>
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

#include <vtkSmartPointer.h>
#include <vtkImageReader2.h>
#include <vtkMatrix4x4.h>
#include <vtkImageReslice.h>
#include <vtkLookupTable.h>
#include <vtkImageMapToColors.h>
#include <vtkImageActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkCommand.h>
#include <vtkImageData.h>
#include <vtkImageMapper3D.h>
#include <vtkStreamingDemandDrivenPipeline.h>
#include <vtkInformation.h>
#include <vtkMetaImageReader.h>


// The mouse motion callback, to turn "Slicing" on and off
class VISP_EXPORT usSlicingCallback : public vtkCommand
{
public:

  static usSlicingCallback *New() {
    return new usSlicingCallback; };

  usSlicingCallback() {
    this->Slicing = 0;
    this->ImageReslice = 0;
    this->Interactor = 0; };

  void SetImageReslice(vtkImageReslice *reslice) {
    this->ImageReslice = reslice; };

  vtkImageReslice *GetImageReslice() {
    return this->ImageReslice; };

  void SetInteractor(vtkRenderWindowInteractor *interactor) {
    this->Interactor = interactor; };

  vtkRenderWindowInteractor *GetInteractor() {
    return this->Interactor; };

  void set3DViewer(usViewer3D* viewer3D) {
    this->m_viewer3D = viewer3D;
  };

  void setSliceOrientation(us::Orientation orientation) {
    this->m_orientation = orientation;
  };

  void SetTextMapper(vtkTextMapper* textMapper) {
    m_textMapper = textMapper;
  };

  void Execute(vtkObject *, unsigned long event, void *)
#if (USTK_HAVE_VTK_VERSION > 0x070100) // vtk-7.1.0
  VTK_OVERRIDE
#endif
  {
    vtkRenderWindowInteractor *interactor = this->GetInteractor();

    int lastPos[2];
    interactor->GetLastEventPosition(lastPos);
    int currPos[2];
    interactor->GetEventPosition(currPos);

    if (event == vtkCommand::LeftButtonPressEvent)
    {
      this->Slicing = 1;
    }
    else if (event == vtkCommand::LeftButtonReleaseEvent)
    {
      this->Slicing = 0;
    }
    else if (event == vtkCommand::MouseWheelForwardEvent) {
      vtkImageReslice *reslice = this->ImageReslice;

      reslice->Update();
      double sliceSpacing = 0;
      if(m_orientation == us::Xorientation) {
        sliceSpacing = reslice->GetOutput()->GetSpacing()[2];
      }
      else if(m_orientation == us::Yorientation) {
        sliceSpacing = reslice->GetOutput()->GetSpacing()[1];
      }
      else if(m_orientation == us::Zorientation) {
        sliceSpacing = reslice->GetOutput()->GetSpacing()[0];
      }
      vtkMatrix4x4 *matrix = reslice->GetResliceAxes();
      // move the center point that we are slicing through
      double point[4];
      point[0] = 0.0;
      point[1] = 0.0;
      point[2] = sliceSpacing; //one frame forward
      point[3] = 1.0;
      double center[4];
      matrix->MultiplyPoint(point, center);
      //m_viewer3D->sliceX(center[2]);
      matrix->SetElement(0, 3, center[0]);
      matrix->SetElement(1, 3, center[1]);
      matrix->SetElement(2, 3, center[2]);
      //update slicing text
      std::stringstream tmp;
      if(m_orientation == us::Xorientation) {
        tmp << "Slice :  " << center[2];
      }
      else if(m_orientation == us::Yorientation) {
        tmp << "Slice :  " << center[1];
      }
      else if(m_orientation == us::Zorientation) {
        tmp << "Slice :  " << center[0];
      }
      std::string msg = tmp.str();
      m_textMapper->SetInput(msg.c_str());

      //update interactor
      interactor->Render();
    }
    else if (event == vtkCommand::MouseWheelBackwardEvent) {
      vtkImageReslice *reslice = this->ImageReslice;

      reslice->Update();
      double sliceSpacing = 0;
      if(m_orientation == us::Xorientation) {
        sliceSpacing = reslice->GetOutput()->GetSpacing()[2];
      }
      else if(m_orientation == us::Yorientation) {
        sliceSpacing = reslice->GetOutput()->GetSpacing()[1];
      }
      else if(m_orientation == us::Zorientation) {
        sliceSpacing = reslice->GetOutput()->GetSpacing()[0];
      }
      vtkMatrix4x4 *matrix = reslice->GetResliceAxes();
      // move the center point that we are slicing through
      double point[4];
      point[0] = 0.0;
      point[1] = 0.0;
      point[2] = - sliceSpacing; //one frame backward
      point[3] = 1.0;
      double center[4];
      matrix->MultiplyPoint(point, center);
      matrix->SetElement(0, 3, center[0]);
      matrix->SetElement(1, 3, center[1]);
      matrix->SetElement(2, 3, center[2]);

      //update slicing text
      std::stringstream tmp;
      if(m_orientation == us::Xorientation) {
        tmp << "Slice :  " << center[2];
      }
      else if(m_orientation == us::Yorientation) {
        tmp << "Slice :  " << center[1];
      }
      else if(m_orientation == us::Zorientation) {
        tmp << "Slice :  " << center[0];
      }
      std::string msg = tmp.str();
      m_textMapper->SetInput(msg.c_str());

      //update interactor
      interactor->Render();
    }
    else if (event == vtkCommand::MouseMoveEvent) {
      if (this->Slicing)
      {
        vtkImageReslice *reslice = this->ImageReslice;

        // Increment slice position by deltaY of mouse
        int deltaY = lastPos[1] - currPos[1];

        reslice->Update();
        double sliceSpacing = 0.;
        if(m_orientation == us::Xorientation) {
          sliceSpacing = reslice->GetOutput()->GetSpacing()[2];
        }
        else if(m_orientation == us::Yorientation) {
          sliceSpacing = reslice->GetOutput()->GetSpacing()[1];
        }
        else if(m_orientation == us::Zorientation) {
          sliceSpacing = reslice->GetOutput()->GetSpacing()[0];
        }
        vtkMatrix4x4 *matrix = reslice->GetResliceAxes();
        // move the center point that we are slicing through
        double point[4];
        point[0] = 0.0;
        point[1] = 0.0;
        point[2] = sliceSpacing * deltaY;
        point[3] = 1.0;
        double center[4];
        matrix->MultiplyPoint(point, center);
        //m_viewer3D->sliceX(center[2]);
        matrix->SetElement(0, 3, center[0]);
        matrix->SetElement(1, 3, center[1]);
        matrix->SetElement(2, 3, center[2]);
        interactor->Render();
      }
      else
      {
        vtkInteractorStyle *style = vtkInteractorStyle::SafeDownCast(
              interactor->GetInteractorStyle());
        if (style)
        {
          style->OnMouseMove();
        }
      }
    }
  };

private:
  //Pointer to 3D Viewer to slince in it //NOT USED FOR NOW
  usViewer3D* m_viewer3D;

  //Slice orientation
  us::Orientation m_orientation;

  // Pointer to the text mapper to update it when slice change
  vtkTextMapper *m_textMapper;

  // Actions (slicing only, for now)
  int Slicing;

  // Pointer to vtkImageReslice
  vtkImageReslice *ImageReslice;

  // Pointer to the interactor
  vtkRenderWindowInteractor *Interactor;
};

#endif
#endif //US_SLICING_CALLBACK_H
