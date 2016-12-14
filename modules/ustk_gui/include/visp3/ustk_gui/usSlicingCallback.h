#ifndef US_SLICING_CALLBACK_H
#define US_SLICING_CALLBACK_H


//ViSP includes
#include<visp3/core/vpConfig.h>
#include<visp3/ustk_gui/usViewer2D.h>
#include<visp3/ustk_gui/usViewer3D.h>

//VTK includes
#include<vtkInteractorStyleImage.h>
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

  void Execute(vtkObject *, unsigned long event, void *) VTK_OVERRIDE
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
    else if (event == vtkCommand::MouseMoveEvent)
    {
      if (this->Slicing)
      {
        vtkImageReslice *reslice = this->ImageReslice;

        // Increment slice position by deltaY of mouse
        int deltaY = lastPos[1] - currPos[1];

        reslice->Update();
        double sliceSpacing = reslice->GetOutput()->GetSpacing()[2];
        vtkMatrix4x4 *matrix = reslice->GetResliceAxes();
        // move the center point that we are slicing through
        double point[4];
        double center[4];
        point[0] = 0.0;
        point[1] = 0.0;
        point[2] = sliceSpacing * deltaY;
        point[3] = 1.0;
        matrix->MultiplyPoint(point, center);
        std::cout << "current slice : " << center[2] << std::endl;
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
  //Pointer to 3D Viewer to slince in it
   usViewer3D* m_viewer3D;

  // Actions (slicing only, for now)
  int Slicing;

  // Pointer to vtkImageReslice
  vtkImageReslice *ImageReslice;

  // Pointer to the interactor
  vtkRenderWindowInteractor *Interactor;
};

#endif //US_SLICING_CALLBACK_H
