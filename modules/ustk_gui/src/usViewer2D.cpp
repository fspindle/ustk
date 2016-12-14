/****************************************************************************
 *
 * This file is part of the UsTk software.
 * Copyright (C) 2014 by Inria. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License ("GPL") as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * See the file COPYING at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact the
 * authors at Alexandre.Krupa@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Authors:
 * Marc Pouliquen
 *
 *****************************************************************************/

#include <visp3/ustk_gui/usViewer2D.h>

/**
* Default constructor.
*/
usViewer2D::usViewer2D(usViewer2D::Orientation orientation ,int slice)
{
  vtkSmartPointer<vtkMetaImageReader> reader =
      vtkSmartPointer<vtkMetaImageReader>::New();
  reader->SetFileName("/home/mpouliqu/Documents/usData/postscan/3D/postscan3d.mhd");
  reader->Update();
  //reader->SetDataExtent(0, 63, 0, 63, 1, 93);
  //reader->SetDataSpacing(3.2, 3.2, 1.5);
  reader->SetDataOrigin(0.0, 0.0, 0.0);
  //reader->SetDataScalarTypeToUnsignedShort();
  //reader->SetDataByteOrderToLittleEndian();
  reader->UpdateWholeExtent();

  // Calculate the center of the volume
  reader->Update();
  int extent[6];
  double spacing[3];
  double origin[3];


  reader->GetOutputInformation(0)->Get(vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT(), extent);
  reader->GetOutput()->GetSpacing(spacing);
  reader->GetOutput()->GetOrigin(origin);

  double center[3];
  center[0] = origin[0] + spacing[0] * 0.5 * (extent[0] + extent[1]);
  center[1] = origin[1] + spacing[1] * 0.5 * (extent[2] + extent[3]);
  center[2] = origin[2] + spacing[2] * 0.5 * (extent[4] + extent[5]);

  // Matrices for axial, coronal, sagittal, oblique view orientations
  static double axialElements[16] = {
           1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1 };

  //static double coronalElements[16] = {
  //         1, 0, 0, 0,
  //         0, 0, 1, 0,
  //         0,-1, 0, 0,
  //         0, 0, 0, 1 };

  //static double sagittalElements[16] = {
  //         0, 0,-1, 0,
  //         1, 0, 0, 0,
  //         0,-1, 0, 0,
  //         0, 0, 0, 1 };

  //static double obliqueElements[16] = {
  //         1, 0, 0, 0,
  //         0, 0.866025, -0.5, 0,
  //         0, 0.5, 0.866025, 0,
  //         0, 0, 0, 1 };

  // Set the slice orientation
  vtkSmartPointer<vtkMatrix4x4> resliceAxes =
    vtkSmartPointer<vtkMatrix4x4>::New();
  resliceAxes->DeepCopy(axialElements);
  // Set the point through which to slice
  resliceAxes->SetElement(0, 3, center[0]);
  resliceAxes->SetElement(1, 3, center[1]);
  resliceAxes->SetElement(2, 3, slice);

  // Extract a slice in the desired orientation
  vtkSmartPointer<vtkImageReslice> reslice =
    vtkSmartPointer<vtkImageReslice>::New();
  reslice->SetInputConnection(reader->GetOutputPort());
  reslice->SetOutputDimensionality(2);
  reslice->SetResliceAxes(resliceAxes);
  reslice->SetInterpolationModeToLinear();

  // Create a greyscale lookup table
  vtkSmartPointer<vtkLookupTable> table =
    vtkSmartPointer<vtkLookupTable>::New();
  table->SetRange(0, 150); // image intensity range
  table->SetValueRange(0.0, 1.0); // from black to white
  table->SetSaturationRange(0.0, 0.0); // no color saturation
  table->SetRampToLinear();
  table->Build();

  // Map the image through the lookup table
  vtkSmartPointer<vtkImageMapToColors> color =
    vtkSmartPointer<vtkImageMapToColors>::New();
  color->SetLookupTable(table);
  color->SetInputConnection(reslice->GetOutputPort());

  // Display the image
  vtkSmartPointer<vtkImageActor> actor =
    vtkSmartPointer<vtkImageActor>::New();
  actor->GetMapper()->SetInputConnection(color->GetOutputPort());

  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(actor);

  vtkSmartPointer<vtkRenderWindow> window =
    vtkSmartPointer<vtkRenderWindow>::New();
  window->AddRenderer(renderer);

  // Set up the interaction
  vtkSmartPointer<vtkInteractorStyleImage> imageStyle =
    vtkSmartPointer<vtkInteractorStyleImage>::New();
  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetInteractorStyle(imageStyle);
  window->SetInteractor(interactor);
  window->Render();

  vtkSmartPointer<usSlicingCallback> callback =
    vtkSmartPointer<usSlicingCallback>::New();
  callback->SetImageReslice(reslice);
  callback->SetInteractor(interactor);

  imageStyle->AddObserver(vtkCommand::MouseMoveEvent, callback);
  imageStyle->AddObserver(vtkCommand::LeftButtonPressEvent, callback);
  imageStyle->AddObserver(vtkCommand::LeftButtonReleaseEvent, callback);

  // Start interaction
  // The Start() method doesn't return until the window is closed by the user
  interactor->Start();
}

/**
* Destructor.
*/
usViewer2D::~usViewer2D()
{

}

/**
* Start display.
*/
void usViewer2D::start()
{
//  /m_renderWindowInteractor->Start();
}

/**
* Init interactor.
*/
void usViewer2D::initInteractorStyle(usViewer3D* viewer)
{
  //m_interactorStyle->SetImageViewer(m_imageReslice,0,100,50);
  //m_interactorStyle->SetViewer3D(viewer);
  //m_interactorStyle->SetRenderWindow2D(m_window);
}

/**
* Set orientation of th view.
*/
void usViewer2D::setOrientation(usViewer2D::Orientation orientation)
{
/*
  std::cout << "setOrientation" << std::endl;
  if(orientation == usViewer2D::Xorientation)
    m_imageViewer->SetSliceOrientationToYZ();
  else if(orientation == usViewer2D::Yorientation)
    m_imageViewer->SetSliceOrientationToXZ();
  else if(orientation == usViewer2D::Zorientation)
    m_imageViewer->SetSliceOrientationToXY();
  std::cout << "setOrientation end" << std::endl;
  m_imageViewer->UpdateDisplayExtent();*/
}

/**
* Set orientation of th view.
*/
void usViewer2D::updateView() {
  //m_imageReslice->Update();
}
