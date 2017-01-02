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
usViewer2D::usViewer2D(us::Orientation orientation ,int sliceOrigin)
{
  m_orientation = orientation;

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

  //static double obliqueElements[16] = {
  //         1, 0, 0, 0,
  //         0, 0.866025, -0.5, 0,
  //         0, 0.5, 0.866025, 0,
  //         0, 0, 0, 1 };

  // Set the slice orientation
  m_resliceAxes = vtkSmartPointer<vtkMatrix4x4>::New();
  if(m_orientation == us::Xorientation) {
    double XElements[16] = {
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1 };

    m_resliceAxes->DeepCopy(XElements);
  }
  else if (m_orientation == us::Yorientation) {
    double YElements[16] = {
      1, 0, 0, 0,
      0, 0, 1, 0,
      0,-1, 0, 0,
      0, 0, 0, 1 };

    m_resliceAxes->DeepCopy(YElements);
  }
  else if (m_orientation == us::Zorientation) {
    double ZElements[16] = {
      0, 0,-1, 0,
      1, 0, 0, 0,
      0,-1, 0, 0,
      0, 0, 0, 1 };

    m_resliceAxes->DeepCopy(ZElements);
  }
  // Set the point through which to slice
  m_resliceAxes->SetElement(0, 3, center[0]);
  m_resliceAxes->SetElement(1, 3, center[1]);
  m_resliceAxes->SetElement(2, 3, center[2]);

  // Extract a slice in the desired orientation
  m_reslice = vtkSmartPointer<vtkImageReslice>::New();
  m_reslice->SetInputConnection(reader->GetOutputPort());
  m_reslice->SetOutputDimensionality(2);
  m_reslice->SetResliceAxes(m_resliceAxes);
  m_reslice->SetInterpolationModeToLinear();

  // Create a greyscale lookup table
  m_table = vtkSmartPointer<vtkLookupTable>::New();
  m_table->SetRange(0, 255); // image intensity range
  m_table->SetValueRange(0.0, 1.0); // from black to white
  m_table->SetSaturationRange(0.0, 0.0); // no color saturation
  m_table->SetRampToLinear();
  m_table->Build();

  // Map the image through the lookup table
  m_color = vtkSmartPointer<vtkImageMapToColors>::New();
  m_color->SetLookupTable(m_table);
  m_color->SetInputConnection(m_reslice->GetOutputPort());

  // Display the image
  m_actor = vtkSmartPointer<vtkImageActor>::New();
  m_actor->GetMapper()->SetInputConnection(m_color->GetOutputPort());

  //Text for slice number
  m_sliceTextProp = vtkSmartPointer<vtkTextProperty>::New();
  m_sliceTextProp->SetFontFamilyToCourier();
  m_sliceTextProp->SetFontSize(20);
  m_sliceTextProp->SetVerticalJustificationToBottom();
  m_sliceTextProp->SetJustificationToLeft();

  m_sliceTextMapper = vtkSmartPointer<vtkTextMapper>::New();
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
  m_sliceTextMapper->SetInput(msg.c_str());
  m_sliceTextMapper->SetTextProperty(m_sliceTextProp);

  m_sliceTextActor = vtkSmartPointer<vtkActor2D>::New();
  m_sliceTextActor->SetMapper(m_sliceTextMapper);
  m_sliceTextActor->SetPosition(15, 10);

  m_renderer = vtkSmartPointer<vtkRenderer>::New();
  m_renderer->AddActor(m_actor);
  m_renderer->AddActor(m_sliceTextActor);

  m_window = vtkSmartPointer<vtkRenderWindow>::New();
  m_window->AddRenderer(m_renderer);

  // Set up the interaction
  m_imageStyle = vtkSmartPointer<usInteractor2D>::New();

  m_interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  m_interactor->SetInteractorStyle(m_imageStyle);

  m_window->SetInteractor(m_interactor);
  m_window->Render();

  m_callback = vtkSmartPointer<usSlicingCallback>::New();
  m_callback->SetImageReslice(m_reslice);
  m_callback->setSliceOrientation(m_orientation);
  m_callback->SetInteractor(m_interactor);
  m_callback->SetTextMapper(m_sliceTextMapper);

  //m_imageStyle->AddObserver(vtkCommand::MouseMoveEvent, m_callback);
  //m_imageStyle->AddObserver(vtkCommand::LeftButtonPressEvent, m_callback);
  //m_imageStyle->AddObserver(vtkCommand::LeftButtonReleaseEvent, m_callback);
  m_imageStyle->AddObserver(vtkCommand::MouseWheelBackwardEvent, m_callback);
  m_imageStyle->AddObserver(vtkCommand::MouseWheelForwardEvent, m_callback);
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
  // Start interaction
  m_interactor->Start();
}

/**
* Set orientation of th view.
*/
void usViewer2D::setOrientation(us::Orientation orientation)
{
  m_orientation = orientation;
}
