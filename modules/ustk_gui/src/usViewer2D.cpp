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
usViewer2D::usViewer2D(vtkPlane *plane, double* origin)
{
  vtkSmartPointer<vtkMetaImageReader> reader =
      vtkSmartPointer<vtkMetaImageReader>::New();
  reader->SetFileName("/home/mpouliqu/Documents/usData/postscan/3D/postscan3d.mhd");
  reader->Update();

    //2D plane separate display test
  m_imageReslice = vtkSmartPointer<vtkImageReslice>::New();
  m_imageReslice->SetInputData(reader->GetOutput());
  //m_imageReslice->SetOutputDimensionality(2);

  std::cout << "origin = " << origin[0] << ", "<< origin[1] << ", "<< origin[2] << std::endl;
  //m_imageReslice->SetInformationInput(reader->GetOutput());
  m_imageReslice->SetInterpolationModeToLinear();




  // Create a greyscale lookup table
  m_table = vtkSmartPointer<vtkLookupTable>::New();
  m_table->SetRange(0, 500); // image intensity range
  m_table->SetValueRange(0.0, 1.0); // from black to white
  m_table->SetSaturationRange(0.0, 0.0); // no color saturation
  m_table->SetRampToLinear();
  m_table->Build();

  // Map the image through the lookup table
  m_color = vtkSmartPointer<vtkImageMapToColors>::New();
  m_color->SetLookupTable(m_table);
  m_color->SetInputConnection(m_imageReslice->GetOutputPort());

  //map the slice in the volume
  m_resliceMapper = vtkSmartPointer<vtkImageResliceMapper>::New();
  m_resliceMapper->SetInputConnection(m_color->GetOutputPort());
  m_resliceMapper->SetSlicePlane(plane);
  //m_resliceMapper->SliceFacesCameraOn();
  //m_resliceMapper->SliceAtFocalPointOn();

  // Display the image
  m_actor = vtkSmartPointer<vtkImageActor>::New();
  m_actor->SetMapper(m_resliceMapper);


  m_imageSlice = vtkSmartPointer<vtkImageSlice>::New();
  m_imageSlice->SetMapper(m_resliceMapper);

  m_renderer = vtkSmartPointer<vtkRenderer>::New();
  m_renderer->AddActor2D(m_imageSlice);
  m_renderer->ResetCamera();

  m_window = vtkSmartPointer<vtkRenderWindow>::New();
  m_window->AddRenderer(m_renderer);

  // Set up the interaction
  m_interactorStyle = vtkSmartPointer<usInteractor2D>::New();


  m_renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  m_renderWindowInteractor->SetInteractorStyle(m_interactorStyle);
  m_window->SetInteractor(m_renderWindowInteractor);

  m_window->Render();
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
  m_renderWindowInteractor->Start();
}

/**
* Init interactor.
*/
void usViewer2D::initInteractorStyle(usViewer3D* viewer)
{
  m_interactorStyle->SetImageViewer(m_imageReslice,0,10,0);
  m_interactorStyle->SetViewer3D(viewer);
  m_interactorStyle->SetRenderWindow2D(m_window);
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
  m_imageReslice->Update();
}
