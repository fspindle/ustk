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

#include <visp3/ustk_gui/usDisplay3D.h>

/**
* Default constructor.
*/
usDisplay3D::usDisplay3D()
{
  //VTK_MODULE_INIT(vtkRenderingWindow);
}

/**
* Destructor.
*/
usDisplay3D::~usDisplay3D()
{

}


void usDisplay3D::Init(vtkRenderWindowInteractor *renderWinInteractor, int window_width,  int window_height, int image_origin[3], int image_dims[3], int nSliceIdx_X, int nSliceIdx_Y, int nSliceIdx_Z, int nCamRotAngle_X, int nCamRotAngle_Y, int nCamRotAngle_Z, int nCamFarCls_t)
{

}

void usDisplay3D::render(const unsigned char *image_data, int data_size)
{
    std::cout << "0" << std::endl;
   // Read and display file for verification that it was written correctly
   vtkSmartPointer<vtkMetaImageReader> reader = vtkSmartPointer<vtkMetaImageReader>::New();
   reader->SetFileName("/home/mpouliqu/Documents/usData/postscan/3D/postscan3d.mhd");
   reader->Update();

    std::cout << "1" << std::endl;
   vtkSmartPointer<vtkImageActor> actor = vtkSmartPointer<vtkImageActor>::New();
   actor->GetMapper()->SetInputConnection(reader->GetOutputPort());

    std::cout << "2" << std::endl;
   vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
   vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
   renderWindow->AddRenderer(renderer);
   vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
   renderWindowInteractor->SetRenderWindow(renderWindow);


    std::cout << "3" << std::endl;
   renderer->AddActor(actor);
   renderer->SetBackground(.2, .3, .4);

    std::cout << "4" << std::endl;
   renderWindow->Render();
   renderWindowInteractor->Start();




/*
  prepareVolumeData(image_data, data_size);

  // TO AVOID image copy but the image are rotated (BUG not found for the moment)
  // ImageX.bitmap = (unsigned char*)m_resliceX->GetOutput()->GetScalarPointer();
  // ImageY.bitmap = (unsigned char*)m_resliceY->GetOutput()->GetScalarPointer();
  // ImageZ.bitmap = (unsigned char*)m_resliceZ->GetOutput()->GetScalarPointer();

  // ViSP display [pchatela 2013-09-09]
  // Initialize the display of the planes
  vpPoseVector planeXpose(0.0, 0.0, m_nSliceIdx_X - m_iD[2] / 2.0, 0.0, 0.0, 0.0);
  vpPoseVector planeYpose(0,0,0,M_PI/2,0,0);
  vpPoseVector planeZpose(m_nSliceIdx_Z - m_iD[0] / 2.0, 0.0, 0.0, 0.0, -M_PI / 2., 0.0);
  m_oMX.buildFrom(planeXpose);
  m_oMY.buildFrom(planeYpose);
  m_oMZ.buildFrom(planeZpose);
  //Init3Dplanes(m_oMX, m_oMY, m_oMZ); // Create, init and render the 3 planes
  //Get3Dplanes();

  //displayImageX();
  //displayImageZ();

  // Render volume
  m_pvtkRenderWindow->Render();*/

}
/*
void usDisplay3D::NewVTK(void)
{
  m_pvtk3DImage =vtkImageData::New();
  m_pvtkPlaneWidgetX =vtkImagePlaneWidget::New();
  m_pvtkPlaneWidgetY =vtkImagePlaneWidget::New();
  m_pvtkPlaneWidgetZ =vtkImagePlaneWidget::New();
  m_pvtkOutlineFilter =vtkOutlineFilter::New();
  m_pvtkOutlineMapper =vtkPolyDataMapper::New();
  m_pvtkOutlineActor =vtkActor::New();
  m_pvtkRenderer = vtkRenderer::New();
  m_pvtkRenderWindow =vtkRenderWindow::New();

  // Needle [pchatela 2013-07-24]
  m_pvtkNeedleMapper = vtkPolyDataMapper::New();
  m_pvtkNeedleActor = vtkActor::New();
  m_pvtkNeedlePolyData = vtkPolyData::New();
  m_pvtkNeedleTransform = vtkTransform::New();
  m_pvtkNeedleTransformFilter = vtkTransformPolyDataFilter::New();

  if(m_OffScreen==true)
    m_pvtkRenderWindowInteractor = vtkRenderWindowInteractor::New();

  m_pvtkAxesActor =vtkAxesActor::New();

  //m_pvtkReslice =vtkImageReslice::New();
  //m_pvtk_oMi = vtkMatrix4x4::New();
  //m_pvtkTransform =vtkTransform::New();

  // lines
  for(int i=0; i<3; i++){
    m_pvtkLineSrc[i] =vtkLineSource::New();
    m_pvtkLineMapper[i] =vtkPolyDataMapper::New();
    m_pvtkLineActor[i]=vtkActor::New();
  }

  // 3 Planes  A.K 12/09/11
  m_resliceX = vtkImageReslice::New();
  m_resliceY = vtkImageReslice::New();
  m_resliceZ = vtkImageReslice::New();

  m_transform_planeX = vtkTransform::New();
  m_transform_planeY = vtkTransform::New();
  m_transform_planeZ = vtkTransform::New();

  m_vtk_oMX = vtkMatrix4x4::New();
  m_vtk_oMY = vtkMatrix4x4::New();
  m_vtk_oMZ = vtkMatrix4x4::New();

}


void usDisplay3D::SetVTK(int window_width,  int window_height)
{
// Read and display file
   vtkSmartPointer<vtkMetaImageReader> reader =
      vtkSmartPointer<vtkMetaImageReader>::New();
   reader->SetFileName(filePath.c_str());
   reader->Update();

   vtkSmartPointer<vtkImageActor> actor =
      vtkSmartPointer<vtkImageActor>::New();
   actor->GetMapper()->SetInputConnection(reader->GetOutputPort());

   vtkSmartPointer<vtkRenderer> renderer =
      vtkSmartPointer<vtkRenderer>::New();
   vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
   renderWindow->AddRenderer(renderer);
   vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
   renderWindowInteractor->SetRenderWindow(renderWindow);

   renderer->AddActor(actor);
   renderer->SetBackground(.2, .3, .4);

   renderWindow->Render();
   renderWindowInteractor->Start();
}*/
