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

#include <visp3/ustk_gui/usViewer3D.h>

/**
* Default constructor.
*/
usViewer3D::usViewer3D()
{
    vtkSmartPointer<vtkMetaImageReader> reader =
    vtkSmartPointer<vtkMetaImageReader>::New();
  reader->SetFileName("/home/mpouliqu/Documents/usData/postscan/3D/postscan3d.mhd");
  reader->Update();

  m_image = reader->GetOutput();
  m_image->GetDimensions(m_imageDims);

  m_imageResliceMapperX = vtkSmartPointer<vtkImageResliceMapper>::New();
  m_imageResliceMapperY = vtkSmartPointer<vtkImageResliceMapper>::New();
  m_imageResliceMapperZ = vtkSmartPointer<vtkImageResliceMapper>::New();
#if VTK_MAJOR_VERSION <= 5
  m_imageResliceMapperX->SetInputConnection(reader->GetOutputPort());
  m_imageResliceMapperY->SetInputConnection(reader->GetOutputPort());
  m_imageResliceMapperZ->SetInputConnection(reader->GetOutputPort());
#else
  m_imageResliceMapperX->SetInputData(reader->GetOutput());
  m_imageResliceMapperY->SetInputData(reader->GetOutput());
  m_imageResliceMapperZ->SetInputData(reader->GetOutput());
#endif


  //plane widget to control the slicing
  //vtkSmartPointer<vtkImagePlaneWidget> planeWidget;
  //planeWidget->SetInputData(reader->GetOutput());
  //planeWidget->SetPlaneOrientationToXAxes();
  //Hard coded plane selection
  //planeWidget->SetSliceIndex(10);

  m_image->SetOrigin(0,0,0);


  m_planeX = vtkSmartPointer<vtkPlane>::New();
  m_planeX->SetNormal(1,0,0);
  m_planeX->SetOrigin(50,0,0);
  m_planeY = vtkSmartPointer<vtkPlane>::New();
  m_planeY->SetNormal(0,1,0);
  m_planeY->SetOrigin(0,100,0);
  m_planeZ = vtkSmartPointer<vtkPlane>::New();
  m_planeZ->SetNormal(0,0,1);
  m_planeZ->SetOrigin(0,0,50);

  //select plane
  m_imageResliceMapperX->SetSlicePlane(m_planeX);
  m_imageResliceMapperY->SetSlicePlane(m_planeY);
  m_imageResliceMapperZ->SetSlicePlane(m_planeZ);

  m_imageSliceX = vtkSmartPointer<vtkImageSlice>::New();
  m_imageSliceX->SetMapper(m_imageResliceMapperX);
  m_imageSliceY = vtkSmartPointer<vtkImageSlice>::New();
  m_imageSliceY->SetMapper(m_imageResliceMapperY);
  m_imageSliceZ = vtkSmartPointer<vtkImageSlice>::New();
  m_imageSliceZ->SetMapper(m_imageResliceMapperZ);

  // Setup renderers
  m_renderer = vtkSmartPointer<vtkRenderer>::New();
  m_renderer->AddActor(m_imageSliceX);
  m_renderer->AddActor(m_imageSliceY);
  m_renderer->AddActor(m_imageSliceZ);
  m_renderer->SetBackground(0.5, 0.5, 0.5);
  m_renderer->ResetCamera();


  // Setup render window
  m_renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  m_renderWindow->SetSize(300, 300);
  m_renderWindow->AddRenderer(m_renderer);

  // Setup render window interactor
  m_renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();

  m_interactorStyle = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();

  m_renderWindowInteractor->SetInteractorStyle(m_interactorStyle);

  // Render and start interaction
  m_renderWindowInteractor->SetRenderWindow(m_renderWindow);
  m_renderWindowInteractor->Initialize();

  m_renderWindow->Render();
}

/**
* Destructor.
*/
usViewer3D::~usViewer3D()
{

}

/**
* Size setter of the view.
* @param height Height of the view in pixels.
* @param width Width of the view in pixels.
*
*/
void usViewer3D::setSize(int height, int width)
{
  m_renderWindow->SetSize(height, width);
}

/**
* Start interaction.
*
*/
void usViewer3D::start()
{
 m_renderWindowInteractor->Start();
}

/**
* Slice the X plane.
* @param sliceNumber Slice number of the image to display, in X axis.
*
*/
void usViewer3D::sliceX(int sliceNumber)
{
  std::cout << "sliceX 0" << std::endl;
  //if(sliceNumber < 0 || sliceNumber > m_imageDims[0])
//   /throw(vpException(vpException::badValue,"X slice number out of image"));

  std::cout << "sliceX 1" << std::endl;
  m_planeX->SetOrigin(sliceNumber,0,0);
  std::cout << "sliceX 2" << std::endl;
  m_renderWindow->Render();
}

/**
* Slice the Y plane.
* @param sliceNumber Slice number of the image to display, in Y axis.
*
*/
void usViewer3D::sliceY(int sliceNumber)
{
  if(sliceNumber < 0 || sliceNumber > m_imageDims[1])
   throw(vpException(vpException::badValue,"Y slice number out of image"));

  m_planeY->SetOrigin(0,sliceNumber,0);
}

/**
* Slice the Z plane.
* @param sliceNumber Slice number of the image to display, in Z axis.
*
*/
void usViewer3D::sliceZ(int sliceNumber)
{
  if(sliceNumber < 0 || sliceNumber > m_imageDims[2])
   throw(vpException(vpException::badValue,"Z slice number out of image"));

  m_planeZ->SetOrigin(0,0,sliceNumber);
}

/**
* Get the matrix corresponding to X image slice.
* @return The 4*4 matrix
*/
vtkMatrix4x4* usViewer3D::getXSliceMatrix()
{
  return m_imageSliceX->GetMatrix();
}

/**
* Get the matrix corresponding to Y image slice.
* @return The 4*4 matrix
*/
vtkMatrix4x4* usViewer3D::getYSliceMatrix()
{
  return m_imageSliceY->GetMatrix();
}

/**
* Get the matrix corresponding to Z image slice.
* @return The 4*4 matrix
*/
vtkMatrix4x4* usViewer3D::getZSliceMatrix()
{
  return m_imageSliceZ->GetMatrix();
}

/**
* Get the origin of the X slice plane.
* @return Array of 3 doubles containig the origin coordinates
*/
double* usViewer3D::getXSliceOrigin()
{
  return m_planeX->GetOrigin();
}

/**
* Get the origin of the Y slice plane.
* @return Array of 3 doubles containig the origin coordinates
*/
double* usViewer3D::getYSliceOrigin()
{
  return m_planeY->GetOrigin();
}

/**
* Get the origin of the X slice plane.
* @return Array of 3 doubles containig the origin coordinates
*/
double* usViewer3D::getZSliceOrigin()
{
  return m_planeZ->GetOrigin();
}



/**
* Get the X plane.
* @return vtkPlane
*/
vtkPlane* usViewer3D::getXPlane()
{
  return m_planeX;
}

/**
* Get the Y plane.
* @return vtkPlane
*/
vtkPlane* usViewer3D::getYPlane()
{
  return m_planeY;
}

/**
* Get the Z plane.
* @return vtkPlane
*/
vtkPlane* usViewer3D::getZPlane()
{
  return m_planeZ;
}
