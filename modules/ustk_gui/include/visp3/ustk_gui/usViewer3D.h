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

/**
 * @file usViewer3D.h
 * @brief Class to display a 3D ultrasound image at screen (3 orthogonal planes), and interact with it.
 */

#ifndef US_VIEWER_3D_H
#define US_VIEWER_3D_H

#include <string>

#include <visp3/ustk_gui/usGuiConfig.h>

#ifdef USTK_HAVE_VTK

#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_core/usImageRF3D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImagePreScan3D.h>
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePostScan3D.h>

// vtk headers
#include <vtkVersion.h>
#include <vtkImageData.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkRenderer.h>
#include <vtkImageMapper.h>
#include <vtkImageResliceMapper.h>
#include <vtkImageReslice.h>
#include <vtkImageSlice.h>
#include <vtkMetaImageReader.h>
#include <vtkImagePlaneWidget.h>
#include <vtkPlane.h>
#include <vtkCamera.h>
#include <vtkLookupTable.h>
#include <vtkImageMapToColors.h>
#include <vtkImageActor.h>
#include <vtkInformation.h>
#include <vtkStreamingDemandDrivenPipeline.h>

/**
 * @class usViewer3D
 * @brief Class to display a 3D ultrasound image at screen, and interact with it.
 * @ingroup module_ustk_gui
 */
class VISP_EXPORT usViewer3D
{
public:

  usViewer3D();

  virtual ~usViewer3D();

  //getters
  vtkMatrix4x4* getXSliceMatrix();
  vtkMatrix4x4* getYSliceMatrix();
  vtkMatrix4x4* getZSliceMatrix();

  double* getXSliceOrigin();
  double* getYSliceOrigin();
  double* getZSliceOrigin();

  vtkPlane* getXPlane();
  vtkPlane* getYPlane();
  vtkPlane* getZPlane();

  void setSize(int height, int width);

  void sliceX(int sliceNumber);
  void sliceY(int sliceNumber);
  void sliceZ(int sliceNumber);

  void start();

private:
  //mappers
  vtkSmartPointer<vtkImageResliceMapper> m_imageResliceMapperX;
  vtkSmartPointer<vtkImageResliceMapper> m_imageResliceMapperY;
  vtkSmartPointer<vtkImageResliceMapper> m_imageResliceMapperZ;

  //planes used to select the slice to display
  vtkSmartPointer<vtkPlane> m_planeX;
  vtkSmartPointer<vtkPlane> m_planeY;
  vtkSmartPointer<vtkPlane> m_planeZ;

  //ImageSlices (actors)
  vtkSmartPointer<vtkImageSlice> m_imageSliceX;
  vtkSmartPointer<vtkImageSlice> m_imageSliceY;
  vtkSmartPointer<vtkImageSlice> m_imageSliceZ;

  //Renderer
  vtkSmartPointer<vtkRenderer> m_renderer;

  //RenderWindow
  vtkSmartPointer<vtkRenderWindow> m_renderWindow;

  //Interactor
  vtkSmartPointer<vtkRenderWindowInteractor> m_renderWindowInteractor;

  //interaction style
  vtkSmartPointer<vtkInteractorStyleTrackballCamera> m_interactorStyle;

  //Image
  vtkSmartPointer<vtkImageData> m_image;
  int m_imageDims[3];
};

#endif
#endif //US_VIEWER_3D_H
