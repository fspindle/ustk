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
 * @file usViewer2D.h
 * @brief Class to display a 3D ultrasound image at screen (3 orthogonal planes), and interact with it.
 */

#ifndef US_VIEWER_2D_H
#define US_VIEWER_2D_H

#include <string>

//ViSP includes
#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_core/usImageRF3D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImagePreScan3D.h>
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePostScan3D.h>

// vtk includes
#include <vtkVersion.h>
#include <vtkImageData.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkRenderer.h>
#include <vtkImageMapper.h>
#include <vtkImageResliceMapper.h>
#include <vtkImageSlice.h>
#include <vtkMetaImageReader.h>
#include <vtkImagePlaneWidget.h>
#include <vtkPlane.h>
#include <vtkImageViewer2.h>
#include <vtkDICOMImageReader.h>
#include <vtkInteractorStyleImage.h>
#include <vtkActor2D.h>
#include <vtkTextProperty.h>
#include <vtkTextMapper.h>

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
#include <vtkAbstractTransform.h>


//UsTK includes
#include <visp3/ustk_gui/usInteractor2D.h>
#include <visp3/ustk_gui/usSlicingCallback.h>


/**
 * @class usViewer2D
 * @brief Class to display a 3D ultrasound image at screen, and interact with it.
 * @ingroup module_ustk_gui
 */
class VISP_EXPORT usViewer2D
{
public:

 /*! Enum to set the orientation of a 2D plane from a 3D image.*/
typedef enum {
    Xorientation,
    Yorientation,
    Zorientation,
  }Orientation;

  usViewer2D(usViewer2D::Orientation orientation, int slice);

  virtual ~usViewer2D();

  void initInteractorStyle(usViewer3D* viewer);

  void setSize(int height, int width);

  void setOrientation(usViewer2D::Orientation orientation);

  void start();

  void updateView();

private:
  //vtkImageReslice
  vtkSmartPointer<vtkImageReslice> m_imageReslice;

  vtkSmartPointer<vtkImageResliceMapper> m_resliceMapper;
  //LUT
  vtkSmartPointer<vtkLookupTable> m_table;

  //Color map
  vtkSmartPointer<vtkImageMapToColors> m_color;

  //actor
  vtkSmartPointer<vtkImageActor> m_actor;
  vtkSmartPointer<vtkImageSlice> m_imageSlice;

  //renderer
  vtkSmartPointer<vtkRenderer> m_renderer;

  //render window
  vtkSmartPointer<vtkRenderWindow> m_window;

  //render window interactor
  vtkSmartPointer<vtkRenderWindowInteractor> m_renderWindowInteractor;

  //interactor style
  vtkSmartPointer<usInteractor2D> m_interactorStyle;

  /*
  //viewer
  vtkSmartPointer<vtkImageViewer2> m_imageViewer;

  //slice text property
  vtkSmartPointer<vtkTextProperty> m_sliceTextProp;

  //slice text mapper
  vtkSmartPointer<vtkTextMapper> m_sliceTextMapper;

  //slice text actor
  vtkSmartPointer<vtkActor2D> m_sliceTextActor;


  //Image
  vtkSmartPointer<vtkImageData> m_image;*/
};

#endif //US_VIEWER_2D_H
