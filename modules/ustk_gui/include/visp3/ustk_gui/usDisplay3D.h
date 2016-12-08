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
 * @file usDisplay3D.h
 * @brief Class to display a ultrasound image at screen, and interact with it.
 */

#ifndef US_DISPLAY_3D_H
#define US_DISPLAY_3D_H

#include <string>

#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_core/usImageRF3D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImagePreScan3D.h>
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePostScan3D.h>

// some standard vtk headers
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

// headers needed for this example
#include <vtkImageData.h>
#include <vtkImageMapper3D.h>
#include <vtkImageCast.h>
#include <vtkMetaImageWriter.h>
#include <vtkMetaImageReader.h>
#include <vtkImageActor.h>
#include <vtkAutoInit.h>

/**
 * @class usDisplay3D
 * @brief Class to display a ultrasound image at screen, and interact with it.
 * @ingroup module_ustk_gui
 */
class VISP_EXPORT usDisplay3D
{
public:

  usDisplay3D();

  virtual ~usDisplay3D();


  /**
   * Initialization method.
   *
   * @param renderWinInteractor Pointer to the vtk render window interactor.
   * @param window_width Window width.
   * @param window_height Window height.
   * @param image_origin Coordinates of the image origin.
   * @param image_dims Image dimensions.
   * @param nSliceIdx_X Index of the rendered slice in the X direction.
   * @param nSliceIdx_Y Index of the rendered slice in the Y direction.
   * @param nSliceIdx_Z Index of the rendered slice in the Z direction.
   * @param nCamRotAngle_X Renderer camera angle.
   * @param nCamRotAngle_Y Renderer camera angle.
   * @param nCamRotAngle_Z Renderer camera angle.
   * @param nCamFarCls_t Renderer camera position.
   */
  void Init(vtkRenderWindowInteractor *renderWinInteractor, int window_width,  int window_height,
            int image_origin[3], int image_dims[3], int nSliceIdx_X, int nSliceIdx_Y, int nSliceIdx_Z,
            int nCamRotAngle_X, int nCamRotAngle_Y, int nCamRotAngle_Z, int nCamFarCls_t);

  /**
   * Renders the volume.
   */
  void render(const unsigned char *image_data, int data_size);

};

#endif //US_DISPLAY_3D_H
