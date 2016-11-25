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
 * @file usScanConverter3D.h
 * @brief 3D scan-converter
 */

#ifndef US_SCAN_CONVERTER_3D_H
#define US_SCAN_CONVERTER_3D_H

#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/ustk_core/usImagePostScan3D.h>
#include <visp3/ustk_core/usImagePreScan3D.h>

/**
 * @class usScanConverter3D
 * @brief 3D scan-converter
 * @ingroup module_ustk_core
 *
 * This class allows to convert 3D pre-scan ultrasound images to post-scan.
 * The converter should be initialized through init() and then applied through run().
 */
class VISP_EXPORT usScanConverter3D
{
 public:

  usScanConverter3D();

  ~usScanConverter3D();

  void init(const usImagePreScan3D<unsigned char> &inputSettings, const double xResolution,
            const double yResolution,const double zResolution);

  void run(const usImagePreScan3D<unsigned char> &preScanImage, usImagePostScan3D<unsigned char> &postScanImage);

  vpImage<unsigned char> m_Image;
 private:
  //double interpolateLinear(const vpImage<unsigned char>& I, double x, double y, double z);

  vpMatrix m_xMap;
  usImage3D<double> m_yMap;
  usImage3D<double> m_zMap;
  int m_dimX, m_dimY, m_dimZ;


  double m_xResolution;
  double m_yResolution;
  double m_zResolution;
};

#endif // US_SCAN_CONVERTER_3D_H
