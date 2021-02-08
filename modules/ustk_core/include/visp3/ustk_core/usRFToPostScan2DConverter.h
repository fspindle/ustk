/****************************************************************************
 *
 * This file is part of the ustk software.
 * Copyright (C) 2016 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ustk with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at ustk@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Authors:
 * Pedro Patlan
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usRFToPostScan2DConverter.h
 * @brief 2D converter from RF to post-scan.
 */

#ifndef __usRFToPostScan2DConverter_h_
#define __usRFToPostScan2DConverter_h_

#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_FFTW)

// visp/ustk includes
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_core/usPreScanToPostScan2DConverter.h>
#include <visp3/ustk_core/usRFToPreScan2DConverter.h>

/**
 * @class usRFToPostScan2DConverter
 * @brief 2D conversion from RF signal to post-scan image
 * @ingroup module_ustk_core
 *
 * This class allows to convert 2D RF ultrasound images to post-scan.
 * Here is an example to show how to use it :
 *
 * \code
#include <visp3/core/vpImage.h>
#include <visp3/ustk_core/usRFToPostScan2DConverter.h>
#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_core/usImagePostScan2D.h>

int main()
{
#if defined(USTK_HAVE_FFTW)
  // example of 2D post-scan image settings
  unsigned int width = 320;
  unsigned int height = 240;
  double transducerRadius = 0.045;
  double scanLinePitch = 0.0012;
  unsigned int scanLineNumber = 256;
  bool isTransducerConvex = true;
  double axialResolution = 0.002;

  vpImage<short int> I(height, width);
  usImageRF2D<short int> rfImage; // to fill (image + settings)
  rfImage.setTransducerRadius(transducerRadius);
  rfImage.setScanLinePitch(scanLinePitch);
  rfImage.setScanLineNumber(scanLineNumber);
  rfImage.setTransducerConvexity(isTransducerConvex);
  rfImage.setAxialResolution(axialResolution);

  usImagePostScan2D<unsigned char> postscanImage;
  postscanImage.setHeightResolution(0.0005);
  postscanImage.setWidthResolution(0.0005); // pixels of 0.5*0.5 mm in output
  usRFToPostScan2DConverter converter;
  converter.setConversionParameters(postscanImage,rfImage.getRFSampleNumber()/10,rfImage.getScanLineNumber(),10);
  converter.convert(rfImage,postscanImage);
#endif

  return 0;
}
 * \endcode
 *
 */
class VISP_EXPORT usRFToPostScan2DConverter
{
public:
  usRFToPostScan2DConverter(int decimationFactor = 10);

  ~usRFToPostScan2DConverter();

  void convert(const usImageRF2D<short int> &rfImage, usImagePostScan2D<unsigned char> &postScanImage);

  void setConversionParameters(const usImagePostScan2D<unsigned char> &inputSettings, const int BModeSampleNumber,
                               const int scanLineNumber, const int decimationFactor);

private:
  usRFToPreScan2DConverter m_RFConverter;
  usPreScanToPostScan2DConverter m_scanConverter;
};

#endif // USTK_HAVE_FFTW
#endif // __usRFToPostScan2DConverter_h_
