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
 * @file usRFToPreScan2DConverter.h
 * @brief 2D scan-converter
 */

#ifndef __usRFToPreScan2DConverter_h_
#define __usRFToPreScan2DConverter_h_

#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_FFTW)
// external includes
#include <fftw3.h>

// std includes
#include <cmath>
#include <complex>
#include <vector>

// visp/ustk includes
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_core/usLogCompressor.h>

/**
 * @class usRFToPreScan2DConverter
 * @brief 2D conversion from RF signal to pre-scan image
 * @ingroup module_ustk_core
 *
 * This class allows to convert 2D RF ultrasound images to pre-scan.
 * Here is an example to show how to use it :
 *
 * \code
#include <visp3/core/vpImage.h>
#include <visp3/ustk_core/usRFToPreScan2DConverter.h>
#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>

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

  usImagePreScan2D<unsigned char> prescanImage; // output
  usRFToPreScan2DConverter converter;
  converter.convert(rfImage, prescanImage);
#endif
  return 0;
}
 * \endcode
 */
class VISP_EXPORT usRFToPreScan2DConverter
{
  friend class usRFToPreScan3DConverter;

public:
  usRFToPreScan2DConverter(int decimationFactor = 10);

  ~usRFToPreScan2DConverter();

  void convert(const usImageRF2D<short int> &rfImage, usImagePreScan2D<unsigned char> &preScanImage);

  int getDecimationFactor();

  void setDecimationFactor(int decimationFactor);

private:
  void init(int widthRF, int heigthRF);
  void enveloppeDetection(const short *s, double *out);

  usLogCompressor m_logCompressor;

  int m_decimationFactor;

  fftw_complex *m_fft_in, *m_fft_out, *m_fft_conv, *m_fft_out_inv;
  fftw_plan m_p, m_pinv;

  double *m_env;
  unsigned char *m_comp;

  int m_signalSize;
  int m_scanLineNumber;

  bool m_isInit;
};

#endif // USTK_HAVE_FFTW
#endif // __usRFToPreScan2DConverter_h_
