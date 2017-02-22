/****************************************************************************
 *
 * This file is part of the UsNeedleDetection software.
 * Copyright (C) 2013 - 2016 by Inria. All rights reserved.
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
 * Authors:
 * Pierre Chatelain
 * Alexandre Krupa
 *
 *****************************************************************************/

#ifndef US_NEEDLE_TIP_DETECTOR_3D
#define US_NEEDLE_TIP_DETECTOR_3D

#include <visp3/ustk_core/usImagePostScan3D.h>
#include <visp/vpColVector.h>
#include <visp/vpMatrix.h>

class VISP_EXPORT usNeedleTipDetector3D {
 public:
  void init(const usImagePostScan3D<unsigned char>& V, unsigned int nbCandidates, double varThreshold);
  bool detect(const usImagePostScan3D<unsigned char>& V);

 private:
  usImagePostScan3D<unsigned char> m_lastVolume;
  usImagePostScan3D<unsigned int> m_difference;
  unsigned int m_nbCandidates;
  double m_variance;
  double m_varThreshold;
  unsigned int m_intThreshold;
  vpMatrix m_candidates;
  vpColVector m_tip;
  bool m_detected;
};

#endif // US_NEEDLE_TIP_DETECTOR_3D
