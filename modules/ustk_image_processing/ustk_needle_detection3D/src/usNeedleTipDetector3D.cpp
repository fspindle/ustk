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

#include <visp3/ustk_needle_detection3D/usNeedleTipDetector3D.h>

#include <visp3/ustk_needle_detection3D/usVolumeProcessing.h>
#include <visp3/ustk_needle_detection/usNeedleDetectionTools.h>

#include <visp3/ustk_gui/usVTKConverter.h>

void usNeedleTipDetector3D::init(const usImagePostScan3D<unsigned char>& V, unsigned int nbCandidates,
				 double varThreshold)
{
  m_lastVolume = V;
  m_difference.resize(V.getDimX(), V.getDimY(), V.getDimZ());
  m_nbCandidates = nbCandidates;
  m_varThreshold = varThreshold;
  m_intThreshold = 0;
  m_candidates.resize(nbCandidates, 3);
  m_tip.resize(3);
  m_detected = false;
}

bool usNeedleTipDetector3D::detect(const usImagePostScan3D<unsigned char>& V)
{
  usVolumeProcessing::absoluteDifference(V, m_lastVolume, m_difference);
  vtkSmartPointer<vtkImageData> vtkImage = vtkSmartPointer<vtkImageData>::New();
  usVTKConverter::convert(m_difference,vtkImage);
  m_intThreshold = usNeedleDetectionTools::getThresholdedCoordinates(vtkImage,
								     m_candidates, m_nbCandidates);
  m_nbCandidates = m_candidates.getRows();
  m_tip = usNeedleDetectionTools::geometricMedian(m_candidates, m_nbCandidates, 3);
  m_variance = usNeedleDetectionTools::variance(m_candidates, m_nbCandidates, 3);
  std::cout << "Variance: " << m_variance << std::endl;
  m_lastVolume = V;
  return (m_variance < m_varThreshold);
}
