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
 * Pierre Chatelain
 *
 *****************************************************************************/

#include <visp3/ustk_core/usScanConverter3D.h>
#include <visp/vpMath.h>

usScanConverter3D::usScanConverter3D() {}

usScanConverter3D::~usScanConverter3D() {}

/**
* Initialize the scan-converter.
* @param inputSettings Pre-scan settings : transducer settings and motor settings.
* @param xResolution Size of a pixel in x direction of the post scan image built.
* @param yResolution Size of a pixel in y direction of the post scan image built.
* @param zResolution Size of a pixel in z direction of the post scan image built.
*/
void usScanConverter3D::init(const usImagePreScan3D<unsigned char> &inputSettings, const double xResolution,
            const double yResolution,const double zResolution)
{
  m_xResolution = xResolution;
  m_yResolution = yResolution;
  m_zResolution = zResolution;

  m_dimX = vpMath::round(inputSettings.getDepth() * sin(inputSettings.getScanLinePitch()*(inputSettings.getScanLineNumber()-1)/2.0) / m_xResolution); // X-axis size (px)

  //!\\ TO CORRECT, IMAGE SHOULD BE SMALLER IN Y : REMOVE DISTANCE BETWEEN CENTER OF SCANLINES AND TOP PLANE IN IMAGE
  m_dimY = vpMath::round(inputSettings.getDepth() / m_yResolution); // Y-axis size (px)

  m_dimZ = vpMath::round(inputSettings.getDepth() * sin(inputSettings.getFramePitch()*(inputSettings.getFrameNumber()-1)/2.0) / m_zResolution); // Z-axis size (px)

  m_Image.resize(m_dimY, m_dimX);
  m_xMap.resize(m_dimX, m_dimY);
  //m_yMap.resize(m_height, m_width);
  //m_zMap.resize(m_height, m_width);
  
  std::cout<< "m_dimX " << m_dimX << std::endl;
  std::cout<< "m_dimY " << m_dimY << std::endl;
  double x, y;
  for (unsigned int i = 0; i < m_dimX; ++i) {
    for (unsigned int j = 0; j < m_dimY; ++j) {
      m_xMap[i][j] = (inputSettings.getAxialResolution()*j + inputSettings.getTransducerRadius()) *
                      sin(-((inputSettings.getScanLinePitch()*(inputSettings.getScanLineNumber()-1)/2.0) +
                          inputSettings.getScanLinePitch()*i)) / xResolution;

      /*for(int k = 0; k < m_dimZ; k++) {
      m_yMap(i,j,k,
                      ((inputSettings.getAxialResolution() * j + inputSettings.getTransducerRadius()) *
                      cos(-((inputSettings.getScanLinePitch()*(inputSettings.getScanLineNumber()-1)/2.0) +
                      inputSettings.getScanLinePitch()*i) -inputSettings.getTransducerRadius() + inputSettings.getMotorRadius()) *
                      cos(inputSettings.getFrameNumber()*inputSettings.getFramePitch()/2 -
                      (i + inputSettings.getScanLineNumber() * k) *
                      inputSettings.getFrameNumber()*inputSettings.getFramePitch() /
                      (inputSettings.getScanLineNumber()*inputSettings.getFrameNumber() -1)) + inputSettings.getTransducerRadius() -
                      inputSettings.getMotorRadius()) / xResolution);

      m_zMap(i,j,k,   ((inputSettings.getAxialResolution() * j + inputSettings.getTransducerRadius()) *
                      cos(-((inputSettings.getScanLinePitch()*(inputSettings.getScanLineNumber())-1)/2.0) +
                      inputSettings.getScanLinePitch()*i) -inputSettings.getTransducerRadius() + inputSettings.getMotorRadius()) *
                      sin(inputSettings.getFrameNumber()*inputSettings.getFramePitch()/2 -
                      (i + inputSettings.getScanLineNumber() * k) *
                      inputSettings.getFrameNumber()*inputSettings.getFramePitch() /
                      (inputSettings.getScanLineNumber()*inputSettings.getFrameNumber() -1))  / xResolution);
      }*/
      m_Image[j][i] = m_xMap[i][j];
      std::cout << m_xMap[i][j] << std::endl;

    }
    std::cout << "END OF SCANLINE" << std::endl << std::endl;
  }
}

/**
* Run the scan-converter.
* @param [out] postScanImage Post-scan image : result of the scan conversion.
* @param [in] preScanImage Pre-scan image to convert.
*/
void usScanConverter3D::run(const usImagePreScan3D<unsigned char> &preScanImage, usImagePostScan3D<unsigned char> &postScanImage)
{
  /*postScanImage.resize(m_height, m_width);
  for (unsigned int i = 0; i < m_height; ++i)
    for (unsigned int j = 0; j < m_width; ++j) {
      double u = m_rMap[i][j];
      double v = m_tMap[i][j];
      postScanImage(i, j, (unsigned char)interpolateLinear(preScanImage, u, v));
    }

  //saving settings in postScanImage
  postScanImage.setHeightResolution(m_yResolution);
  postScanImage.setWidthResolution(m_xResolution);
  postScanImage.setScanLineNumber(m_scanLineNumber);
  postScanImage.setTransducerConvexity(preScanImage.isTransducerConvex());
  postScanImage.setScanLinePitch(m_settings.getScanLinePitch());
  postScanImage.setTransducerRadius(m_settings.getTransducerRadius());*/
}

/*
double usScanConverter3D::interpolateLinear(const vpImage<unsigned char>& I, double x, double y)
{
  int x1 = (int)floor(x);
  int x2 = (int)ceil(x);
  int y1 = (int)floor(y);
  int y2 = (int)ceil(y);
  double val1, val2;

  if ((0 <= x) && (x < I.getHeight()) && (0 <= y) && (y < I.getWidth())) {
    // Check whether the indices are within the image extent
    if (x1 < 0) ++x1;
    if (y1 < 0) ++y1;
    if (x2 >= static_cast<int>(I.getHeight())) --x2;
    if (y2 >= static_cast<int>(I.getWidth())) --y2;

    // Check whether the target is on the grid
    if (x1==x2) {
      val1 = I(x1, y1);
      val2 = I(x1, y2);
    }
    else {
      val1 = (x2 - x) * I(x1, y1) + (x - x1) * I(x2, y1);
      val2 = (x2 - x) * I(x1, y2) + (x - x1) * I(x2, y2);
    }
    if (y1==y2)
      return val1;
    else
      return (y2 - y) * val1 + (y - y1) * val2;
  }
  return 0.0;
}*/
