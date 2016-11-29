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

/*                                                                -*-c++-*-
#----------------------------------------------------------------------------
#  
#----------------------------------------------------------------------------
*/

#include <visp3/ustk_needle_detection/us3DNeedleTracker.h>

//#include <visp3/ustk_needle_detection/usNeedleDetectionTools.h>

us3DNeedleTracker::us3DNeedleTracker()
{
  m_kalmanFilter = vpLinearKalmanFilterInstantiation();
  m_sigmaState = vpColVector(12);
  m_sigmaMeasure = vpColVector(6); // 1-3: entry ; 4-6: tip
  m_sigmaState = 1;
  m_sigmaMeasure = 1;
  m_kalmanFilter.initStateConstVel_MeasurePos(6, m_sigmaState, m_sigmaMeasure, 1);
  m_nPoints = 2;
  m_VOI = new int[6];
  m_margin = 10;
  m_initialized = false;
  m_points = vpMatrix(2,3);
  m_velocities = vpMatrix(2,3);
  m_maxLengthError = 50.0;
  m_maxAngleError = 0.8;
}

us3DNeedleTracker::us3DNeedleTracker(unsigned int nPoints)
{
  m_kalmanFilter = vpLinearKalmanFilterInstantiation();
  m_sigmaState = vpColVector(6*nPoints);
  m_sigmaMeasure = vpColVector(3*nPoints); // 1-3: entry ; 4-6: tip
  m_kalmanFilter.initStateConstVel_MeasurePos(3*nPoints, m_sigmaState, m_sigmaMeasure, 1.0);
  m_nPoints = nPoints;
  m_VOI = new int[6];
  m_margin = 10;
  m_initialized = false;
  m_points = vpMatrix(nPoints,3);
  m_velocities = vpMatrix(nPoints,3);
  m_maxLengthError = 50.0;
  m_maxAngleError = 0.8;
}

void us3DNeedleTracker::init()
{
  m_kalmanFilter.initStateConstVel_MeasurePos(3*m_nPoints, m_sigmaState, m_sigmaMeasure, 1.0);
}

void us3DNeedleTracker::setExtent(int extent[6])
{
  m_extent = extent;
}

bool us3DNeedleTracker::isInitialized()
{
  return m_initialized;
}

bool us3DNeedleTracker::addMeasure(vpColVector measure)
{
  /*
  std::cerr << "Adding measure ";
  for (unsigned int i=0; i<m_nPoints; ++i)
    for (unsigned int j=0; j<3; ++j)
      std::cerr << measure[j+3*i] << " ";
  std::cerr << "... ";
  */
  bool valid = true;
  if (!m_initialized)
    {
      init();
      m_kalmanFilter.filter(measure);
      for (int i=0; i<3*m_nPoints; i++)
	{
	  m_points[i/3][i%3] = m_kalmanFilter.Xest[2*i];
	  m_velocities[i/3][i%3] = m_kalmanFilter.Xest[2*i+1];
	}
      m_initialized = true;
    }
  else
    {
      vpColVector prediction = vpColVector(3*m_nPoints);
      //std::cerr << std::endl << "   predicting... ";
      m_kalmanFilter.prediction();
      //std::cerr << "done." << std::endl;
      for (int i=0; i<3*m_nPoints; i++)
	prediction[i] = m_kalmanFilter.Xpre[2*i];
      //std::cerr << "   checking measure coherence... ";
      double error = usNeedleDetectionTools::dist3(prediction.data+3*(m_nPoints-1), measure.data+3*(m_nPoints-1));
      //double angularError = UsNeedleDetectionTools::angle(prediction.data, prediction.data+3*(m_nPoints-1),measure.data, measure.data+3*(m_nPoints-1));
      //std::cerr << "done." << std::endl;
      std::cerr << "     Kalman position error: " << error << std::endl;
      if (error<m_maxLengthError)
	m_kalmanFilter.filtering(measure);
      else {
	//std::cerr << "Incoherent detection - using kalman prediction instead." << std::endl;
	valid = false;
	m_kalmanFilter.filtering(prediction);
      }
      for (int i=0; i<3*m_nPoints; i++) {
	  m_points[i/3][i%3] = m_kalmanFilter.Xest[2*i];
	  m_velocities[i/3][i%3] = m_kalmanFilter.Xest[2*i+1];
       }
    }
  for (int i=0; i<3; i++)
    {
      m_VOI[2*i] =
	vpMath::maximum(vpMath::minimum(vpMath::round(m_kalmanFilter.Xpre[2*i]-m_margin),
					vpMath::round(m_kalmanFilter.Xpre[2*i+6*(m_nPoints-1)]-m_margin)),
			m_extent[2*i]);
      m_VOI[2*i+1] =
	vpMath::minimum(vpMath::maximum(vpMath::round(m_kalmanFilter.Xpre[2*i]+m_margin),
				       vpMath::round(m_kalmanFilter.Xpre[2*i+6*(m_nPoints-1)]+m_margin)),
			m_extent[2*i+1]);
    }
  //std::cerr << "done." << std::endl;
  return valid;
}

vpMatrix us3DNeedleTracker::getPrediction() {
  vpMatrix prediction(m_nPoints, 3);
  for (int i=0; i<3*m_nPoints; i++) {
    prediction[i/3][i%3] = m_kalmanFilter.Xpre[2*i];
  }
  return prediction;
}

int *us3DNeedleTracker::getVOI() { return m_VOI; }

void us3DNeedleTracker::getPose(int i, double *pose)
{
  for (int j=0; j<3; j++)
      pose[j] = m_points[i][j];
}
 
void us3DNeedleTracker::getVelocity(int i, double *vel)
{
  for (int j=0; j<3; j++)
    vel[j] = m_velocities[i][j];
}

vpMatrix us3DNeedleTracker::getPoints() { return m_points; }

vpMatrix us3DNeedleTracker::getVelocities() { return m_velocities; }

double us3DNeedleTracker::getPredictedLength()
{
  double *prediction = new double[3*m_nPoints];
  for (int i=0; i<3*m_nPoints; i++)
    prediction[i] = m_kalmanFilter.Xpre[2*i];
  return usNeedleDetectionTools::dist3(prediction, prediction+3*(m_nPoints-1));
}

void us3DNeedleTracker::setSigmaStateEntry(double s0, double s1, double s2, double s3, double s4, double s5)
{
  m_sigmaState[0] = s0;
  m_sigmaState[1] = s1;
  m_sigmaState[2] = s2;
  m_sigmaState[3] = s3;
  m_sigmaState[4] = s4;
  m_sigmaState[5] = s5;
}

void us3DNeedleTracker::setSigmaStateEntry(double sigma)
{
  for (unsigned int i=0; i<6; i++)
    m_sigmaState[i] = sigma;
}

void us3DNeedleTracker::setSigmaStateTip(double s0, double s1, double s2, double s3, double s4, double s5)
{
  m_sigmaState[6] = s0;
  m_sigmaState[7] = s1;
  m_sigmaState[8] = s2;
  m_sigmaState[9] = s3;
  m_sigmaState[10] = s4;
  m_sigmaState[11] = s5;
}

void us3DNeedleTracker::setSigmaStateTip(double sigma)
{
  for (unsigned int i=6; i<6*m_nPoints; i++)
    m_sigmaState[i] = sigma;
}

void us3DNeedleTracker::setSigmaMeasureEntry(double s0, double s1, double s2)
{
  m_sigmaMeasure[0] = s0;
  m_sigmaMeasure[1] = s1;
  m_sigmaMeasure[2] = s2;
}

void us3DNeedleTracker::setSigmaMeasureEntry(double sigma)
{
  for (unsigned int i=0; i<3; i++)
    m_sigmaMeasure[i] = sigma;
}

void us3DNeedleTracker::setSigmaMeasureTip(double s0, double s1, double s2)
{
  m_sigmaMeasure[3] = s0;
  m_sigmaMeasure[4] = s1;
  m_sigmaMeasure[5] = s2;
}

void us3DNeedleTracker::setSigmaMeasureTip(double sigma)
{
  for (unsigned int i=3; i<3*m_nPoints; i++)
    m_sigmaMeasure[i] = sigma;
}

void us3DNeedleTracker::setMaxLengthError(double maxError) { m_maxLengthError = maxError; }
void us3DNeedleTracker::setMaxAngleError(double maxError) { m_maxAngleError = maxError; }
void us3DNeedleTracker::setVOIMargin(double margin) { m_margin = margin; }
