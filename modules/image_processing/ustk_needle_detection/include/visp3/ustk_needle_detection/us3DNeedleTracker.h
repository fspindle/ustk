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

#ifndef us3DNeedleTracker_h
#define us3DNeedleTracker_h

#include <stdio.h>
#include <string.h>

// ViSP
#include <visp/vpMath.h>
#include <visp/vpMatrix.h>
#include <visp/vpLinearKalmanFilterInstantiation.h>

// UsNeedleDetection
#include <visp3/ustk_needle_detection/usNeedleDetectionTools.h>

/**
 * @brief Kalman filter for needle tracking
 * @author Pierre Chatelain
 * @date 2013-09-10
 *
 * This class implements the needle tracking based on a Kalman filter.
 *
 */
class us3DNeedleTracker
{
 public:
  /**
   * Default Contructor.
   * Initializes the Kalman filter to default values.
   * Do not use this if you need more than 2 controls points!
   */
  us3DNeedleTracker();

  /**
   * Default Contructor.
   * Initializes the Kalman filter to default values.
   */
  us3DNeedleTracker(unsigned int nPoints);

  /**
   * Reinitializes the Kalman filter.
   */
  void init();

  /**
   * Set the image extent.
   */
  void setExtent(int extent[6]);

  /**
   * Returns true iff the tracking has been initialized (i.e. has received at least one measure.
   */
  bool isInitialized();

  /**
   * Provides a new measure to the Kalman filter.
   */
  bool addMeasure(vpColVector measure);

  /**
   * Returns the predicted volume of interest.
   */
  int *getVOI();

  /**
   * Returns the filtered points.
   */
  vpMatrix getPoints();

  /**
   * Returns the filtered velocities.
   */
  vpMatrix getVelocities();

  /**
   * Provides the filtered position of the ith point.
   */
  void getPose(int i, double *pose);

  /**
   * Provides the velocity of the ith point.
   */
  void getVelocity(int i, double *vel);

  /**
   * Returns the predicted position of the control points.
   */
  vpMatrix getPrediction();

  /**
   * Returns the predicted needle length.
   */
  double getPredictedLength();

  /**
   * Set the state variance for the entry point.
   */
  void setSigmaStateEntry(double sigma);

  /**
   * Set the state variance for all points.
   */
  void setSigmaStateTip(double sigma);

  /**
   * Set the measure variance for the entry point.
   */
  void setSigmaMeasureEntry(double sigma);

  /**
   * Set the measure variance for all points.
   */
  void setSigmaMeasureTip(double sigma);
  void setSigmaStateEntry(double s0, double s1, double s2, double s3, double s4, double s5);
  void setSigmaStateTip(double s0, double s1, double s2, double s3, double s4, double s5);
  void setSigmaMeasureEntry(double s0, double s1, double s2);
  void setSigmaMeasureTip(double s0, double s1, double s2);
  void setMaxLengthError(double maxError);
  void setMaxAngleError(double maxError);
  void setVOIMargin(double margin);

 private:
  /**
   * Number of control points.
   */
  unsigned int m_nPoints;

  /**
   * Position of the control points.
   */
  vpMatrix m_points;

  /**
   * Velocities of the control points.
   */
  vpMatrix m_velocities;

  /**
   * Margin around the curve used to define the volume of interest.
   */
  double m_margin;
  int *m_extent;
  bool m_initialized;

  /**
   * Vector containing the variance of the state noise.
   */
  vpColVector m_sigmaState;

  /**
   * Vector containing the variance of the measurement noise.
   */
  vpColVector m_sigmaMeasure;

  /**
   * Predicted volume of interest.
   */
  int *m_VOI;

  /**
   * Kalman filter.
   */
  vpLinearKalmanFilterInstantiation m_kalmanFilter;

  /**
   * Maximum length error.
   */
  double m_maxLengthError;

  /**
   * Maximum angular error.
   */
  double m_maxAngleError;
  
};

#endif
