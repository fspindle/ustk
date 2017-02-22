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

#include <stdio.h>
#include <string.h>
#include <math.h>

// ViSP
#include <visp/vpMath.h>
#include <visp/vpMatrix.h>
#include <visp/vpLinearKalmanFilterInstantiation.h>

// VTK
#include <vtkPlane.h>

/**
 * @brief Kalman filter for needle entry point tracking
 * @author Pierre Chatelain
 * @date 2013-09-10
 *
 * This class implements a Kalman filter-based tracking of the needle entry point.
 *
 */
class VISP_EXPORT us3DEntryTracker
{
 public:
  /**
   * Constructor.
   */
  us3DEntryTracker();

  /**
   * Set the image extent.
   */
  void setExtent(int extent[6]);

  /**
   * Get the right-hand plane limit of the field of view as a vtk plane.
   */
  vtkPlane *getRPlane();

  /**
   * Get the left-hand plane limit of the field of view as a vtk plane.
   */
  vtkPlane *getLPlane();

  /**
   * Get the plane limit of the field of view where the needle is inserted as a vtk plane.
   */
  vtkPlane *getPlane();

  /**
   * Set the right-hand plane limit of the field of view as a vtk plane.
   */
  void setRPlane(vtkPlane *rPlane);

  /**
   * Set the left-hand plane limit of the field of view as a vtk plane.
   */
  void setLPlane(vtkPlane *lPlane);

  /**
   * Set the plane limit of the field of view where the needle is inserted as a vtk plane.
   */
  void setPlanes(vtkPlane *lPlane, vtkPlane *rPlane);

  /**
   * Set the angle of the field of view (in rad).
   */
  void setFOV(double fov);

  /**
   * Get the tracked pose of the entry point.
   */
  double *getPose();

  /**
   * Get the velocity of the entry point.
   */
  double *getVelocity();

  /**
   * Get the count of entry detections on the left-hand side.
   */
  int getLCount();

  /**
   * Get the count of entry detections on the right-hand side.
   */
  int getRCount();

  /**
   * Set the origin of the field of view.
   */
  void setOrigin(const vpColVector &origin);

  /**
   * Get the origin of the field of view.
   */
  double *getOrigin();

  /**
   * Get the index of the entry face (-1: left, +1: right, 0: not detected)
   */
  int getEntryFace();

  /**
   * Set the index of the entry face (-1: left, +1: right, 0: not detected)
   */
  void setEntryFace(int entryFace);

  /**
   * Get the coordinates of the volume of interest.
   */
  int *getVOI();

  /**
   * Check whether the tracker is initialized.
   */
  bool isInitialized();

  /**
   * Update the tracker with a new measurement.
   */
  void update(double point[3]);

  /**
   * Update the tracker with a new measurement.
   */
  void update(double x, double y, double z);
  
 private:
  int *m_extent;
  bool m_initialized;
  int m_entryFace;
  int *m_VOI;
  int m_lCount;
  int m_rCount;
  int m_requiredVotes;
  vpColVector m_sigmaState;
  vpColVector m_sigmaMeasure;
  double *m_origin;
  double *m_pose;
  double *m_velocity;
  vtkPlane *m_rPlane;
  vtkPlane *m_lPlane;
  vpLinearKalmanFilterInstantiation m_kalmanFilter;
  int m_margin;
};


