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

#include <visp3/ustk_needle_detection3D/us3DEntryTracker.h>

us3DEntryTracker::us3DEntryTracker()
{
  m_kalmanFilter = vpLinearKalmanFilterInstantiation();
  m_sigmaState = vpColVector(6);
  m_sigmaMeasure = vpColVector(3);
  for (int i = 0; i < 3; i++)
    {
      m_sigmaState[i] = 1;
      m_sigmaState[i+3] = 1;
      m_sigmaMeasure[i] = 10;
    }
  m_kalmanFilter.initStateConstVel_MeasurePos(3, m_sigmaState, m_sigmaMeasure, 1);
  m_initialized = false;
  m_pose = new double[3];
  m_velocity = new double[3];
  m_origin = new double[3];
  m_rPlane = vtkPlane::New();
  m_lPlane = vtkPlane::New();
  m_entryFace = 0;
  m_VOI = new int[6];
  m_lCount = 0;
  m_rCount = 0;
  m_requiredVotes = 3;
  m_margin = 20;
  m_extent = new int[6];
}

void us3DEntryTracker::setExtent(int extent[6])
{
  m_extent = extent;
}

vtkPlane *us3DEntryTracker::getRPlane() { return m_rPlane; }

vtkPlane *us3DEntryTracker::getLPlane() { return m_lPlane; }

vtkPlane* us3DEntryTracker::getPlane()
{
  if (m_entryFace==-1)
    return m_lPlane;
  else if (m_entryFace==1)
    return m_rPlane;
  else
    std::cerr << "Error: entry point not detected." << std::endl;
  return m_lPlane;
}

void us3DEntryTracker::setRPlane(vtkPlane *rPlane) { m_rPlane = rPlane; }

void us3DEntryTracker::setLPlane(vtkPlane *lPlane) { m_lPlane = lPlane; }

void us3DEntryTracker::setPlanes(vtkPlane *lPlane, vtkPlane *rPlane)
{
  m_lPlane = lPlane;
  m_rPlane = rPlane;
}

void us3DEntryTracker::setFOV(double fov)
{
  m_lPlane->SetNormal(sin(fov/2), cos(fov/2), 0);
  m_rPlane->SetNormal(sin(fov/2), -cos(fov/2), 0);
}

double *us3DEntryTracker::getPose() { return m_pose; }

double *us3DEntryTracker::getVelocity() { return m_velocity; }

int us3DEntryTracker::getLCount() { return m_lCount; }

int us3DEntryTracker::getRCount() { return m_rCount; }

void us3DEntryTracker::setOrigin(const vpColVector &origin)
{
  for (int i=0; i<3; i++)
    m_origin[i] = origin[i];
  m_lPlane->SetOrigin(m_origin);
  m_rPlane->SetOrigin(m_origin);
}

double *us3DEntryTracker::getOrigin() { return m_origin; }

int us3DEntryTracker::getEntryFace() { return m_entryFace; }

void us3DEntryTracker::setEntryFace(int entryFace) { m_entryFace = entryFace; }

int *us3DEntryTracker::getVOI()
{
  return m_VOI;
}

bool us3DEntryTracker::isInitialized() { return m_initialized; }

void us3DEntryTracker::update(double point[3])
{
  double proj[3];
  vpColVector vpProj(3);
  double lDist;
  double rDist;
  if (m_entryFace == 0)
    {
      lDist = m_lPlane->DistanceToPlane(point);
      rDist = m_rPlane->DistanceToPlane(point);
      std::cerr << "lDist: " << lDist << std::endl;
      std::cerr << "rDist: " << rDist << std::endl;
      if (rDist < 50)
	m_rCount++;
      else if (lDist < 50)
	m_lCount++;
      if (m_lCount >= m_requiredVotes)
	{
	  m_entryFace = -1;
	  //std::cerr << "Needle detected on the left." << std::endl;
	}
      if (m_rCount >= m_requiredVotes)
	{
	  m_entryFace = 1;
	  //std::cerr << "Needle detected on the right." << std::endl;
	}
    }
  if (m_entryFace != 0)
    {
      if (m_entryFace == -1)
	m_lPlane->ProjectPoint(point, proj);
      else
	m_rPlane->ProjectPoint(point, proj);
      for (int i=0; i<3; i++)
	vpProj[i] = proj[i];
      if (!m_initialized)
	{
	  m_kalmanFilter.filter(vpProj);
	  m_initialized = true;
	}
      else
	{
	  m_kalmanFilter.filtering(vpProj);
	  m_kalmanFilter.prediction();
	}
      m_pose[0] = m_kalmanFilter.Xpre[0];
      m_pose[1] = m_kalmanFilter.Xpre[2];
      m_pose[2] = m_kalmanFilter.Xpre[4];
      m_velocity[0] = m_kalmanFilter.Xpre[1];
      m_velocity[1] = m_kalmanFilter.Xpre[3];
      m_velocity[2] = m_kalmanFilter.Xpre[5];
      m_VOI[0] = vpMath::maximum(vpMath::round(m_kalmanFilter.Xpre[0] - m_margin), m_extent[0]);
      m_VOI[1] = vpMath::minimum(vpMath::round(m_kalmanFilter.Xpre[0] + m_margin), m_extent[1]);
      m_VOI[2] = vpMath::maximum(vpMath::round(m_kalmanFilter.Xpre[2] - m_margin), m_extent[2]);
      m_VOI[3] = vpMath::minimum(vpMath::round(m_kalmanFilter.Xpre[2] + m_margin), m_extent[3]);
      m_VOI[4] = vpMath::maximum(vpMath::round(m_kalmanFilter.Xpre[4] - m_margin), m_extent[4]);
      m_VOI[5] = vpMath::minimum(vpMath::round(m_kalmanFilter.Xpre[4] + m_margin), m_extent[5]);
    }
}

void us3DEntryTracker::update(double x, double y, double z)
{
  double point[3];
  point[0] = x;
  point[1] = y;
  point[2] = z;
  double proj[3];
  vpColVector vpProj(3);
  double lDist;
  double rDist;
  if (m_entryFace == 0)
    {
      lDist = m_lPlane->DistanceToPlane(point);
      rDist = m_rPlane->DistanceToPlane(point);
      if (lDist > rDist)
	m_rCount++;
      else
	m_lCount++;
      if (m_lCount >= m_requiredVotes)
	  m_entryFace = -1;
      if (m_rCount >= m_requiredVotes)
	  m_entryFace = 1;
    }
  if (m_entryFace != 0)
    {
      if (m_entryFace == -1)
	m_lPlane->ProjectPoint(point, proj);
      else
	m_rPlane->ProjectPoint(point, proj);
      for (int i=0; i<3; i++)
	vpProj[i] = proj[i];
      if (!m_initialized)
	{
	  m_kalmanFilter.filter(vpProj);
	  m_initialized = true;
	}
      else
	{
	  m_kalmanFilter.filtering(vpProj);
	  m_kalmanFilter.prediction();
	}
      m_pose[0] = m_kalmanFilter.Xpre[0];
      m_pose[1] = m_kalmanFilter.Xpre[2];
      m_pose[2] = m_kalmanFilter.Xpre[4];
      m_velocity[0] = m_kalmanFilter.Xpre[1];
      m_velocity[1] = m_kalmanFilter.Xpre[3];
      m_velocity[2] = m_kalmanFilter.Xpre[5];
    }
}
