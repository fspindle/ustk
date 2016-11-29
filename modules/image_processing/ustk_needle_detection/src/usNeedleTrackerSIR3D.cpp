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

#include <visp3/ustk_needle_detection/usNeedleTrackerSIR3D.h>

usNeedleTrackerSIR3D::usNeedleTrackerSIR3D()
{
  m_needleModel = NULL;
  m_particles = NULL;
  m_weights = NULL;
  m_noise.setSigmaMean(1.0, 0.0);
  m_noise.seed(42);
  m_sample = vpUniRand(42);
  m_sigma.eye(3);
}

usNeedleTrackerSIR3D::~usNeedleTrackerSIR3D()
{
  if (m_needleModel) {
    delete m_needleModel;
    m_needleModel = NULL;
  }
  if (m_particles) {
    for (unsigned int i = 0; i < m_nParticles; ++i) {
      if (m_particles[i]) {
	delete m_particles[i];
	m_particles[i] = NULL;
      }
      delete [] m_particles;
      m_particles = NULL;
    }
  }
  if (m_weights) {
    delete [] m_weights;
    m_weights = NULL;
  }
}

void usNeedleTrackerSIR3D::init(double resolution, unsigned int dims[3], double origin[3], double fov,
				unsigned int nPoints, unsigned int nParticles,
				const us3DNeedleModel &needle)
{
  m_resolution = resolution;
  m_fov = fov;
  m_dims[0] = dims[0];
  m_dims[1] = dims[1];
  m_dims[2] = dims[2];
  m_origin[0] = origin[0];
  m_origin[1] = origin[1];
  m_origin[2] = origin[2];
  m_nPoints = nPoints;
  m_nPointsCurrent = needle.getOrder();
  m_nParticles = nParticles;
  m_needleModel = new us3DNeedleModel(needle);
  m_particles = new us3DNeedleModel*[m_nParticles];
  for (unsigned int i = 0; i < m_nParticles; ++i)
    m_particles[i] = new us3DNeedleModel(needle);
  m_weights = new double[m_nParticles];
  for (unsigned int i = 0; i < m_nParticles; ++i)
    m_weights[i] = 1.0 / m_nParticles;
}

void usNeedleTrackerSIR3D::init(const usImagePostScan3D<unsigned char> &vol, double origin[3], double fov, unsigned int nPoints,
        unsigned int nParticles, const us3DNeedleModel &needle) {
  unsigned int dims[3];
  dims[0] = vol.getDimX();
  dims[1] = vol.getDimY();
  dims[2] = vol.getDimZ();
  init(1.0, dims, origin, fov, nPoints, nParticles, needle);
}

/*
#ifdef USNEEDLEDETECTION_HAVE_US3DSCAN
void usNeedleTrackerSIR3D::init(CUSScanConv *converter, double resolution, unsigned int nPoints,
				unsigned int nParticles, const us3DNeedleModel &needle)
{
  
}
#endif
*/

us3DNeedleModel *usNeedleTrackerSIR3D::getNeedle() { return m_needleModel; }

us3DNeedleModel *usNeedleTrackerSIR3D::getParticle(unsigned int i) {
  if (i >= m_nParticles) {
    std::cerr << "Error: in usNeedleTrackerSIR3D::getParticle(): "
	      << "Particle index is out of range." << std::endl;
    exit(EXIT_FAILURE);
  }
  return m_particles[i];
}

double usNeedleTrackerSIR3D::getWeight(unsigned int i) {
  if (i >= m_nParticles) {
    std::cerr << "Error: in usNeedleTrackerSIR3D::getWeight(): "
	      << "Particle index is out of range." << std::endl;
    exit(EXIT_FAILURE);
  }
  return m_weights[i];
}

void usNeedleTrackerSIR3D::setSigma(double s) {
  m_noise.setSigmaMean(s, 0.0);  
}

void usNeedleTrackerSIR3D::setSigma1(double s) {
  m_sigma[0][0] = s;
}

void usNeedleTrackerSIR3D::setSigma2(double s) {
  m_sigma[1][1] = s;
  m_sigma[2][2] = s;
}

void usNeedleTrackerSIR3D::run(usImagePostScan3D<unsigned char>& vol, double dx, double dy, double dz)
{
  vpMatrix controlPoints;
  vpMatrix meanControlPoints;

  // Sample particles
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    controlPoints = m_particles[i]->getControlPoints();
    //controlPoints[0][0] += m_noise();
    //controlPoints[1][0] += m_noise();
    //controlPoints[2][0] += m_noise();
    for (unsigned int j = 1; j < m_nPointsCurrent-1; ++j) {
      controlPoints[0][j] += (dx / (m_nPointsCurrent-1) + m_noise() / 10.0);
      controlPoints[1][j] += (dy / (m_nPointsCurrent-1) + m_noise() / 10.0);
      controlPoints[2][j] += (dz / (m_nPointsCurrent-1) + m_noise() / 10.0);
    }
    controlPoints[0][m_nPointsCurrent-1] += (dx + m_noise());
    controlPoints[1][m_nPointsCurrent-1] += (dy + m_noise());
    controlPoints[2][m_nPointsCurrent-1] += (dz + m_noise());
    m_particles[i]->setControlPoints(controlPoints.t());
  }

  // Compute weights
  double sumWeights = 0.0;
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    m_weights[i] *= computeLikelihood(m_particles[i], vol);
    sumWeights += m_weights[i];
  }
  
  // Normalize the weights and estimate the effective number of particles
  double sumSquare = 0.0;
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    m_weights[i] /= sumWeights;
    sumSquare += vpMath::sqr(m_weights[i]);
  }
  
  double n_eff = 1.0 / sumSquare;
  std::cout << "Neff = " << n_eff << std::endl;

  // Resampling
  if (n_eff < m_nParticles / 2.0) {
    resample();
  }

  // Compute mean
  meanControlPoints.resize(3, m_nPointsCurrent);
  meanControlPoints = 0.0;
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    meanControlPoints += m_weights[i] * m_particles[i]->getControlPoints();
  }
 
  m_needleModel->setControlPoints(meanControlPoints.t());

  if (m_needleModel->getLength() > 10.0) {
    m_needleModel->changePolynomialOrder(m_nPoints);
    for (unsigned int i = 0; i < m_nParticles; ++i) {
      m_particles[i]->changePolynomialOrder(m_nPoints);
    }
  }
}


void usNeedleTrackerSIR3D::run(usImagePostScan3D<unsigned char>& vol, double v)
{
  vpMatrix controlPoints;
  vpMatrix meanControlPoints;
  vpColVector direction;
  vpMatrix S_noise_tip;
  vpMatrix S_noise_inter;
  vpColVector noise(3);
  vpMatrix U(3,3);

  // Sample particles
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    controlPoints = m_particles[i]->getControlPoints();

    // Get needle direction
    direction = m_particles[i]->getTangent(1.0);
    direction /= direction.euclideanNorm();

    // Compute eigen vectors
    for (unsigned int j = 0; j < 3; ++j)
      U[j][0] = direction[j];
    U[0][1] = - direction[1];
    U[1][1] = direction[0];
    U[2][1] = 0.0;

    U.insert(vpColVector::crossProd(U.getCol(0), U.getCol(1)), 0, 2);

    // Compute noise covariance
    S_noise_tip = U * m_sigma * U.t();
    S_noise_inter.eye(3);
    S_noise_inter *= m_sigma[1][1];

    // Compute tip velocity vector
    direction *= v;

    // Entry point
    //controlPoints[0][0] += m_noise();
    //controlPoints[1][0] += m_noise();
    //controlPoints[2][0] += m_noise();

    // Intermediate control points
    for (unsigned int j = 1; j < m_nPointsCurrent-1; ++j) {

      // Generate noise
      for (unsigned int k = 0; k < 3; ++k)
	noise[k] = m_noise();
      noise = S_noise_inter * noise;

      // Update control points
      for (unsigned int k = 0; k < 3; ++k)
	controlPoints[k][j] += direction[k] / (m_nPointsCurrent-1) + noise[k] / 4.0;
    }

    // Tip

    // Generate noise
    for (unsigned int k = 0; k < 3; ++k)
      noise[k] = m_noise();
    noise = S_noise_tip * noise;
    
    // Update control points
    for (unsigned int k = 0; k < 3; ++k)
      controlPoints[k][m_nPointsCurrent-1] += direction[k] + noise[k];

    m_particles[i]->setControlPoints(controlPoints.t());
  }

  // Compute weights
  double sumWeights = 0.0;
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    m_weights[i] *= computeLikelihood(m_particles[i], vol);
    sumWeights += m_weights[i];
  }
  
  // Normalize the weights and estimate the effective number of particles
  double sumSquare = 0.0;
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    m_weights[i] /= sumWeights;
    sumSquare += vpMath::sqr(m_weights[i]);
  }
  
  double n_eff = 1.0 / sumSquare;
  //std::cout << "Neff = " << n_eff << std::endl;

  // Resampling
  if (n_eff < m_nParticles / 2.0) {
    resample();
  }

  // Compute mean
  meanControlPoints.resize(3, m_nPointsCurrent);
  meanControlPoints = 0.0;
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    meanControlPoints += m_weights[i] * m_particles[i]->getControlPoints();
  }
 
  m_needleModel->setControlPoints(meanControlPoints.t());

  if ((m_needleModel->getLength()) > 50.0 && (m_nPoints != m_nPointsCurrent)) {
    std::cout << "Changing polynomial order from " << m_nPointsCurrent << " to " << m_nPoints
	      << std::endl;
    us3DNeedleModel *newModel = new us3DNeedleModel(m_needleModel->changePolynomialOrder(m_nPoints));
    delete m_needleModel;
    m_needleModel = newModel;
    for (unsigned int i = 0; i < m_nParticles; ++i) {
      us3DNeedleModel *newModel =
	new us3DNeedleModel(m_particles[i]->changePolynomialOrder(m_nPoints));
      delete m_particles[i];
    m_particles[i] = newModel;
    }
    m_nPointsCurrent = m_nPoints;
  }
}

double usNeedleTrackerSIR3D::computeLikelihood(us3DNeedleModel *model, usImagePostScan3D<unsigned char> &vol)
{
  double l = 0.0;
  vpColVector point;
  unsigned int c = 0;
  for (double t = 0; t <= 1.0; t += 0.01) {
    point = model->getPoint(t);
    int x = vpMath::round(point[0]);
    int y = vpMath::round(point[1]);
    int z = vpMath::round(point[2]);
    if ((0 <= x) && (x < m_dims[0]) && (0 <= y) && (y < m_dims[1]) && (0 <= z) && (z < m_dims[2])) {
      if (vol(x, y, z) == 0.0)
	return 0.0;
      l += vol(x, y, z);
    }
    ++c;
  }
  l /= c;
  point = model->getPoint(1.0);
  int x = vpMath::round(point[0]);
  int y = vpMath::round(point[1]);
  int z = vpMath::round(point[2]);
  if ((0 <= x) && (x < m_dims[0]) && (0 <= y) && (y < m_dims[1]) && (0 <= z) && (z < m_dims[2]))
    l += 0.15 * vol(x, y, z);
  return l;
}

void usNeedleTrackerSIR3D::resample() {
  // std::cout << "Resampling..." << std::endl;
  double x;
  double sumWeights;
  int *idx = new int[m_nParticles];
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    x = m_sample();
    sumWeights = 0.0;
    for (unsigned int j = 0; j < m_nParticles; ++j) {
      if (x < sumWeights + m_weights[j]) {
	idx[i] = j;
	break;
      }
      sumWeights += m_weights[j];
    }
  }
  us3DNeedleModel **oldParticles = new us3DNeedleModel*[m_nParticles];
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    oldParticles[i] = new us3DNeedleModel(*m_particles[i]);
  }
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    delete m_particles[i];
    // std::cout << idx[i] << " (" << m_weights[i] << ") ";
    m_particles[i] = new us3DNeedleModel(*oldParticles[idx[i]]);
  }
  // std::cout << std::endl;
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    delete oldParticles[i];
    oldParticles[i] = NULL;
  }
  delete [] oldParticles;
  delete [] idx;
  for (unsigned int i = 0; i < m_nParticles; ++i) {
    m_weights[i] = 1.0 / m_nParticles;
  }
}
