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

#ifndef US_NEEDLE_TRACKER_SIR_3D_H
#define US_NEEDLE_TRACKER_SIR_3D_H

// ViSP
#include <visp/vpNoise.h>

// UsNeedleDetection
#include <visp3/ustk_needle_detection3D/us3DNeedleModel.h>

// UsTK
#include <visp3/ustk_core/usImagePostScan3D.h>

/**
 * @brief Needle detection pipeline with particle filter.
 * @author Pierre Chatelain
 * @date 2014-07-02
 *
 * This class defines the particle-based needle detection pipeline.
 */
class VISP_EXPORT usNeedleTrackerSIR3D
{
 public:
  /**
   * Default constructor.
   */
  usNeedleTrackerSIR3D();
  
  /**
   * Destructor.
   */
  ~usNeedleTrackerSIR3D();
 
  /**
   * Initialization method.
   * Initializes all attributes with default values and the given image resolution.
   * Should be called before to start the detection.
   */
  void init(double resolution, unsigned int dims[3], double origin[3], double fov,
	    unsigned int nPoints, unsigned int nParticles, const us3DNeedleModel &needle);

  /**
   * Initialization method.
   * Initializes all attributes with default values and the given image resolution.
   * Should be called before to start the detection.
   * @param vol Reference to the post-scan volume.
   * @param resolution The image resolution (in meters).
   * @param nPoints The number of control points.
   */
  void init(const usImagePostScan3D<unsigned char>& vol, double fov, unsigned int nPoints,
	    unsigned int nParticles, const us3DNeedleModel &needle);
  /*
#ifdef USNEEDLEDETECTION_HAVE_US3DSCAN
  void usNeedleTrackerSIR3D::init(CUSScanConv *converter, double resolution, unsigned int nPoints,
				  unsigned int nParticles, const us3DNeedleModel &needle);
#endif
  */

  /**
   * Returns the detected needle model.
   */
  us3DNeedleModel *getNeedle();

  /**
   * Get a specific particle.
   */
  us3DNeedleModel *getParticle(unsigned int i);

  /**
   * Get a specific particle's weight.
   */
  double getWeight(unsigned int i);

  /**
   * Set the standard deviation for the update noise.
   */
  void setSigma(double s);

  /**
   * Set the standard deviation along the insertion direction.
   */
  void setSigma1(double s);

  /**
   * Set the standard deviation orthogonal to the insertion direction.
   */
  void setSigma2(double s);

  /**
   * Runs the needle detection process with the current post-scan volume.
   *
   * @param vol The input post-scan volume.
   */
  void run(usImagePostScan3D<unsigned char>& vol, double dx, double dy, double dz);

  /**
   * Runs the needle detection process with the current post-scan volume.
   *
   * @param vol The input post-scan volume.
   */
  void run(usImagePostScan3D<unsigned char>& vol, double v);

  /**
   * Computing the likelihood of the model for a given observation.
   */
  double computeLikelihood(us3DNeedleModel *model, usImagePostScan3D<unsigned char> &vol);

  /**
   * Resample the particles proportionnaly to their weights.
   */
  void resample();

 private:
  unsigned int m_nParticles;
  unsigned int m_nPoints;
  unsigned int m_nPointsCurrent;
  us3DNeedleModel *m_needleModel;
  us3DNeedleModel **m_particles;
  double *m_weights;
  vpGaussRand m_noise;
  vpUniRand m_sample;
  vpMatrix m_sigma;

  // Image parameters
  double m_resolution;
  unsigned int m_dims[3];
  double m_origin[3];
  double m_fov;
};

#endif // US_NEEDLE_TRACKER_SIR_3D
