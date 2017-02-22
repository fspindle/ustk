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
#include <float.h>
#include <ctime>

#include <visp3/ustk_gui/usGuiConfig.h>

#include <visp3/ustk_needle_detection/usNeedleDetectionTools.h>
#include <visp3/ustk_needle_detection3D/us3DNeedleModel.h>

#include <visp/vpNoise.h> // random number generation
#include <visp/vpDebug.h> // debug and trace
#include <visp/vpMatrix.h>
#include <visp/vpMath.h>
#include <visp/vpColVector.h>

#if defined USTK_HAVE_VTK_QT
#include <vtkImageData.h>
#endif

/**
 * This class implements the RANSAC algorithm for detection of lines in 3D images.
 *
 * @brief Needle-specific RANSAC algorithm
 * @author Pierre Chatelain
 *
 */
class VISP_EXPORT softRansac
{
public:
  /**
   * Default constructor.
   */
  softRansac();
  /**
   * Constructor.
   */
  softRansac(unsigned int s, double costThresh, double failThresh);
  /**
   * Initializes the RANSAC algorithm parameters.
   */
  void init(unsigned int s, double costThresh, double failThresh);
  /**
   * Set the maximal number of trials performed by the RANSAC algorithm.
   */
  void setMaxTrials(unsigned int maxTrials);
  /**
   * Returns the maximal number of trial being performed.
   */
  unsigned int getMaxTrials();
  void setCostThreshold(double value);
  void setFailThreshold(double value);
  void setModelLikelihoodWeight(double value);
  /**
   * Forces the position of point i to the value x.
   */
  void forcePoint(unsigned int i, vpColVector &x);
  void forcePoint(unsigned int i, double *x);
  /**
   * Unset the value of point i. Point i is then chosen randomly during the RANSAC algorithm.
   */
  void freePoint(unsigned int i);

#if defined USTK_HAVE_VTK_QT
  /**
   * Runs the RANSAC algorithm on the given data.
   *
   * @param x The point cloud.
   * @param bestModel The best found model.
   * @param bestInliers The set of inliers for the best model.
   * @param n_inliers_p The number of inliers for the best model.
   */
  int run(vpMatrix &candidates,
	  vtkImageData *imageData,
	  us3DNeedleModel &previousModel,
	  us3DNeedleModel &needleModel,
	  const vpColVector &origin,
	  const vpColVector &entryPlane,
	  int VOI[6],
	  unsigned int dropThreshold,
	  unsigned int nActivePoints);
#endif

  /**
   * Returns the permutation sorting the vpColVector C.
   */
  static unsigned int *argSort(const vpColVector &C, bool ascent = 1);
  /**
   * Checks wether a set of indexes is degenerate. Returns true iff at least two indexes are equal.
   */
  static bool isDegenerate(unsigned int *ind, unsigned int npts);
  /**
   * Cost function.
   */
  static void computeCost(vpMatrix &X, vpMatrix &M, vpMatrix &T, double *cost);
private:
  /**
   * The order of the polynomial curve to detect.
   */
  unsigned int m_order;
  /**
   * The cost threshold. Points with a cost lower than this threshold (and only them) are considered as inliers.
   */
  double m_costThresh;
  /**
   * The maximal probability of failure.
   */
  double m_failThresh;
  /**
   * The maximal number of trials to perform.
   */
  int m_maxTrials;
  /**
   * The maximal number of iterations.
   */
  int m_maxIter;
  /**
   * The best-so-far model.
   */
  vpMatrix m_bestModel;
  /**
   * The best-so-far inlier set.
   */ 
  vpMatrix m_bestInliers;
  /**
   * The best-so-far number of inliers.
   */
  unsigned int m_nInliers;
  /**
   * The number of fixed points.
   */
  unsigned int m_nForced;
  /**
   * The indexes of the fixed points.
   */
  int *m_forced;
  /**
   * The fixed points.
   */
  vpMatrix m_forcedPoints;
  double m_modelLikelihoodWeight;
};

