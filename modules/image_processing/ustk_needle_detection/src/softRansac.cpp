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

#include <visp3/ustk_needle_detection/softRansac.h>

SoftRansac::SoftRansac()
{
  //m_bestInliers.init();
  m_nInliers = 0;
}

SoftRansac::SoftRansac(unsigned int s, double costThresh, double failThresh)
{
  m_order = s;
  m_costThresh = costThresh;
  m_failThresh = failThresh;
  m_bestModel = vpMatrix(s,s);
  //m_bestInliers.init();
  m_nInliers = 0;
  m_nForced = 0;
  m_forced = new int[s];
  for (unsigned int i=0; i<s; i++)
    m_forced[i] = false;
  m_forcedPoints = vpMatrix(s,3);
  m_maxIter = 200;
  m_maxTrials = 1000;
  m_modelLikelihoodWeight = 1.0;
}

void SoftRansac::init(unsigned int s, double costThresh, double failThresh)
{
  m_order = s;
  m_costThresh = costThresh;
  m_failThresh = failThresh;
  m_bestModel = vpMatrix(s,s);
  m_nInliers = 0;
  m_nForced = 0;
  m_forced = new int[s];
  for (unsigned int i=0; i<s; i++)
    m_forced[i] = false;
  m_forcedPoints = vpMatrix(s,3);
  m_maxIter = 200;
  m_maxTrials = 1000;
  m_modelLikelihoodWeight = 1.0;
}

void SoftRansac::setMaxTrials(unsigned int maxTrials) { m_maxTrials = maxTrials; }

unsigned int SoftRansac::getMaxTrials() { return m_maxTrials; }

void SoftRansac::setCostThreshold(double value) { m_costThresh = value; }
void SoftRansac::setFailThreshold(double value) { m_failThresh = value; }
void SoftRansac::setModelLikelihoodWeight(double value) { m_modelLikelihoodWeight = value; }

void SoftRansac::forcePoint(unsigned int i, vpColVector &x)
{
  if (!m_forced[i])
    {
      m_nForced++;
      m_forced[i] = true;
    }
  m_forcedPoints.getRow(i) = x.t();
}

void SoftRansac::forcePoint(unsigned int i, double *x)
{
  if (!m_forced[i])
    {
      m_nForced++;
      m_forced[i] = true;
    }
  for (unsigned int j=0; j<3; j++)
    m_forcedPoints[i][j] = x[j];
}

void SoftRansac::freePoint(unsigned int i)
{
  if (m_forced[i])
    {
      m_nForced--;
      m_forced[i] = false;
    }
}

//#if defined USNEEDLEDETECTION_HAVE_VTK
int SoftRansac::run(vpMatrix &candidates,
		    vtkImageData *imageData,
		    us3DNeedleModel &previousModel,
		    us3DNeedleModel &needleModel,
		    const vpColVector &origin,
		    const vpColVector &entryPlane,
		    int VOI[6],
		    unsigned int dropThreshold,
		    unsigned int nActivePoints)
{
  //std::cerr << "In SoftRansac::run()" << std::endl;
  double eps = 1e-6 ;

  // Add forced points
  unsigned int nCandidates = candidates.getRows();
  unsigned int npts = nCandidates + m_nForced;
  vpMatrix x(npts, 3);
  for (unsigned int i=0; i<nCandidates; i++)
    for (unsigned int j=0; j<3; j++)
      x[i][j] = candidates[i][j];
  unsigned int counter = 0;
  for (unsigned int i=0; i<m_order; i++)
    if (m_forced[i])
      {
	//std::cerr << "        point " << i << " forced to "
	//	  << m_forcedPoints[i][0] << " "
	//	  << m_forcedPoints[i][1] << " "
	//	  << m_forcedPoints[i][2] << std::endl;
	for (unsigned int j=0; j<3; j++)
	  x[counter+nCandidates][j] = m_forcedPoints[i][j];
	counter++;
      }

  // Sentinel value allowing detection of solution failure.
  bool solutionFind = false ;
  double bestscore = -DBL_MAX;
  unsigned int bestNumberOfInliers = 0;
  double score;
  double N = 1;            // Dummy initialisation for number of trials.
  
  vpUniRand random(42/*(const long)time(NULL)*/);
  vpColVector inliers(npts);
  unsigned int *ind = new unsigned int[m_order];
  int numiter = 0;
  int numtrials = 0;
  int ninliers = 0;
  double residual = 0.0;
  vpMatrix mss(nActivePoints,3); // Minimal Sample Set
  vpMatrix tapprox(nActivePoints,npts); // Approximate curvilinear coordinate
  vpMatrix tapprox_mss(nActivePoints,nActivePoints); // Approximate curvilinear coordinate
  double tapp;
  double tapps;
  double entry[3];
  double tip[3];
  vpColVector dirVec(3);
  vpColVector ndirVec(3);
  unsigned int *perm;
  bool degenerate;
  double *cost = new double[npts];
  double orientation;
  double norm;
  us3DNeedleModel candidateModel(nActivePoints);
  candidateModel.setNumberOfRenderingLines(previousModel.getNumberOfRenderingLines());

  inliers = vpColVector(npts);

  //std::cerr << "Expected length: " << previousModel.getLength() << std::endl;

  while( N > numiter)
    {
      // Generate s random indicies in the range 1..npts
      degenerate = true;
      while (degenerate && numtrials < m_maxTrials) {
	for  (unsigned int i=0; i < nActivePoints; i++) {
	  counter = 0;
	  if (m_forced[i]) {
	    ind[i] = counter + nCandidates;
	    counter++;
	  }
	  else
	    ind[i] = (unsigned int)ceil(random()*nCandidates) - 1;
	}
	degenerate = isDegenerate(ind, nActivePoints);
	numtrials++;
      }
      if (degenerate) {
	vpTRACE("All sampled configurations are degenerate. Less than %d sample points?", m_order);
	break;
      }
      
      // Safeguard against being stuck in this loop forever
      if (numtrials >= m_maxTrials)
	{
	  vpTRACE("ransac reached the maximum number of %d trials", m_maxTrials);
	  break ;
	}

      // Fit model to this random selection of data points
      for  (unsigned int i=0 ; i < nActivePoints ; i++)
	for (unsigned int j=0; j<3; j++)
	    mss[i][j] = x[ind[i]][j];

      // Estimate approximate curvilinear coordinates
      tapprox_mss = usNeedleDetectionTools::approximateCoordinates(mss, mss, nActivePoints);

      // Sort points and reestimate coordinates
      perm = argSort(tapprox_mss.getRow(1).t());

      if (isDegenerate(perm, nActivePoints))
	continue;

      for (unsigned int i=0 ; i < nActivePoints ; i++)
	for (unsigned int j=0; j < 3; j++)
	    mss[i][j] = x[ind[perm[i]]][j];

      // Reject if parallel to the limit plane
      double dotProduct = abs((mss.t().getCol(0) - mss.t().getCol(nActivePoints-1)) * entryPlane)
	/ (mss.t().getCol(0) -  mss.t().getCol(nActivePoints-1)).euclideanNorm();

      if (dotProduct < eps)
	continue;

      // Compute model parameters
      candidateModel.setControlPoints(mss);

      // Find entry point and tip
      if (!usNeedleDetectionTools::findEntry(*candidateModel.getModel(), entry, nActivePoints, origin,
					     entryPlane, VOI))
	continue;

      if (!usNeedleDetectionTools::findTipUsingMeanValues(imageData, *candidateModel.getModel(), VOI,
							  tip, nActivePoints, dropThreshold,
							  previousModel.getLength()))
	continue;

      for (unsigned int j=0; j<3; ++j) {
	mss[0][j] = entry[j];
	mss[nActivePoints-1][j] = tip[j];
      }

      // Re-compute model parameters
      candidateModel.setControlPoints(mss);

      // Compute distance to curve for all candidate points
      tapprox = usNeedleDetectionTools::approximateCoordinates(x, mss, nActivePoints);
      computeCost(x, *candidateModel.getModel(), tapprox, cost);

      // Find the indices of points that are inliers to this model.
      ninliers = 0;
      for (unsigned int i=0; i < npts; i++) {
	if (cost[i] < m_costThresh) {
	  inliers[i] = 1;
	  ninliers++;
	}
	else inliers[i] = 0;
      }

      // Compute model score
      score = ninliers
	- m_modelLikelihoodWeight * us3DNeedleModel::curveDistance(previousModel, candidateModel)
	- 10.0 * candidateModel.getCurvature() * candidateModel.getCurvature();

      if (score >= bestscore) {   // Largest set of inliers so far...
	bestscore = score;  // Record data for this model
	bestNumberOfInliers = ninliers;
	if (m_order==nActivePoints)
	  needleModel = candidateModel;
	else
	  needleModel = candidateModel.changePolynomialOrder(m_order);
	solutionFind = true ;
	
	// Update estimate of N, the number of trials to ensure we pick,
	// with probability p, a data set with no outliers.
	
	double fracinliers =  (double)ninliers / (double)npts;
	
	double pNoOutliers = 1 - pow(fracinliers,static_cast<int>(nActivePoints));
	
	pNoOutliers = vpMath::maximum(eps, pNoOutliers);  // Avoid division by -Inf
	pNoOutliers = vpMath::minimum(1-eps, pNoOutliers);// Avoid division by 0.
	N = vpMath::minimum(vpMath::round(log(1-m_failThresh)/log(pNoOutliers)),m_maxIter);
      }
      numiter++;
    }
  //std::cerr << "done." << std::endl;
  
  if (solutionFind==true)   // We got a solution
    std::cout << "RANSAC: Score after " << numiter << " iterations: " << bestscore
  	      << " with " << bestNumberOfInliers << " inliers." << std::endl;
  else
    vpTRACE("ransac was unable to find a useful solution");
  std::cerr << "Distance to previous solution: "
	    << us3DNeedleModel::curveDistance(previousModel, needleModel) << std::endl;
  
  //if(ninliers > 0)
  //  residual /= ninliers;
  delete [] ind;
  //std::cerr << "Out SoftRansac::run()" << std::endl;
  return bestscore;
}
//#endif

unsigned int *SoftRansac::argSort(const vpColVector &C, bool ascent)
{
  //std::cerr << " Sorting ";
  //for (unsigned int i=0; i<3; ++i) {
  //  std::cerr << C[i] << " ";
  //}
  //std::cerr << std::endl;
  unsigned int n = C.getRows();
  unsigned int *perm = new unsigned int[n];
  double *ptr;
  double min;
  double mmin = -DBL_MAX;
  unsigned int amin;
  for (unsigned int i=0; i<n; i++)
    {
      ptr = C.data;
      min = DBL_MAX;
      amin = 0;
      for (unsigned int j=0; j<n; j++)
	{
	  if ((*ptr < min) && (*ptr > mmin))
	    {
	      min = *ptr;
	      amin = j;
	    }
	  ptr++;
	}
      if (ascent)
	perm[i] = amin;
      else
	perm[n-i-1] = amin;
      mmin = min;
    }
  return perm;
}

bool SoftRansac::isDegenerate(unsigned int *ind, unsigned int npts)
{
  bool deg = false;
  for (unsigned int i=0; i<npts-1; i++)
    for (unsigned int j=i+1; j<npts; j++)
      deg = deg || (ind[i]==ind[j]);
  return deg;
}

void SoftRansac::computeCost(vpMatrix &X, vpMatrix &M, vpMatrix &T, double *cost)
{
  unsigned int n = X.getRows();
  vpMatrix Y = M * T;
  
  for (unsigned int j = 0; j < T.getCols(); ++j)
    if (T[1][j] > 1.0)
      for (unsigned int i = 1; i < T.getRows(); ++i)
	T[i][j] = 1.0;
  
  /*vpColVector P1(2);
  vpColVector P2(2);
  vpColVector Q(2);
  Q[1] = 1;
  P1 = M * Q;
  Q[2] = 1;
  P2 = M * Q;
  double al1;
  double al2;*/
  for (unsigned int i=0; i<n; i++)
    /*{
      al1 = vpColVector::dotProd(X.row(i+1).t()-P1,P2-P1);
      al2 = vpColVector::dotProd(X.row(i+1).t()-P2,P1-P2);
      if (al1 > 0 && al2 > 0)*/
	cost[i] = (X.getRow(i).t()-Y.getCol(i)).euclideanNorm();
  /*else
    cost[i] = DBL_MAX;
    }*/
}
