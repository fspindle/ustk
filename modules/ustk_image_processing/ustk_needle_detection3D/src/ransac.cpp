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

#include <visp3/ustk_needle_detection3D/ransac.h>

Ransac::Ransac()
{
  //m_bestInliers.init();
  m_nInliers = 0;
}

Ransac::Ransac(unsigned int s, double costThresh, double failThresh)
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
  m_maxIter = 50;
  m_maxTrials = 1000;
}

void Ransac::init(unsigned int s, double costThresh, double failThresh)
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
  m_maxIter = 50;
  m_maxTrials = 1000;
}

void Ransac::setMaxTrials(unsigned int maxTrials) { m_maxTrials = maxTrials; }

unsigned int Ransac::getMaxTrials() { return m_maxTrials; }

void Ransac::setCostThreshold(double value) { m_costThresh = value; }
void Ransac::setFailThreshold(double value) { m_failThresh = value; }

void Ransac::forcePoint(unsigned int i, vpColVector &x)
{
  if (!m_forced[i])
    {
      m_nForced++;
      m_forced[i] = true;
    }
  m_forcedPoints.getRow(i) = x.t();
}

void Ransac::forcePoint(unsigned int i, double *x)
{
  if (!m_forced[i])
    {
      m_nForced++;
      m_forced[i] = true;
    }
  for (unsigned int j=0; j<3; j++)
    m_forcedPoints[i][j] = x[j];
}

void Ransac::freePoint(unsigned int i)
{
  if (m_forced[i])
    {
      m_nForced--;
      m_forced[i] = false;
    }
}

int Ransac::run(vpMatrix &candidates, vpMatrix &bestM, vpMatrix &bestMSS,
		vpColVector &bestInliers, int prevscore)
{
  std::cerr << "     starting RANSAC" << std::endl;
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
	std::cerr << "        point " << i << " forced to "
		  << m_forcedPoints[i][0] << " "
		  << m_forcedPoints[i][1] << " "
		  << m_forcedPoints[i][2] << std::endl;
	for (unsigned int j=0; j<3; j++)
	  x[counter+nCandidates][j] = m_forcedPoints[i][j];
	counter++;
      }

  // Sentinel value allowing detection of solution failure.
  bool solutionFind = false ;
  vpMatrix model(3,m_order);
  int  bestscore;
  double N = 1;            // Dummy initialisation for number of trials.
  
  vpUniRand random((const long)time(NULL));
  vpColVector inliers(npts);
  unsigned int *ind = new unsigned int[m_order];
  int numiter = 0;
  int numtrials = 0;
  int ninliers = 0;
  double residual = 0.0;
  vpMatrix mss(m_order,3); // Minimal Sample Set
  vpMatrix tapprox(m_order,npts); // Approximate curvilinear coordinate
  vpMatrix tapprox_mss(m_order,m_order); // Approximate curvilinear coordinate
  double tapp;
  double tapps;
  vpColVector dirVec(3);
  vpColVector ndirVec(3);
  unsigned int *perm;
  bool degenerated;
  double *cost = new double[npts];
  double orientation;
  double norm;
  vpMatrix prevMSS;

  // improve on previous run
  //if (prevscore>0)
  //  {
  //    bestscore = prevscore*4/10;
  //  }
  //else
  //  {
  inliers = vpColVector(npts);
  bestscore = -1;
  //  }
  prevMSS = vpMatrix(bestMSS);

  unsigned int tooSmall = 0;
  unsigned int tooFar = 0;
  unsigned int tooCurved = 0;

  while( N > numiter)
    {
      // Generate s random indicies in the range 1..npts
      degenerated = true;
      while (degenerated && numtrials < m_maxTrials) {
	for  (unsigned int i=0; i < m_order; i++) {
	  counter = 0;
	  if (m_forced[i]) {
	    ind[i] = counter + nCandidates;
	    counter++;
	  }
	  else
	    ind[i] = (unsigned int)ceil(random()*nCandidates) - 1;
	  degenerated = isDegenerated(ind, m_order);
	}
	numtrials++;
      }
      if (degenerated) {
	vpTRACE("All sampled configurations are degenerated. Less than %d sample points?", m_order);
	break;
      }
      
      // Safeguard against being stuck in this loop forever
      if (numtrials >= m_maxTrials)
	{
	  vpTRACE("ransac reached the maximum number of %d trials", m_maxTrials);
	  break ;
	}
      // Fit model to this random selection of data points.
      for  (unsigned int i=0 ; i < m_order ; i++)
	for (unsigned int j=0; j<3; j++)
	    mss[i][j] = x[ind[i]][j];

      // Estimate approximate curvilinear coordinates
      tapprox_mss = usNeedleDetectionTools::approximateCoordinates(mss, mss, m_order);

      // Sort points and reestimate coordinates [WARNING: only works for insertion from right]
      perm = argSort(tapprox_mss.getRow(1).t());
      for  (unsigned int i=0 ; i < m_order ; i++)
	for (unsigned int j=0; j < 3; j++)
	    mss[i][j] = x[ind[perm[i]]][j];

      // Discard degenerate configurations
      double minNorm = DBL_MAX;
      for (unsigned int i=1; i<m_order; ++i) {
	norm = (mss.getRow(i-1).t()-mss.getRow(i).t()).euclideanNorm();
	if (norm < minNorm)
	  minNorm = norm;
      }
      if (minNorm < 4) {
	++tooSmall;
	continue;
      }
      if (m_order > 2) {
	double minCurvature = 1.0;
	for (unsigned int i=1; i<m_order-1; ++i) {
	  double curvature = vpColVector::dotProd(mss.getRow(i-1).t()-mss.getRow(i).t(),
						  mss.getRow(i).t()-mss.getRow(i+1).t())
      	    / ((mss.getRow(i-1).t()-mss.getRow(i).t()).euclideanNorm() * (mss.getRow(i).t()-mss.getRow(i+1).t()).euclideanNorm());
	  if (curvature < minCurvature)
	    minCurvature = curvature;
	}
	if (minCurvature < 0.9) {
	  ++tooCurved;
	  continue;
	}
      }
      /*
      if (prevscore>0) {
	double worstOrientation = 1.0;
	for (unsigned int i=1; i<m_order; ++i) {
      	  orientation = vpColVector::dotProd(mss.row(i).t()-mss.row(i+1).t(),
	  				     prevMSS.row(i).t()-prevMSS.row(i+1).t())
      	    / ((mss.row(i).t()-mss.row(i+1).t()).euclideanNorm() * (prevMSS.row(i).t()-prevMSS.row(i+1).t()).euclideanNorm());
	  if (orientation < worstOrientation)
	    worstOrientation = orientation;
	}
	if (worstOrientation<0.90) {
	  ++tooFar;
	  continue;
	}
	}*/
      tapprox_mss = usNeedleDetectionTools::approximateCoordinates(mss, mss, m_order);
      
      // Compute model parameters
      model = mss.t() * tapprox_mss.inverseByLU();

      // Evaluate distances between points and model.
      tapprox = usNeedleDetectionTools::approximateCoordinates(x, mss, m_order);
      computeCost(x,model,tapprox,cost);
      // Find the indices of points that are inliers to this model.
      ninliers = 0;
      for (unsigned int i=0; i < npts; i++) {
	if (cost[i] < m_costThresh) {
	  inliers[i] = 1;
	  ninliers++;
	}
	else inliers[i] = 0;
      }
      if (ninliers >= bestscore) {   // Largest set of inliers so far...
	bestscore = ninliers;  // Record data for this model
	bestInliers = inliers;
	bestM = model;
	bestMSS = mss;
	solutionFind = true ;
	
	// Update estimate of N, the number of trials to ensure we pick,
	// with probability p, a data set with no outliers.
	
	double fracinliers =  (double)ninliers / (double)npts;
	
	double pNoOutliers = 1 - pow(fracinliers,static_cast<int>(m_order));
	
	pNoOutliers = vpMath::maximum(eps, pNoOutliers);  // Avoid division by -Inf
	pNoOutliers = vpMath::minimum(1-eps, pNoOutliers);// Avoid division by 0.
	N = vpMath::minimum(vpMath::round(log(1-m_failThresh)/log(pNoOutliers)),m_maxIter);
      }
      numiter++;
    }
  
  std::cerr << "Candidate discarded " << tooSmall << " times for being too short and "
	    << tooCurved << " times for being too curved." << std::endl;
  
  if (solutionFind==true)   // We got a solution
    {
      std::cerr << bestscore << " inliers found in " << numiter << " iterations." << std::endl;
    }
  else
    {
      vpTRACE("ransac was unable to find a useful solution");
      bestM = 0;
      bestMSS = mss;
   }
  
  //if(ninliers > 0)
  //  residual /= ninliers;
  delete [] ind;
  return bestscore;
}

unsigned int *Ransac::argSort(const vpColVector &C, bool ascent)
{
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

bool Ransac::isDegenerated(unsigned int *ind, unsigned int npts)
{
  bool deg = false;
  for (unsigned int i=0; i<npts-1; i++)
    for (unsigned int j=i+1; j<npts; j++)
      deg = deg || (ind[i]==ind[j]);
  return deg;
}

void Ransac::computeCost(vpMatrix &X, vpMatrix &M, vpMatrix &T, double *cost)
{
  unsigned int n = X.getRows();
  vpMatrix Y = M * T;
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
