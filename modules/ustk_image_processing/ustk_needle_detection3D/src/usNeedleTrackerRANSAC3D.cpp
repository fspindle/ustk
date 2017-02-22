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

#include <visp3/ustk_needle_detection3D/usNeedleTrackerRANSAC3D.h>

usNeedleTrackerRANSAC3D::usNeedleTrackerRANSAC3D()
{
  m_nPoints = 3;
  m_nActivePoints = 2;
  m_nRenderSegments = 10;
  m_varianceThreshold = 1000.0;
  m_nCandidatePointsDetection = 800;
  m_nCandidatePointsTracking = 200;
  m_tipIntensityDropThreshold = 20.0;
  m_extent = new int[6];
  m_dimensions = new int[3];
  m_VOI = NULL;
  m_imageImport = (vtkImageImport**)malloc(sizeof(vtkImageImport*)*3);
  m_extractVOI = (vtkExtractVOI**)malloc(sizeof(vtkExtractVOI*)*3);
  m_castUInt2Short = (vtkImageCast**)malloc(sizeof(vtkImageCast*)*3);
  for(int i=0; i<3; i++)
    {
      m_imageImport[i] = vtkImageImport::New();
      m_extractVOI[i] = vtkExtractVOI::New();
      m_castUInt2Short[i] = vtkImageCast::New();
    }
  m_difference = vtkImageMathematics::New();
  m_difference->SetOperationToSubtract();
  m_absoluteValue = vtkImageMathematics::New();
  m_absoluteValue->SetOperationToAbsoluteValue();
  m_ransac = softRansac();
  m_model = vpMatrix();
  m_inliers = vpColVector();
  m_ninliers = 0;
  m_median = vpColVector(3);
  m_needle = NULL;
  m_predictedNeedle = NULL;
  m_principalDir = vpColVector(3);
  m_probeFilter = vtkProbeFilter::New();
  m_tipLimit = vtkPlane::New();
  m_needleTracker = us3DNeedleTracker();
  m_entryTracker = us3DEntryTracker();
  m_fixEntryPoint = false;
  m_missCount = 0;
  m_startIndex = 1;
  m_pose = new double[3];
  m_velocity = new double[3];
}

usNeedleTrackerRANSAC3D::~usNeedleTrackerRANSAC3D()
{
  if (m_points) {
    for (unsigned int i=0; i<m_nPoints; ++i) {
      if (m_points[i]) {
	delete [] m_points[i];
	m_points[i] = NULL;
      }
    }
    delete [] m_points;
    m_points = NULL;
  }
  if (m_needle) {
    delete m_needle;
    m_needle = NULL;
  }
  //if (m_extent) {
  //  delete [] m_extent;
  //  m_extent = NULL;
  //}
  //if (m_dimensions) {
  //  delete [] m_dimensions;
  //  m_dimensions = NULL;
  //}
  if (m_pose) {
    delete [] m_pose;
    m_pose = NULL;
  }
  if (m_velocity) {
    delete [] m_velocity;
    m_velocity = NULL;
  }
  free (m_imageImport);
  free (m_extractVOI);
  free (m_castUInt2Short);
}

void usNeedleTrackerRANSAC3D::init(double resolution, unsigned int dims[3], double origin[3], double fov,
			      unsigned int nPoints)
{
  m_nPoints = nPoints;
  m_nActivePoints = 2;;
  m_points = new double*[m_nPoints];
  m_needle = new us3DNeedleModel(m_nPoints);
  for (unsigned int i=0; i<m_nPoints; ++i)
    m_points[i] = new double[3];
  m_mss = vpMatrix(m_nPoints,3);
  m_measure = vpColVector(3*m_nPoints);
  m_volIndex = 0;
  m_lastVolIdx = -1;
  m_state = IDLE;
  m_resolution = resolution;
  m_dimensions[0] = dims[0];
  m_dimensions[1] = dims[1];
  m_dimensions[2] = dims[2];
  // field of view parameters [pchatela 2013-09-08]
  m_fov = fov;
  m_origin.resize(3);
  m_origin[0] = origin[0];
  m_origin[1] = origin[1];
  m_origin[2] = origin[2];
  m_entryTracker.setOrigin(m_origin);
  for (int i=0; i<3; i++)
    {
      m_imageImport[i]->SetDataSpacing(1, 1, 1);
      m_imageImport[i]->SetDataOrigin(0, 0, 0);
      m_imageImport[i]->SetWholeExtent(0, m_dimensions[0]-1,
				       0, m_dimensions[1]-1,
				       0, m_dimensions[2]-1);
      m_imageImport[i]->SetDataExtentToWholeExtent();
      m_imageImport[i]->SetDataScalarTypeToUnsignedChar();
      m_imageImport[i]->SetNumberOfScalarComponents(1);
      m_extractVOI[i]->SetSampleRate(1,1,1);
      m_castUInt2Short[i]->SetOutputScalarTypeToShort();
    }
  m_extent = m_imageImport[0]->GetDataExtent();
  m_VOI = m_extent;
  m_entryTracker.setFOV(m_fov);
  m_entryTracker.setExtent(m_extent);
  m_needleTracker = us3DNeedleTracker(m_nPoints);
  m_needleTracker.setExtent(m_extent);
  m_needleTracker.setSigmaStateTip(0.1);
  m_needleTracker.setSigmaMeasureTip(10);
  m_needleTracker.init();
  m_fixEntryPoint = false;
  m_missCount = 0;
  m_startIndex = 1;
  // VTK pipeline
  for (int i=0; i<3; i++)
    {
      m_extractVOI[i]->SetInputConnection(m_imageImport[i]->GetOutputPort());
      m_castUInt2Short[i]->SetInputConnection(m_extractVOI[i]->GetOutputPort());
    }
  //m_probeFilter->SetInputConnection(m_needle->GetOutputPort());
  //m_probeFilter->SpatialMatchOn();
  m_ransac.init(m_nPoints, 5.0, 1-1e-7);
  
  std::cout << "3DUS Needle detector initialized." << std::endl
	    << "Input data parameters:" << std::endl
	    << "     dimensions: " << dims[0] << " " << dims[1] << " " << dims[2] << std::endl
	    << "     origin: " << origin[0] << " " << origin[1] << " " << origin[2] << std::endl
	    << "     field of view: " << m_fov << std::endl << std::endl;
}

void usNeedleTrackerRANSAC3D::init(const usImagePostScan3D<unsigned char> &vol, double fov, unsigned int nPoints)
{
  unsigned int dims[3];
  double origin[3];
  dims[0] = vol.getDimX();
  dims[1] = vol.getDimY();
  dims[2] = vol.getDimZ();
  /*origin[0] = vol.getOriginX();
  origin[1] = vol.getOriginY();
  origin[2] = vol.getOriginZ();*/

  origin[0] = 0;
  origin[1] = 0;
  origin[2] = 0;

  init(vol.getElementSpacingX(), dims, origin, fov, nPoints);
}

#ifdef USTK_HAVE_CUDA
void usNeedleTrackerRANSAC3D::init(usScanConverter3D *converter, double resolution,
				   unsigned int nPoints)
{
  unsigned int dims[3];
  double origin[3];
  if (!converter) {
    std::cerr << "Error: invalid converter value." << std::endl;
    exit(EXIT_SUCCESS);
  }

  m_converter = converter;
  dims[0] = m_converter->GetPostscanParams()->dims[0];
  dims[1] = m_converter->GetPostscanParams()->dims[1];
  dims[2] = m_converter->GetPostscanParams()->dims[2];
  origin[0] = -m_converter->GetPostscanParams()->origin[0];
  origin[1] = -m_converter->GetPostscanParams()->origin[1];
  origin[2] = -m_converter->GetPostscanParams()->origin[2];
  
  init(resolution, dims, origin, m_converter->GetPostscanParams()->frm_fov, nPoints);
}
#endif

int usNeedleTrackerRANSAC3D::getLastVolIdx() {
  return m_lastVolIdx;
}

double usNeedleTrackerRANSAC3D::getVarianceThreshold() {
  return m_varianceThreshold;
}

void usNeedleTrackerRANSAC3D::setVarianceThreshold(double varianceThreshold) {
  m_varianceThreshold = varianceThreshold;
}

void usNeedleTrackerRANSAC3D::setNCandidatePointsDetection(unsigned int nPoints) {
  m_nCandidatePointsDetection = nPoints;
}

void usNeedleTrackerRANSAC3D::setTipIntensityDropThreshold(double value) {
  m_tipIntensityDropThreshold = value;
}

void usNeedleTrackerRANSAC3D::setNCandidatePointsTracking(unsigned int nPoints) {
  m_nCandidatePointsTracking = nPoints;
}

void usNeedleTrackerRANSAC3D::setEntryStateVariance(double sigma) {
  m_needleTracker.setSigmaStateEntry(sigma);
}

void usNeedleTrackerRANSAC3D::setNeedleStateVariance(double sigma) {
  m_needleTracker.setSigmaStateTip(sigma);
}

void usNeedleTrackerRANSAC3D::setEntryMeasureVariance(double sigma) {
  m_needleTracker.setSigmaMeasureEntry(sigma);
}

void usNeedleTrackerRANSAC3D::setNeedleMeasureVariance(double sigma) {
  m_needleTracker.setSigmaMeasureTip(sigma);
}

void usNeedleTrackerRANSAC3D::setMaximumAngularDifference(double value) {
  m_needleTracker.setMaxAngleError(value);
}

void usNeedleTrackerRANSAC3D::setMaximumLengthDifference(double value) {
  m_needleTracker.setMaxLengthError(value);
}

void usNeedleTrackerRANSAC3D::setVOIMargin(double margin) {
  m_needleTracker.setVOIMargin(margin);
}

void usNeedleTrackerRANSAC3D::setRansacCostThreshold(double value) {
  m_ransac.setCostThreshold(value);
}

void usNeedleTrackerRANSAC3D::setRansacFailThreshold(double value) {
  m_ransac.setFailThreshold(value);
}

void usNeedleTrackerRANSAC3D::fixEntryPointOn() {
  m_fixEntryPoint = true;
}

void usNeedleTrackerRANSAC3D::fixEntryPointOff() {
  m_fixEntryPoint = false;
}

void usNeedleTrackerRANSAC3D::setModelLikelihoodWeight(double value) {
  m_ransac.setModelLikelihoodWeight(value);
}

int *usNeedleTrackerRANSAC3D::getExtent() { return m_extent; }

//void usNeedleTrackerRANSAC3D::setVOI(int VOI[6]) { m_VOI = VOI; }

/*void usNeedleTrackerRANSAC3D::setVOI(int xmin, int xmax, int ymin, int ymax, int zmin, int zmax)
{
  m_VOI[0] = xmin;
  m_VOI[1] = xmax;
  m_VOI[2] = ymin;
  m_VOI[3] = ymax;
  m_VOI[4] = zmin;
  m_VOI[5] = zmax;
  }*/

int *usNeedleTrackerRANSAC3D::getVOI() { return m_VOI; }

#ifdef USTK_HAVE_CUDA
void usNeedleTrackerRANSAC3D::setConverter(usScanConverter3D *converter) {m_converter = converter; }

usScanConverter3D *usNeedleTrackerRANSAC3D::getConverter() { return m_converter; }
#endif

us3DNeedleModel *usNeedleTrackerRANSAC3D::getNeedle() { return m_needle; }

bool usNeedleTrackerRANSAC3D::isNeedleDetected() { return (m_needleTracker.isInitialized()); }

double *usNeedleTrackerRANSAC3D::getTipVelocity() { return m_needleTracker.getVelocities()[m_nPoints-1]; }

//double *usNeedleTrackerRANSAC3D::getTipPose() { return m_needleTracker.getPoints()[m_nPoints-1]; }

vpColVector usNeedleTrackerRANSAC3D::getTipPose() {
  return m_needle->getControlPoints().getCol(m_nPoints-1);
}

double *usNeedleTrackerRANSAC3D::getEntryVelocity() { return m_needleTracker.getVelocities()[0]; }
double usNeedleTrackerRANSAC3D::getEntryVelocity(int c) { return m_needleTracker.getVelocities()[0][c]; }

//double *usNeedleTrackerRANSAC3D::getEntryPose() { return m_needleTracker.getPoints()[0]; }
double usNeedleTrackerRANSAC3D::getEntryPose(int c) { return m_needleTracker.getPoints()[0][c]; }

vpColVector usNeedleTrackerRANSAC3D::getEntryPose() {
  return m_needle->getControlPoints().getCol(0); }

us3DNeedleTracker *usNeedleTrackerRANSAC3D::getNeedleTracker() { return &m_needleTracker; }

vpColVector usNeedleTrackerRANSAC3D::getPoseError()
{
  vpColVector error(3);
  if (this->isNeedleDetected())// && UsNeedleDetectionTools::dist3(tip, entry) > 15)
    {
      error[0] = m_resolution * (m_points[m_nPoints-1][2] - m_dimensions[2] / 2); // in m
      error[1] = m_resolution * (m_points[m_nPoints-1][1] - m_dimensions[1] / 2); // in m
      double lz = m_points[0][2]-m_points[m_nPoints-1][2];
      double ly = m_points[0][1]-m_points[m_nPoints-1][1];
      if (ly==0)
	error[2] = M_PI/2;
      else
	error[2] = atan2(lz,ly); // in rad
    }
  else
    error = 0;
  return error;
}

void usNeedleTrackerRANSAC3D::run(usImagePostScan3D<unsigned char> &vol)
{
  // Import data
  m_imageImport[m_volIndex%3]->CopyImportVoidPointer(vol.getData(),
						     vol.getSize() * sizeof(unsigned char));
  m_imageImport[m_volIndex%3]->Update();
  m_imageImport[m_volIndex%3]->Modified();
  
  // Run needle detection
  run();
}

#ifdef USTK_HAVE_CUDA
void usNeedleTrackerRANSAC3D::run(int volIdx)
{
  // Import data
  m_imageImport[m_volIndex%3]->CopyImportVoidPointer(m_converter->GetPtPostscanVolData(),
						     m_converter->GetPostscanVolSize());
  m_imageImport[m_volIndex%3]->Update();
  m_imageImport[m_volIndex%3]->Modified();

  // Run needle detection
  run();
}
#endif

void usNeedleTrackerRANSAC3D::detect(bool verbose)
{
  unsigned int npts;
  double threshold;
  double var;
  vpMatrix vpPoints;
  
  // Extract VOI
  m_extractVOI[m_volIndex%3]->SetVOI(m_VOI);
  m_extractVOI[(m_volIndex-2)%3]->SetVOI(m_VOI);
  m_extractVOI[m_volIndex%3]->SetInputConnection(m_imageImport[m_volIndex%3]->GetOutputPort());
  m_extractVOI[(m_volIndex-2)%3]
    ->SetInputConnection(m_imageImport[(m_volIndex-2)%3]->GetOutputPort());
  m_extractVOI[m_volIndex%3]->UpdateWholeExtent();
  m_extractVOI[(m_volIndex-2)%3]->UpdateWholeExtent();
  m_extractVOI[m_volIndex%3]->Update();
  m_extractVOI[(m_volIndex-2)%3]->Update();
  
  // Cast data
  m_castUInt2Short[m_volIndex%3]->SetInputConnection(m_extractVOI[m_volIndex%3]->GetOutputPort());
  m_castUInt2Short[(m_volIndex-2)%3]
    ->SetInputConnection(m_extractVOI[(m_volIndex-2)%3]->GetOutputPort());
  m_castUInt2Short[m_volIndex%3]->UpdateWholeExtent();
  m_castUInt2Short[(m_volIndex-2)%3]->UpdateWholeExtent();
  m_castUInt2Short[m_volIndex%3]->Update();
  m_castUInt2Short[(m_volIndex-2)%3]->Update();
  
  // Compute image difference
  m_difference->SetInput1Data(m_castUInt2Short[m_volIndex%3]->GetOutput());
  m_difference->SetInput2Data(m_castUInt2Short[(m_volIndex-2)%3]->GetOutput());
  m_difference->UpdateWholeExtent();
  m_difference->Update();
  m_absoluteValue->SetInput1Data(m_difference->GetOutput());
  m_absoluteValue->UpdateWholeExtent();
  m_absoluteValue->Update();
  
  // Get the candidate points
  threshold = usNeedleDetectionTools::getThresholdedCoordinates(m_absoluteValue->GetOutput(),
								vpPoints,
								m_nCandidatePointsDetection);
  npts = vpPoints.getRows();
  if (verbose)
    std::cerr << "   " << npts << " candidate points" << std::endl;
  
  // Detect entry
  m_median = usNeedleDetectionTools::geometricMedian(vpPoints, npts, 3);
  if (verbose) {
    std::cerr << "   Median: ";
    for (unsigned int j=0; j<3; ++j)
      std::cerr << m_median[j] << " ";
    std::cerr << std::endl;
  }
  var = usNeedleDetectionTools::variance(vpPoints, npts, 3);
  if (verbose)
    std::cerr << "   variance: " << var << std::endl;
  
  if (var < m_varianceThreshold) {
    m_entryTracker.update(m_median.data);
    m_pose = m_entryTracker.getPose();
    for (int i=0; i<3; ++i)
      {
	m_mss[0][i] = m_pose[i];
	m_mss[m_nPoints-1][i] = m_median[i];
	if (m_nPoints>2)
	  for (unsigned int j=1; j<m_nPoints-1; ++j) {
	    m_mss[j][i] = m_mss[0][i] + j * (m_mss[m_nPoints-1][i] - m_mss[0][i])
	      / (m_nPoints-1);
	  }
      }
    m_needle->setControlPoints(m_mss);
  }
}

void usNeedleTrackerRANSAC3D::track(bool verbose)
{
  double needleLength;
  unsigned int npts;
  double threshold;
  vpMatrix vpPoints;
  
  // Extract VOI
  if (m_needleTracker.isInitialized())
    m_VOI = m_needleTracker.getVOI();
  else
    m_VOI = m_entryTracker.getVOI();

  if (verbose) {
    std::cerr << "     extract VOI: " << m_VOI[0] << "-" << m_VOI[1] << " "
	      << m_VOI[2] << "-" << m_VOI[3] << " "
	      << m_VOI[4] << "-" << m_VOI[5] << std::endl;
  }
  
  m_extractVOI[m_volIndex%3]->SetVOI(m_VOI);
  m_extractVOI[m_volIndex%3]->SetInputConnection(m_imageImport[m_volIndex%3]->GetOutputPort());
  m_extractVOI[m_volIndex%3]->UpdateWholeExtent();
  m_extractVOI[m_volIndex%3]->Update();
  m_extractVOI[m_volIndex%3]->Modified();

  // Cast data
  m_castUInt2Short[m_volIndex%3]->SetInputConnection(m_extractVOI[m_volIndex%3]->GetOutputPort());
  m_castUInt2Short[m_volIndex%3]->UpdateWholeExtent();
  m_castUInt2Short[m_volIndex%3]->Update();
  m_castUInt2Short[m_volIndex%3]->Modified();
  
  if (m_needleTracker.isInitialized())
    needleLength = usNeedleDetectionTools::dist3(m_points[0], m_points[m_nPoints-1]);
  else
    needleLength = 10;
  
  // Get the candidate points
  
  threshold = usNeedleDetectionTools::getThresholdedCoordinates(m_castUInt2Short[m_volIndex%3]->GetOutput(),
								vpPoints,
								m_nCandidatePointsTracking);
  /*
  UsNeedleDetectionTools::getThresholdedCoordinates(m_castUInt2Short[m_volIndex%3]->GetOutput(),
						    vpPoints,
						    220.0);
  */
  npts = vpPoints.getRows();

  if (verbose) {
    std::cerr << "     threshold = " << threshold << std::endl;
    std::cerr << "     " << npts << " candidate points" << std::endl;
  }
  
  // Run RANSAC
  m_predictedNeedle = new us3DNeedleModel(m_nPoints);
  m_predictedNeedle->setControlPoints(m_needleTracker.getPrediction());
  m_ninliers = m_ransac.run(vpPoints, m_imageImport[m_volIndex%3]->GetOutput(),
			    *m_predictedNeedle, *m_needle, m_origin, m_entryPlane, m_VOI,
			    m_tipIntensityDropThreshold, m_nActivePoints);

  if (verbose)
    std::cerr << "     RANSAC finished." << std::endl;
  //m_needle->getControlPoints().print(std::cerr, 5, "RANSAC");
  
  // Tracking
  for (int j=0; j<m_nPoints; ++j)
    for (int i=0; i<3; i++)
      m_measure[i+3*j] = m_needle->getControlPoints()[i][j];
  if (!m_needleTracker.addMeasure(m_measure)) {
    if (verbose)
      std::cerr << "     Skipping measure." << std::endl;
    ++m_missCount;
  }
  
  for (unsigned int i=0; i<m_nPoints; ++i)
    m_needleTracker.getPose(i,m_points[i]);
  m_needle->setControlPoints(m_points);
  needleLength = m_needle->getLength();

  if (verbose) {
    std::cerr << "     Needle lenght: " << needleLength << std::endl;
    std::cerr << "     Pose error: " << getPoseError()[0]
	      << " " << getPoseError()[1]
	      << " " << getPoseError()[2] << std::endl;
  }

  m_needle->setControlPoints(m_points);
  //m_needle->getControlPoints().print(std::cout, 5, "NEEDLE");
  
  // Adjust polynomial order [pchatela 2013-09-10]
  if (m_needle->getLength()>50.0)
    m_nActivePoints = m_nPoints;
  
  // Stop tracking if the needle is lost [pchatela 2013-09-10]
  if (m_missCount > 9) {
    std::cout << "-----------------------------" << std::endl
	      << "Needle lost, returning to IDLE state." << std::endl
	      << "-----------------------------" << std::endl;
    m_state = IDLE;
    m_missCount = 0;
    m_startIndex = m_volIndex + 1;
    m_entryTracker = us3DEntryTracker();
    m_entryTracker.setFOV(m_fov);
    m_entryTracker.setExtent(m_extent);
    m_needleTracker = us3DNeedleTracker();
    m_needleTracker = us3DNeedleTracker(m_nPoints);
    m_needleTracker.setExtent(m_extent);
    m_needleTracker.setSigmaStateTip(0.1);
    m_needleTracker.setSigmaMeasureTip(10);
    m_needleTracker.init();
  }
}

void usNeedleTrackerRANSAC3D::run()
{
  bool verbose = true;
  if (verbose)
    std::cerr << "Needle detection (it " << m_volIndex << ")..." << std::endl;

  switch (m_state)
    { 
    case IDLE:
      if (verbose)
	std::cerr << "   in idle mode" << std::endl;
      break;
    case DETECTION:
      if (verbose)
	std::cerr << "   in detection mode" << std::endl;
      detect(verbose);
      break;
    case TRACKING:
      if (verbose)
	std::cerr << "   in tracking mode" << std::endl;
      track(verbose);
      break;
    default:
      break;
    }

  // Update detector state
  m_lastVolIdx = m_volIndex;
  m_volIndex++;
  if (m_state==IDLE && m_volIndex>m_startIndex)
    m_state = DETECTION;
  if (m_state==DETECTION && m_entryTracker.getEntryFace() !=0)
    {
      std::cout << "-----------------------------" << std::endl
		<< "Entry detected at " << m_entryTracker.getPose()[0]
		<< " " << m_entryTracker.getPose()[1]
		<< " " << m_entryTracker.getPose()[2]
		<< " --- Starting tracking" << std::endl
		<< "-----------------------------" << std::endl;
      m_state = TRACKING;
      m_VOI = m_entryTracker.getVOI();
      m_entryPlane.resize(3);
      m_entryPlane[0] = sin(m_fov / 2.0);
      if (m_entryTracker.getEntryFace() == -1) // Needle inserted from the left
	m_entryPlane[1] = cos(m_fov / 2.0);
      else
	m_entryPlane[1] = -cos(m_fov / 2.0); // Needle inserted from the right
      m_entryPlane[2] = 0.0;
      if (m_fixEntryPoint)
	m_ransac.forcePoint(0, m_entryTracker.getPose());
      //m_mss.print(std::cerr, 5);
      //std::cerr << "MSS: ";
      for (unsigned int j=0; j<m_nPoints; ++j) {
	for (int i=0; i<3; ++i)
	  {
	    //std::cerr << m_mss[j][i] << " ";
	    m_measure[i+3*j] = m_mss[j][i];
	  }
      }
      //std::cerr << std::endl;
      m_needleTracker.addMeasure(m_measure);
    }
  //std::cerr << " done." << std::endl;
}
