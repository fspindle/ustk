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

#ifndef US_NEEDLE_TRACKER_RANSAC_3D
#define US_NEEDLE_TRACKER_RANSAC_3D

#include <stdio.h>
#include <string.h>

// UsNeedleDetection
#include <visp3/ustk_gui/usGuiConfig.h> //replaced usNeedleDetectionConfig.h (for Cuda & Vtk defines)
#include <visp3/ustk_needle_detection3D/softRansac.h>
//#include <visp3/ustk_needle_detection/usNeedleDetectionTools.h>
#include <visp3/ustk_needle_detection3D/us3DEntryTracker.h>
#include <visp3/ustk_needle_detection3D/us3DNeedleTracker.h>
#include <visp3/ustk_needle_detection3D/us3DNeedleModel.h>

// ViSP
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpColVector.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/core/vpConfig.h>

// UsTK
#include <visp3/ustk_core/usImagePostScan3D.h>

#ifdef USTK_HAVE_CUDA
#include <UsTk/usScanConversion3D.h>
#endif

// VTK
#ifdef USTK_HAVE_VTK_QT
	#include <vtkImageData.h>
	#include <vtkUnstructuredGrid.h>
	#include <vtkSmartPointer.h>
	#include <vtkImageImport.h>
	#include <vtkDoubleArray.h>
	#include <vtkImageMathematics.h>
	#include <vtkImageCast.h>
	#include <vtkDataSetAttributes.h>
	#include <vtkAppendFilter.h>
	#include <vtkExtractVOI.h>
	#include <vtkInformation.h>
	#include <vtkDataSet.h>
	#include <vtkProbeFilter.h>
  #include <vtkPointData.h>
  #include <vtkLineSource.h>
  #include <vtkImageMathematics.h>
#endif

/**
 * @brief Needle detection pipeline.
 * @author Pierre Chatelain
 * @date 2013-09-10
 *
 * This class defines the needle detection pipeline.
 */
class VISP_EXPORT usNeedleTrackerRANSAC3D
{

public:

  /**
   * Default constructor.
   */
  usNeedleTrackerRANSAC3D();

  /**
   * Destructor.
   */
  ~usNeedleTrackerRANSAC3D();

  /**
   * Initialization method.
   * Initializes all attributes with default values and the given image resolution.
   * Should be called before to start the detection.
   */
  void init(double resolution, unsigned int dims[3], double origin[3], double fov,
	    unsigned int nPoints);

  /**
   * Initialization method.
   * Initializes all attributes with default values and the given image resolution.
   * Should be called before to start the detection.
   * @param vol Reference to the post-scan volume.
   * @param resolution The image resolution (in meters).
   * @param nPoints The number of control points.
   */
  void init(const usImagePostScan3D<unsigned char>& vol, double fov, unsigned int nPoints);

#ifdef USTK_HAVE_CUDA
  /**
   * Conveniance initialization method with Us3DScan
   * Initializes all attributes with default values and the given image resolution.
   * Should be called before to start the detection.
   * @param converter Pointer the usScanConverter3D performing volume reconstruction.
   * @param resolution The image resolution (in meters).
   * @param nPoints The number of control points.
   */
  void init(usScanConverter3D *converter, double resolution, unsigned int nPoints);
#endif

  /**
   * Returns the index of the last processed volume.
   */
  int getLastVolIdx();

  /**
   * Returns the variance threshold.
   */
  double getVarianceThreshold();

  /**
   * Sets the variance threshold.
   */
  void setVarianceThreshold(double varianceTheshold);

  /**
   * Sets the number of candidate points used for detection.
   */
  void setNCandidatePointsDetection(unsigned int nPoints);

  /**
   * Sets the number of candidate points used during traking.
   */
  void setNCandidatePointsTracking(unsigned int nPoints);
  void setTipIntensityDropThreshold(double value);

  /**
   * Sets the tracker state variance for the entry point.
   */
  void setEntryStateVariance(double sigma);

  /**
   * Sets the tracker state variance for the needle control points.
   */
  void setNeedleStateVariance(double sigma); 

  /**
   * Sets the tracker measure variance for the entry point.
   */
  void setEntryMeasureVariance(double sigma);

  /**
   * Sets the tracker measure variance for the needle control points.
   */
  void setNeedleMeasureVariance(double sigma);

  /**
   * Sets the maximum allowed angle (in radians) between two consecutive predictions.
   */
  void setMaximumAngularDifference(double value);

  /**
   * Sets the maximum allowed lenght difference between two consecutive predictions.
   */
  void setMaximumLengthDifference(double value);

  /**
   * Sets the margin around the predicted needle position used to compute the volume interest.
   */
  void setVOIMargin(double margin);
  void setRansacCostThreshold(double value);
  void setRansacFailThreshold(double value);
  void fixEntryPointOn();
  void fixEntryPointOff();
  void setModelLikelihoodWeight(double value);

  /**
   * Returns the volume extent.
   */
  int *getExtent();

  /**
   * Sets the Volume Of Interest. The needle detection algorithm will be reduced to this volume.
   * The values in VOI shall be interpreted as follows: [xmin, xmax, ymin, ymax, zmin, zmax].
   */
  //void setVOI(int VOI[6]);

  /**
   * Sets the Volume Of Interest. The needle detection algorithm will be reduced to this volume.
   */
  //void setVOI(int xmin, int xmax, int ymin, int ymax, int zmin, int zmax);
  /**
   * Returns the Volume Of Interest as an array in the following format: [xmin, xmax, ymin, ymax, zmin, zmax].
   */
  int *getVOI();

#ifdef USTK_HAVE_CUDA
  /**
   * Sets the usScanConverter3D performing volume reconstruction.
   */
  void setConverter(usScanConverter3D *converter);

  /**
   * Return a pointer to the usScanConverter3D performing volume reconstruction.
   */
  usScanConverter3D *getConverter();
#endif

  /**
   * Returns the detected needle model.
   */
  us3DNeedleModel *getNeedle();

  /**
   * Returns true iff the needle has been detected.
   */
  bool isNeedleDetected();

  /**
   * Returns the velocity of the needle tip.
   */
  double *getTipVelocity();

  /**
   * Returns the position of the needle tip.
   */
  vpColVector getTipPose();

  /**
   * Returns the velocity of the entry point.
   */
  double *getEntryVelocity();

  /**
   * Returns the velocity of the entry point in the direction of axis c.
   */
  double getEntryVelocity(int c);

  /**
   * Returns the position of the entry point.
   */
  vpColVector getEntryPose();

  /**
   * Returns the position of the entry point along the axis c.
   */
  double getEntryPose(int c);

  /**
   * Returns a pointer to the us3DNeedleTracker.
   */
  us3DNeedleTracker *getNeedleTracker();
  
  /**
   * Returns the probe pose error wrt the desired pose.
   * This should rather be in the us3DController class.
   */
  vpColVector getPoseError();
  
  /**
   * Runs the needle detection process with the current volume.
   */
  void run();

  /**
   * Runs the needle detection process with the current post-scan volume.
   *
   * @param vol The input post-scan volume.
   */
  void run(usImagePostScan3D<unsigned char>& vol);

#ifdef USTK_HAVE_CUDA
  /**
   * Runs the needle detection process with the current pre-scan volume.
   *
   * @param volIdx The index of the volume where to detect the needle.
   */
  void run(int volIdx);
#endif

private:

  /**
   * usNeedleTrackerRANSAC3D state.
   * IDLE: wait for next volume.
   * DETECTION: try do detect a needle insertion using volume differences.
   * TRACKING: detect the needle using its previous position.
   */
  enum State
    {
      IDLE,
      DETECTION,
      TRACKING
    };

  /**
   * The number of control points defining the needle.
   */
  unsigned int m_nPoints;

  /**
   * The current number of active control Points.
   */
  unsigned int m_nActivePoints;

  /**
   * Number of segments used to render the needle.
   */
  unsigned int m_nRenderSegments;

  /**
   * The current volume index.
   */
  int m_volIndex;

  /**
   * The previous volume index.
   */
  int m_lastVolIdx;

  /**
   * The current state of the usNeedleTrackerRANSAC3D.
   */
  int m_state;

  /**
   * Variance threshold used to discriminate between global and local motion.
   */
  double m_varianceThreshold;

  /**
   * Number of candidate points used for detection.
   */
  unsigned int m_nCandidatePointsDetection;

  /**
   * Number of candidate points used during tracking.
   */
  unsigned int m_nCandidatePointsTracking;
  double m_tipIntensityDropThreshold;

  // Image processing

  /**
   * The volume extent.
   */
  int *m_extent;

  /**
   * The volume dimensions.
   */
  int *m_dimensions;

  /**
   * Coordinates of the Volume Of Interest.
   */
  int *m_VOI;

#ifdef USTK_HAVE_CUDA
  /**
   * Pointer to the usScanConverter3D providing the reconstructed volume.
   */
  usScanConverter3D *m_converter;
#endif

  /**
   * Array of pointers to the vtkImageImport object for conversion of images to the vtk format.
   */
  vtkImageImport **m_imageImport;

  /**
   * Array of pointers to the vtkExtract VOI objects for extraction of the volume of interest.
   */
  vtkExtractVOI **m_extractVOI;

  /**
   * Array of pointers to the vtkImageCast objects for type casting.
   */
  vtkImageCast **m_castUInt2Short;

  /**
   * Pointer to a difference image filter.
   */
  vtkImageMathematics *m_difference;

  /**
   * Pointer to an absolute value image filter.
   */
  vtkImageMathematics *m_absoluteValue;

  // Needle detection

  /**
   * The RANSAC algorithm manager.
   */
  softRansac m_ransac;

  /**
   * Curve model parameters.
   */
  vpMatrix m_model;

  /**
   * Minimal sample set
   */
  vpMatrix m_mss;

  /**
   * Set of inliers.
   */
  vpColVector m_inliers;

  /**
   * Number of inliers.
   */
  int m_ninliers;

  /**
   * Median point of the candidate points.
   */
  vpColVector m_median;

  /**
   * vtkLineSource representation of the needle (used for display)
   */
  //vtkLineSource *m_needle;

  /**
   * Principal direction of the set of candidate points.
   */
  vpColVector m_principalDir;

  /**
   * Needle control points (ordered from entry to tip).
   */
  double **m_points;

  /**
   * Needle representation.
   */
  us3DNeedleModel *m_needle;

  /**
   * Predicted needle model.
   */
  us3DNeedleModel *m_predictedNeedle;

  /**
   * Pointer to the vtkProbeFilter used for interpolation along the needle axis.
   */
  vtkProbeFilter *m_probeFilter;

  /**
   * Pointer to the vtkPlane object representing a boundary for the tip detection.
   */
  vtkPlane *m_tipLimit;

  // Needle Tracking
  /**
   * Needle tracker.
   */
  us3DNeedleTracker m_needleTracker;

  /**
   * Entry point tracker.
   */
  us3DEntryTracker m_entryTracker;
  double *m_pose;
  double *m_velocity;
  vpColVector m_measure;
  bool m_fixEntryPoint;
  unsigned int m_missCount;
  unsigned int m_startIndex;

  // Plot
  unsigned int m_plotIdx;
  vpPlot m_plot;

  // Control
  double m_resolution;
  vpHomogeneousMatrix m_iMc; // Initial probe pose -> current probe pose
  vpHomogeneousMatrix m_rMp; // Probe frame -> image frame

  // FOV parameters
  /**
   * Frame field of view (in degrees).
   */
  float m_fov;

  /**
   * Acquisition origin.
   */
  vpColVector m_origin;

  /**
   * Entry plane normal vector.
   */
  vpColVector m_entryPlane;
  
  /**
   * Run a detection step on the current volume.
   */
  void detect(bool verbose);

  /**
   * Run a tracking step on the current volume.
   */
  void track(bool verbose);

};

#endif
