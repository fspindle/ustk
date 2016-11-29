/****************************************************************************
 *
 * This file is part of the UsTk software.
 * Copyright (C) 2014 by Inria. All rights reserved.
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
 *
 * Authors:
 * Pierre Chatelain
 *
 *****************************************************************************/

/**
 * @file usImage3DProcessing.h
 * @brief Volume processing.
 * @author Pierre Chatelain
 */

#ifndef US_IMAGE_3D_PROCESSING_H
#define US_IMAGE_3D_PROCESSING_H

#include <float.h>
#include <cmath>
#include <cstring>

#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>

#include <visp3/ustk_core/usImage3D.h>
/**
 * @namespace usImage3DProcessing
 * @brief Processing tools for the usImage3D class.
 *
 * This namespace contains processing tools (derivative, filtering...) for the usVolume class.
 */
namespace usImage3DProcessing
{
  /**
   * Get the max value.
   */
  template<class T>
    T max(const usImage3D<T> &V);

  /**
   * Apply a 3x1x1 derivative filter to a voxel.
   */
  template<class T>
    double derivativeX(const usImage3D<T> &V, unsigned int x, unsigned int y, unsigned int z);
  
  /**
   * Apply a 1x3x1 derivative filter to a voxel.
   */
  template<class T>
    double derivativeY(const usImage3D<T> &V, unsigned int x, unsigned int y, unsigned int z);
  
  /**
   * Apply a 1x1x3 derivative filter to a voxel.
   */
  template<class T>
    double derivativeZ(const usImage3D<T> &V, unsigned int x, unsigned int y, unsigned int z);
  
  /**
   * Apply a 3x1x1 derivative filter to a volume.
   */
  template<class T>
    void derivativeX(const usImage3D<T> &Src, usImage3D<double> &Dst);

  /**
   * Compute the volume gradient.
   */
  template<class T>
    void gradient(const usImage3D<T> &Src, usImage3D<vpColVector> &Dst);

  /**
   * Compute the volume hessian.
   */
  template<class T>
    void hessian(const usImage3D<T> &Src, usImage3D<vpMatrix> &Dst);

  /**
   * Compute the norm of a vector image.
   */
  void norm(const usImage3D<vpColVector> &Src, usImage3D<double> &Dst);

  /**
   * Compute the volume difference.
   */
  template<class T1, class T2>
    void difference(const usImage3D<T1> &Src1, const usImage3D<T1> &Src2, usImage3D<T2> &Dst);

  /**
   * Compute the volume absolute difference.
   */
  template<class T1, class T2>
    void absoluteDifference(const usImage3D<T1> &Src1, const usImage3D<T1> &Src2, usImage3D<T2> &Dst);

  /**************************************************************
   * Instanciations
   **************************************************************/

  template<class T>
    T max(const usImage3D<T> &V)
    {
      T max = V(0);
      for (unsigned int i = 1; i < V.getSize(); ++i)
	if (V(i) > max)
	  max = V(i);
      return max;
    }

  template<class T>
    double derivativeX(const usImage3D<T> &V, unsigned int x, unsigned int y, unsigned int z)
    {
      return (V(x+1, y, z) - V(x-1, y, z)) / 2.0;
    }

  template<class T>
    double derivativeY(const usImage3D<T> &V, unsigned int x, unsigned int y, unsigned int z)
    {
      return (V(x, y+1, z) - V(x, y-1, z)) / 2.0;
    }

  template<class T>
    double derivativeZ(const usImage3D<T> &V, unsigned int x, unsigned int y, unsigned int z)
    {
      return (V(x, y, z+1) - V(x, y, z-1)) / 2.0;
    }

  template<class T>
    void derivativeX(const usImage3D<T> &Src, usImage3D<double> &Dst)
    {
      unsigned int dimx(Src.getDimX()), dimy(Src.getDimY()), dimz(Src.getDimZ());
      Dst.resize(dimx, dimy, dimz);
      // Access in order z-y-x for performance
      for (unsigned int z = 0; z < dimz; ++z) {
	for (unsigned int y = 0; y < dimy; ++y) {
	  Dst(0, y, z, 0.0);
	  for (unsigned int x = 1; x < dimx-1; ++x)
	    Dst(x, y, z, derivativeX(Src, x, y, z));
	  Dst(dimx-1, y, z, 0.0);
	}
      }
    }

  template<class T>
    void derivativeY(const usImage3D<T> &Src, usImage3D<double> &Dst)
    {
      unsigned int dimx(Src.getDimX()), dimy(Src.getDimY()), dimz(Src.getDimZ());
      Dst.resize(dimx, dimy, dimz);
      // Access in order z-y-x for performance
      for (unsigned int z = 0; z < dimz; ++z) {
	for (unsigned int x = 0; x < dimx; ++x)
	  Dst(x, 0, z, 0.0);
	for (unsigned int y = 1; y < dimy-1; ++y)
	  for (unsigned int x = 0; x < dimx; ++x)
	    Dst(x, y, z, derivativeY(Src, x, y, z));
	for (unsigned int x = 0; x < dimx; ++x)
	  Dst(x, dimy-1, z, 0.0);
      }
    }

  template<class T>
    void derivativeZ(const usImage3D<T> &Src, usImage3D<double> &Dst)
    {
      unsigned int dimx(Src.getDimX()), dimy(Src.getDimY()), dimz(Src.getDimZ());
      Dst.resize(dimx, dimy, dimz);
      // Access in order z-y-x for performance
      for (unsigned int y = 0; y < dimy; ++y)
	for (unsigned int x = 0; x < dimx; ++x)
	  Dst(x, y, 0, 0.0);
      for (unsigned int z = 1; z < dimz-1; ++z) {
	for (unsigned int y = 0; y < dimy; ++y) {
	  for (unsigned int x = 0; x < dimx; ++x)
	    Dst(x, y, z, derivativeZ(Src, x, y, z));
	}
      }
      for (unsigned int y = 0; y < dimy; ++y)
	for (unsigned int x = 0; x < dimx; ++x)
	  Dst(x, y, dimz-1, 0.0);
    }
  
  template<class T>
    void gradient(const usImage3D<T> &Src, usImage3D<vpColVector> &Dst)
    {
      unsigned int dimx(Src.getDimX()), dimy(Src.getDimY()), dimz(Src.getDimZ());
      usImage3D<double> Gx, Gy, Gz;
      derivativeX(Src, Gx);
      derivativeY(Src, Gy);
      derivativeZ(Src, Gz);
      Dst.resize(dimx, dimy, dimz);
      for (unsigned int i = 0; i < Src.getSize(); ++i) {
	vpColVector v(3);
	v[0] = Gx(i);
	v[1] = Gy(i);
	v[2] = Gz(i);
	Dst(i, v);
      }
    }

  template<class T>
    void hessian(const usImage3D<T> &Src, usImage3D<vpMatrix> &Dst)
    {
      unsigned int dimx(Src.getDimX()), dimy(Src.getDimY()), dimz(Src.getDimZ());
      usImage3D<double> Gx, Gy, Gz, Gxx, Gxy, Gxz, Gyy, Gyz, Gzz;
      derivativeX(Src, Gx);
      derivativeY(Src, Gy);
      derivativeZ(Src, Gz);
      derivativeX(Gx, Gxx);
      derivativeX(Gy, Gxy);
      derivativeX(Gz, Gxz);
      derivativeY(Gy, Gyy);
      derivativeY(Gz, Gyz);
      derivativeZ(Gz, Gzz);
      Dst.resize(dimx, dimy, dimz);
      for (unsigned int i = 0; i < Src.getSize(); ++i) {
	vpMatrix M(3, 3);
	M[0][0] = Gxx(i);
	M[0][1] = Gxy(i);
	M[0][2] = Gxz(i);
	M[1][0] = Gxy(i);
	M[1][1] = Gyy(i);
	M[1][2] = Gyz(i);
	M[2][0] = Gxz(i);
	M[2][1] = Gyz(i);
	M[2][2] = Gzz(i);
	Dst(i, M);
      }
    }
  
  template<class T1, class T2>
    void difference(const usImage3D<T1> &Src1, const usImage3D<T1> &Src2, usImage3D<T2> &Dst)
  {
    unsigned int dimx = Src1.getDimX();
    unsigned int dimy = Src1.getDimY();
    unsigned int dimz = Src1.getDimZ();
    unsigned int n = Src1.getSize();
    Dst.resize(dimx, dimy, dimz);
    for (unsigned int i = 0; i < n; ++i) {
      Dst(i, Src1(i) - Src2(i));
    }
  }

  template<class T1, class T2>
    void absoluteDifference(const usImage3D<T1> &Src1, const usImage3D<T1> &Src2, usImage3D<T2> &Dst)
  {
    unsigned int dimx = Src1.getDimX();
    unsigned int dimy = Src1.getDimY();
    unsigned int dimz = Src1.getDimZ();
    unsigned int n = Src1.getSize();
    Dst.resize(dimx, dimy, dimz);
    for (unsigned int i = 0; i < n; ++i) {
      Dst(i, abs(Src1(i) - Src2(i)));
    }
  }
}

#endif
