/****************************************************************************
 *
 * This file is part of the ustk software.
 * Copyright (C) 2016 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ustk with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at ustk@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author:
 * Jason Chevrie
 *
 *****************************************************************************/

#include <visp3/ustk_needle_modeling/usTissueModelPolynomial.h>

usTissueModelPolynomial::usTissueModelPolynomial() : m_surface(), m_path() {}

usTissueModelPolynomial::usTissueModelPolynomial(const usTissueModelPolynomial &tissue)
  : m_surface(tissue.m_surface), m_path(tissue.m_path)
{
}

usTissueModelPolynomial::~usTissueModelPolynomial() {}

const usTissueModelPolynomial &usTissueModelPolynomial::operator=(const usTissueModelPolynomial &tissue)
{
  m_surface = tissue.m_surface;
  m_path = tissue.m_path;

  return *this;
}

usTissueModelPolynomial *usTissueModelPolynomial::clone() const { return new usTissueModelPolynomial(*this); }

const usOrientedPlane3D &usTissueModelPolynomial::accessSurface() const { return m_surface; }

usOrientedPlane3D &usTissueModelPolynomial::accessSurface() { return m_surface; }

const usPolynomialCurve3D &usTissueModelPolynomial::accessPath() const { return m_path; }

usPolynomialCurve3D usTissueModelPolynomial::accessPath() { return m_path; }

bool usTissueModelPolynomial::moveInWorldFrame(const vpHomogeneousMatrix &H)
{
  m_surface.moveInWorldFrame(H);
  m_path.move(H);

  return true;
}

bool usTissueModelPolynomial::moveInWorldFrame(double x, double y, double z, double tx, double ty, double tz)
{
  return this->moveInWorldFrame(vpHomogeneousMatrix(x, y, z, tx, ty, tz));
}

bool usTissueModelPolynomial::move(const vpHomogeneousMatrix &H)
{
  vpPoseVector ptu(this->getPose());

  vpHomogeneousMatrix M(ptu);
  M = M * H * M.inverse();
  m_surface.moveInWorldFrame(M);
  m_path.move(M);

  return true;
}

bool usTissueModelPolynomial::move(double x, double y, double z, double tx, double ty, double tz)
{
  return this->move(vpHomogeneousMatrix(x, y, z, tx, ty, tz));
}

bool usTissueModelPolynomial::setPose(const vpPoseVector &p)
{
  vpPoseVector ptu(this->getPose());

  vpHomogeneousMatrix M(vpHomogeneousMatrix(p) * vpHomogeneousMatrix(ptu).inverse());
  m_surface.moveInWorldFrame(M);
  m_path.move(M);

  return true;
}

vpPoseVector usTissueModelPolynomial::getPose() const
{
  vpPoseVector ptu = m_surface.getPose();

  vpColVector p = m_path.getPoint(0);
  for (int i = 0; i < 3; i++)
    ptu[i] = p[i];

  return ptu;
}

std::ostream &operator<<(std::ostream &s, const usTissueModelPolynomial &tissue)
{
  s << "usTissueModelPolynomial\n";
  s << tissue.m_surface;
  s << tissue.m_path;

  s.flush();
  return s;
}

std::istream &operator>>(std::istream &s, usTissueModelPolynomial &tissue)
{
  std::string c;
  s >> c;
  if (c != "usTissueModelPolynomial") {
    vpException e(vpException::ioError, "Stream does not contain usTissueModelPolynomial data");
    throw e;
  }
  s >> tissue.m_surface;
  s >> tissue.m_path;
  return s;
}

std::ostream &operator<<=(std::ostream &s, const usTissueModelPolynomial &tissue)
{
  s.write("usTissueModelPolynomial", 24);
  s <<= tissue.m_surface;
  s <<= tissue.m_path;

  s.flush();
  return s;
}

std::istream &operator>>=(std::istream &s, usTissueModelPolynomial &tissue)
{
  char c[24];
  s.read(c, 24);
  if (strcmp(c, "usTissueModelPolynomial")) {
    vpException e(vpException::ioError, "Stream does not contain usTissueModelPolynomial data");
    throw e;
  }
  s >>= tissue.m_surface;
  s >>= tissue.m_path;
  return s;
}
