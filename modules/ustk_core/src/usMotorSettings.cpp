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
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usMotorSettings.cpp
 * @brief Generic ultrasound 3D image settings.
 */

//std includes
#include <iostream>

//visp includes
#include <visp3/ustk_core/usMotorSettings.h>

//ustk includes

/**
* Basic Constructor, all settings set to default.
*/
usMotorSettings::usMotorSettings() : m_motorRadius(0.0), m_framePitch(0.0), m_isMotorRotating(true) {}

/**
* Full Constructor, all settings availables
* @param motorRadius Distance between the rotation center of the probe motor and the first pixel arc acquired, in meters (m).
* @param framePitch Radius between 2 successives acquisiton planes in the probe, in radians (rad).
* @param isMotorRotating True if the probe motor is rotating, false otherwise.
*/
usMotorSettings::usMotorSettings(double motorRadius, double framePitch, bool isMotorRotating) : m_motorRadius(motorRadius), m_framePitch(framePitch), m_isMotorRotating(isMotorRotating) {}

/**
* Copy Constructor, all settings availables
* @param other usMotorSettings you want to copy.
*/
usMotorSettings::usMotorSettings(const usMotorSettings &other) : m_motorRadius(other.getMotorRadius()), m_framePitch(other.getFramePitch()), m_isMotorRotating(other.isMotorRotating()) {}

/**
* Destructor.
*/
usMotorSettings::~usMotorSettings() {}

/**
* Assignment operator.
* @param other usMotorSettings you want to copy.
*/
usMotorSettings& usMotorSettings::operator=(const usMotorSettings& other)
{
  m_motorRadius = other.getMotorRadius();
  m_framePitch = other.getFramePitch();
  m_isMotorRotating = other.isMotorRotating();
  return *this;
}

bool usMotorSettings::operator==(const usMotorSettings& other)
{
  return (this->getFramePitch() == other.getFramePitch() &&
          this->getMotorRadius() == other.getMotorRadius() &&
          this->isMotorRotating() == other.isMotorRotating());
}

/**
* Print probe settings information.
*/
VISP_EXPORT std::ostream& operator<<(std::ostream& out, const usMotorSettings& other)
{
  return out << "motor radius : " << other.getMotorRadius() << std::endl
    << "frame angle : " << other.getFramePitch() << std::endl
    << "is motor rotating : " << other.isMotorRotating() << std::endl;
}

//probe settings getters/setters

/**
* Set the motor  radius (m).
* @param motorRadius Motor radius in meters.
*/
void usMotorSettings::setMotorRadius(double motorRadius) { m_motorRadius = motorRadius; }

/**
* Get the probe radius (m).
* @return motorRadius Motor radius in meters.
*/
double usMotorSettings::getMotorRadius() const { return m_motorRadius; }

/**
* Set the frame angle (rad).
* @param framePitch Frame angle of the probe in radians.
*/
void usMotorSettings::setFramePitch(double framePitch) { m_framePitch = framePitch; }

/**
* Get the frame angle (rad).
* @return m_lineAngle Frame angle of the probe in radians.
*/
double usMotorSettings::getFramePitch() const { return m_framePitch; }

/**
* Set the motor type : convex or linear (from probe type used to acquire the image).
* @param isMotorRotating Boolean to specify the motor type : true for a rotating motor, false for a linear motor.
*/
void usMotorSettings::setMotorConvexity(bool isMotorRotating) {
  m_isMotorRotating = isMotorRotating;
  if (!isMotorRotating) {
    setMotorRadius(0.0);
  }
}

/**
* Get the motor type : convex or linear (from probe type used to acquire the image).
* @return isMotorRotating Boolean to specify the motor type : true for convex, false for linear.
*/
bool usMotorSettings::isMotorRotating() const { return m_isMotorRotating; }

/**
* Get the motor type : convex or linear (from probe type used to acquire the image).
* @return isMotorRotating Boolean to specify the motor type : true for convex, false for linear.
*/
void usMotorSettings::setMotorSettings(const usMotorSettings &other)
{
  *this = other;
}
