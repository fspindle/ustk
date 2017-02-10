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
 * Authors:
 * Alexandre Krupa
 * Pierre Chatelain
 *
 *****************************************************************************/

/**
 * @file usGrabberThreadPreScan2D.cpp
 */

#include <visp3/ustk_grabber_volume/usGrabberThreadPreScan2D.h>

#ifdef USTK_HAVE_VTK_QT


usGrabberThreadPreScan2D::usGrabberThreadPreScan2D(usImagePreScan2D<unsigned char> * frame, QMutex* frameMutex) :
  m_stopGrabbing(false), m_grabberUltrasonix(), m_grabberFramePreScan(), m_frameMutex(frameMutex), m_framePreScan(frame){

}

usGrabberThreadPreScan2D::~usGrabberThreadPreScan2D() {

}

void usGrabberThreadPreScan2D::stopGrabbing() {
  m_stopGrabbing = false;
}

void usGrabberThreadPreScan2D::run() {

  m_grabberUltrasonix.start();
  if(m_grabberUltrasonix.getImageType() != us::PRESCAN_2D)
    throw(vpException(vpException::badValue,"usGrabberThreadPreScan2D receiving no 2D pre-scan images"));

  //resize frame
  m_frameMutex->lock();
  m_framePreScan->resize(m_grabberUltrasonix.getCommunicationsInformations()->m_header.height,
                         m_grabberUltrasonix.getCommunicationsInformations()->m_header.width);


  m_grabberFramePreScan.setCommunicationInformation(m_grabberUltrasonix.getCommunicationsInformations());
  m_grabberFramePreScan.setTransducerSettings(m_grabberUltrasonix.getTransducerSettings());
  m_frameMutex->unlock();

  int frameIncrement = -1;
  int frameIndex = 0;
  int volumeIndex = 0;

  while (! m_stopGrabbing) {
    //get new frame
    m_frameMutex->lock();
    m_grabberFramePreScan.grabFrame(m_framePreScan);
    m_frameMutex->unlock();

    //calculate the frame index
    frameIndex = m_grabberUltrasonix.getCommunicationsInformations()->m_totFrmIdx % m_grabberUltrasonix.getCommunicationsInformations()->m_header.framesPerVolume;

    //update frame increment if we reach last frame in a direction
    if(m_grabberUltrasonix.getCommunicationsInformations()->m_totFrmIdx % (m_grabberUltrasonix.getCommunicationsInformations()->m_header.framesPerVolume - 1) == 0)
      frameIncrement = -1 * frameIncrement;

    volumeIndex = m_grabberUltrasonix.getCommunicationsInformations()->m_totFrmIdx / m_grabberUltrasonix.getCommunicationsInformations()->m_header.framesPerVolume;
    frameIndex = m_grabberUltrasonix.getCommunicationsInformations()->m_totFrmIdx % m_grabberUltrasonix.getCommunicationsInformations()->m_header.framesPerVolume;

    if (volumeIndex % 2)
      frameIndex = m_grabberUltrasonix.getCommunicationsInformations()->m_header.framesPerVolume - frameIndex - 1;

    //frameNumber += frameIncrement;
    std::cout << "total frame index = " << m_grabberUltrasonix.getCommunicationsInformations()->m_totFrmIdx << std::endl;
    std::cout << "frame index = " << frameIndex << std::endl;
    emit(newFrameGrabbed(frameIndex));
  }
}


#endif // USTK_HAVE_VTK_QT
