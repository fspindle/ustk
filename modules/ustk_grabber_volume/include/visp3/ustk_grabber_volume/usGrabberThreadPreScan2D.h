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
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usGrabberThreadPreScan2D.h
 * @brief Grabber thread for 2D pre-scan images.
 */

#ifndef __usGrabberThreadPreScan2D_h_
#define __usGrabberThreadPreScan2D_h_

#include <visp3/ustk_gui/usGuiConfig.h>

#ifdef USTK_HAVE_VTK_QT

#include <visp3/ustk_grabber/usGrabberUltrasonix.h>
#include <visp3/ustk_grabber/usGrabberFrame.h>

// Qt includes
#if defined(USTK_HAVE_VTK_QT4)
#  include <QtCore/QThread>
#  include <QtCore/QMutex>
#include <QtCore/QObject>
#elif defined(USTK_HAVE_VTK_QT5)
#  include <QtCore/QThread>
#  include <QtCore/QMutex>
#include <QtCore/QObject>
#endif
/**
 * @class usGrabberThreadPreScan2D
 * @brief Grabber thread for 2D pre-scan images.
 * @ingroup module_ustk_grabber_volume
 */
class VISP_EXPORT usGrabberThreadPreScan2D : public QThread {
  Q_OBJECT
public:
  usGrabberThreadPreScan2D(usImagePreScan2D<unsigned char> *frame, QMutex* frameMutex);

  virtual ~usGrabberThreadPreScan2D();


public slots :
  //to stop the grabbing loop.
  void stopGrabbing();

signals:
  void newFrameGrabbed(int frameNumber);

protected :
  void run();

private:
  bool m_stopGrabbing;

  usGrabberUltrasonix m_grabberUltrasonix;
  usGrabberFrame<usImagePreScan2D<unsigned char> > m_grabberFramePreScan;

  QMutex * m_frameMutex;
  usImagePreScan2D<unsigned char> * m_framePreScan;
};

#endif // USTK_HAVE_VTK_QT
#endif // __usGrabberThreadPreScan2D_h_
