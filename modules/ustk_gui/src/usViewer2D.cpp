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
 * Marc Pouliquen
 *
 *****************************************************************************/

#include <visp3/ustk_gui/usViewer2D.h>

/**
* Default constructor.
*/
usViewer2D::usViewer2D()
{
  vtkSmartPointer<vtkMetaImageReader> reader =
      vtkSmartPointer<vtkMetaImageReader>::New();
  reader->SetFileName("/home/mpouliqu/Documents/usData/postscan/3D/postscan3d.mhd");
  reader->Update();

  // Visualize
  m_imageViewer = vtkSmartPointer<vtkImageViewer2>::New();
  m_imageViewer->SetInputConnection(reader->GetOutputPort());

  std::cout << "1" << std::endl;
  //m_imageViewer->SetSliceOrientationToYZ();
  //std::cout << m_imageViewer->GetSliceOrientation() << std::endl;

  //this->setOrientation(usViewer2D::Yorientation);

  // slice status message
  std::cout << "2" << std::endl;
  m_sliceTextProp = vtkSmartPointer<vtkTextProperty>::New();
  m_sliceTextProp->SetFontFamilyToCourier();
  m_sliceTextProp->SetFontSize(20);
  m_sliceTextProp->SetVerticalJustificationToBottom();
  m_sliceTextProp->SetJustificationToLeft();

  std::cout << "3" << std::endl;
  m_sliceTextMapper = vtkSmartPointer<vtkTextMapper>::New();
  std::stringstream tmp;
  tmp << "Slice " << m_imageViewer->GetSliceMin() + 1 << "/" << m_imageViewer->GetSliceMax() + 1;
  std::string msg = tmp.str();
  m_sliceTextMapper->SetInput(msg.c_str());
  m_sliceTextMapper->SetTextProperty(m_sliceTextProp);

  std::cout << "4" << std::endl;
  m_sliceTextActor = vtkSmartPointer<vtkActor2D>::New();
  m_sliceTextActor->SetMapper(m_sliceTextMapper);
  m_sliceTextActor->SetPosition(15, 10);

  std::cout << "5" << std::endl;
  // create an interactor with our own style (inherit from vtkInteractorStyleImage)
  // in order to catch mousewheel and key events
  m_renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();

  m_interactorStyle = vtkSmartPointer<usInteractor2D>::New();
  std::cout << "6" << std::endl;

  // make imageviewer2 and sliceTextMapper visible to our interactorstyle
  // to enable slice status message updates when scrolling through the slices
  m_interactorStyle->SetImageViewer(m_imageViewer);
  m_interactorStyle->SetStatusMapper(m_sliceTextMapper);

  std::cout << "7" << std::endl;
  m_imageViewer->SetupInteractor(m_renderWindowInteractor);
  // make the interactor use our own interactorstyle
  // cause SetupInteractor() is defining it's own default interatorstyle
  // this must be called after SetupInteractor()
  m_renderWindowInteractor->SetInteractorStyle(m_interactorStyle);
  // add slice status message and usage hint message to the renderer
  m_imageViewer->GetRenderer()->AddActor2D(m_sliceTextActor);

  std::cout << "8" << std::endl;
  // initialize rendering and interaction
  //imageViewer->GetRenderWindow()->SetSize(400, 300);
  //imageViewer->GetRenderer()->SetBackground(0.2, 0.3, 0.4);
  std::cout << "9" << std::endl;
  m_imageViewer->Render();
  std::cout << "10" << std::endl;
  m_imageViewer->GetRenderer()->ResetCamera();
  std::cout << "11" << std::endl;
  m_imageViewer->Render();
  std::cout << "12" << std::endl;
}

/**
* Destructor.
*/
usViewer2D::~usViewer2D()
{

}

/**
* Start display.
*/
void usViewer2D::start()
{
  m_renderWindowInteractor->Start();
}

/**
* Set orientation of th view.
*/
void usViewer2D::setOrientation(usViewer2D::Orientation orientation)
{
  std::cout << "setOrientation" << std::endl;
  if(orientation == usViewer2D::Xorientation)
    m_imageViewer->SetSliceOrientationToYZ();
  else if(orientation == usViewer2D::Yorientation)
    m_imageViewer->SetSliceOrientationToXZ();
  else if(orientation == usViewer2D::Zorientation)
    m_imageViewer->SetSliceOrientationToXY();
  std::cout << "setOrientation end" << std::endl;
  m_imageViewer->UpdateDisplayExtent();
}

