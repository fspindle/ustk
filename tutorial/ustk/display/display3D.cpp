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

/*                                                                -*-c++-*-
#----------------------------------------------------------------------------
#  
#	Example file for needle detection.
#
#       Pierre Chatelain
#       July 10, 2015
#
#----------------------------------------------------------------------------
*/

//ustk
#include <visp3/ustk_io/usImageIo.h>
#include <visp3/ustk_gui/usDisplay3D.h>

using namespace std;

/* -------------------------------------------------------------------------- */
/*                         COMMAND LINE OPTIONS                               */
/* -------------------------------------------------------------------------- */

// List of allowed command line options
#define GETOPTARGS	"cdo:h"

void usage(const char *name, const char *badparam, const std::string& opath, const std::string& user);
bool getOptions(int argc, const char **argv, std::string &opath, const std::string& user);

/*!

Print the program options.

\param name : Program name.
\param badparam : Bad parameter name.
\param opath : Output image path.
\param user : Username.

 */
void usage(const char *name, const char *badparam, const std::string& opath, const std::string& user)
{
  fprintf(stdout, "\n\
          Write and read ultrasound sequences in 2d image files, and the associated xml settings file.\n\
          \n\
          SYNOPSIS\n\
          %s [-o <output image path>] [-h]\n", name);

      fprintf(stdout, "\n\
              OPTIONS:                                               Default\n\
              -o <output data path>                               %s\n\
              Set data output path.\n\
              From this directory, creates the \"%s\"\n\
              subdirectory depending on the username, where \n\
              sequenceRF2D.xml file is written.\n\
              \n\
              -h\n\
              Print the help.\n\n", opath.c_str(), user.c_str());

              if (badparam) {
                fprintf(stderr, "ERROR: \n" );
                fprintf(stderr, "\nBad parameter [%s]\n", badparam);
              }
}

/*!
  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param opath : Output data path.
  \param user : Username.
  \return false if the program has to be stopped, true otherwise.
*/
/*bool getOptions(int argc, const char **argv, std::string &opath, const std::string& user)
{
  const char *optarg_;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'o': opath = optarg_; break;
    case 'h': usage(argv[0], NULL, opath, user); return false; break;

    case 'c':
    case 'd':
      break;

    default:
      usage(argv[0], optarg_, opath, user); return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, opath, user);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}*/


int main(int argc, const char *argv[])
{
  ///////////////////
  ///   OPTIONS   ///
  ///////////////////
/*
  std::string opt_opath;
  std::string ipath;
  std::string username;

  char *logFilename = new char [FILENAME_MAX];
  const char *opath = new char [FILENAME_MAX];
  char *windowTitle = new char[32];

  // Set the default output path
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  opt_opath = "/tmp";
#elif defined(_WIN32)
  opt_opath = "C:\\temp";
#endif

  // Get the user login name
  vpIoTools::getUserName(username);

  // Read the command line options
  if (getOptions(argc, argv, opt_opath, username) == false) {
    exit (-1);
  }

  // Get the option values
  if (!opt_opath.empty())
    opath = opt_opath.c_str();

  // Append to the output path string, the login name of the user
  std::string dirname = vpIoTools::createFilePath(opath, username);

  // Test if the output path exist. If no try to create it
  if (vpIoTools::checkDirectory(dirname) == false) {
    try {
      // Create the dirname
      vpIoTools::makeDirectory(dirname);
    }
    catch (...) {
      usage(argv[0], NULL, opath, username);
      std::cerr << std::endl
                << "ERROR:" << std::endl;
      std::cerr << "  Cannot create " << dirname << std::endl;
      std::cerr << "  Check your -o " << opath << " option " << std::endl;
      exit(-1);
    }
  }
*/
  ///////////////////////////
  ///   INITIALISATIONS   ///
  ///////////////////////////

  //image reading
  usImagePostScan3D<unsigned char> postScan3D;
  usImageIo::read(postScan3D,"/home/mpouliqu/Documents/usData/postscan/3D/postscan3d.mhd");

// Initialize the display of the volume
  vtkRenderWindowInteractor *renderWindowInteractor = vtkRenderWindowInteractor::New();

  int centerOrigin[3];
  centerOrigin[0] = -postScan3D.getDimX()/2;
  centerOrigin[1] = -postScan3D.getDimY()/2;
  centerOrigin[2] = -postScan3D.getDimZ()/2;

  int dims[3];
  dims[0] = postScan3D.getDimX();
  dims[1] = postScan3D.getDimY();
  dims[2] = postScan3D.getDimZ();

  usDisplay3D display = usDisplay3D();
  display.Init(renderWindowInteractor, 800,  800,  centerOrigin, dims, dims[0]/2, dims[1]/2, dims[2]/2, 10, 45, 0, 3*dims[0]);

  /////////////////////
  ///   RENDERING   ///
  /////////////////////

  display.render(postScan3D.getConstData(), postScan3D.getSize());


  return 0;
}
