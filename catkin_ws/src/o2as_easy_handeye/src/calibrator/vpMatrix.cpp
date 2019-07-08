/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Matrix manipulation.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/
/*!
\file vpMatrix.cpp
\brief Definition of the vpMatrix class
*/

#include <limits> // numeric_limits
#include <cmath>

#include <visp3/core/vpConfig.h>

#include <gsl/gsl_eigen.h>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMatrix.h>


void vpMatrix::eigenValues(vpColVector &evalue, vpMatrix &evector) const
{
  if (rowNum != colNum) {
    throw(vpException(vpException::dimensionError, "Cannot compute eigen values on a non square matrix (%dx%d)", rowNum,
                      colNum));
  }

  {
    // Check if the matrix is symetric: At - A = 0
    vpMatrix At_A = (*this).t() - (*this);
    for (unsigned int i = 0; i < rowNum; i++) {
      for (unsigned int j = 0; j < rowNum; j++) {
        // if (At_A[i][j] != 0) {
        if (std::fabs(At_A[i][j]) > std::numeric_limits<double>::epsilon()) {
          throw(vpException(vpException::fatalError, "Cannot compute eigen values on a non symetric matrix"));
        }
      }
    }

    // Resize the output matrices
    evalue.resize(rowNum);
    evector.resize(rowNum, colNum);

    gsl_vector *eval = gsl_vector_alloc(rowNum);
    gsl_matrix *evec = gsl_matrix_alloc(rowNum, colNum);

    gsl_eigen_symmv_workspace *w = gsl_eigen_symmv_alloc(rowNum);
    gsl_matrix *m = gsl_matrix_alloc(rowNum, colNum);

    unsigned int Atda = (unsigned int)m->tda;
    for (unsigned int i = 0; i < rowNum; i++) {
      unsigned int k = i * Atda;
      for (unsigned int j = 0; j < colNum; j++)
        m->data[k + j] = (*this)[i][j];
    }
    gsl_eigen_symmv(m, eval, evec, w);

    gsl_eigen_symmv_sort(eval, evec, GSL_EIGEN_SORT_ABS_ASC);

    for (unsigned int i = 0; i < rowNum; i++) {
      evalue[i] = gsl_vector_get(eval, i);
    }
    Atda = (unsigned int)evec->tda;
    for (unsigned int i = 0; i < rowNum; i++) {
      unsigned int k = i * Atda;
      for (unsigned int j = 0; j < rowNum; j++) {
        evector[i][j] = evec->data[k + j];
      }
    }

    gsl_eigen_symmv_free(w);
    gsl_vector_free(eval);
    gsl_matrix_free(m);
    gsl_matrix_free(evec);
  }
}

