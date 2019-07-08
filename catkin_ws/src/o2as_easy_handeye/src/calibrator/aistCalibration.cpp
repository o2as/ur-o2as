/*!
  \file aistCalibration.cpp
  \brief Tools for camera calibration.
*/

#include <visp3/core/vpImage.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpColVector.h>
#include <visp3/vision/vpCalibrationException.h>
#include "aistCalibration.h"

#define DEBUG

namespace aistCalibration
{
Transform
calibrationAIST(const std::vector<Transform>& cMo,
		const std::vector<Transform>& wMe)
{
    const auto	nposes = cMo.size();

  // Compute rotation.
    vpMatrix	C(4, 4);	// 4x4 matrix initialized with zeros.
    for (size_t i = 0; i < nposes; ++i)
	for (size_t j = i + 1; j < nposes; ++j)
	{
	    const auto			A = wMe[j].inverse() * wMe[i];
	    const auto			B = cMo[j] * cMo[i].inverse();
	    const vpTranslationVector	a(A.q(0), A.q(1), A.q(2));
	    const vpTranslationVector	b(B.q(0), B.q(1), B.q(2));
	    const auto			ab  = vpColVector::dotProd(a, b);
	    const auto			axb = vpColVector::cross(a, b);
	    const auto			a3  = A.q(3);
	    const auto			b3  = B.q(3);

	    for (size_t r = 0; r < 3; ++r)
	    {
		C[r][r] += (1 + ab - a3*b3);
		C[3][r] += axb[r];
		C[r][3] += axb[r];

		for (size_t c = 0; c < 3; ++c)
		    C[r][c] -= (a[r]*b[c] + b[r]*a[c]);
	    }
	    C[3][3] += (1 - ab - a3*b3);
	}

    vpColVector	lambda;
    vpMatrix	V;
    C.eigenValues(lambda, V);
    vpQuaternionVector	qx(V.getCol(0));	// computed rotation
#ifdef DEBUG
    std::cerr << "--- Rotation: lambda ---\n" << lambda << std::endl;
#endif

  // Compute translation.
    vpMatrix	D(3, 3);	// 3x3 matrix initialized with zeros.
    vpColVector	d(3, 0);	// 3x1 vector initialized with zeros.
    for (size_t i = 0; i < nposes; ++i)
	for (size_t j = i + 1; j < nposes; ++j)
	{
	    const auto	A = wMe[j].inverse() * wMe[i];
	    const auto	B = cMo[j] * cMo[i].inverse();

	    vpMatrix	M = A.R();
	    M[0][0] -= 1;
	    M[1][1] -= 1;
	    M[2][2] -= 1;
	    D += M.AtA();

	    const auto	m = vpRotationMatrix(qx)*B.t() - A.t();
	    d += (M.t() * m);
	}
    
    const vpTranslationVector	tx(D.inverseByCholesky() * d);

    return Transform(tx, qx);
}

Transform
objectToWorld(const std::vector<Transform>& cMo,
	      const std::vector<Transform>& wMe, const Transform& eMc)
{
    const auto	nposes = cMo.size();
    vpMatrix	C(4, 4);	// 4x4 matrix initialized with zeros.
    vpColVector	ty(3, 0.0);
    for (size_t i = 0; i < nposes; ++i)
    {
      // Transformation from object to world computed from estimated eMc.
	const auto	M = wMe[i] * eMc * cMo[i];

	for (size_t r = 0; r < 4; ++r)
	{
	    C[r][r] += 1;

	    for (size_t c = 0; c < 4; ++c)
		C[r][c] -= M.q(r)*M.q(c);
	}

	ty += M.t();
    }
    vpColVector	lambda;
    vpMatrix	V;
    C.eigenValues(lambda, V);
#ifdef DEBUG
    std::cerr << "--- Translation: lambda ---\n" << lambda << std::endl;
#endif
    vpQuaternionVector	qy(V.getCol(0));
    ty *= (1.0/double(nposes));

    return Transform(vpTranslationVector(ty), qy);
}
    
void
evaluateAccuracy(std::ostream& out,
		 const std::vector<Transform>& cMo,
		 const std::vector<Transform>& wMe,
		 const Transform& eMc, const Transform& wMo)
{
    constexpr double	degree = 180.0/M_PI;

    const auto	nposes = cMo.size();
    double	tdiff_mean = 0;	// mean translational difference
    double	tdiff_max  = 0;	// max. translational difference
    double	adiff_mean = 0;	// mean angular difference
    double	adiff_max  = 0;	// max. angular difference
    
    for (size_t i = 0; i < nposes; ++i)
    {
	const auto	AX    = wMe[i] * eMc;
	const auto	YBinv = wMo * cMo[i].inverse();
	const auto	tdiff = AX.translational_difference(YBinv);
	const auto	adiff = AX.angular_difference(YBinv);

	if (tdiff > tdiff_max)
	    tdiff_max = tdiff;
	if (adiff > adiff_max)
	    adiff_max = adiff;

	tdiff_mean += tdiff*tdiff;
	adiff_mean += adiff*adiff;
    }
    tdiff_mean = std::sqrt(tdiff_mean/nposes);
    adiff_mean = std::sqrt(adiff_mean/nposes);

    out << "trans. err(m): (mean, max) = ("
	<< tdiff_mean << ", " << tdiff_max
	<< ")\nangle err(deg.): (mean, max) = ("
	<< adiff_mean * degree << ", " << adiff_max * degree << ')'
	<< std::endl;

    out << "estimated eMc:\n" << eMc << std::endl
	<< "estimated wMo:\n" << wMo << std::endl << std::endl;

    std::cout << "=== estimated eMc: ====\n";
    eMc.print();
    std::cout << std::endl;
    std::cout << "=== estimated wMo: ===\n";
    wMo.print();
    std::cout << std::endl;
}
    
    
void
calibrationTsai(const std::vector<vpHomogeneousMatrix>& cMo,
		const std::vector<vpHomogeneousMatrix>& wMe,
		vpHomogeneousMatrix& eMc)
{
    unsigned int	nbPose = (unsigned int)cMo.size();

    if (cMo.size() != wMe.size())
	throw vpCalibrationException(vpCalibrationException::dimensionError,
				     "cMo and wMe have different sizes");
    
    vpMatrix		A;
    vpColVector		B;
    unsigned int	k = 0;
  // for all couples ij
    for (unsigned int i = 0; i < nbPose; i++)
    {
	vpRotationMatrix	rRei, ciRo;
	wMe[i].extract(rRei);
	cMo[i].extract(ciRo);
      // std::cout << "wMei: " << std::endl << wMe[i] << std::endl;

	for (unsigned int j = i + 1; j < nbPose; j++)
	{
	    vpRotationMatrix	rRej, cjRo;
	    wMe[j].extract(rRej);
	    cMo[j].extract(cjRo);
	  // std::cout << "wMej: " << std::endl << wMe[j] << std::endl;

	    vpRotationMatrix	rReij = rRej.t() * rRei;
	    vpRotationMatrix	cijRo = cjRo * ciRo.t();
	    vpThetaUVector	rPeij(rReij);
	    double		theta = sqrt(rPeij[0] * rPeij[0] +
					     rPeij[1] * rPeij[1] +
					     rPeij[2] * rPeij[2]);

	    for (unsigned int m = 0; m < 3; m++)
		rPeij[m] = rPeij[m] * vpMath::sinc(theta / 2);

	    vpThetaUVector	cijPo(cijRo);
	    theta = sqrt(cijPo[0] * cijPo[0] +
			 cijPo[1] * cijPo[1] + cijPo[2] * cijPo[2]);
	    for (unsigned int m = 0; m < 3; m++)
		cijPo[m] = cijPo[m] * vpMath::sinc(theta / 2);

	    vpMatrix	As = vpColVector::skew(vpColVector(rPeij) +
					       vpColVector(cijPo));
	    vpColVector	b = (vpColVector)cijPo - (vpColVector)rPeij; // A.40;

	    if (k == 0)
	    {
		A = As;
		B = b;
	    }
	    else
	    {
		A = vpMatrix::stack(A, As);
		B = vpColVector::stack(B, b);
	    }
	    k++;
	}
    }

  // the linear system is defined
  // x = AtA^-1AtB is solved
    vpMatrix	AtA = A.AtA();
    vpMatrix	Ap;
    AtA.pseudoInverse(Ap, 1e-6); // rank 3
    vpColVector	x = Ap * A.t() * B;

  //     {
  //       // Residual
  //       vpColVector residual;
  //       residual = A*x-B;
  //       std::cout << "Residual: " << std::endl << residual << std::endl;

  //       double res = 0;
  //       for (int i=0; i < residual.getRows(); i++)
  // 	res += residual[i]*residual[i];
  //       res = sqrt(res/residual.getRows());
  //       printf("Mean residual = %lf\n",res);
  //     }

  // extraction of theta and U
    double	theta;
    double	d = x.sumSquare();
    for (unsigned int i = 0; i < 3; i++)
	x[i] = 2 * x[i] / sqrt(1 + d);
    theta = sqrt(x.sumSquare()) / 2;
    theta = 2 * asin(theta);
  // if (theta !=0)
    if (std::fabs(theta) > std::numeric_limits<double>::epsilon())
    {
	for (unsigned int i = 0; i < 3; i++)
	    x[i] *= theta / (2 * sin(theta / 2));
    }
    else
	x = 0;

  // Building of the rotation matrix eRc
    vpThetaUVector	xP(x[0], x[1], x[2]);
    vpRotationMatrix	eRc(xP);

  //vpMatrix	A;
  //	vpColVector	B;
  // Building of the system for the translation estimation
  // for all couples ij
    vpRotationMatrix	I3;
    I3.eye();
    k = 0;
    for (unsigned int i = 0; i < nbPose; i++)
    {
	vpRotationMatrix	rRei, ciRo;
	vpTranslationVector	rTei, ciTo;
	wMe[i].extract(rRei);
	cMo[i].extract(ciRo);
	wMe[i].extract(rTei);
	cMo[i].extract(ciTo);

	for (unsigned int j = i + 1; j < nbPose; j++)
	{
	    vpRotationMatrix	rRej, cjRo;
	    wMe[j].extract(rRej);
	    cMo[j].extract(cjRo);

	    vpTranslationVector	rTej, cjTo;
	    wMe[j].extract(rTej);
	    cMo[j].extract(cjTo);

	    vpRotationMatrix	rReij = rRej.t() * rRei;
	    vpTranslationVector	rTeij = rTej + (-rTei);
	    rTeij = rRej.t() * rTeij;

	    vpMatrix		a = vpMatrix(rReij) - vpMatrix(I3);
	    vpTranslationVector	b = eRc * cjTo - rReij * eRc * ciTo
		+ rTeij;

	    if (k == 0)
	    {
		A = a;
		B = b;
	    }
	    else
	    {
		A = vpMatrix::stack(A, a);
		B = vpColVector::stack(B, b);
	    }
	    k++;
	}
    }

  // the linear system is solved
  // x = AtA^-1AtB is solved
    AtA = A.AtA();
    AtA.pseudoInverse(Ap, 1e-6);
    vpColVector	AeTc = Ap * A.t() * B;

  //     {
  //       // residual
  //       vpColVector residual;
  //       residual = A*AeTc-B;
  //       std::cout << "Residual: " << std::endl << residual << std::endl;
  //       double res = 0;
  //       for (int i=0; i < residual.getRows(); i++)
  // 	res += residual[i]*residual[i];
  //       res = sqrt(res/residual.getRows());
  //       printf("mean residual = %lf\n",res);
  //     }

    vpTranslationVector eTc(AeTc[0], AeTc[1], AeTc[2]);

    eMc.insert(eTc);
    eMc.insert(eRc);
}
    
}	// namespace aistCalibration

