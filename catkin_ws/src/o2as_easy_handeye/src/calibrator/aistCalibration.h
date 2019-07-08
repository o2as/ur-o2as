/*!
  \file aistCalibration.h
  \brief Tools for camera calibration.

  \author Eric Marchand (INRIA) using code from Francois Chaumette (INRIA)

  \sa the example in calibrate.cpp
*/
#ifndef aistCalibration_h
#define aistCalibration_h

#include <geometry_msgs/Transform.h>
#include <vector>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpQuaternionVector.h>

namespace aistCalibration
{
class Transform
{
  public:
    Transform()
	:_t(0, 0, 0), _q(0, 0, 0, 1)
    {
    }

    Transform(const vpTranslationVector& t, const vpQuaternionVector& q)
	:_t(t), _q(q)
    {
	if (_q[3] < 0)
	    _q = -_q;
	_q.normalize();
    }

    Transform(const geometry_msgs::Transform& trans)
	:_t(trans.translation.x, trans.translation.y, trans.translation.z),
	 _q(trans.rotation.x, trans.rotation.y,
	    trans.rotation.z, trans.rotation.w)
    {
	if (_q[3] < 0)
	    _q = -_q;
	_q.normalize();
    }

    operator geometry_msgs::Transform() const
    {
	geometry_msgs::Transform	ret;
	ret.translation.x = _t[0];
	ret.translation.y = _t[1];
	ret.translation.z = _t[2];
	ret.rotation.x    = _q[0];
	ret.rotation.y    = _q[1];
	ret.rotation.z    = _q[2];
	ret.rotation.w    = _q[3];

	return ret;
    }

    const auto&	t()		const	{ return _t;    }
    auto	t(size_t i)	const	{ return _t[i]; }
    const auto&	q()		const	{ return _q;    }
    auto	q(size_t i)	const	{ return _q[i]; }
    auto	R()		const	{ return vpRotationMatrix(_q);}
    
    Transform	inverse() const
		{
		    Transform	ret;
		    ret._q = _q.inverse();
		    ret._t = -(vpRotationMatrix(ret._q)*_t);

		    return ret;
		}

    Transform	operator *(const Transform& trans) const
		{
		    return {_t + vpRotationMatrix(_q)*trans._t, _q * trans._q};
		}


    auto	translational_difference(const Transform& trans) const
		{
		    return (t() - trans.t()).euclideanNorm();
		}

    auto	angular_difference(const Transform& trans) const
		{
		    return (vpColVector(vpThetaUVector(q())) -
			    vpColVector(vpThetaUVector(trans.q())))
			.euclideanNorm();
		}

    void	print() const
		{
		    constexpr double	degree = 180.0/M_PI;
		    
		    std::cout << "xyz(m)    = "
			      << _t[0] << ' ' << _t[1] << ' ' << _t[2]
			      << std::endl;
		    std::cout << "rot(deg.) = ";
		    vpRotationMatrix	rpy(_q);
		    rpy *= degree;
		    rpy.printVector();
		}
		
    friend std::istream&
    operator >>(std::istream& in, Transform& trans)	;
    
  private:
    vpTranslationVector	_t;
    vpQuaternionVector	_q;
};

inline std::ostream&
operator <<(std::ostream& out, const Transform& trans)
{
    return out << trans.t(0) << ' ' << trans.t(1) << ' ' << trans.t(2) << "; "
	       << trans.q(0) << ' ' << trans.q(1) << ' '
	       << trans.q(2) << ' ' << trans.q(3);
}
    
inline std::istream&
operator >>(std::istream& in, Transform& trans)
{
    char	c;
    in >> trans._t[0] >> trans._t[1] >> trans._t[2] >> c
       >> trans._q[0] >> trans._q[1] >> trans._q[2] >> trans._q[3];
    if (trans._q[3] < 0)
	    trans._q = -trans._q;
    trans._q.normalize();

    return in;
}
    
Transform	calibrationAIST(const std::vector<Transform>& cMo,
				const std::vector<Transform>& wMe)	;
void		calibrationTsai(const std::vector<vpHomogeneousMatrix>& cMo,
				const std::vector<vpHomogeneousMatrix>& wMe,
				vpHomogeneousMatrix& eMc)		;
Transform	objectToWorld(const std::vector<Transform>& cMo,
			      const std::vector<Transform>& wMe,
			      const Transform& eMc)			;
void		evaluateAccuracy(std::ostream& out,
				 const std::vector<Transform>& cMo,
				 const std::vector<Transform>& wMe,
				 const Transform& eMc,
				 const Transform& wMo)			;
};

#endif
