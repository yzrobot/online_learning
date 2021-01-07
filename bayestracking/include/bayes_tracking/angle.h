/*
 * (c) Michael Stevens 2002, 2003
 * $Header: /home/nbellotto/cvsroot/projects/libtrack/src/angle.h,v 1.1.1.1 2007/09/12 10:27:00 belush Exp $
 * $NoKeywords: $
 */

/*
 * Normalised Angular Arithmetic
 *	Manipulates angles using a normalised -PI <= angle < PI representation
 *	The algorithm has proven to be the most numerically acurate.
 *  For implementations where the cost of fmod is low it is also the most
 *	efficient in all cases.
 *	The class is templatised for a model of real for which std::fmod and arithmatic
 *  and comparison operation are defined.
 */

namespace angleArith
{

template <class real>
class angle
{
private:
	real a;	// normalised angle

	// mymod - provide the most efficient system mod function
	inline static real myfmod(real x, real y)
	{
		return std::fmod(x,y);
	}

public:
	// The mathematical constants
	static const real Pi, Pi2, Deg2Rad;

	angle (const real& unNorm)
	/* Conversion constructor from an unnormalised angle */
	{
		// normalised in range
		a = myfmod(unNorm, Pi2);
		if (a >= Pi) {
			a -= Pi2;
		}
		else if (a < -Pi) {
			a += Pi2;
		}
	}

	angle (const angle& right)
	/* Copy construtor */
	{
		a = right.a;
	}
	
	const angle& operator= (const angle& right)
	/* Assigment */
	{
		a = right.a;
		return *this;
	}
	
	operator real () const
	/* Automatic conversion to real normalised angle */
	{
		return a;
	}

	real from (real a2) const
	/* Return the angle for which it's difference from a2 is normalised */
	{
		// normalised difference
		angle<real> diff(this->a - a2);

		// rebase to a2
		return a2 + real(diff);
	}

};

/*
 * Angular constants. Only to sufficient accuracy for an IEEE double
 */
template <class real>
const real angle<real>::Pi = real(3.1415926535898);
template <class real>
const real angle<real>::Pi2 = real(3.1415926535898) * 2;
template <class real>
const real angle<real>::Deg2Rad = real(3.1415926535898) / 180;


/*
 * Template function constructor for simple syntax
 */

template <class real>
angle<real> normAngle(const real& r)
{
	return angle<real>(r);
}

template <class real>
const real Pi ()
{
	return angle<real>::Pi;
}

template <class real>
const real Pi2 ()
{
	return angle<real>::Pi2;
}

template <class real>
const real Deg2Rad ()
{
	return angle<real>::Deg2Rad;
}

}//namespace angleArith
