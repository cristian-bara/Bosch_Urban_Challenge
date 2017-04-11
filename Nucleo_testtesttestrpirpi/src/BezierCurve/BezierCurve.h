#ifndef BEZIERCURVE_H
#define BEZIERCURVE_H

#include "PolynomialFunction.h"
#include <complex>
#define BEZIER_ORDER 3


template<class T>
class BezierCurve
{
    public:
        BezierCurve();
        BezierCurve(std::complex<T> points[BEZIER_ORDER+1]);
        BezierCurve(std::complex<T> a,std::complex<T> b,std::complex<T> c,std::complex<T> d);
        virtual ~BezierCurve();

        void setBezierCurve(std::complex<T> a,std::complex<T> b,std::complex<T> c,std::complex<T> d);
        
        std::complex<T> getValue(float input_value);
        std::complex<T> get_FO_DerivateValue(float input_value);
        std::complex<T> get_SO_DerivateValue(float input_value);


        PolynomialFunction<std::complex<T>,BEZIER_ORDER > getBezierCurve();
        PolynomialFunction<std::complex<T>,BEZIER_ORDER-1> getFODerivate();
        PolynomialFunction<std::complex<T>,BEZIER_ORDER-2> getSODerivate();

    protected:

    private:
//        PRIVATE FUNCTIONS
//      Convert the complex points to polynomial function
        PolynomialFunction<std::complex<float>,BEZIER_ORDER> CP2PF();

//        PRIVATE PARAMETERS
        std::complex<T> points[BEZIER_ORDER+1];
        PolynomialFunction<std::complex<T>,BEZIER_ORDER> bezierCurve;
        PolynomialFunction<std::complex<T>,BEZIER_ORDER-1> FOder_bezierCurve;
        PolynomialFunction<std::complex<T>,BEZIER_ORDER-2> SOder_bezierCurve;
};

#include "BezierCurve.hpp"

#endif // BEZIERCURVE_H
