#ifndef POLYNOMIALFUNCTION_H
#define POLYNOMIALFUNCTION_H

#include <stdint.h>

template <class T,int32_t N>
class PolynomialFunction
{
    public:
        PolynomialFunction();
        PolynomialFunction(T coefficients[N+1]);
        virtual ~PolynomialFunction();

//        template<int32_t N2> void add(PolynomialFunction<T,N2> poli);
        template<int32_t N2> PolynomialFunction<T,(N2<N?N:N2)> add(PolynomialFunction<T,N2> b);
        template<int32_t N2> PolynomialFunction<T,(N2+N)> multip(PolynomialFunction<T,N2> b);

        T calculateValue(T input_value);
        PolynomialFunction<T,N-1> derivateFO();

        int32_t getGrade();
        T getCoeffienceValue(int32_t index);
        void setCoeffienceValue(int32_t index,T value );

    private:
    T coefficients[N+1];
};





#include "PolynomialFunction.hpp"

#endif // POLYNOMIALFUNCTION_H
