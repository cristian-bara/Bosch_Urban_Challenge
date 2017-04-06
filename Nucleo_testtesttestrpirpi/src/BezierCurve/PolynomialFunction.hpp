#ifndef POLYNOMIALFUNCTION_PP_INCLUDED
#define POLYNOMIALFUNCTION_HPP_INCLUDED

#include<math.h>

#include <iostream>

template<class T,int32_t N> PolynomialFunction<T,N>::PolynomialFunction()
{
    for(int i=0;i<=N;++i){
        this->coefficients[i]=static_cast<T>(0);
    }
}

template<class T,int32_t N> PolynomialFunction<T,N>::PolynomialFunction(T coefficients[N+1])
{
    for(int i=0;i<=N;++i){
        this->coefficients[i]=coefficients[i];
    }
}

template<class T,int32_t N> PolynomialFunction<T,N>::~PolynomialFunction()
{

}

template<class T,int32_t N> int32_t PolynomialFunction<T,N>::getGrade()
{
    return N;
}

template<class T,int32_t N> T PolynomialFunction<T,N>::getCoeffienceValue(int32_t index)
{
    if(index>N || index<0) return static_cast<T>(0);
    return this->coefficients[index];
}


template<class T,int32_t N> void PolynomialFunction<T,N>::setCoeffienceValue(int32_t index, T value ){
    if(index<=N){
        this->coefficients[index]=value;
    }
}



template<class T,int32_t N>
template<int32_t N2>
PolynomialFunction<T,(N2<N?N:N2)> PolynomialFunction<T,N>::add(PolynomialFunction<T,N2> b){
    PolynomialFunction<T,N2<N?N:N2> p;
    for(int i=0;i<=N2 || i<=N;++i){
        p.setCoeffienceValue(i,this->getCoeffienceValue(i)+b.getCoeffienceValue(i));
    }
    return p;
}

template<class T,int32_t N>
template<int32_t N2>
PolynomialFunction<T,(N2+N)> PolynomialFunction<T,N>::multip(PolynomialFunction<T,N2> b){
    PolynomialFunction<T,(N2+N)> result;
    for(int32_t i=0;i<=N2+N;++i){
        T sum=static_cast<T>(0);
        for(int32_t j=0;j<=i;++j){
            sum+=this->getCoeffienceValue(j)*b.getCoeffienceValue(i-j);
        }
        result.setCoeffienceValue(i,sum);
    }

    return result;
}

template<class T,int32_t N>
T PolynomialFunction<T,N>::calculateValue(T input_value){


    T output_value=static_cast<T>(0);
    for(int32_t i=0;i<=N;++i){
        output_value+=this->coefficients[i]*static_cast<T>(pow(input_value,i));
    }
//    std::cout<<"OOO"<<output_value<<std::endl;
    return output_value;
}

template<class T, int32_t N>
PolynomialFunction<T,N-1> PolynomialFunction<T,N>::derivateFO(){
    PolynomialFunction<T,N-1> derivate;
    for(int32_t i=0;i<N;++i){
        T coeff=static_cast<T>(i+1);
        derivate.setCoeffienceValue(i,coeff*this->getCoeffienceValue(i+1));
    }
    return derivate;
};


#endif // POLYNOMIALFUNCTION_HPP_INCLUDED
