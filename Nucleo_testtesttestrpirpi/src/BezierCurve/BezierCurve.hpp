#ifndef BEZIERCURVE_HPP_INCLUDED
#define BEZIERCURVE_HPP_INCLUDED

template<class T>
BezierCurve<T>::BezierCurve(){
}

template<class T>
BezierCurve<T>::BezierCurve(std::complex<T> a,std::complex<T> b,std::complex<T> c,std::complex<T> d){
    this->points[0]=a;
    this->points[1]=b;
    this->points[2]=c;
    this->points[3]=d;

    this->bezierCurve=this->CP2PF();
    this->FOder_bezierCurve=this->bezierCurve.derivateFO();
    this->SOder_bezierCurve=this->FOder_bezierCurve.derivateFO();
}

template<class T>
BezierCurve<T>::BezierCurve(std::complex<T> points[BEZIER_ORDER+1])
{
    for(int32_t i=0;i<=BEZIER_ORDER;++i){
        this->points[i]=points[i];
    }
    this->bezierCurve=this->CP2PF();
    this->FOder_bezierCurve=this->bezierCurve.derivateFO();
    this->SOder_bezierCurve=this->FOder_bezierCurve.derivateFO();
}

template<class T>
BezierCurve<T>::~BezierCurve()
{
    //dtor
}


template<class T>
PolynomialFunction<std::complex<float>,BEZIER_ORDER> BezierCurve<T>::CP2PF(){
    PolynomialFunction<std::complex<float>,BEZIER_ORDER> pf;

    const std::complex<T> temp_cst_3(3,0);
    const std::complex<T> temp_cst_2(2,0);

    std::complex<T> coef1=temp_cst_3*(this->points[1]-this->points[0]);
    std::complex<T> coef2=temp_cst_3*(this->points[0]-temp_cst_2*this->points[1]+this->points[2]);
    std::complex<T> coef3=temp_cst_3*(this->points[1]-this->points[2])+this->points[3]-this->points[0];
    pf.setCoeffienceValue(0,this->points[0]);
    pf.setCoeffienceValue(1,coef1);
    pf.setCoeffienceValue(2,coef2);
    pf.setCoeffienceValue(3,coef3);
    return pf;
}


template<class T>
std::complex<T> BezierCurve<T>::getValue(float input_value){
    T input_value_T=static_cast<T>(input_value);
    return this->bezierCurve.calculateValue(input_value);
}

template<class T>
std::complex<T> BezierCurve<T>::get_FO_DerivateValue(float input_value){
    T input_value_T=static_cast<T>(input_value);
    return this->FOder_bezierCurve.calculateValue(input_value_T);
}


template<class T>
std::complex<T> BezierCurve<T>::get_SO_DerivateValue(float input_value){
    T input_value_T=static_cast<T>(input_value);

    return this->SOder_bezierCurve.calculateValue(input_value_T);
}

template<class T>
PolynomialFunction<std::complex<T>,BEZIER_ORDER> BezierCurve<T>::getBezierCurve(){
    return this->bezierCurve;
}


template<class T>
PolynomialFunction<std::complex<T>,BEZIER_ORDER-1> BezierCurve<T>::getFODerivate(){
    return this->FOder_bezierCurve;
}

template<class T>
PolynomialFunction<std::complex<T>,BEZIER_ORDER-2> BezierCurve<T>::getSODerivate(){
    return this->SOder_bezierCurve;
}


template<class T>
void  BezierCurve<T>::setBezierCurve(std::complex<T> a,std::complex<T> b,std::complex<T> c,std::complex<T> d){
    this->points[0]=a;
    this->points[1]=b;
    this->points[2]=c;
    this->points[3]=d;

    this->bezierCurve=this->CP2PF();
    this->FOder_bezierCurve=this->bezierCurve.derivateFO();
    this->SOder_bezierCurve=this->FOder_bezierCurve.derivateFO();
}


#endif // BEZIERCURVE_HPP_INCLUDED
