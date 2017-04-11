#ifndef FILTER_H
#define FILTER_H

#include <linalg.h>

namespace filter
{
    namespace lti
    {
        namespace siso
        {
            template <class T, uint32_t NA, uint32_t NB>
            class CIIRFilter
            {
            public:
                CIIRFilter(const linalg::CRowVector<T,NA>& f_A,const linalg::CRowVector<T,NB>& f_B) : m_A(f_A), m_B(f_B), m_Y(), m_U() {}
                T operator()(T& f_u)
                {
                    for (uint32_t l_idx = 1; l_idx < NB; ++l_idx)
                    {
                        m_U[l_idx] = m_U[l_idx-1];
                    }
                    m_U[0] = f_u;

                    linalg::CMatrix<T,1,1> l_y = m_B*m_U - m_A*m_Y;
                    // T l_y = m_B*m_U - m_A*m_Y;

                    for (uint32_t l_idx = 1; l_idx < NA; ++l_idx)
                    {
                        m_Y[l_idx] = m_Y[l_idx-1];
                    }
                    m_Y[0] = l_y;

                    return m_Y[0];
                }
            private:
                CIIRFilter(){}
                linalg::CRowVector<T,NA> m_A;
                linalg::CRowVector<T,NB> m_B;
                linalg::CColVector<T,NA> m_Y;
                linalg::CColVector<T,NB> m_U;
            };
            template <class T, uint32_t NB>
            class CFIRFilter
            {
            public:                
                CFIRFilter(const linalg::CRowVector<T,NB>& f_B) : m_B(f_B), m_U() {}
                T operator()(T& f_u)
                {
                    for (uint32_t l_idx = 1; l_idx < NB; ++l_idx)
                    {
                        m_U[l_idx] = m_U[l_idx-1];
                    }
                    m_U[0] = f_u;

                    linalg::CMatrix<T,1,1> l_y = m_B*m_U;

                    return l_y[0][0];
                }
            private:
                CFIRFilter() {}
                linalg::CRowVector<T,NB> m_B;
                linalg::CColVector<T,NB> m_U;
            };
            template <class T, uint32_t NB>
            class CMeanFilter
            {
            public:
                CMeanFilter() : m_B(1./NB), m_U() {}

                T operator()(T& f_u)
                {
                    T l_y =0;

                    for (uint32_t l_idx = 1; l_idx < NB; ++l_idx)
                    {
                        m_U[l_idx] = m_U[l_idx-1];
                        l_y += m_U[l_idx];
                    }
                    m_U[0] = f_u;
                    l_y += m_U[0];

                    return m_B*l_y;
                }
            private:
                T m_B;
                linalg::CColVector<T,NB> m_U;
            };
            template <class T, uint32_t NB>
            class CMedianFilter
            {

            };
        };
        namespace mimo
        {
            template <class T, uint32_t NA, uint32_t NB, uint32_t NC>
            class CSSModel
            {

            };
        };
    };
    namespace nonlinear
    {
        namespace mimo
        {
            class CEKF
            {
                
            }
        };
    };
};

#include <filter.inl>
#endif // FILTER_H
