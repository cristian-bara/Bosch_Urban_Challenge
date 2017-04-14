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
            public:

                using CStateType = linalg::CColVector<T,NA>;
                using CStateTransitionType = linalg::CMatrix<T,NA,NA>;
                using CInputType = linalg::CColVector<T,NB>;
                using CMeasurementType = linalg::CColVector<T,NC>;
                using CInputMatrixType = linalg::CMatrix<T,NA,NB>;
                using CMeasurementMatrixType = linalg::CMatrix<T,NC,NA>;
                using CDirectTransferMatrixType = linalg::CMatrix<T,NC,NB>;

                CSSModel(
                    const CStateTransitionType& f_stateTransitionMatrix,
                    const CInputMatrixType& f_inputMatrix,
                    const CMeasurementMatrixType& f_measurementMatrix
                ) 
                : m_stateVector()
                , m_stateTransitionMatrix(f_stateTransitionMatrix)
                , m_inputMatrix(f_inputMatrix)
                , m_measurementMatrix(f_measurementMatrix)
                , m_directTransferMatrix()
                {
                    // do nothing
                }

                CSSModel(
                    const CStateTransitionType& f_stateTransitionMatrix,
                    const CInputMatrixType& f_inputMatrix,
                    const CMeasurementMatrixType& f_measurementMatrix,
                    const CDirectTransferMatrixType& f_directTransferMatrix
                ) 
                : m_stateVector()
                , m_stateTransitionMatrix(f_stateTransitionMatrix)
                , m_inputMatrix(f_inputMatrix)
                , m_measurementMatrix(f_measurementMatrix)
                , m_directTransferMatrix(f_directTransferMatrix)
                {
                    // do nothing
                }

                CSSModel(
                    const CStateTransitionType& f_stateTransitionMatrix,
                    const CInputMatrixType& f_inputMatrix,
                    const CMeasurementMatrixType& f_measurementMatrix,
                    const CDirectTransferMatrixType& f_directTransferMatrix,
                    const CStateType& f_state                    
                ) 
                : m_stateVector(f_state)
                , m_stateTransitionMatrix(f_stateTransitionMatrix)
                , m_inputMatrix(f_inputMatrix)
                , m_measurementMatrix(f_measurementMatrix)
                , m_directTransferMatrix(f_directTransferMatrix)
                {
                    // do nothing
                }

                const CStateType& state() const {return m_stateVector;} 
                CStateType& state() {return m_stateVector;} 

                CMeasurementType operator()(const CInputType& f_inputVector)
                {
                    updateState(f_inputVector);

                    return getOutput(f_inputVector);
                }

                void updateState(const CInputType& f_inputVector)
                {
                    m_stateVector = m_stateTransitionMatrix * m_stateVector + m_inputMatrix * f_inputVector;
                }

                CMeasurementType getOutput(const CInputType& f_inputVector)
                {
                    return m_measurementMatrix * m_stateVector + m_directTransferMatrix * f_inputVector;
                }

            private:
                CSSModel() {}
                CStateType m_stateVector;
                CStateTransitionType m_stateTransitionMatrix;
                CInputMatrixType m_inputMatrix;
                CMeasurementMatrixType m_measurementMatrix;
                CDirectTransferMatrixType m_directTransferMatrix;
            };

            template <class T, uint32_t NA, uint32_t NB, uint32_t NC>
            class CKalmanFilter
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

            };
        };
    };
};

#include <filter.inl>
#endif // FILTER_H
