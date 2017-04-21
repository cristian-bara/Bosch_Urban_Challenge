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
                    const CStateTransitionType& f_stateTransition,
                    const CInputMatrixType& f_inputMatrix,
                    const CMeasurementMatrixType& f_measurementMatrix
                )
                : m_stateTransition(f_stateTransition)
                , m_inputMatrix(f_inputMatrix)
                , m_measurementMatrix(f_measurementMatrix)
                , m_directTransferMatrix()
                {
                    // do nothing
                }

                CSSModel(
                    const CStateTransitionType& f_stateTransition,
                    const CInputMatrixType& f_inputMatrix,
                    const CMeasurementMatrixType& f_measurementMatrix,
                    const CDirectTransferMatrixType& f_directTransferMatrix
                )
                : m_stateTransition(f_stateTransition)
                , m_inputMatrix(f_inputMatrix)
                , m_measurementMatrix(f_measurementMatrix)
                , m_directTransferMatrix(f_directTransferMatrix)
                {
                    // do nothing
                }

                CSSModel(
                    const CStateTransitionType& f_stateTransition,
                    const CInputMatrixType& f_inputMatrix,
                    const CMeasurementMatrixType& f_measurementMatrix,
                    const CDirectTransferMatrixType& f_directTransferMatrix,
                    const CStateType& f_state
                )
                : m_state(f_state)
                , m_stateTransition(f_stateTransition)
                , m_inputMatrix(f_inputMatrix)
                , m_measurementMatrix(f_measurementMatrix)
                , m_directTransferMatrix(f_directTransferMatrix)
                {
                    // do nothing
                }

                const CStateType& state() const {return m_state;}
                CStateType& state() {return m_state;}

                CMeasurementType operator()(const CInputType& f_input)
                {
                    updateState(f_input);

                    return getOutput(f_input);
                }

                void updateState(const CInputType& f_input)
                {
                    m_state = m_stateTransition * m_state + m_inputMatrix * f_input;
                }

                CMeasurementType getOutput(const CInputType& f_input)
                {
                    return m_measurementMatrix * m_state + m_directTransferMatrix * f_input;
                }

            private:
                CSSModel() {}
                CStateType m_state;
                CStateTransitionType m_stateTransition;
                CInputMatrixType m_inputMatrix;
                CMeasurementMatrixType m_measurementMatrix;
                CDirectTransferMatrixType m_directTransferMatrix;
            };

            template <class T, uint32_t NA, uint32_t NB, uint32_t NC>
            class CKalmanFilter
            {
            public:
              using CStateType                 = linalg::CVector<T, NA>;
              using CInputVectorType           = linalg::CVector<T, NB>;
              using COutputVectorType          = linalg::CVector<T, NC>;
              using CInputType                 = linalg::CMatrix<T, NA, NB>;
              using CControInputType           = linalg::CMatrix<T, NA, NC>;
              using CModelCovarianceType       = linalg::CMatrix<T, NA, NA>;
              using CMeasurementCovarianceType = linalg::CMatrix<T, NC, NC>;
              using CStateTransType            = linalg::CMatrix<T, NA, NA>;
              using CMeasurementType           = linalg::CMatrix<T, NC, NA>;
              using CKalmanGainType            = linalg::CMatrix<T, NA, NC>;

              CKalmanFilter(
                  const CStateTransType&            f_stateTransitionModel,
                  const CControInputType&           f_controlInput,
                  const CMeasurementType&           f_observationModel,
                  const CModelCovarianceType&       f_covarianceProcessNoise,        // <-
                  const CMeasurementCovarianceType& f_observationNoiseCovariance     // <-

              )
              : m_stateTransitionModel(f_stateTransitionModel)
              , m_controlInput(f_controlInput)
              , m_observationModel(f_observationModel)
              , m_covarianceProcessNoise(f_covarianceProcessNoise)
              , m_observationNoiseCovariance(f_observationNoiseCovariance)
              {
              }

              const CStateType& state() const
              {
                  return m_posterioriState;
              }

              CStateType& state()
              {
                  return m_posterioriState;
              }

              CMeasurementType operator()(const CInputVectorType& f_input)
              {
                  predict(f_input);
                  return update();
              }

              CMeasurementType operator()()
              {
                  predict();
                  return update();
              }

              void predict()
              {
                  m_previousState      = m_prioriState;
                  m_previousCovariance = m_prioriCovariance;
                  m_prioriState        = m_stateTransitionModel * m_previousState;
                  m_prioriCovariance   = m_stateTransitionModel * m_previousCovariance * transpose(m_stateTransitionModel) + m_covarianceProcessNoise;
              }

              void predict(const CInputVectorType& f_input)
              {
                  m_previousState      = m_prioriState;
                  m_previousCovariance = m_prioriCovariance;
                  m_prioriState        = m_stateTransitionModel * m_previousState + m_controlInput * f_input;
                  m_prioriCovariance   = m_stateTransitionModel * m_previousCovariance * transpose(m_stateTransitionModel) + m_covarianceProcessNoise;
              }

              const CMeasurementType& update(void)
              {
                  m_measurementResidual  = m_measurement - m_observationModel * m_prioriState;
                  m_measurement          = m_observationModel * m_posterioriState;
                  m_residualCovariance   = m_observationModel * m_prioriCovariance * transpose(m_observationModel) + m_observationNoiseCovariance;
                  m_kalmanGain           = m_prioriCovariance * transpose(m_observationModel) * m_residualCovariance.inv();
                  m_posterioriState      = m_prioriState + m_kalmanGain * m_measurementResidual;
                  m_posterioriCovariance = ( CStateTransType::eye() - m_kalmanGain * m_observationModel ) * m_prioriCovariance;
                  return m_measurementResidual;
              }

            private:
              CKalmanFilter() {}

              CStateType                 m_previousState;              // previous state
              CStateType                 m_prioriState;                // priori state
              CStateType                 m_posterioriState;            // posteriori state
              CControInputType           m_controlInput;               // control input modelþ
              CModelCovarianceType       m_previousCovariance;         // previous covariance estimate         // <-
              CModelCovarianceType       m_prioriCovariance;           // priori covariance estimate           // <-
              CModelCovarianceType       m_posterioriCovariance;       // posteriori covariance estimate       // <-
              CStateTransType            m_stateTransitionModel;       // state transition model
              CModelCovarianceType       m_covarianceProcessNoise;     // covariance of process noise          // done
              CMeasurementType           m_measurementResidual;        // innovation or measurement residual
              CMeasurementType           m_measurement;                // observation (or measurement)
              CMeasurementType           m_observationModel;           // observation model
              CMeasurementCovarianceType m_residualCovariance;         // innovation or residual covariance    // <-
              CMeasurementCovarianceType m_observationNoiseCovariance; // covariance of observation noise      // <-
              CKalmanGainType            m_kalmanGain;                 // optimal kalman gain

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
