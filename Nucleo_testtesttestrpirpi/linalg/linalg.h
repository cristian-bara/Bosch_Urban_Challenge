#ifndef LINALG_H 
#define LINALG_H 

#include <mbed.h>
#include <array>

namespace linalg 
{
    template <class T, uint32_t M, uint32_t N>
    class CMatrix
    {
    public:
        using CThisType = CMatrix<T,M,N>;
        using CContainerType = std::array<std::array<T,N>,M>;
        using CDataType =T;

        template <uint32_t P>
        using CLeftMultipliableType = CMatrix<T,P,M>;

        template <uint32_t P>
        using CRightMultipliableType = CMatrix<T,N,P>;

        template <uint32_t P>
        using CLeftMultiplicationResultType = CMatrix<T,P,N>;

        template <uint32_t P>
        using CRightMultiplicationResultType = CMatrix<T,M,P>;

        CMatrix() : m_data() {}
        CMatrix(const CThisType& f_matrix) : m_data(f_matrix.m_data) {}
        CMatrix(const CThisType&& f_matrix) : m_data(f_matrix.m_data) {}
        CMatrix(const CContainerType& f_data) : m_data(f_data) {}
        CMatrix(const CContainerType&& f_data) : m_data(f_data) {}

        CThisType& operator=(const CThisType& f_matrix)
        {
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    this->m_data[l_row][l_col] = f_matrix.m_data[l_row][l_col];
                }
            }
            return *this;
        }
        CThisType& operator=(const CThisType&& f_matrix)
        {
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    this->m_data[l_row][l_col] = f_matrix.m_data[l_row][l_col];
                }
            }
            return *this;
        }

        std::array<T,N>& operator[](uint32_t f_row)
        {
            return m_data[f_row];
        }

        CDataType& operator()(uint32_t f_row, uint32_t f_col)
        {
            return m_data[f_row][f_col];
        }

        const std::array<T,N>& operator[](uint32_t f_row) const
        {
            return m_data[f_row];
        }

        const CDataType& operator()(uint32_t f_row, uint32_t f_col) const
        {
            return m_data[f_row][f_col];
        }

        CThisType& operator+() {return *this;}
        CThisType operator-() 
        {
            CThisType l_matrix;
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    l_matrix.m_data[l_row][l_col] = -this->m_data[l_row][l_col];
                }
            }
            return l_matrix;
        }
        CThisType operator+(const CThisType& f_matrix) 
        {
            CThisType l_matrix;
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    l_matrix.m_data[l_row][l_col] = this->m_data[l_row][l_col] + f_matrix.m_data[l_row][l_col];
                }
            }
            return l_matrix;
        }
        CThisType operator-(const CThisType& f_matrix) 
        {
            CThisType l_matrix;
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    l_matrix.m_data[l_row][l_col] = this->m_data[l_row][l_col] - f_matrix.m_data[l_row][l_col];
                }
            }
            return l_matrix;
        }
        CThisType& operator+=(const CThisType& f_matrix) 
        {
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    this->m_data[l_row][l_col] += f_matrix.m_data[l_row][l_col];
                }
            }
            return *this;
        }
        CThisType& operator-=(const CThisType& f_matrix) 
        {
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    this->m_data[l_row][l_col] -= f_matrix.m_data[l_row][l_col];
                }
            }
            return *this;
        }
        
        CThisType operator+(const CDataType& f_val) 
        {
            CThisType l_matrix;
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    l_matrix.m_data[l_row][l_col] = this->m_data[l_row][l_col] + f_val;
                }
            }
            return l_matrix;
        }
        CThisType operator-(const CDataType& f_val) 
        {
            CThisType l_matrix;
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    l_matrix.m_data[l_row][l_col] = this->m_data[l_row][l_col] - f_val;
                }
            }
            return l_matrix;
        }
        CThisType& operator+=(const CDataType& f_val) 
        {
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    this->m_data[l_row][l_col] += f_val;
                }
            }
            return *this;
        }
        CThisType& operator-=(const CDataType& f_val) 
        {
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    this->m_data[l_row][l_col] -= f_val;
                }
            }
            return *this;
        }
        CThisType& operator*=(const CDataType& f_val) 
        {
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    this->m_data[l_row][l_col] *= f_val;
                }
            }
            return *this;
        }
        CThisType& operator*=(const CThisType& f_val) 
        {
            CThisType& l_thisRef(*this);
            l_thisRef = l_thisRef * f_val;
            return l_thisRef;
        }
        CThisType& operator/=(const CDataType& f_val) 
        {
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    this->m_data[l_row][l_col] /= f_val;
                }
            }
            return *this;
        }
        CThisType operator*(const CDataType& f_val) 
        {
            CThisType l_matrix;
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    l_matrix.m_data[l_row][l_col] = this->m_data[l_row][l_col] * f_val;
                }
            }
            return l_matrix;
        }
        template <uint32_t P>
        CRightMultiplicationResultType<P> operator*(const CRightMultiplicationResultType<P>& f_matrix)
        {
            CLeftMultiplicationResultType<P> l_matrix;
            for (uint32_t l_row = 0; l_row < M; ++l_row)
            {
                for (uint32_t l_col = 0; l_col < N; ++l_col)
                {
                    l_matrix.m_data[l_row][l_col] = 0;
                    for (uint32_t l_idx = 0; l_idx < P; ++l_idx)
                    {
                        l_matrix.m_data[l_row][l_col] += this->m_data[l_row][l_idx] * f_matrix.m_data[l_idx][l_col];
                    }
                }
            }
            return l_matrix;
        }
        CThisType invert()
        {
            return CLUDecomposition(*this).invert();
        }

        template <uint32_t P>
        CRightMultiplicationResultType<P> solve(const CRightMultipliableType<P>& f_B)
        {
            return CLUDecomposition(*this).solve();
        }

    private:
        std::array<std::array<T,N>,M> m_data;
    };

    template<class T, uint32_t N>
    class CLUDecomposition
    {
    public:
        using CThisType = CLUDecomposition<T,N>;
        using COriginalType = CMatrix<T,N,N>;

        template <uint32_t P>
        using CLeftMultipliableType = CMatrix<T,P,N>;

        template <uint32_t P>
        using CRightMultipliableType = CMatrix<T,N,P>;

        template <uint32_t P>
        using CLeftMultiplicationResultType = CMatrix<T,P,N>;

        template <uint32_t P>
        using CRightMultiplicationResultType = CMatrix<T,N,P>;

        CLUDecomposition(const CThisType& f_decomposition) : m_LU(f_decomposition.m_LU) {}
        CLUDecomposition(const CThisType&& f_decomposition) : m_LU(f_decomposition.m_LU) {}
        CLUDecomposition(const COriginalType& f_matrix) : m_LU() {decompose(f_matrix);}
        CLUDecomposition(const COriginalType&& f_matrix) : m_LU() {decompose(f_matrix);}

        operator COriginalType()
        {
            COriginalType l_result;

            return l_result;
        }

        COriginalType invert()
        {
            return triUInv() * triLInv();
        }
        COriginalType triUInv()
        {
            COriginalType l_invU;
            return l_invU;
        }
        COriginalType triLInv()
        {
            COriginalType l_invL;
            return l_invL;
        }

        template <uint32_t P>
        CRightMultiplicationResultType<P> solve(const CRightMultipliableType<P>& f_B)
        {
            return sTriL(sTriU(f_B));
        }

        template <uint32_t P>
        CRightMultiplicationResultType<P> sTriU(const CRightMultipliableType<P>& f_B)
        {
            CRightMultiplicationResultType<P> l_U;

            return l_U;
        }

        template <uint32_t P>
        CRightMultiplicationResultType<P> sTriL(const CRightMultipliableType<P>& f_B)
        {
            CRightMultiplicationResultType<P> l_L;

            for(uint32_t l_idx = 0; l_idx < P; ++l_idx)
            {
                for(uint32_t l_jdx = 0; l_jdx < N; ++l_jdx)
                {
                    l_L[l_idx][l_jdx] = f_B[l_idx][l_jdx];
                    for(uint32_t l_kdx = 0; l_kdx < (l_jdx-1); ++l_kdx)
                    {
                        // l_L[l_idx][l_jdx] -= L(l_jdx,l_kdx)
                    }
                }
            }

            return l_L;
        }


    private:
        void decompose(CMatrix<T,N,N> f_matrix)
        {

        }
        CMatrix<T,N,N> m_LU;
        std::array<uint32_t,N> m_P;
    };

    template <class T, uint32_t N>
    using CColVector = CMatrix<T,N,1>;

    template <class T, uint32_t N>
    using CVector = CColVector<T,N>;

    template <class T, uint32_t N>
    using CRowVector = CMatrix<T,1,N>;
};

#endif // LINALG_H 
