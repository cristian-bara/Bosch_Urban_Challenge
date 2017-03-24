#ifndef QUEUE_H
#define QUEUE_H

template <class T, unsigned int N>
class CQueue
{
public:
    CQueue(): m_buffer(), m_start(0), m_end(N-1), m_size(0) {}
    virtual ~CQueue() {}
    inline bool isFull();
    inline bool isEmpty();
    inline T peek();
    inline T pop();
    inline unsigned int getSize();
    inline void push(T& f_char);
    inline void push(T *f_char, unsigned int f_len);
    inline void empty();
private:
    volatile T m_buffer[N];
    volatile unsigned int  m_start;  
    volatile unsigned int  m_end; 
    volatile unsigned int  m_size;       
};

// Inline Implementation

template <class T, unsigned int N>
bool CQueue<T,N>::isFull()
{
    return (m_start + 1)%N == m_end;
}
template <class T, unsigned int N>
bool CQueue<T,N>::isEmpty()
{
    return (m_end + 1)%N == m_start;
}
template <class T, unsigned int N>
T CQueue<T,N>::peek()
{
    return m_buffer[(m_end+1) % N];
}
template <class T, unsigned int N>
T CQueue<T,N>::pop()
{
    if(!isEmpty())
    {
        m_end = (m_end+1) % N;
        T l_char = m_buffer[m_end];
        m_size--;
        return l_char;
    }
    else
    {
        return 0;
    }
}
template <class T, unsigned int N>
void CQueue<T,N>::push(T& f_char)
{
    if (!isFull())
    {
        m_buffer[m_start] = f_char;
        m_start = (m_start+1) % N; 
        m_size++;           
    }
}
template <class T, unsigned int N>
inline void CQueue<T,N>::push(T *f_char, unsigned int f_len)
{
    for ( unsigned int l_idx = 0; l_idx < f_len; ++l_idx)
    {
        push(f_char[l_idx]);
    }
}
template <class T, unsigned int N>
unsigned int CQueue<T,N>::getSize()
{
    return m_size;
}
template <class T, unsigned int N>
inline void CQueue<T,N>::empty()
{
    m_start = 0;
    m_end = N-1;
    m_size = 0;
}

#endif
