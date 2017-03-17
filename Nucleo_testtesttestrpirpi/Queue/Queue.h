#ifndef QUEUE_H
#define QUEUE_H

template <unsigned int N>
class CQueue
{
public:
    CQueue(): m_buffer(), m_start(0), m_end(N-1), m_size(0) {}
    virtual ~CQueue() {}
    inline bool isFull();
    inline bool isEmpty();
    inline unsigned char peek();
    inline unsigned char pop();
    inline unsigned int getSize();
    inline void push(unsigned char f_char);
    inline void push(char *f_char, unsigned int f_len);
    inline void empty();
private:
    volatile unsigned char m_buffer[N];
    volatile unsigned int  m_start;  
    volatile unsigned int  m_end; 
    volatile unsigned int  m_size;       
};

// Inline Implementation

template <unsigned int N>
bool CQueue<N>::isFull()
{
    return (m_start + 1)%N == m_end;
}
template <unsigned int N>
bool CQueue<N>::isEmpty()
{
    return (m_end + 1)%N == m_start;
}
template <unsigned int N>
unsigned char CQueue<N>::peek()
{
    return m_buffer[(m_end+1) % N];
}
template <unsigned int N>
unsigned char CQueue<N>::pop()
{
    if(!isEmpty())
    {
        m_end = (m_end+1) % N;
        unsigned char l_char = m_buffer[m_end];
        m_size--;
        return l_char;
    }
    else
    {
        return 0;
    }
}
template <unsigned int N>
void CQueue<N>::push(unsigned char f_char)
{
    if (!isFull())
    {
        m_buffer[m_start] = f_char;
        m_start = (m_start+1) % N; 
        m_size++;           
    }
}
template <unsigned int N>
inline void CQueue<N>::push(char *f_char, unsigned int f_len)
{
    for ( unsigned int l_idx = 0; l_idx < f_len; ++l_idx)
    {
        push(f_char[l_idx]);
    }
}
template <unsigned int N>
unsigned int CQueue<N>::getSize()
{
    return m_size;
}
template <unsigned int N>
inline void CQueue<N>::empty()
{
    m_start = 0;
    m_end = N-1;
    m_size = 0;
}

#endif
