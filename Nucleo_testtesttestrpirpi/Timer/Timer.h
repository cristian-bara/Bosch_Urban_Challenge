#ifndef __TIMER_H__
#define __TIMER_H__

template <unsigned int N, unsigned int D = 1>
class CTimer
{
public:
    CTimer(): m_ticker(), m_millisValue(0), m_started(0) {}
    virtual ~CTimer() { if (m_started) stop ();}
    inline void start () {
        m_started = 1;
        m_ticker.attach (mbed::callback(&CTimer::callback, this), 1.f*N/D);  
    }
     
    inline void stop () {
        m_started = 0;
        m_ticker.detach ();
    }
    inline uint32_t get()
    {
        return m_millisValue;
    }
private:
    static void callback(void *thisPointer)
    {
        CTimer<N,D>* self = static_cast<CTimer<N,D>*>(thisPointer);
        self->millisTicker(); 
    }
    void millisTicker ()
    {
        m_millisValue ++;
    }
    Ticker m_ticker;
    volatile uint32_t m_millisValue;
    bool m_started;
};

typedef CTimer<1,1000> CTimer_ms;
typedef CTimer<1,1000000> CTimer_us;
typedef CTimer<1,10000> CTimer_100us;
typedef CTimer<60> CTimer_min;

#endif