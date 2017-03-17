#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

class CTask
{
public:
    CTask(uint32_t f_period) : m_period(f_period),m_ticks(0),m_triggered(false) {}
    virtual ~CTask() {}
    virtual void run()
    {
        if (m_triggered)
        {
            m_triggered = false;
            _run();
        }
    }
    void timerCallback()
    {
        m_ticks++;
        if (m_ticks >= m_period)
        {
            m_ticks = 0;
            Trigger();
        }
    }
    void Trigger()
    {
        m_triggered = true;
    }
protected:
    virtual void _run() = 0;
    const uint32_t m_period;
    uint32_t m_ticks;
    bool m_triggered;
};

class CTaskManager
{
public:
    CTaskManager(CTask** f_taskList, uint32_t f_taskCount, float f_baseFreq):
        m_taskList(f_taskList), m_taskCount(f_taskCount) 
    {
        m_ticker.attach(mbed::callback(&CTaskManager::callback, this), f_baseFreq);
    }
    virtual ~CTaskManager() 
    {
        m_ticker.detach();
    }
    void mainCallback()
    {
        for(uint32_t i = 0; i < m_taskCount; i++)
        {
            m_taskList[i]->run();
        }
    }
    void timerCallback()
    {
        for(uint32_t i = 0; i < m_taskCount; i++)
        {
            m_taskList[i]->timerCallback();
        }
    }
private:    
    static void callback(void *thisPointer)
    {
        CTaskManager* self = static_cast<CTaskManager*>(thisPointer);
        self->timerCallback(); 
    }
    CTask** m_taskList;
    uint32_t m_taskCount;
    Ticker m_ticker;
};

#endif