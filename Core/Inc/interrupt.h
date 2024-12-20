#pragma once

struct Interrupting
{
    virtual void interrupt() = 0;
};

class Interrupt
{
    Interrupting* pointer;

public:

    void subscribe(Interrupting* ps)
    {
      pointer = ps;
    }

    void clear_subscribe() { pointer = 0; }

    void interrupt()
    {
      pointer->interrupt();
    }
};
