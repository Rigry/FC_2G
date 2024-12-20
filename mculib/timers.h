/*
 * timers.h
 *
 *  Created on: 9 февр. 2023 г.
 *      Author: Alex
 */

//#ifndef TIMERS_H_
//#define TIMERS_H_

#pragma once

#include <functional>
#include "subscriber.h"
#include "systick.h"

enum Faster {
     x2 = 2, x4 = 4, x5 = 5, x8 = 8, x10 = 10,
};

struct TickUpdater : Publisher
{
    TickUpdater() { systemtick.initInterrupt<1000>(); }
    template<Faster multiplier> // multiplier how many subticks in tick
    void subscribe_subtick (Subscriber& v);
    void interrupt();


private:
    struct Subtick : Publisher {} subtick {};
    size_t subtick_cnt {0};
    size_t subtick_qty {1};
} tickUpdater{};



void HAL_IncTick()
{
    tickUpdater.interrupt();
}

class TickSubscriber : Subscriber
{
protected:
   bool subscribed;
   void subscribe()
   {
     if (not subscribed) {
       subscribed = true;
       tickUpdater.subscribe (this);
     }
   }
   void unsubscribe()
   {
     if (subscribed) {
       subscribed = false;
       tickUpdater.unsubscribe (this);
     }
   }
};


class Timer : TickSubscriber
{
  volatile uint32_t time_passed;
public:
  Timer(){
    time_passed = 0;
    subscribed = false;
  };
  Timer(uint32_t ms){
    time_passed = 0;
    subscribed = false;
    start(ms);
  }

  uint32_t time_set;

  void     start   (uint32_t ms); /// запускает счёт с текущего значения счётчика, устанавливает время
  bool     event();   /// возвращает true, если таймер натикал и перезапускает его
  bool     done();    /// возвращает true, если таймер натикал и НЕ перезапускает его
  void     pause();   /// останавливает счёт, но не сбрасывает счётчик
  void     start();   /// запускает счёт с текущего значения счётчика
  void     stop();    /// останавливает счёт и сбрасывает счётчик
  bool     isGreater (uint32_t val); /// возвращает true, если счётчик натикал больше значения val
  bool     isCount(); /// возвращает true если считает, но не досчитал
  uint32_t timePassed(); /// возвращает сколько натикал
  uint32_t timeLeft();   /// возвращает сколько осталось
  template<class function>
  void     event (function); /// выполняет function, когда дотикал и перезапускает таймер

  void notify() {
    time_passed++;
  }
};


//template<Faster multiplier> // how many subticks in tick
//void TickUpdater::subscribe_subtick (Subscriber& v)
//{
//    subtick.subscribe(v);
//    subtick_qty = multiplier;
//    REF(SysTick).initInterrupt<1000_us/multiplier>();
//}

template<class Function>
inline void every_qty_cnt_call(size_t& cnt, size_t qty, Function call)
{
    if (not(cnt++ % qty))
        call();
}

void TickUpdater::interrupt()
{
//    subtick.notify();
//    every_qty_cnt_call (subtick_cnt, subtick_qty, [this]{
        notify();
//    });
}


void Timer::start(uint32_t ms){
   time_passed = 0;
   time_set = ms;
   subscribe();
}

bool Timer::event()
{
  if (time_passed >= time_set) {
      time_passed = 0;
      return (true);
  } else {
    return (false);
  }
}

template<class function>
void Timer::event (function functor)
{
    if (time_passed >= time_set) {
        time_passed = 0;
        functor();
    }
}

void Timer::stop() {
   time_passed = 0;
   unsubscribe();
}

bool     Timer::done()       { return time_passed >= time_set; }
void     Timer::pause()      { unsubscribe(); }
void     Timer::start()      { subscribe();  }
bool     Timer::isCount()    { return subscribed; }
uint32_t Timer::timePassed() { return time_passed; }
uint32_t Timer::timeLeft(  ) { return time_set - time_passed; }
bool     Timer::isGreater (uint32_t val) { return time_passed > val; }


//#endif /* TIMERS_H_ */
