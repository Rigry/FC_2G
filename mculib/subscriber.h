/*
 * subscriber.h
 *
 *  Created on: 9 февр. 2023 г.
 *      Author: Alex
 */

//#ifndef SUBSCRIBER_H_
//#define SUBSCRIBER_H_

#pragma once

#include "static_vector.h"

struct Subscriber  {
   virtual void notify() = 0;
};

struct Publisher
{
    Static_vector<Subscriber*, 12> subscriber;

    void subscribe  (Subscriber* p) { subscriber.push_back(p); }
    void unsubscribe(Subscriber* p) { subscriber.remove(p);  }
    void notify() {
      for(uint8_t i = 0; i < subscriber.cend(); i++){
          subscriber[i]->notify();
      }
    }
};



//#endif /* SUBSCRIBER_H_ */
