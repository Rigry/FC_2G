/*
 * static_vector.h
 *
 *  Created on: 9 февр. 2023 г.
 *      Author: Alex
 */

//#ifndef STATIC_VECTOR_H_
//#define STATIC_VECTOR_H_
#pragma once

template<class T, size_t size_>
class Static_vector
{
    T vector [size_] ;
    size_t end_i;
public:
    Static_vector (){}
    void clear()  {end_i = 0;}
    uint8_t cend  () const {return end_i;}
    size_t size ()  {return end_i;}
    void push_back(T p)
    {
        if (end_i < size_)
            vector[end_i++] = p;
    }
    void remove (T p)
    {
      uint8_t index = 0;

      for (uint8_t i = 0; i < end_i; i++) {
        if (vector[i] == p) {
          index = i;
          break;
        }
      }

      for (uint8_t i = index; i < end_i; i++) {
          vector[i] = vector[i+1];
      }
      end_i--;
    }

    T operator[] (const size_t index) const
    {
        return vector[index];
    }

};




//#endif /* STATIC_VECTOR_H_ */
