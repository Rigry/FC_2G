#pragma once

template<uint8_t size_>
class Net_buffer {
public:

    Net_buffer() {
      clear();
    }

    void clear() { begin_i = 0; end_i = 0; }
    uint8_t begin() { return begin_i; }
    uint8_t end()   { return end_i; }
    uint8_t size()  { return end_i - begin_i; }
    void set_size (size_t v) { begin_i = 0; end_i = v; }
    uint8_t* ptr() {return &buffer[0];}

    Net_buffer& operator<< (uint8_t);
    Net_buffer& operator<< (char);
    Net_buffer& operator<< (uint16_t);
    Net_buffer& operator<< (int16_t);

    Net_buffer& operator>> (uint8_t&);
    Net_buffer& operator>> (uint16_t&);

    Net_buffer& operator<< (char*);


    uint8_t operator[](int n){return buffer[n];}
    uint8_t pop_back();
    uint8_t pop_front();

private:

   uint8_t begin_i;
   uint8_t end_i;
   uint8_t buffer[size_];

    void to_bytes (uint16_t v, uint8_t& low, uint8_t& hi)
    {
        union {
            uint8_t d8[2];
            uint16_t d16;
        } u;
        u.d16 = v;
        low = u.d8[0];
        hi = u.d8[1];
    }

    uint16_t from_bytes (uint8_t v1, uint8_t v2)
    {
        union {
            uint8_t d8[2];
            uint16_t d16;
        } u;
        u.d8[0] = v1;
        u.d8[1] = v2;
        return u.d16;
    }

};








template<uint8_t size_>
Net_buffer<size_>& Net_buffer<size_>::operator<< (uint8_t v)
{
    if (end_i < size_) {
      buffer[end_i] = v;
      end_i++;
    }
    return *this;
}

template<uint8_t size_>
Net_buffer<size_>& Net_buffer<size_>::operator<< (char v)
{
    if (end_i < size_) {
      buffer[end_i] = v;
      end_i++;
    }
    return *this;
}

template<uint8_t size_>
Net_buffer<size_>& Net_buffer<size_>::operator<< (char *s)
{
   for (int i = 0; ; i++) {
        if (s[i] == '\0') return *this;
        buffer[end_i] = s[i];
        end_i++;
   }
}

template<uint8_t size_>
Net_buffer<size_>& Net_buffer<size_>::operator<< (uint16_t v)
{
    uint8_t low, hi;
    to_bytes(v, low, hi);
    *this << hi << low;
    return *this;
}

template<uint8_t size_>
Net_buffer<size_>& Net_buffer<size_>::operator<< (int16_t v)
{
    uint8_t low, hi;
    to_bytes(v, low, hi);
    *this << hi << low;
    return *this;
}

template<uint8_t size_>
Net_buffer<size_>& Net_buffer<size_>::operator>> (uint8_t& v)
{
    if (end_i > begin_i)
        v = buffer[begin_i++];
    else
        v = 0;
    return *this;
}

template<uint8_t size_>
Net_buffer<size_>& Net_buffer<size_>::operator>> (uint16_t& v)
{
    if (end_i > begin_i + 1) {
        uint8_t _1 = pop_front();
        uint8_t _2 = pop_front();
        v = from_bytes (_1, _2);
    } else {
        v = 0;
    }

    return *this;
}

template<uint8_t size_>
uint8_t Net_buffer<size_>::pop_back()
{
    if (end_i > begin_i)
        return buffer[--end_i];
//    return 0;
}

template<uint8_t size_>
uint8_t Net_buffer<size_>::pop_front()
{
    if (end_i > begin_i)
        return buffer[begin_i++];
//    return 0;
}
