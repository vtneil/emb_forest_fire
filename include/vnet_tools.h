#ifndef VNET_TOOLS_H
#define VNET_TOOLS_H

#include <Arduino.h>

#define DMAP(x, in_min, in_max, out_min, out_max) (((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))

template<typename T>
T map_val(T x, T in_min, T in_max, T out_min, T out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/**
 * Build Arduino String from multiple fields, separated by commas.
 * @param last Value
 * @return built Arduino String
 */
template<typename T>
String build_string(T last) {
    return String(last);
}

/**
 * Build Arduino String from multiple fields, separated by commas.
 * @param first Value
 * @param args Values
 * @return built Arduino String
 */
template<typename T, typename... Ts>
String build_string(T first, Ts... args) {
    String s0 = "";
    s0 += String(first) + "," + build_string<Ts...>(args...);
    return s0;
}

template<typename S = HardwareSerial, S *stream = &Serial, typename T>
void write_stream(T last) {
    stream->write(last);
}

template<typename S = HardwareSerial, S *stream = &Serial, typename T, typename... Ts>
void write_stream(T first, Ts... args) {
    stream->write(first);
    write_stream<S, stream, Ts...>(args...);
}

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char *sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

/**
 * Get Free RAM Memory
 * @return free memory remaining
 */
size_t free_memory() {
    char top;
#ifdef __arm__
    return &top - reinterpret_cast<char *>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
    return &top - __brkval;
#else  // __arm__
    return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

#endif //VNET_TOOLS_H
