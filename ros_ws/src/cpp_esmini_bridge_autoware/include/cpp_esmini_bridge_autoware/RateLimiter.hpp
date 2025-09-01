#pragma once

#include <chrono>
#include <iostream>
#include <thread>

class RateLimiter {
  public:
    RateLimiter(std::chrono::milliseconds interval)
        : interval(interval),
          last_call(std::chrono::steady_clock::now() - interval) {}

    template <typename Func> void call(Func &&func) {
        auto now = std::chrono::steady_clock::now();
        if (now - last_call >= interval) {
            last_call = now;
            func();
        }
    }

  private:
    std::chrono::milliseconds interval;
    std::chrono::steady_clock::time_point last_call;
};
