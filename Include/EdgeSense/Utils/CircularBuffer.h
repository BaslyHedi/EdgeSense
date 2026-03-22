/**
 * @file CircularBuffer.h
 * @author Hedi Basly
 * @brief Header for CircularBuffer module to store recent sensor readings
 * @date 2026-02-16
 */
#pragma once
#include <vector>
#include <mutex>
#include <algorithm>
#include <cstdint>

namespace EdgeSense {
    namespace Utils {

    template <typename T, size_t Size>
    class CircularBuffer {
        public:
            CircularBuffer() : head(0), count(0) {}

            /**
             * @brief Add a new sample (Harvester Thread 1ms)
             * @param item Sample to add
             */
            void push(const T& item) {
                std::lock_guard<std::mutex> lock(mtx);
                buffer[head] = item;
                head = (head + 1) % Size;
                if (count < Size) {
                    count++;
                }
            }

            /**
             * @brief Retrieve 'n' samples (Refiner Thread 5ms)
             * @param n Number of samples to retrieve
             * @return Vector containing the requested samples in order from OLDEST to NEWEST
             */
            std::vector<T> getLatest(size_t n) {
                std::lock_guard<std::mutex> lock(mtx);
                std::vector<T> result;
                
                /* Don't try to grab more than we have or more than the buffer holds */
                size_t available = std::min(n, count);
                result.reserve(available);

                for (size_t i = 0; i < available; ++i) {
                    /* Logic to find the 'i-th' oldest element in the requested window */
                    size_t idx = (head + Size - available + i) % Size;
                    result.push_back(buffer[idx]);
                }
                return result;
            }

            size_t getCount() const { return count; }

        private:
            T buffer[Size];
            size_t head;
            size_t count;
            std::mutex mtx;
        };

    } /* namespace Utils */
} /* namespace EdgeSense */