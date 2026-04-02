/**
 * @file audio_ring_buffer.hpp
 * @brief Lock-free ring buffer for audio sample streaming.
 */

#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstring>

/**
 * @brief Single-producer/single-consumer ring buffer for audio blocks.
 * @tparam T Sample type.
 * @tparam Size Buffer capacity in samples (must be power of 2).
 */
template <typename T, size_t Size> class AudioRingBuffer {
  static_assert((Size & (Size - 1)) == 0, "Size must be a power of 2");

public:
  /** @brief Construct an empty ring buffer. */
  AudioRingBuffer() : pWrite_(0), pRead_(0), isReady_(false) {}

  /**
   * @brief Write samples into the ring buffer.
   * @param input Source sample pointer.
   * @param length Number of samples to write.
   */
  void write(const T *input, size_t length) {
    size_t current_write = pWrite_.load(std::memory_order_relaxed);

    for (size_t i = 0; i < length; ++i) {
      data_[(current_write + i) & (Size - 1)] = input[i];
    }

    pWrite_.store(current_write + length, std::memory_order_release);
  }

  /**
   * @brief Read samples from the ring buffer.
   * @param output Destination sample pointer.
   * @param length Number of samples to read.
   * @note Outputs zeros while the buffer is not ready or underflowed.
   */
  void read(T *output, size_t length) {
    size_t current_read = pRead_.load(std::memory_order_relaxed);
    size_t current_write = pWrite_.load(std::memory_order_acquire);
    size_t available = current_write - current_read;

    if (!isReady_) {
      if (available >= (Size / 4)) {
        isReady_ = true;
      }
    }

    if (isReady_) {
      if (available >= length) {
        for (size_t i = 0; i < length; ++i) {
          output[i] = data_[(current_read + i) & (Size - 1)];
        }
        pRead_.store(current_read + length, std::memory_order_release);
      } else {
        std::memset(output, 0, length * sizeof(T));
        isReady_ = false;
      }
    } else {
      std::memset(output, 0, length * sizeof(T));
    }
  }

  /** @brief Reset read/write pointers and ready state. */
  void reset() {
    pWrite_.store(0, std::memory_order_release);
    pRead_.store(0, std::memory_order_release);
    isReady_ = false;
  }

  /**
   * @brief Get number of currently available samples.
   * @return Available sample count.
   */
  size_t getAvailableSamples() const {
    return pWrite_.load(std::memory_order_acquire) -
           pRead_.load(std::memory_order_relaxed);
  }

  /**
   * @brief Get number of currently available stereo frames.
   * @return Available frame count.
   */
  size_t getAvailableFrames() const { return getAvailableSamples() / 2; }

  /**
   * @brief Query whether buffer has reached ready threshold.
   * @return true if ready, false otherwise.
   */
  bool getIsReady() const { return isReady_; }

private:
  std::array<T, Size> data_;
  std::atomic<size_t> pWrite_{};
  std::atomic<size_t> pRead_{};
  std::atomic<bool> isReady_{};
};
