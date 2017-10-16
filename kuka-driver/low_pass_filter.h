#pragma once

#include <stdexcept>

/**
 * A discrete time first order low pass filter. This filter maintains one
 * internal state `val`, and given input `input`, the update rule is:
 * <pre>
 * val = alpha * val + (1 - alpha) * input,
 * <\pre>
 * where alpha is a constant between 0 and 1.
 */
template <typename T> class DiscreteTimeLowPassFilter {
 public:
  /**
   * Constructs a DiscreteTimeLowPassFilter by specifying the desired cutoff
   * frequency and fixed update period.
   * @param cutoff_hz Cutoff frequency in Hz.
   * @param dt Time step in seconds.
   *
   * @throws std::logic_error if the equivalent alpha is smaller than 0 or
   * bigger than 1.
   */
  DiscreteTimeLowPassFilter(T cutoff_hz, T dt) {
    // https://en.wikipedia.org/wiki/Low-pass_filter
    // This alpha is 1 - wiki's alpha.
    T rc = 1 / (2 * M_PI * cutoff_hz);
    alpha_ = 1 - dt / (dt + rc);
    check_alpha();
  }

  /**
   * Constructs a DiscreteTimeLowPassFilter using by specifying the coefficient
   * directly.
   * @param alpha The coefficient.
   *
   * @throws std::logic_error if @p alpha is smaller than 0 or bigger than 1.
   */
  DiscreteTimeLowPassFilter(T alpha) : alpha_(alpha) { check_alpha(); }

  /**
   * Initializes the internal value to @p val.
   */
  void set_initial(T val = 0.) { val_ = val; }

  /**
   * Performs one filtering step given input @p input, and returns the filtered
   * value.
   */
  T filter(T input) {
    val_ = alpha_ * val_ + (1. - alpha_) * input;
    return val_;
  }

  /**
   * Returns the current filtered value.
   */
  T get_filtered() const { return val_; }

 private:
  void check_alpha() {
    if (alpha_ > 1 || alpha_ < 0)
      throw std::logic_error("invalid alpha" + std::to_string(alpha_));
  }

  T val_{0};
  T alpha_{0};
};
