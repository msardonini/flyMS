/*
Copyright (c) 2014, Mike Sardonini
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the FreeBSD Project.
*/

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>

#ifndef FILTER_H
#define FILTER_H

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/** \var typedef struct digital_filter_t
    \brief A type definition for a struct that contains all the info needed for a digital filter
*/
typedef struct digital_filter_t {
  uint8_t order;           /**< Specified order of the filter*/
  uint8_t filter_len;      /**< Specified order of the filter*/
  uint8_t current_index_f; /**< Index showing where in the vector current and past data is */
  uint8_t initialized;     /**< Boolean saying if filter has been initialized*/
  float data[];            /**< Denominator Coefficients*/

} digital_filter_t;

/**
 * @brief Apply the filter on a new data sample
 *
 * @param filter A pointer to a pre-generated filter object
 * @param raw_val The new raw value to apply to the filter
 * @return float The filtered output value
 */
float update_filter(digital_filter_t* filter, float raw_val);

/**
 * @brief Simple Saturation filter
 *
 * @param value input value to filter
 * @param min min value of saturation filter
 * @param max max value of saturation filter
 * @return float filtered value
 */
float saturateFilter(float value, float min, float max);

/**
 * @brief Zero out all data in a filter
 *
 * @param filter A pointer to a pre-generated filter object
 */
int zeroFilter(digital_filter_t* filter);

/**
 * @brief Fill the history of the filter with a value to prevent a harsh step function
 *
 * @param filter A pointer to a pre-generated filter object
 * @param value value to fill the filter with
 */
int prefill_filter(digital_filter_t* filter, float value);

/**
 * @brief Dynamically allocate memory for a filter of specified order and set transfer function constants.
          Note: A normalized transfer function should have a leading 1 in the denominator but can be !=1 in this library
 *
 * @param order The order of the filter
 * @param num numerator coefficients, should be size (order + 1)
 * @param den denominator coefficients, should be size (order + 1
 * @return digital_filter_t*
 */
digital_filter_t* initialize_filter(uint8_t order, float num[], float den[]);

/**
 * @brief discrete-time implementation of a parallel PID controller with derivative filter similar to Matlab pid command
 *
 * @param kp Proportional coeff
 * @param ki Integrator coeff
 * @param kd Derivative coeff
 * @param Tf the pole location for derivative filter. Must be greater than 2*DT smaller Tf gives faster filter decay
 * @param dt The time difference between sample updates
 * @return digital_filter_t* a digital filter
 */
digital_filter_t* generatePID(float kp, float ki, float kd, float Tf, float dt);

/**
 * @brief Print the contents of the digital filter
 *
 * @param filter The value of the filter to print
 */
void print_filter(digital_filter_t* filter);

#endif

#ifdef __cplusplus
}
#endif
