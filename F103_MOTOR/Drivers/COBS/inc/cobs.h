/*
 * cobs.h
 *
 *  Created on: Nov 1, 2026
 *      Author: pk
 */

#ifndef COBS_COBS_H_
#define COBS_COBS_H_

#include <stddef.h>  // needed for size_t data type
#include <stdint.h>  // needed for uint8_t data type
#include <stdbool.h>

size_t getCOBSBufferSize(size_t input_size,
                         bool with_trailing_zero); // default for bool is true

size_t encodeCOBS(const uint8_t *inptr,
                   size_t inputlen,
                   uint8_t *outptr,
                   size_t outlen,
                   bool add_trailing_zero); // default for bool is true

size_t decodeCOBS(const uint8_t *inptr,
                  size_t inputlen,
                  uint8_t *outptr,
                  size_t outputlen);

size_t decodeCOBS_inplace(uint8_t *inptr, size_t inputlen);


#endif /* COBS_COBS_H_ */
