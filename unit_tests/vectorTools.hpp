/*!
 * @file vectorTools.hpp
 * @author Steven Knudsen
 * @date December 14, 2021
 *
 * @details A collection of vector tools.
 *
 * @copyright AlbertaSat 2021
 *
 * @license
 * This software may not be modified or distributed in any form, except as described in the LICENSE file.
 */

#ifndef EX2_ERROR_CONTROL_UNIT_TESTS_VECTOR_TOOLS_H_
#define EX2_ERROR_CONTROL_UNIT_TESTS_VECTOR_TOOLS_H_

#include <cstdint>
#include <vector>

namespace ex2 {
  namespace error_control {

    /*!
     * @brief Misc tools to manipulate vectors.
     *
     * @details For now, the methods are all static, so no object is needed; no
     * constructor or destructor.
     */
    class VectorTools {
    public:


      /*!
       * @brief Convert a vector of floats to packed bytes according to the threshold.
       *
       * @ details Sets of 8 values in the input vector are thresholded and turned
       * into packed uint8_t values. The lowest index value in each set (the "first"
       * value) is made to be the msb of the resultign uint8_t (aka byte) unless
       * @p reverseBitOrder is true.
       *
       * @param[in] threshold Input floats greater than @p threshold are deemed a
       * binary 1, otherwise they are deemed a binary 0
       * @param[in] reverseBitOrder If false, the first value in each group of 8 values
       * in @p in is the msb of the resulting byte in @p out. If true, the order
       * is reversed.
       * @param[in] in Vector of floats
       * @param[inout] out Vector of packed uint8_t (bytes). The length is the length of
       * @p in / 8, plus one if there is a remainder
       */
      static void floatToBytes(float threshold, bool reverseBitOrder, std::vector<float>& in, std::vector<uint8_t>& out);

      /*!
       * @brief Convert a vector of packed bytes to a vector of floats.
       *
       * @param[in] in Vector of packed packed bytes
       * @param[in] packed If true, there are 8 1-bit symbols per byte. Otherwise
       * a byte contains 1, 1-bit symbol in bit position lsb or msb as per @p lsbFirst
       * @param[in] lsbFirst If true, the first symbol in an input byte is the
       * lsb, otherwise it's the msb
       * @param[in] nrz True if the input bits are to be converted to non-return-to-zero floats
       * @param[in] magnitude Magnitude of the corresponding float values
       * @param[inout] out Vector of floats
       */
      static void bytesToFloat(std::vector<uint8_t>& in, bool packed, bool lsbFirst, bool nrz, float magnitude, std::vector<float>& out);

      /*!
       * @brief Switch the order of elements in blocks of the vector.
       *
       * @details A vector comprises a number of blocks of elements that are to
       * be reversed in order. The number of elements in each block is the same
       * and may be even or odd.
       *
       * @todo This could be templated.
       *
       * @param[in] blockSize The number of contiguous elements to reverse in a
       * block-wise fashion
       * @param[inout] v The vector
       */
      static void blockReverse(uint32_t blockSize, std::vector<float>& v);

    private:
    };

  } /* namespace error_control */
} /* namespace ex2 */

#endif /* EX2_ERROR_CONTROL_UNIT_TESTS_VECTOR_TOOLS_H_ */
