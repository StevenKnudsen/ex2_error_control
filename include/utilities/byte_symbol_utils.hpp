/*!
 * @file byte_symbol_utils.hpp
 * @author Steven Knudsen
 * @date Jan 18, 2023
 *
 * @details Utilities to manage packing and unpacking bytes.
 *
 * @copyright University of Alberta 2023
 *
 * @license
 * This software may not be modified or distributed in any form, except as described in the LICENSE file.
 */

#ifndef EX2_SDR_ERROR_CONTROL_BYTE_SYMBOL_UTILS_H_
#define EX2_SDR_ERROR_CONTROL_BYTE_SYMBOL_UTILS_H_

#include <cstdint>
#include <vector>

namespace ex2 {
  namespace error_control {
    /*!
     * @brief Utility class that packs or unpacks @p uint8_t symbols.
     *
     * @details This class defines static methods to pack and unpack byte symbols.
     *
     * Symbols may be "unpacked", which means there is one symbol per byte
     * in the PDU. Or they may be "packed" so that a minimum number of bits
     * in the PDU are used. For example, a symbol with 2 data bits will
     * occupy 2 bits per byte in an unpacked representation. Four, 2-bit symbols
     * occupy a byte in a packed representation.
     *
     * In the unpacked representation, symbols bits are placed in the byte's
     * least significant bits.
     */

    class ByteSymbolUtility
    {

    public:

      enum  BitsPerSymbol {
        BPSymb_1 = 1,
        BPSymb_2 = 2,
        BPSymb_3 = 3,
        BPSymb_4 = 4,
        BPSymb_5 = 5,
        BPSymb_6 = 6,
        BPSymb_7 = 7,
        BPSymb_8 = 8
      };

      /*!
       * @brief Repack the symbols.
       *
       * @details Repack the symbols based on the new bits per symbol setting.
       *
       * @note It's assumed that the caller knows the current bits per symbol
       *
       * @param[in out] payload A byte vector
       * @param[in] currentBps Current number of bits per symbol
       * @param[in] newBps New number of bits per symbol
       */
      static void repack(std::vector<uint8_t>& payload, BitsPerSymbol currentBps, BitsPerSymbol newBps);

      /*!
       * @brief Reverse the order of the payload
       *
       * @param[in out] payload A byte vector
       * @param[in] currentBps Current number of bits per symbol
       * @param[in] byteLevel If true, the bytes of the payload are reversed,
       * otherwise the payload bits are reversed.
       */
      static void reverse(std::vector<uint8_t>& payload, BitsPerSymbol currentBps, bool byteLevel);

      /*!
       * @brief Roll the payload bits right or left a number of bit positions.
       *
       * @param[in out] payload A byte vector
       * @param[in] currentBps Current number of bits per symbol
       * @param[in] numBits The number of bit positions to roll the vector
       * @param[in] left If true, roll the bits left. Otherwise roll right.
       */
      static void roll(std::vector<uint8_t>& payload, BitsPerSymbol currentBps, uint32_t numBits, bool left);

    private:


      /*!
       * @brief Pack 1-bit symbols into bytes
       */
      static void pack(std::vector<uint8_t>& payload);

      /*!
       * @brief Unpack bytes into 1-bit symbols
       */
      static void unpack(std::vector<uint8_t>& payload);

    };

  } /* namespace error_control */
} /* namespace ex2 */

#endif /* EX2_SDR_ERROR_CONTROL_BYTE_SYMBOL_UTILS_H_ */
