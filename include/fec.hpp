/*!
 * @file fec.hpp
 * @author StevenKnudsen
 * @date June 21, 2021
 *
 * @details A Forward Error Correction factory that creates instances of various FEC codecs.
 * 
 * @note Unencoded messages are assumed to contain 1 bit per byte. If this is not true, results
 * will be unpredictable.
 * 
 * Codewords (encoded messages) are assumed to be packed.
 * 
 * The FEC factory assumes that all FEC schemes work on a fixed length message.
 *
 * @copyright AlbertaSat 2021
 *
 * @license
 * This software may not be modified or distributed in any form, except as described in the LICENSE file.
 */

#ifndef EX2_MAC_ERROR_CONTROL_FEC_H_
#define EX2_MAC_ERROR_CONTROL_FEC_H_

#include <cstdint>
#include <string>
#include <vector>
#include <stdexcept>

#include "error_correction.hpp"

namespace ex2 {
  namespace error_control {


    class FECException: public std::runtime_error {

    public:
      FECException(const std::string& message);
    };

    /*!
     * @brief Define a forward error correction scheme.
     */
    class FEC {
    public:
      /*!
       * @brief The factory make method
       * 
       * @note The FEC factory assumes that all FEC schemes work on a fixed length message.
       * 
       * @param[in] ecScheme The error correction scheme
       * @param[in] msgLengthBytes The message length in bytes. If the number of bits is 
       * not a multiple of 8, the last byte is assumed to be zero-padded.
       */
      static FEC *makeFECCodec(ErrorCorrection::ErrorCorrectionScheme ecScheme, const uint32_t msgLengthBytes);

      virtual ~FEC() {}

      /*!
       * @brief A virtual function to encode a payload using the FEC scheme
       *
       * @param[in] message The message to encode; assumed 8 message bits per byte. If the 
       * number of bits is not a multiple of 8, the last byte is assumed to be zero-padded.
       * @return The codeword (encoded message) packed, 8 bits per byte
       */
      virtual std::vector<uint8_t> encode(const std::vector<uint8_t>& message) = 0;

      /*!
       * @brief A virtual function to decode a codeword using the FEC scheme
       *
       * @todo It may be better to not have @p encodedPayload as const. What if
       * it needs to be manipulated? After all, there is no contract saying it's
       * not destroyed after decoding
       *
       * @param[in] codeword The codeword (encoded message) assume packed, 8 bits per byte
       * @param[in] snrEstimate An estimate of the SNR for FEC schemes that need it.
       * @param[out] message The resulting decoded message with 8 bits per byte. If the 
       * number of bits is not a multiple of 8, the last byte is assumed to be zero-padded.
       * @return The number of bit errors from the decoding process
       */
      virtual uint32_t decode(std::vector<uint8_t>& codeword, const float snrEstimate,
        std::vector<uint8_t>& message) = 0;

    protected:

     /*!
      * @brief FEC Constructor
      * 
      * @note The FEC factory assumes that all FEC schemes work on a fixed length message.
      * 
      * @param[in] ecScheme The error correction scheme
      * @param[in] msgLengthBytes The message length in bytes
      */
      FEC(ErrorCorrection::ErrorCorrectionScheme ecScheme, const uint32_t msgLengthBytes);

      ErrorCorrection *m_errorCorrection = 0;

    };

  } /* namespace error_control */
} /* namespace ex2 */

#endif /* EX2_MAC_ERROR_CONTROL_FEC_H_ */
