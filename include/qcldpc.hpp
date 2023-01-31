/*!
 * @file qcldpc.hpp
 * @author StevenKnudsen
 * @date Sept 27, 2021
 *
 * @details Quasi-Cyclic Low-Density Parity Check based on 802.11 QC generators.
 *
 * @copyright AlbertaSat 2021
 *
 * @license
 * This software may not be modified or distributed in any form, except as described in the LICENSE file.
 */

#ifndef EX2_ERROR_CONTROL_QCLDPC_H_
#define EX2_ERROR_CONTROL_QCLDPC_H_

#include <stdexcept>

#include "fec.hpp"

namespace ex2 {
  namespace error_control {

    /*!
     * @brief Define a forward error correction scheme.
     */
    class QCLDPC : public FEC {
    public:

      QCLDPC(ErrorCorrection::ErrorCorrectionScheme ecScheme, const uint32_t messageLength);

      ~QCLDPC();

      /*!
       * @brief A virtual function to encode a payload using the FEC scheme
       *
       * @param[in] message The message to encode; assumed one message bit per byte
       * @return The codeword (encoded message)
       */
      std::vector<uint8_t> encode(const std::vector<uint8_t>& message);

      /*!
       * @brief A virtual function to decode a codeword using the FEC scheme
       *
       * @todo It may be better to not have @p encodedPayload as const. What if
       * it needs to be manipulated? After all, there is no contract saying it's
       * not destroyed afrter decoding
       *
       * @param[in] codeword The codeword (encoded message)
       * @param[in] snrEstimate An estimate of the SNR for FEC schemes that need it.
       * @param[out] message The resulting decoded message with one bit per byte
       * @return The number of bit errors from the decoding process
       */
      uint32_t decode(std::vector<uint8_t>& codeword, float snrEstimate,
        std::vector<uint8_t>& message);

    private:
      ErrorCorrection *m_errorCorrection = 0;
    };

  } /* namespace error_control */
} /* namespace ex2 */

#endif /* EX2_ERROR_CONTROL_QCLDPC_H_ */
