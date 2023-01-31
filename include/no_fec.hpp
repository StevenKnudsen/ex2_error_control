/*!
 * @file no_fec.hpp
 * @author StevenKnudsen
 * @date July 12, 2021
 *
 * @details A passthrough FEC codec.
 *
 * @copyright AlbertaSat 2021
 *
 * @license
 * This software may not be modified or distributed in any form, except as described in the LICENSE file.
 */

#ifndef EX2_ERROR_CONTROL_NOFEC_H_
#define EX2_ERROR_CONTROL_NOFEC_H_

#include "fec.hpp"

namespace ex2 {
  namespace error_control {

    /*!
     * @brief Define a forward error correction scheme.
     */
    class NoFEC : public FEC {
    public:

      NoFEC(ErrorCorrection::ErrorCorrectionScheme ecScheme, const uint32_t messageLength) : FEC(ecScheme, messageLength) { }

      ~NoFEC();

      std::vector<uint8_t> encode(const std::vector<uint8_t>& payload);

      uint32_t decode(std::vector<uint8_t>& encodedPayload, float snrEstimate,
        std::vector<uint8_t>& decodedPayload);

    };

  } /* namespace error_control */
} /* namespace ex2 */

#endif /* EX2_ERROR_CONTROL_NOFEC_H_ */
