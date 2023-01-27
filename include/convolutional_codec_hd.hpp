/*!
 * @file convolutional_codec_hd.hpp
 * @author Steven Knudsen
 * @date Dec 1, 2021
 *
 * @details The Convolutional Codec provides convolutional encoding and
 * hard-decision decoding for the CCSDS schemes defined in @p error_correction.hpp
 *
 * @copyright AlbertaSat 2021
 *
 * @license
 * This software may not be modified or distributed in any form, except as described in the LICENSE file.
 */

#ifndef EX2_ERROR_CONTROL_CONVOLUTIONAL_CODEC_HD_H_
#define EX2_ERROR_CONTROL_CONVOLUTIONAL_CODEC_HD_H_

#include "fec.hpp"

#include "third_party/viterbi/viterbi.hpp"


namespace ex2 {
  namespace error_control {

    /*!
     * @brief Define a forward error correction scheme.
     */
    class ConvolutionalCodecHD : public FEC {
    public:

      ConvolutionalCodecHD(ErrorCorrection::ErrorCorrectionScheme ecScheme);

      ~ConvolutionalCodecHD();

      std::vector<uint8_t> encode(const std::vector<uint8_t>& payload);

      uint32_t decode(std::vector<uint8_t>& encodedPayload, float snrEstimate,
        std::vector<uint8_t>& decodedPayload);

    private:
      ErrorCorrection *m_errorCorrection = 0;
      ViterbiCodec *m_codec = 0;
    };

  } /* namespace error_control */
} /* namespace ex2 */

#endif /* EX2_ERROR_CONTROL_CONVOLUTIONAL_CODEC_HD_H_ */
