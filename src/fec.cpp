/*!
 * @file FEC.cpp
 * @author StevenKnudsen
 * @date June 21, 2021
 *
 * @details A Forward Error Correction factory that creates instances of various
 * FEC codecs.
 *
 * @copyright AlbertaSat 2021
 *
 * @license
 * This software may not be modified or distributed in any form, except as described in the LICENSE file.
 */


#include "fec.hpp"
#include "no_fec.hpp"
#include "qcldpc.hpp"
#include "convolutional_codec_hd.hpp"

namespace ex2 {
  namespace error_control {

    FECException::FECException(const std::string& message) :
           runtime_error(message) { }

    FEC *
    FEC::makeFECCodec(ErrorCorrection::ErrorCorrectionScheme ecScheme,const uint32_t messageLengthBits)
    {

      // Create a new FEC-based object according to the desired FEC scheme.
      FEC *newFEC;
      switch(ecScheme) {
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_REED_SOLOMON_255_239_INTERLEAVING_1:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_REED_SOLOMON_255_239_INTERLEAVING_2:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_REED_SOLOMON_255_239_INTERLEAVING_3:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_REED_SOLOMON_255_239_INTERLEAVING_4:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_REED_SOLOMON_255_239_INTERLEAVING_5:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_REED_SOLOMON_255_239_INTERLEAVING_8:
          newFEC = NULL; // @TODO change when this is implemented
          break;
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_REED_SOLOMON_255_223_INTERLEAVING_1:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_REED_SOLOMON_255_223_INTERLEAVING_2:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_REED_SOLOMON_255_223_INTERLEAVING_3:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_REED_SOLOMON_255_223_INTERLEAVING_4:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_REED_SOLOMON_255_223_INTERLEAVING_5:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_REED_SOLOMON_255_223_INTERLEAVING_8:
          newFEC = NULL; // @TODO change when this is implemented
          break;
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_TURBO_1784_R_1_2:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_TURBO_1784_R_1_3:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_TURBO_1784_R_1_4:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_TURBO_1784_R_1_6:
          newFEC = NULL; // @TODO change when this is implemented
          break;
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_TURBO_3568_R_1_2:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_TURBO_3568_R_1_3:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_TURBO_3568_R_1_4:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_TURBO_3568_R_1_6:
          newFEC = NULL; // @TODO change when this is implemented
          break;
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_TURBO_7136_R_1_2:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_TURBO_7136_R_1_3:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_TURBO_7136_R_1_4:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_TURBO_7136_R_1_6:
          newFEC = NULL; // @TODO change when this is implemented
          break;
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_LDPC_ORANGE_BOOK_1280:
          newFEC = NULL; // @TODO change when this is implemented
          break;
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_LDPC_ORANGE_BOOK_1536:
          newFEC = NULL; // @TODO change when this is implemented
          break;
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_LDPC_ORANGE_BOOK_2048:
          newFEC = NULL; // @TODO change when this is implemented
          break;
        case ErrorCorrection::ErrorCorrectionScheme::IEEE_802_11N_QCLDPC_648_R_1_2:
        case ErrorCorrection::ErrorCorrectionScheme::IEEE_802_11N_QCLDPC_648_R_2_3:
        case ErrorCorrection::ErrorCorrectionScheme::IEEE_802_11N_QCLDPC_648_R_3_4:
        case ErrorCorrection::ErrorCorrectionScheme::IEEE_802_11N_QCLDPC_648_R_5_6:
          newFEC = new QCLDPC(ecScheme, messageLengthBits); // @TODO change when this is implemented
          break;
        case ErrorCorrection::ErrorCorrectionScheme::IEEE_802_11N_QCLDPC_1296_R_1_2:
        case ErrorCorrection::ErrorCorrectionScheme::IEEE_802_11N_QCLDPC_1296_R_2_3:
        case ErrorCorrection::ErrorCorrectionScheme::IEEE_802_11N_QCLDPC_1296_R_3_4:
        case ErrorCorrection::ErrorCorrectionScheme::IEEE_802_11N_QCLDPC_1296_R_5_6:
          newFEC = new QCLDPC(ecScheme, messageLengthBits); // @TODO change when this is implemented
          break;
        case ErrorCorrection::ErrorCorrectionScheme::IEEE_802_11N_QCLDPC_1944_R_1_2:
        case ErrorCorrection::ErrorCorrectionScheme::IEEE_802_11N_QCLDPC_1944_R_2_3:
        case ErrorCorrection::ErrorCorrectionScheme::IEEE_802_11N_QCLDPC_1944_R_3_4:
        case ErrorCorrection::ErrorCorrectionScheme::IEEE_802_11N_QCLDPC_1944_R_5_6:
          newFEC = new QCLDPC(ecScheme, messageLengthBits); // @TODO change when this is implemented
          break;
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_CONVOLUTIONAL_CODING_R_1_2:
          newFEC = new ConvolutionalCodecHD(ecScheme, messageLengthBits);
          break;
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_CONVOLUTIONAL_CODING_R_2_3:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_CONVOLUTIONAL_CODING_R_3_4:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_CONVOLUTIONAL_CODING_R_5_6:
        case ErrorCorrection::ErrorCorrectionScheme::CCSDS_CONVOLUTIONAL_CODING_R_7_8:
          newFEC = NULL; // @TODO change when this is implemented
          break;
        case ErrorCorrection::ErrorCorrectionScheme::NO_FEC:
          newFEC = new NoFEC(ecScheme, messageLengthBits);
          break;


        default:
          newFEC = NULL; // @TODO change when this is implemented
          break;
      }
      return newFEC;
    }

    FEC::FEC(ErrorCorrection::ErrorCorrectionScheme ecScheme, const uint32_t messageLengthBits)
    {
      m_errorCorrection = new ErrorCorrection(ecScheme, messageLengthBits);
    }

  } /* namespace error_control */
} /* namespace ex2 */
