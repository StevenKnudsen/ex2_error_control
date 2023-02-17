/*!
 * @file qa_viterbit.cpp
 * @author Steven Knudsen
 * @date Dec 15, 2021
 *
 * @details Unit test for the viterbi class as written by vitalsong.
 *
 *
 * @copyright AlbertaSat 2021
 *
 * @license
 * This software may not be modified or distributed in any form, except as described in the LICENSE file.
 */

#include <cstdio>
#include <iostream>
#include <random>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

#include "third_party/viterbi/viterbi.hpp"
#include "utilities/byte_symbol_utils.hpp"
#include "viterbi-utils.hpp"
#include "vectorTools.hpp"

using namespace std;
using namespace ex2::error_control;

#include "gtest/gtest.h"

// Set this to 1 if the trellis column type is uint8_t instead of int.
// See third_party/viterbi/viterbi.hpp
#define QA_VITERBI_TRELLIS_COL_8_BIT 1

/*!
 * @brief Count the number of '1' bits in a byte
 * @param b The byte
 * @return Number of '1' bits
 */
uint8_t numOnesInByte(uint8_t b) {
  uint8_t count = (b >> 7) & 0x01;
  count += (b >> 6) & 0x01;
  count += (b >> 5) & 0x01;
  count += (b >> 4) & 0x01;
  count += (b >> 3) & 0x01;
  count += (b >> 2) & 0x01;
  count += (b >> 1) & 0x01;
  count += b & 0x01;

  return count;
}

/*!
 * @brief Check FEC decoding for the scheme provided.
 *
 * @details Using randomly generated messages at the specificy SNR, test if the
 * specified bit-error rate can be achieved.
 *
 * @param errorCorrectionScheme The error correction scheme.
 * @param snr The signal to noise ratio of the generated data in the range [-20,60]
 * @param ber The target bit-error rate in the range [1e-6,0.1]
 * @param berExceedExpected If true, the target @p ber is expected to be exceeded
 * @param berTolerance The percent tolerance to allow for the BER to be exceeded, in the range [0,100]
 */
double check_ber (const ViterbiCodec &codec, double snr, unsigned int maxBitErrors, unsigned long maxBits)
{
  if (snr > 60.0f or snr < -20.0f) {
    printf("The SNR must be in the range [-20,60]\n");
    throw std::exception();
  }

  if (maxBitErrors == 0) {
    printf("The maximum number of bit errors must be > 0\n");
    throw std::exception();
  }

  if (maxBits == 0) {
    printf("The maximum number of bit encoded/decoded must be > 0\n");
    throw std::exception();
  }

  // @note we assume bpsk for this test
  double bitsPerSymbol = 1;
  double rate = 0.5; // 1/2 rate

  double snrAdjusted = snr + 10.0*std::log10(bitsPerSymbol*rate);

  // Get ready for simulating noise at the input SNR
  // The channel is assumed to be complex, so the noise power is split
  // between the I and Q channels. For example, to generate complext noise
  // with variance 1, we need real and imaginary variances to be 1/2.
  double sigma2 = 1.0 / pow (10.0, snrAdjusted / 10.0); // noise variance
  double sigma = sqrt(sigma2)*sqrt(2.0)/2.0;

  std::mt19937 generator;
  std::normal_distribution<double> dist(0.0, sigma);

  // Deal with data in packed format, i.e., 8 bits per byte.
  // The message length is returned in bits. It's easiest to handle the data as
  // being 1 bit per 8-bit symbol, aka unpacked
  unsigned int payloadByteCount = 1000;
  std::vector<uint8_t> packedMessage(payloadByteCount);

  uint32_t numBits = 0;
  uint32_t numErrors = 0;

  std::srand(std::time(0));

  while (numBits < maxBits && numErrors < maxBitErrors)
  {
    // Make a random data payload
    for (unsigned int i = 0; i < payloadByteCount; i++) {
      packedMessage[i] = (uint8_t) (std::rand () & 0x00FF);
    }

    // Encode the packet
    std::vector<uint8_t> payload = codec.encodePacked (packedMessage);

    std::vector<float> payloadFloat;
    VectorTools::bytesToFloat(payload, true, false, true, 1.0f, payloadFloat);

    // Add noise. The bytesToFloat method makes float symbols of mag 1.
//#if QA_CC_HD_DEBUG
    double pSignal = 0.0f;
    double pNoise = 0.0f;
//#endif
    double noise;
    for (uint32_t i = 0; i < payloadFloat.size(); i++) {
      noise = dist(generator);
//#if QA_CC_HD_DEBUG
      pNoise += noise*noise;
      pSignal += payloadFloat[i]*payloadFloat[i];
//#endif
      payloadFloat[i] += noise;
    }

//#if QA_CC_HD_DEBUG
    pSignal /= (float) payloadFloat.size();
    pNoise /= (float) payloadFloat.size();
    printf("pSignal = %g pNoise = %g calculated snr = %g\n", pSignal, pNoise, 10.0f*std::log10(pSignal/pNoise));
//#endif
    // We convert back to binary data and impose our own hard decision.
    std::vector<uint8_t> payloadPlusNoise;
    float threshold = 0.0; // The float payload was NRZ, so in [-1,1]
    VectorTools::floatToBytes(threshold, false, payloadFloat, payloadPlusNoise);

    printf("payload size %ld payloadPlusNoise size %ld\n",payload.size(),payloadPlusNoise.size());
    uint8_t diffPByte;
    unsigned long payloadErrors = 0;
    for (unsigned int i = 0; i < payload.size(); i++) {
      diffPByte = payload[i] ^ payloadPlusNoise[i];
      if (diffPByte > 0) {
//        printf("diff[%ld]\n",i);
        payloadErrors += numOnesInByte(diffPByte);
      }
    }
    printf("payloadErrors %ld\n",payloadErrors);

    // Try to decode the noisy codeword
    ByteSymbolUtility::repack(payloadPlusNoise, ByteSymbolUtility::BPSymb_8, ByteSymbolUtility::BPSymb_1);
    std::vector<uint8_t> decodedMessage = codec.decode(payloadPlusNoise);
    ByteSymbolUtility::repack(decodedMessage, ByteSymbolUtility::BPSymb_1, ByteSymbolUtility::BPSymb_8);

    // count the bit errors in the decoded message
    printf("packedMessage size %ld decodedMessage size %ld\n",packedMessage.size(),decodedMessage.size());
    uint8_t diffByte;
    for (unsigned int i = 0; i < packedMessage.size(); i++) {
      diffByte = packedMessage[i] ^ decodedMessage[i];
      if (diffByte > 0) {
        printf("diff[%ld]\n",i);
        numErrors += numOnesInByte(diffByte);
      }
    }
    numBits += payloadByteCount*8;
  } // while not enough bits for BER

  double calcBER = (double) numErrors / (double) numBits;
//#if QA_CC_HD_DEBUG
  printf("@%g dB (adjusted %g), numbBits %d numErrors %d calculated BER %g\n", snr, snrAdjusted, numBits, numErrors, calcBER);
//#endif
  return calcBER;
//  delete(generator);
} // check decoder



static ViterbiCodec::bitarr_t _gen_message(unsigned long num_bits)
{
  ViterbiCodec::bitarr_t msg(num_bits);
  for (unsigned long j = 0; j < num_bits; j++)
  {
    msg[j] = (std::rand() & 0x1);
  }
  return msg;
}

#define QA_VITERBI_DEBUG 0 // set to 1 for debugging output

#if 1
/*!
 * @brief Test Main Constructors, the one that is parameterized, and the one
 * that takes the received packet as input
 */
TEST(viterbi, Poly_7x5_err)
{
  /* ----------------------------------------------------------------------
   * Confirm codec with constraint length 3 and polynomials 0b111 and 0b101
   * ----------------------------------------------------------------------
   */
  ViterbiCodec codec(3, {7, 5});
  ASSERT_EQ(codec.decodeTruncated("001110000110011111100010110011"_b), "010111001010001"_b);

  // Inject 1 error bit.
  ASSERT_EQ(codec.decodeTruncated("001110000110011111000010110011"_b), "010111001010001"_b);
}

TEST(Viterbi, Poly_7x6_err)
{
  /* ----------------------------------------------------------------------
   * Confirm codec with constraint length 3 and polynomials 0b111 and 0b110
   * ----------------------------------------------------------------------
   */
  ViterbiCodec codec(3, {7, 6});
  ASSERT_EQ(codec.decodeTruncated("101101010011"_b), "101100"_b);

  // Inject 1 error bit.
  ASSERT_EQ(codec.decodeTruncated("101101110011"_b), "101100"_b);
}

TEST(Viterbi, Poly_6x5_err)
{
  /* ----------------------------------------------------------------------
   * Confirm codec with constraint length 3 and polynomials 0b110 and 0b101
   * ----------------------------------------------------------------------
   */
  ViterbiCodec codec(3, {6, 5});
  ASSERT_EQ(codec.decodeTruncated("01101101110110"_b), "1001101"_b);

#if QA_VITERBI_TRELLIS_COL_8_BIT
  // Inject 1 error bits.
  ASSERT_EQ(codec.decodeTruncated("01101101110010"_b), "1001101"_b);
#else
  // Inject 2 error bits.
  ASSERT_EQ(codec.decodeTruncated("11101101110010"_b), "1001101"_b);
#endif
}

TEST(Viterbi, Poly_91x117x121_err)
{
  /* ----------------------------------------------------------------------
   * Confirm codec with constraint length 7 and polynomials 0b1011011,
   * 0b1110101, and 0b1111001
   * ----------------------------------------------------------------------
   */
  ViterbiCodec codec(7, {91, 117, 121});
  ASSERT_EQ(codec.decode("111100101110001011110101"_b), "10110111"_b);

  // Inject 4 error bits.
  ASSERT_EQ(codec.decode("100100101110001011110101"_b), "10110111"_b);
}

TEST(Viterbi, Voyager_err)
{
  /* ----------------------------------------------------------------------
   * Confirm codec with constraint length 7 and polynomials 0b1101101, and
   * 0b1001111 and 5% errors induced
   * ----------------------------------------------------------------------
   */
  ViterbiCodec codec(7, {109, 79});

  double snr = 5.0;
  unsigned int maxBitErrs = 100;
  unsigned long maxBits = 1000000;
  double ber = check_ber (codec, snr, maxBitErrs, maxBits);
  printf("ber = %g\n",ber);

}

#if 0
TEST(Viterbi, CCSDS_err)
{
  /* ----------------------------------------------------------------------
   * Confirm codec with constraint length 7 and polynomials 0b1111001, and
   * 0b1011011 and 5% errors induced
   * ----------------------------------------------------------------------
   */
  ViterbiCodec codec(7, {121, 91});

  for (unsigned int trial = 0; trial < 100; trial++) {
    printf("Trial %d\n",trial);
    auto message = _gen_message(32);
    auto encoded = codec.encode(message);

    // add 5% errors
    unsigned int nerr = encoded.size() * 0.05;
    for (size_t i = 0; i < nerr; i++)
    {
      int idx = rand() % encoded.size();
      encoded[idx] = (encoded[idx] == 0) ? (1) : (0);
    }

    auto decoded = codec.decode(encoded);
    ASSERT_EQ(message, decoded);
  }
}
#endif
// Test the given ViterbiCodec by randomly generating 10 input sequences of
// length 8, 16, 32 respectively, encode and decode them, then test if the
// decoded string is the same as the original input.
void TestViterbiCodecAutomatic(const ViterbiCodec &codec, bool truncated)
{
  for (int num_bits = 8; num_bits <= 32; num_bits <<= 1)
  {
    for (int i = 0; i < 10; i++)
    {
      auto message = _gen_message(num_bits);
      // for (int j = 0; j < message.size(); j++)
      //   printf("0x%02x ",message[j]);
      // printf("\n");
      auto encoded = codec.encode(message);
      if (truncated)
      {
        auto decoded = codec.decodeTruncated(encoded);
#if QA_VITERBI_DEBUG
        printf("lengths: message %ld encoded %ld decoded %ld\n",
               message.size(), encoded.size(), decoded.size());
#endif
        ASSERT_EQ(decoded, message);
      }
      else
      {
        auto decoded = codec.decode(encoded);
#if QA_VITERBI_DEBUG
        printf("lengths: message %ld encoded %ld decoded %ld\n",
               message.size(), encoded.size(), decoded.size());
#endif
        ASSERT_EQ(decoded, message);
      }
    }
  }
}

TEST(Viterbi, Poly_7x5)
{
  /* ----------------------------------------------------------------------
   * Confirm codec with constraint length 3 and polynomials 0b111 and 0b101
   * ----------------------------------------------------------------------
   */
  ViterbiCodec codec(3, {7, 5});
  TestViterbiCodecAutomatic(codec, true);
}

TEST(Viterbi, Poly_6x5)
{
  /* ----------------------------------------------------------------------
   * Confirm codec with constraint length 3 and polynomials 0b110 and 0b101
   * ----------------------------------------------------------------------
   */
  ViterbiCodec codec(3, {6, 5});
  TestViterbiCodecAutomatic(codec, true);
}

TEST(Viterbi, Voyager)
{
  /* ----------------------------------------------------------------------
   * Confirm codec with constraint length 7 and polynomials 0b1101101, and
   * 0b1001111
   * ----------------------------------------------------------------------
   */
  ViterbiCodec codec(7, {109, 79});
  TestViterbiCodecAutomatic(codec, true);
}

TEST(Viterbi, LTE)
{
  /* ----------------------------------------------------------------------
   * Confirm codec with constraint length 7 and polynomials 0b1011011,
   * 0b1110101, and 0b1111001
   * ----------------------------------------------------------------------
   */
  ViterbiCodec codec(7, {91, 117, 121});
  TestViterbiCodecAutomatic(codec, false);
}

TEST(Viterbi, CDMA_2000)
{
  /* ----------------------------------------------------------------------
   * Confirm codec with constraint length 9 and polynomials 0x1F5, 0x1B9,
   * 0x14B, and 0x13B
   * ----------------------------------------------------------------------
   */
  ViterbiCodec codec(9, {501, 441, 331, 315});
  TestViterbiCodecAutomatic(codec, false);
}

#if QA_VITERBI_TRELLIS_COL_8_BIT
#else
TEST(Viterbi, Cassini)
{
  // Cassini / Mars Pathfinder
  /* ----------------------------------------------------------------------
   * Confirm codec with constraint length 15 and polynomials 0x000F, 0x4599,
   * 0x4EA5, 0x5D47, 0x76F3, 0x7EB7, and 0x695F
   * ----------------------------------------------------------------------
   */
  ViterbiCodec codec(15, {15, 17817, 20133, 23879, 30451, 32439, 26975});
  TestViterbiCodecAutomatic(codec);
}
#endif
#endif
// Test the given ViterbiCodec by randomly generating 10 input sequences of
// length 8, 16, 32, and 64 bytes respectively, encode and decode them, then
// test if the decoded string is the same as the original input.
void TestViterbiCodecAutomaticPacked(const ViterbiCodec &codec, bool truncated)
{
  for (unsigned long num_bytes = 8; num_bytes <= 64; num_bytes <<= 1)
  {
    std::vector<uint8_t> message;

    for (int i = 0; i < 10; i++)
    {
      message.resize(0);

      // Set the payload to readable ASCII
      for (unsigned long i = 0; i < num_bytes; i++)
      {
        message.push_back(std::rand() & 0xFF);
      }
      auto encoded = codec.encodePacked(message);

      // Need to unpack for decoder
      ByteSymbolUtility::repack(encoded, ByteSymbolUtility::BPSymb_8, ByteSymbolUtility::BPSymb_1);

      if (truncated)
      {
        auto decoded = codec.decodeTruncated(encoded);
#if QA_VITERBI_DEBUG
        printf("lengths: message %ld encoded %ld decoded %ld\n",
               message.size(), encoded.size(), decoded.size());
#endif
      // Need to pack for comparison
        ByteSymbolUtility::repack(decoded, ByteSymbolUtility::BPSymb_1, ByteSymbolUtility::BPSymb_8);

        ASSERT_EQ(decoded, message);
      }
      else
      {
        auto decoded = codec.decode(encoded);
#if QA_VITERBI_DEBUG
        printf("lengths: message %ld encoded %ld decoded %ld\n",
               message.size(), encoded.size(), decoded.size());
#endif
      // Need to pack for comparison
        ByteSymbolUtility::repack(decoded, ByteSymbolUtility::BPSymb_1, ByteSymbolUtility::BPSymb_8);
        ASSERT_EQ(decoded, message);
      }
    }
  }
}

#if 0
TEST(Viterbi, Poly_7x5Packed)
{
  /* ----------------------------------------------------------------------
   * Confirm codec with constraint length 3 and polynomials 0b111 and 0b101
   * ----------------------------------------------------------------------
   */
  ViterbiCodec codec(3, {7, 5});
  TestViterbiCodecAutomaticPacked(codec, true);
}

TEST(Viterbi, Poly_6x5Packed)
{
  /* ----------------------------------------------------------------------
   * Confirm codec with constraint length 3 and polynomials 0b110 and 0b101
   * ----------------------------------------------------------------------
   */
  ViterbiCodec codec(3, {6, 5});
  TestViterbiCodecAutomaticPacked(codec, true);
}
#endif
TEST(Viterbi, VoyagerPacked)
{
  /* ----------------------------------------------------------------------
   * Confirm codec with constraint length 7 and polynomials 0b1101101, and
   * 0b1001111
   * ----------------------------------------------------------------------
   */
  ViterbiCodec codec(7, {109, 79});
  TestViterbiCodecAutomaticPacked(codec, true);
}
#if 0
TEST(Viterbi, LTEPacked)
{
  /* ----------------------------------------------------------------------
   * Confirm codec with constraint length 7 and polynomials 0b1011011,
   * 0b1110101, and 0b1111001
   * ----------------------------------------------------------------------
   */
  ViterbiCodec codec(7, {91, 117, 121});
  TestViterbiCodecAutomaticPacked(codec, false);
}

TEST(Viterbi, CDMA_2000Packed)
{
  /* ----------------------------------------------------------------------
   * Confirm codec with constraint length 9 and polynomials 0x1F5, 0x1B9,
   * 0x14B, and 0x13B
   * ----------------------------------------------------------------------
   */
  ViterbiCodec codec(9, {501, 441, 331, 315});
  TestViterbiCodecAutomaticPacked(codec, false);
}
#endif
