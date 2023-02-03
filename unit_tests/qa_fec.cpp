/*!
 * @file qa_fec.cpp
 * @author Steven Knudsen
 * @date July 15, 2021
 *
 * @details Unit test for the FEC class. Since this is an abstract class, we simply use a concrete 
 * example to exercise FEC-specific things
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

#include "no_fec.hpp"
//#include "byte_symbol_utils.hpp"

using namespace std;
using namespace ex2::error_control;

#include "gtest/gtest.h"

#define QA_FEC_DEBUG 0 // set to 1 for debugging output

/*!
 * @brief Test Constructor and no FEC encode and decode
 */
TEST(noFEC, ConstructorAndEncodeDecode )
{

  /* ---------------------------------------------------------------------
   * Confirm the NoFEC object can be constructed
   * ---------------------------------------------------------------------
   */

  // Set the length of the test packet so it all fits into a transparent mode payload
  const unsigned long int testPacketLength = 10; // bytes

  FEC * noFEC = new NoFEC(ErrorCorrection::ErrorCorrectionScheme::NO_FEC, testPacketLength*8);

  ASSERT_TRUE(noFEC != NULL) << "NoFEC failed to instantiate";

  if (noFEC) {
    std::vector<uint8_t> packet;

    for (unsigned long i = 0; i < testPacketLength; i++) {
      packet.push_back( i | 0x30 ); // ASCII numbers
    }

#if QA_FEC_DEBUG
    printf("packet length = %d\n", packet.size());
    // Look at the contents :-)
    for (int i = 0; i < packet.size(); i++) {acket
      printf("p[%d] = 0x%02x\n", i, packet[i]);
    }
#endif

    // Technically we should be unpacking the packet to make it 1 bit per byte
    // as per the FEC interface. So that there is an example, we will do it.
//    ByteSymbolUtility::repack(packet, ByteSymbolUtility::BPSymb_8, ByteSymbolUtility::BPSymb_1);

    std::vector<uint8_t> encodedPayload = noFEC->encode(packet);

    bool same = true;
    for (unsigned long i = 0; i < packet.size(); i++) {
      same = same & (packet[i] == encodedPayload[i]);
    }
    ASSERT_TRUE(same) << "encoded payload does not match input payload";

    std::vector<uint8_t> dPayload;
    uint32_t bitErrors = noFEC->decode(encodedPayload, 100.0, dPayload);

    same = true;
    for (unsigned long i = 0; i < packet.size(); i++) {
      same = same & (packet[i] == dPayload[i]);
    }

    // Let's repack the decoded payload so that it's in the same format as the original packet
    // just so there is an example of how to do it
//    ByteSymbolUtility::repack(dPayload, ByteSymbolUtility::BPSymb_1, ByteSymbolUtility::BPSymb_8);

    ASSERT_TRUE(same) << "decoded payload does not match input payload";
    ASSERT_TRUE(bitErrors == 0) << "Bit error count > 0";

    delete(noFEC);
  }

}

