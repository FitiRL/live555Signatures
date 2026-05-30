/**********
This library is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the
Free Software Foundation; either version 3 of the License, or (at your
option) any later version. (See <http://www.gnu.org/copyleft/lesser.html>.)

This library is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License
along with this library; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
**********/
// "liveMedia"
// Copyright (c) 1996-2026 Live Networks, Inc.  All rights reserved.
// RTP sink for H.264 or H.265 video
// Implementation

#include "H264or5VideoRTPSink.hh"
#include "H264or5VideoStreamFramer.hh"
#include <stdint.h>
#include <stdio.h>

// If hardware acceleration is enabled, the signature is done in the hardware
// (fpga core by using the specific driver).
// #define HW_ACCELERATION 1

#ifdef HW_ACCELERATION
#define CONTEXT_LENGTH 16
static uint8_t context[CONTEXT_LENGTH] = {0}; // Initialize the context data
#endif

// Checksum can be calculated every packet or only for keyframe.
SignatureMode g_SignatureMode = SIGNATURE_EVERY_PACKET;

// This function is just an example of checksum calculation. The checksum is
// 
int computeSignature(const unsigned char* data, unsigned size, unsigned char* signature) {
  #ifdef HW_ACCELERATION
  pqb_error_code_t result = wait_for_hardware_ready(hif);
  if (result != PQB_SUCCESS) {
    return 0;
  }

  uint32_t* signature;
  result = pqb_dsa_sign_mldsa_44(hif, signature, data, size, context, 
    CONTEXT_LENGTH);
  if (result != PQB_SUCCESS) {
    PQB_PRINTF("ERROR: Signing failed with status %d (%s)\n", result, 
      pqb_core_error_string(result));
    return -1;
  }

  return 0;
  #else
  // If not accelerated via hardware, the signature is just an example (checksum).
  for (unsigned i = 0; i < FRAME_SIGNATURE_SIZE; ++i) {
    signature[i] = 0x33; // DEBUG ONLY
  }
  return 0;
  #endif
}

////////// H264or5Fragmenter definition //////////

// Because of the ideosyncracies of the H.264 RTP payload format, we implement
// "H264or5VideoRTPSink" using a separate "H264or5Fragmenter" class that delivers,
// to the "H264or5VideoRTPSink", only fragments that will fit within an outgoing
// RTP packet.  I.e., we implement fragmentation in this separate "H264or5Fragmenter"
// class, rather than in "H264or5VideoRTPSink".
// (Note: This class should be used only by "H264or5VideoRTPSink", or a subclass.)

class H264or5Fragmenter: public FramedFilter {
public:
  H264or5Fragmenter(int hNumber, UsageEnvironment& env, FramedSource* inputSource,
		    unsigned inputBufferMax, unsigned maxOutputPacketSize);
  virtual ~H264or5Fragmenter();

  Boolean lastFragmentCompletedNALUnit() const { return fLastFragmentCompletedNALUnit; }
  uint8_t* nalSignature() { return fNALSignature; }

private: // redefined virtual functions:
  virtual void doGetNextFrame();
  virtual void doStopGettingFrames();

private:
  static void afterGettingFrame(void* clientData, unsigned frameSize,
				unsigned numTruncatedBytes,
                                struct timeval presentationTime,
                                unsigned durationInMicroseconds);
  void afterGettingFrame1(unsigned frameSize,
                          unsigned numTruncatedBytes,
                          struct timeval presentationTime,
                          unsigned durationInMicroseconds);
  void reset();

private:
  int fHNumber;
  unsigned fInputBufferSize;
  unsigned fMaxOutputPacketSize;
  unsigned char* fInputBuffer;
  unsigned fNumValidDataBytes;
  unsigned fCurDataOffset;
  unsigned fSaveNumTruncatedBytes;
  Boolean fLastFragmentCompletedNALUnit;
  uint8_t fNALSignature[FRAME_SIGNATURE_SIZE]; // signature of the full NAL unit, computed before fragmentation
};


////////// H264or5VideoRTPSink implementation //////////

H264or5VideoRTPSink
::H264or5VideoRTPSink(int hNumber,
		      UsageEnvironment& env, Groupsock* RTPgs, unsigned char rtpPayloadFormat,
		      u_int8_t const* vps, unsigned vpsSize,
		      u_int8_t const* sps, unsigned spsSize,
		      u_int8_t const* pps, unsigned ppsSize)
  : VideoRTPSink(env, RTPgs, rtpPayloadFormat, 90000, hNumber == 264 ? "H264" : "H265"),
    fHNumber(hNumber), fOurFragmenter(NULL), fFmtpSDPLine(NULL) {
  if (vps != NULL) {
    fVPSSize = vpsSize;
    fVPS = new u_int8_t[fVPSSize];
    memmove(fVPS, vps, fVPSSize);
  } else {
    fVPSSize = 0;
    fVPS = NULL;
  }
  if (sps != NULL) {
    fSPSSize = spsSize;
    fSPS = new u_int8_t[fSPSSize];
    memmove(fSPS, sps, fSPSSize);
  } else {
    fSPSSize = 0;
    fSPS = NULL;
  }
  if (pps != NULL) {
    fPPSSize = ppsSize;
    fPPS = new u_int8_t[fPPSSize];
    memmove(fPPS, pps, fPPSSize);
  } else {
    fPPSSize = 0;
    fPPS = NULL;
  }
}

H264or5VideoRTPSink::~H264or5VideoRTPSink() {
  fSource = fOurFragmenter; // hack: in case "fSource" had gotten set to NULL before we were called
  delete[] fFmtpSDPLine;
  delete[] fVPS; delete[] fSPS; delete[] fPPS;
  stopPlaying(); // call this now, because we won't have our 'fragmenter' when the base class destructor calls it later.

  // Close our 'fragmenter' as well:
  Medium::close(fOurFragmenter);
  fSource = NULL; // for the base class destructor, which gets called next
}

Boolean H264or5VideoRTPSink::continuePlaying() {
  // First, check whether we have a 'fragmenter' class set up yet.
  // If not, create it now:
  if (fOurFragmenter == NULL) {
    fOurFragmenter = new H264or5Fragmenter(fHNumber, envir(), fSource, OutPacketBuffer::maxSize,
					   ourMaxPacketSize() - 12/*RTP hdr size*/ - specialHeaderSize());
  } else {
    fOurFragmenter->reassignInputSource(fSource);
  }
  fSource = fOurFragmenter;

  // Then call the parent class's implementation:
  return MultiFramedRTPSink::continuePlaying();
}

int pendingSignatureNParts = 0; // keep the count of how many parts the signature is split
uint8_t pendingSignatureData[2424] = {0};

unsigned H264or5VideoRTPSink::specialHeaderSize() const {
  if (pendingSignatureNParts > 0) {
    return 820; // Size of the header size required. Padding is applied.
  } else {
    return 0;
  }
}

// TODO: Improve this to manage the size dynamically. We use 4 parts by now, fixed.
struct signaturePacket {
  uint8_t hdr1[2];
  uint8_t seg1[202];
  uint8_t hdr2[2];
  uint8_t seg2[202];
  uint8_t hdr3[2];
  uint8_t seg3[202];
  uint8_t hdr4[2];
  uint8_t seg4[202];
};

void H264or5VideoRTPSink::doSpecialFrameHandling(unsigned /*fragmentationOffset*/,
						 unsigned char* frameStart,
						 unsigned numBytesInFrame,
						 struct timeval framePresentationTime,
						 unsigned /*numRemainingBytes*/) {
  if (fOurFragmenter != NULL) {
    H264or5VideoStreamFramer* framerSource
      = (H264or5VideoStreamFramer*)(fOurFragmenter->inputSource());

    // ---- Keyframe detection ----
    // H.264: IDR = NAL type 5.  FU-A = NAL type 28; underlying type in [1]&0x1F.
    // H.265: IDR_W_RADL/IDR_N_LP = types 19/20.  FU = type 49; underlying in [2]&0x3F.
    bool isKeyframe = false;
    uint8_t rawNalType = 0;
    if (numBytesInFrame >= 1) {
      if (fHNumber == 264) {
        rawNalType = frameStart[0] & 0x1F;
        if (rawNalType == 5) {
          isKeyframe = true;
        } else if (rawNalType == 28 && numBytesInFrame >= 2) {
          uint8_t fuUnderlying = frameStart[1] & 0x1F;
          rawNalType = fuUnderlying;
          isKeyframe = (fuUnderlying == 5);
        }
      } else { // H.265
        rawNalType = (frameStart[0] & 0x7E) >> 1;
        if (rawNalType == 19 || rawNalType == 20) {
          isKeyframe = true;
        } else if (rawNalType == 49 && numBytesInFrame >= 3) {
          uint8_t fuUnderlying = frameStart[2] & 0x3F;
          rawNalType = fuUnderlying;
          isKeyframe = (fuUnderlying == 19 || fuUnderlying == 20);
        }
      }
    }

    // ---- Checksum from the fragmenter (computed over the full NAL unit) ----
    // All RTP fragments of the same NAL unit share the same value because it
    // was computed once in afterGettingFrame1() before any fragmentation.
    H264or5Fragmenter* frag = (H264or5Fragmenter*)fOurFragmenter;
    bool isLastFragment = frag->lastFragmentCompletedNALUnit();

    if (isKeyframe && isLastFragment) {
      // First packet that contains the signature
      pendingSignatureNParts = 3;

      memcpy(pendingSignatureData, frag->nalSignature(), sizeof(pendingSignatureData));
      #ifdef DEBUG
      for(int i=0; i<2420; ++i) {
        printf("%x", pendingSignatureData[i]);
      }
      printf("\n");
      #endif

    } else {
      if (pendingSignatureNParts > 0) {
        pendingSignatureNParts--;
        // ---- Write RTP header extension ----
        // The 8-byte slot is always reserved by specialHeaderSize(); we always
        // fill it so the H.264 payload lands at the correct offset.
        setExtensionBit();

        // Extension header: profile=0x1001 (our custom tag), length=1 (one 32-bit word)
        const unsigned char extHdr[] = { 0x10, 0x01, 0x00, 0xcc };
        setSpecialHeaderBytes(extHdr, 4, 0);

        struct signaturePacket pk1 = {0};
        
        pk1.hdr4[0]=0x01;pk1.hdr4[1]=0xca;  memcpy(&pk1.seg4, pendingSignatureData + (sizeof(pk1.seg4)*3) + (pendingSignatureNParts*sizeof(pk1.seg4)*4), sizeof(pk1.seg4));
        pk1.hdr3[0]=0x01;pk1.hdr3[1]=0xca;  memcpy(&pk1.seg3, pendingSignatureData + (sizeof(pk1.seg4)*2) + (pendingSignatureNParts*sizeof(pk1.seg4)*4), sizeof(pk1.seg4));
        pk1.hdr2[0]=0x01;pk1.hdr2[1]=0xca;  memcpy(&pk1.seg2, pendingSignatureData + (sizeof(pk1.seg4)*1) + (pendingSignatureNParts*sizeof(pk1.seg4)*4), sizeof(pk1.seg4));
        pk1.hdr1[0]=0x01;pk1.hdr1[1]=0xca;  memcpy(&pk1.seg1, pendingSignatureData + (sizeof(pk1.seg4)*0) + (pendingSignatureNParts*sizeof(pk1.seg4)*4), sizeof(pk1.seg4));

        setSpecialHeaderBytes((unsigned char*)&pk1, sizeof(pk1), 4);

        #ifdef DEBUG
        fprintf(stdout, "[RTP SEND] nalType=%-2u  %-8s  lastFrag=%d  bytes=%-5u \n",
                rawNalType,
                isKeyframe ? "[IDR]" : "",
                isLastFragment ? 1 : 0,
                numBytesInFrame
                );
        fflush(stdout);
        #endif
      }
    }

  if (((H264or5Fragmenter*)fOurFragmenter)->lastFragmentCompletedNALUnit()
	&& framerSource != NULL && framerSource->pictureEndMarker()) {
      setMarkerBit();
      framerSource->pictureEndMarker() = False;
    }
  }

  setTimestamp(framePresentationTime);
}

Boolean H264or5VideoRTPSink
::frameCanAppearAfterPacketStart(unsigned char const* /*frameStart*/,
				 unsigned /*numBytesInFrame*/) const {
  return False;
}


////////// H264or5Fragmenter implementation //////////

H264or5Fragmenter::H264or5Fragmenter(int hNumber,
				     UsageEnvironment& env, FramedSource* inputSource,
				     unsigned inputBufferMax, unsigned maxOutputPacketSize)
  : FramedFilter(env, inputSource),
    fHNumber(hNumber),
    fInputBufferSize(inputBufferMax+1), fMaxOutputPacketSize(maxOutputPacketSize) {
  fInputBuffer = new unsigned char[fInputBufferSize];
  reset();
}

H264or5Fragmenter::~H264or5Fragmenter() {
  delete[] fInputBuffer;
  detachInputSource(); // so that the subsequent ~FramedFilter() doesn't delete it
}

void H264or5Fragmenter::doGetNextFrame() {
  if (fNumValidDataBytes == 1) {
    // We have no NAL unit data currently in the buffer.  Read a new one:
    fInputSource->getNextFrame(&fInputBuffer[1], fInputBufferSize - 1,
			       afterGettingFrame, this,
			       FramedSource::handleClosure, this);
  } else {
    // We have NAL unit data in the buffer.  There are three cases to consider:
    // 1. There is a new NAL unit in the buffer, and it's small enough to deliver
    //    to the RTP sink (as is).
    // 2. There is a new NAL unit in the buffer, but it's too large to deliver to
    //    the RTP sink in its entirety.  Deliver the first fragment of this data,
    //    as a FU packet, with one extra preceding header byte (for the "FU header").
    // 3. There is a NAL unit in the buffer, and we've already delivered some
    //    fragment(s) of this.  Deliver the next fragment of this data,
    //    as a FU packet, with two (H.264) or three (H.265) extra preceding header bytes
    //    (for the "NAL header" and the "FU header").

    if (fMaxSize < fMaxOutputPacketSize) { // shouldn't happen
      envir() << "H264or5Fragmenter::doGetNextFrame(): fMaxSize ("
	      << fMaxSize << ") is smaller than expected\n";
    } else {
      fMaxSize = fMaxOutputPacketSize;
    }

    fLastFragmentCompletedNALUnit = True; // by default
    if (fCurDataOffset == 1) { // case 1 or 2
      if (fNumValidDataBytes - 1 <= fMaxSize) { // case 1
	memmove(fTo, &fInputBuffer[1], fNumValidDataBytes - 1);
	fFrameSize = fNumValidDataBytes - 1;
	fCurDataOffset = fNumValidDataBytes;
      } else { // case 2
	// We need to send the NAL unit data as FU packets.  Deliver the first
	// packet now.  Note that we add "NAL header" and "FU header" bytes to the front
	// of the packet (overwriting the existing "NAL header").
	if (fHNumber == 264) {
	  fInputBuffer[0] = (fInputBuffer[1] & 0xE0) | 28; // FU indicator
	  fInputBuffer[1] = 0x80 | (fInputBuffer[1] & 0x1F); // FU header (with S bit)
	} else { // 265
	  u_int8_t nal_unit_type = (fInputBuffer[1]&0x7E)>>1;
	  fInputBuffer[0] = (fInputBuffer[1] & 0x81) | (49<<1); // Payload header (1st byte)
	  fInputBuffer[1] = fInputBuffer[2]; // Payload header (2nd byte)
	  fInputBuffer[2] = 0x80 | nal_unit_type; // FU header (with S bit)
	}
	memmove(fTo, fInputBuffer, fMaxSize);
	fFrameSize = fMaxSize;
	fCurDataOffset += fMaxSize - 1;
	fLastFragmentCompletedNALUnit = False;
      }
    } else { // case 3
      // We are sending this NAL unit data as FU packets.  We've already sent the
      // first packet (fragment).  Now, send the next fragment.  Note that we add
      // "NAL header" and "FU header" bytes to the front.  (We reuse these bytes that
      // we already sent for the first fragment, but clear the S bit, and add the E
      // bit if this is the last fragment.)
      unsigned numExtraHeaderBytes;
      if (fHNumber == 264) {
	fInputBuffer[fCurDataOffset-2] = fInputBuffer[0]; // FU indicator
	fInputBuffer[fCurDataOffset-1] = fInputBuffer[1]&~0x80; // FU header (no S bit)
	numExtraHeaderBytes = 2;
      } else { // 265
	fInputBuffer[fCurDataOffset-3] = fInputBuffer[0]; // Payload header (1st byte)
	fInputBuffer[fCurDataOffset-2] = fInputBuffer[1]; // Payload header (2nd byte)
	fInputBuffer[fCurDataOffset-1] = fInputBuffer[2]&~0x80; // FU header (no S bit)
	numExtraHeaderBytes = 3;
      }
      unsigned numBytesToSend = numExtraHeaderBytes + (fNumValidDataBytes - fCurDataOffset);
      if (numBytesToSend > fMaxSize) {
	// We can't send all of the remaining data this time:
	numBytesToSend = fMaxSize;
	fLastFragmentCompletedNALUnit = False;
      } else {
	// This is the last fragment:
	fInputBuffer[fCurDataOffset-1] |= 0x40; // set the E bit in the FU header
	fNumTruncatedBytes = fSaveNumTruncatedBytes;
      }
      memmove(fTo, &fInputBuffer[fCurDataOffset-numExtraHeaderBytes], numBytesToSend);
      fFrameSize = numBytesToSend;
      fCurDataOffset += numBytesToSend - numExtraHeaderBytes;
    }

    if (fCurDataOffset >= fNumValidDataBytes) {
      // We're done with this data.  Reset the pointers for receiving new data:
      fNumValidDataBytes = fCurDataOffset = 1;
    }

    // Complete delivery to the client:
    FramedSource::afterGetting(this);
  }
}

void H264or5Fragmenter::doStopGettingFrames() {
  // Make sure that we don't have any stale data fragments lying around, should we later resume:
  reset();
  FramedFilter::doStopGettingFrames();
}

void H264or5Fragmenter::afterGettingFrame(void* clientData, unsigned frameSize,
					  unsigned numTruncatedBytes,
					  struct timeval presentationTime,
					  unsigned durationInMicroseconds) {
  H264or5Fragmenter* fragmenter = (H264or5Fragmenter*)clientData;
  fragmenter->afterGettingFrame1(frameSize, numTruncatedBytes, presentationTime,
				 durationInMicroseconds);
}

void H264or5Fragmenter::afterGettingFrame1(unsigned frameSize,
					   unsigned numTruncatedBytes,
					   struct timeval presentationTime,
					   unsigned durationInMicroseconds) {
  fNumValidDataBytes += frameSize;
  fSaveNumTruncatedBytes = numTruncatedBytes;
  fPresentationTime = presentationTime;
  fDurationInMicroseconds = durationInMicroseconds;

  // Compute signature over the *complete* NAL unit now, before it gets
  // fragmented.  fInputBuffer[1] is the first byte of the NAL unit;
  // frameSize is the number of valid NAL bytes.
  computeSignature(&fInputBuffer[1], frameSize, fNALSignature);

  // Deliver data to the client:
  doGetNextFrame();
}

void H264or5Fragmenter::reset() {
  fNumValidDataBytes = fCurDataOffset = 1;
  fSaveNumTruncatedBytes = 0;
  fLastFragmentCompletedNALUnit = True;
}
