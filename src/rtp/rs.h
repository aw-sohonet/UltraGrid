/**
 * @file   rtp/rs.h
 * @author Martin Pulec     <pulec@cesnet.cz>
 */
/*
 * Copyright (c) 2013-2021 CESNET, z. s. p. o.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, is permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of CESNET nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESSED OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __RS_H__
#define __RS_H__

#include <cstdint>
#include <map>
#include <memory>

#include "fec.h"

struct video_frame;

class FecChannel;

struct rs : public fec {
        rs(unsigned int k, unsigned int n);
        rs(const char *cfg);
        virtual ~rs();
        std::shared_ptr<video_frame> encode(std::shared_ptr<video_frame> frame) override;
        virtual audio_frame2 encode(audio_frame2 const &) override;
        bool decode(char *in, int in_len, char **out, int *len,
                const std::map<int, int> &) override;
        void decodeAudio(FecChannel* channel);
        static void initialiseChannel(FecChannel* channel, uint32_t fecHeader);

private:
        int get_ss(int hdr_len, int len);
        uint32_t get_buf_len(const char *buf, std::map<int, int> const & c_m);
        void *state = nullptr;
        /** param k the number of blocks required to reconstruct
            param m the total number of blocks created
            Currently both are limited to a max of 256 */
        unsigned int m_k, m_n;
};

enum FecRecoveryState {
    FEC_COMPLETE,
    FEC_RECOVERABLE,
    FEC_UNRECOVERABLE
};

class FecChannel {
public:
        FecChannel();
        FecChannel(int kBlocks, int mBlocks, int segmentSize);
        ~FecChannel();
        void initialise();
        void addBlock(char* data, size_t dataSize, size_t offset);
        void addBlockCopy(char* data, size_t dataSize, size_t offset);
        FecRecoveryState generateRecovery();
        void recover();
        // Provide functions for access into the class
        char* getSegment(int index);
        char* operator[](std::size_t index);
        const char* operator[](std::size_t index) const;

        char** getRecoverySegments();
        char** getOutputSegments();
        unsigned int* getRecoveryIndex();
        void setSegmentSize(int segmentSize);
        int getSegmentSize();
        void setKBlocks(int kBlocks);
        int getKBlocks();
        void setMBlocks(int mBlocks);
        int getMBlocks();
        bool getInitialised();
private:
        bool initialised;
        /**
         *  K is the number of blocks required to reconstruct
         *  M is the total number of blocks created
         */
        int kBlocks;
        int mBlocks;
        // We want to know how many parity segments to expect.
        // The block delta also lets us know how many segments
        // can contain errors in order to recover the data
        int blockDelta;

        // This is the byte size of each segment
        int segmentSize;

        // In order to reconstruct the channel data we need an array
        // of all of the segments that we received. The segment indexes
        // represent what the index of the segement is in the overall
        // buffer
        char** segments;
        char** paritySegments;
        unsigned int* segmentIndexes;
        unsigned int* parityIndexes;
        // It's nice to have a seperate list for constructing the decoding indexes
        char** recoverySegments;
        unsigned int* recoveryIndex;

        char** outputSegments;
        int outputSize;
        bool outputCreated;
};

#endif /* __RS_H__ */

