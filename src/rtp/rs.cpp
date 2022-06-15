/**
 * @file   rtp/rs.cpp
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#include "config_unix.h"
#include "config_win32.h"
#endif

#include <bitset>
#include <stdlib.h>

#include "debug.h"
#include "rtp/rs.h"
#include "rtp/rtp_callback.h"
#include "transmit.h"
#include "ug_runtime_error.hpp"
#include "video.h"

#define DEFAULT_K 200
#define DEFAULT_N 240

#define MAX_K 255
#define MAX_N 255

#ifdef HAVE_ZFEC
extern "C" {
#ifndef _MSC_VER
#define restrict __restrict
#endif
#include <fec.h>
}
#endif

static void usage();

using namespace std;

/**
 * Constructs RS state. Since this constructor is currently used only for the decoder,
 * it allows creation of dummy state even if zfec was not compiled in.
 */
rs::rs(unsigned int k, unsigned int n)
        : m_k(k), m_n(n)
{
        assert (k <= MAX_K);
        assert (n <= MAX_N);
        assert (m_k <= m_n);
#ifdef HAVE_ZFEC
        state = fec_new(m_k, m_n);
        assert(state != NULL);
#else
        LOG(LOG_LEVEL_ERROR) << "zfec support is not compiled in, error correction is disabled\n";
#endif
}

rs::rs(const char *c_cfg)
{
        if (strcmp(c_cfg, "help") == 0) {
                usage();
                throw 0;
        }
        char *cfg = strdup(c_cfg);
        char *item, *save_ptr;
        item = strtok_r(cfg, ":", &save_ptr);
        if (item != NULL) {
                m_k = atoi(item);
                item = strtok_r(NULL, ":", &save_ptr);
                assert(item != NULL);
                m_n = atoi(item);
        } else {
                m_k = DEFAULT_K;
                m_n = DEFAULT_N;
        }
        free(cfg);
        if (m_k > MAX_K || m_n > MAX_N || m_k >= m_n) {
                usage();
                throw 1;
        }

#ifdef HAVE_ZFEC
        state = fec_new(m_k, m_n);
        assert(state != NULL);
#else
        throw ug_runtime_error("zfec support is not compiled in");
#endif
}

rs::~rs()
{
#ifdef HAVE_ZFEC
        if (state != nullptr) {
                fec_free((fec_t *) state);
        }
#endif
}

shared_ptr<video_frame> rs::encode(shared_ptr<video_frame> in)
{
#ifdef HAVE_ZFEC
        assert(state != nullptr);

        video_payload_hdr_t hdr;
        format_video_header(in.get(), 0, 0, hdr);
        size_t hdr_len = sizeof(hdr);
        size_t len = in->tiles[0].data_len;
        char *data = in->tiles[0].data;

        struct video_frame *out = vf_alloc_desc(video_desc_from_frame(in.get()));
      
        //int encode(char *hdr, int hdr_len, char *in, int len, char **out) {
        int ss = get_ss(hdr_len, len);
        int buffer_len = ss * m_n;
        char *out_data;
        out_data = out->tiles[0].data = (char *) malloc(buffer_len);
        uint32_t len32 = len + hdr_len;
        memcpy(out_data, &len32, sizeof(len32));
        memcpy(out_data + sizeof(len32), hdr, hdr_len);
        memcpy(out_data + sizeof(len32) + hdr_len, data, len);
        memset(out_data + sizeof(len32) + hdr_len + len, 0, ss * m_k - (sizeof(len32) + hdr_len + len));

#if 0
        void *src[m_k];
        for (int k = 0; k < m_k; ++k) {
                src[k] = *out + ss * k;
        }

        for (int m = 0; m < m_n - m_k; ++m) {
                fec_encode(state, src, *out + ss * (m_k + m), m, ss);
        }
#else
        void *src[m_k];
        for (unsigned int k = 0; k < m_k; ++k) {
                src[k] = out_data + ss * k;
        }
        void *dst[m_n-m_k];
        unsigned int dst_idx[m_n-m_k];
        for (unsigned int m = 0; m < m_n-m_k; ++m) {
                dst[m] = out_data + ss * (m_k + m);
                dst_idx[m] = m_k + m;
        }

        fec_encode((const fec_t *)state, (gf **) src,
                        (gf **) dst, dst_idx, m_n-m_k, ss);
#endif

        out->tiles[0].data_len = buffer_len;
        out->fec_params = fec_desc(FEC_RS, m_k, m_n - m_k, 0, 0, ss);

        return {out,
                [](video_frame *frame) {
                        free(frame->tiles[0].data);
                        vf_free(frame);
                }
        };
#else
        (void) in;
        return {};
#endif // defined HAVE_ZFEC
}

audio_frame2 rs::encode(const audio_frame2 &in)
{
#ifdef HAVE_ZFEC
        audio_frame2 out;
        out.init(in.get_channel_count(), in.get_codec(), in.get_bps(), in.get_sample_rate());
        out.reserve(3 * in.get_data_len() / in.get_channel_count()); // just an estimate

        for (int i = 0; i < in.get_channel_count(); ++i) {
                audio_payload_hdr_t hdr;
                format_audio_header(&in, i, 0, (uint32_t *) &hdr);
                size_t hdr_len = sizeof(hdr);
                size_t len = in.get_data_len(i);
                uint32_t len32 = len + hdr_len;
                //const char *data = in->get_data(i);
                out.append(i, (char *) &len32, sizeof len32);
                out.append(i, (char *) &hdr, sizeof hdr);
                out.append(i, in.get_data(i), in.get_data_len(i));

                int ss = get_ss(hdr_len, len);
                int buffer_len = ss * m_n;
                out.resize(i, buffer_len);
                memset(out.get_data(i) + sizeof(len32) + hdr_len + len, 0, ss * m_k - (sizeof(len32) + hdr_len + len));

                out.set_fec_params(i, fec_desc(FEC_RS, m_k, m_n - m_k, 0, 0, ss));

                void *src[m_k];
                for (unsigned int k = 0; k < m_k; ++k) {
                        src[k] = out.get_data(i) + ss * k;
                }

                void *dst[m_n-m_k];
                unsigned int dst_idx[m_n-m_k];
                for (unsigned int m = 0; m < m_n-m_k; ++m) {
                        dst[m] = out.get_data(i) + ss * (m_k + m);
                        dst_idx[m] = m_k + m;
                }

                fec_encode((const fec_t *)state, (gf **) src,
                                (gf **) dst, dst_idx, m_n-m_k, ss);
        }

        return out;
#else
        (void) in;
        return {};
#endif // defined HAVE_ZFEC
}

/**
 * Returns symbol size (?) for given headers len and with configured m_k
 */
int rs::get_ss(int hdr_len, int len) {
        return ((sizeof(uint32_t) + hdr_len + len) + m_k - 1) / m_k;
}

/**
 * @returns stored buffer data length or 0 if first packet (header) is missing
 */
uint32_t rs::get_buf_len(const char *buf, std::map<int, int> const & c_m)
{
        if (auto it = c_m.find(0); it != c_m.end() && it->second >= 4) {
                uint32_t out_sz;
                memcpy(&out_sz, buf, sizeof(out_sz));
                return out_sz;
        }
        return 0U;
}

bool rs::decode(char *in, int in_len, char **out, int *len,
                std::map<int, int> const & c_m)
{
        std::map<int, int> m = c_m; // make private copy
        unsigned int ss = in_len / m_n;

        // compact neighbouring segments
        for (auto it = m.begin(); it != m.end(); ++it) {
                int start = it->first;
                int size = it->second;

                auto neighbour = m.end();
                while ((neighbour = m.find(start + size)) != m.end()) {
                        it->second += neighbour->second;
                        size = it->second;
                        m.erase(neighbour);
                }
        }

        if (state == nullptr) { // zfec was not compiled in - dummy mode
                *len = get_buf_len(in, c_m);
                *out = (char *) in + sizeof(uint32_t);
                auto fst_sgmt = m.find(0);
                return fst_sgmt != m.end() && (unsigned) fst_sgmt->second >= ss * m_k;
        }

#ifdef HAVE_ZFEC
        void *pkt[m_n];
        unsigned int index[m_n];
        unsigned int i = 0;
#if 0

        ///fprintf(stderr, "%d\n\n%d\n%d\n", in_len, malloc_usable_size((void *)in), sizeof(short));


        for (auto it = m.begin(); it != m.end(); ++it) {
                int start = it->first;
                int offset = it->second;

                int first_symbol_start = (start + ss - 1) / ss * ss;
                int last_symbol_end = (start + offset) / ss * ss;
                //fprintf(stderr, "%d %d %d\n", first_symbol_start, last_symbol_end, start);
                for (int j = first_symbol_start; j < last_symbol_end; j += ss) {
                        //fprintf(stderr, "%d\n", j);
                        pkt[i] = (void *) (in + j);
                        index[i] = j / ss;
                        i++;
                        if (i == m_k) break;
                }
                if (i == m_k) break;
        }

        if (i != m_k) {
                *len = 0;
                return;
        }

        assert (i == m_k);

        int ret = fec_decode(state, pkt, index, ss);
        if (ret != 0) {
                *len = 0;
                return;
        }
        uint32_t out_sz;
        memcpy(&out_sz,  pkt[0], sizeof(out_sz));
        fprintf(stderr, "%d %d\n\n", out_sz, index[0]);
        *len = out_sz;
        *out = (char *) in + 4;
#else
        //const unsigned int bitset_size = m_k;

        std::bitset<MAX_K> empty_slots;
        std::bitset<MAX_K> repaired_slots;

        for (auto it = m.begin(); it != m.end(); ++it) {
                int start = it->first;
                int size = it->second;

                unsigned int first_symbol_start = (start + ss - 1) / ss * ss;
                unsigned int last_symbol_end = (start + size) / ss * ss;
                for (unsigned int j = first_symbol_start; j < last_symbol_end; j += ss) {
                        if (j/ss < m_k) {
                                pkt[j/ss] = in + j;
                                index[j/ss] = j/ss;
                                empty_slots.set(j/ss);
                                //fprintf(stderr, "%d\n", j/ss);
                        } else {
                                for (unsigned int k = 0; k < m_k; ++k) {
                                        if (!empty_slots.test(k)) {
                                                pkt[k] = in + j;
                                                index[k] = j/ss;
                                                //fprintf(stderr, "%d\n", j/ss);
                                                empty_slots.set(k);
                                                repaired_slots.set(k);
                                                break;
                                        }
                                        //fprintf(stderr, "what???\n", j/ss);
                                }
                        }
                        i++;
                        //fprintf(stderr, " %d\n", i);
                        if (i == m_k) break;
                }
                if (i == m_k) break;
        }

        //fprintf(stderr, "       %d\n", i);

        if (i != m_k) {
                *len = get_buf_len(in, c_m);
                *out = (char *) in + sizeof(uint32_t);
                return false;
        }

        char **output = (char **) malloc(m_k * sizeof(char *));
        for (unsigned int i = 0; i < m_k; ++i) {
                output[i] = (char *) malloc(ss);
        }

        fec_decode((const fec_t *) state, (const gf *const *) pkt,
                        (gf *const *) output, index, ss);

        i = 0;
        for (unsigned int j = 0; j < m_k; ++j) {
                if (repaired_slots.test(j)) {
                        memcpy((void *) (in + j * ss), output[i], ss);
                        i++;
                }
        }

        for (unsigned int i = 0; i < m_k; ++i) {
                free(output[i]);
        }
        free(output);

        uint32_t out_sz;
        memcpy(&out_sz, in, sizeof(out_sz));
        //fprintf(stderr, "       %d\n", out_sz);
        *len = out_sz;
        *out = (char *) in + sizeof(uint32_t);
#endif
#endif // defined HAVE_ZFEC

        return true;
}

void rs::decodeAudio(FecChannel* channel) {
        fec_decode((const fec_t*) this->state, (gf**) channel->getRecoverySegments(), (gf**) channel->getOutputSegments(), channel->getRecoveryIndex(), channel->getSegmentSize());
}

void rs::initialiseChannel(FecChannel* channel, uint32_t fecHeader) {
    channel->setKBlocks(fecHeader >> 24);
    channel->setMBlocks((fecHeader >> 16) & 0XFF);
    channel->setSegmentSize(fecHeader & 0XFFFF);
    channel->initialise();
}

FecChannel::FecChannel() : outputSize(0), outputCreated(false), initialised(false) {}

FecChannel::FecChannel(int kBlocks, int mBlocks, int segmentSize) : kBlocks(kBlocks), mBlocks(mBlocks), segmentSize(segmentSize), outputSize(0), outputCreated(false) {
    // Initialise the channel
    this->initialise();
}

void FecChannel::initialise() {
    this->initialised = true;
    this->blockDelta = this->mBlocks - this->kBlocks;
    // Allocate enough pointers to store an pointer to every channel segment
    this->segments = (char**)calloc(this->kBlocks, sizeof(char*));
    for(int i = 0; i < this->kBlocks; i++) {
        this->segments[i] = (char *)calloc(this->segmentSize, sizeof(char));
    }
    this->segmentIndexes = (unsigned int*)calloc(this->kBlocks, sizeof(unsigned int));
    // Allocate enough pointers to store a pointer to every parity segment
    this->paritySegments = (char**)calloc(this->blockDelta, sizeof(char*));
    for(int i = 0; i < this->blockDelta; i++) {
        this->paritySegments[i] = (char *)calloc(this->segmentSize, sizeof(char));
    }
    this->parityIndexes = (unsigned int*)calloc(this->blockDelta, sizeof(unsigned int));
    // Allocate the memory for the recovery segments, recovery index, and output segments
    this->recoverySegments = (char**)calloc(this->kBlocks, sizeof(char*));
    this->recoveryIndex = (unsigned int*)calloc(this->kBlocks, sizeof(unsigned int));
}

FecChannel::~FecChannel() {
    if(this->initialised) {
        // Free the resources we allocated for pointing at
        // the segments and indexes
        free(this->segments);
        for(int i = 0; i < this->kBlocks; i++) {
            free(this->segments[i]);
        }
        free(this->segmentIndexes);
        free(this->paritySegments);
        for(int i = 0; i < this->blockDelta; i++) {
            free(this->paritySegments[i]);
        }
        free(this->parityIndexes);
        free(this->recoverySegments);
        free(this->recoveryIndex);
        if(this->outputCreated) {
            for(int i = 0; i < this->outputSize; i++) {
                free(this->outputSegments[i]);
            }
            free(this->outputSegments);
        }
    }
}

void FecChannel::addBlock(char* data, size_t dataSize, size_t offset) {
    LOG(LOG_LEVEL_VERBOSE) << "Size " << dataSize << " Offset " << offset;
    // Calculate the number of segments given in the block of data
    int segments = dataSize / this->segmentSize;
    // Calculate the initial index of the data
    int initialIndex = offset / this->segmentSize;
    // Insert the indexes, and segments in
    for(int i = 0; i < segments; i++) {
        if((initialIndex + i) < this->kBlocks) {
            // Calculate the new index (as this is a data segment)
            int newIndex = initialIndex + i;
            this->segments[newIndex] = data + (this->segmentSize * i);
            this->segmentIndexes[newIndex] = initialIndex + i;
        }
        else {
            // Calculate the new index (as this is a parity segment)
            int newIndex = (initialIndex + i) - this->kBlocks;
            this->paritySegments[newIndex] = data + (this->segmentSize * i);
            this->parityIndexes[newIndex] = initialIndex + i;
        }
    }
}

void FecChannel::addBlockCopy(char* data, size_t dataSize, size_t offset) {
    LOG(LOG_LEVEL_VERBOSE) << "Size " << dataSize << " Offset " << offset;
    // Calculate the number of segments given in the block of data
    int segments = dataSize / this->segmentSize;
    // Calculate the initial index of the data
    int initialIndex = offset / this->segmentSize;
    // Insert the indexes, and segments in
    for(int i = 0; i < segments; i++) {
        if((initialIndex + i) < this->kBlocks) {
            // Calculate the new index (as this is a data segment)
            int newIndex = initialIndex + i;
            // Instead of taking a reference, allocate the memory and copy it in.
            memcpy(this->segments[newIndex], data + (this->segmentSize * i), this->segmentSize);
            this->segmentIndexes[newIndex] = initialIndex + i;
        }
        else {
            // Calculate the new index (as this is a parity segment)
            int newIndex = (initialIndex + i) - this->kBlocks;
            memcpy(this->paritySegments[newIndex], data + (this->segmentSize * i), this->segmentSize);
            this->parityIndexes[newIndex] = initialIndex + i;
        }
    }
}

FecRecoveryState FecChannel::generateRecovery() {
    // Keep track of how many parity segments require usage
    int parityCounter = 0;
    for(int i = 0; i < this->kBlocks; i++) {
        // Check if the segment is NULL or not (calloc sets all pointers to be zero / null)
        if(this->segments[i]) {
            this->recoverySegments[i] = this->segments[i];
            this->recoveryIndex[i] = i;
        }
        // Since the segment has not been filled in, we replace it with a parity segment
        else {
            // Track whether or not the parity segment is set. If not, then there are not
            // enough segments to recover the data
            bool setParity = false;
            do {
                // Test to see if the parity segment has been set
                if(this->paritySegments[parityCounter]) {
                    this->recoverySegments[i] = this->paritySegments[parityCounter];
                    this->recoveryIndex[i] = this->parityIndexes[parityCounter];
                    
                    // Increment the counter, so we know we have used a parity segment
                    // when we look at the final outcome
                    parityCounter++;
                    setParity = true;
                    break;
                }
                else {
                    parityCounter++;
                }
            } while(parityCounter < blockDelta);

            // If we were unable to set the parity segment then there are not enough segments set
            // to recover the data
            if(!setParity) {
                return FecRecoveryState::FEC_UNRECOVERABLE;
            }
        }
    }

    // If no parity segments have been used then the recovery segments will be the complete the data
    if(parityCounter == 0) {
        return FecRecoveryState::FEC_COMPLETE;
    }
    // If we have used any parity segments, and have reached this far, then it is possible for the data
    // to be recovered.
    else {
        // Since the data is recoverable there should be a need to generate additional output buffers.
        this->outputSegments = (char**) calloc(parityCounter, sizeof(char*));
        for(int i = 0; i < parityCounter; i++) {
            this->outputSegments[i] = (char*) calloc(this->segmentSize, sizeof(char));
        }
        this->outputSize = parityCounter;
        this->outputCreated = true;

        return FecRecoveryState::FEC_RECOVERABLE;
    }
}

void FecChannel::recover() {
    // Check that the output has been created
    if(outputCreated) {
        int outputIndex = 0;
        for(int i = 0; i < this->kBlocks; i++) {
            // Check to see if the original data was in the index or not
            if(this->segmentIndexes[i] != i) {
                // This should not happen if the output has been created as it is
                // sized to match the number of indexes which do not match their index. 
                if(outputIndex >= this->outputSize) {
                    return;
                }
                // Update the segments (and the indexes) to match the latest!
                else {
                    this->segments[i] = this->outputSegments[outputIndex];
                    this->segmentIndexes[i] = i;
                    outputIndex++;
                }
            }
        }


        // The output buffer is set to be the size it needs to be, so in order
        // to recover all of the output buffer needs to be used
        for(int i = 0; i < this->outputSize; i++) {
        
        }
    }
}

char** FecChannel::getRecoverySegments() {
    return this->recoverySegments;
}

unsigned int* FecChannel::getRecoveryIndex() {
    return this->recoveryIndex;
}

char** FecChannel::getOutputSegments() {
    return this->outputSegments;
}

void FecChannel::setKBlocks(int kBlocks) {
    this->kBlocks = kBlocks;
}

int FecChannel::getKBlocks() {
    return this->kBlocks;
}

void FecChannel::setMBlocks(int mBlocks) {
    this->mBlocks = mBlocks;
}

int FecChannel::getMBlocks() {
    return this->mBlocks;
}

char* FecChannel::operator[](std::size_t index) {
    // If the index is within the kBlocks then, it's in
    // a segment.
    if(index < this->kBlocks) {
        return this->segments[index];
    }
    // If it's outside of the kBlocks then it's likely within the
    // parity segments.
    else {
        return this->paritySegments[index - this->kBlocks];
    }
}

const char* FecChannel::operator[](std::size_t index) const {
    if(index < this->kBlocks) {
        return this->segments[index];
    }
    else {
        return this->paritySegments[index - this->kBlocks];
    }
}

char* FecChannel::getSegment(int index) {
    // Use the operator for getting a segment from the class
    return (*this)[index];
}

int FecChannel::getSegmentSize() {
    return this->segmentSize;
}

void FecChannel::setSegmentSize(int segmentSize) {
    this->segmentSize = segmentSize;
}

static void usage() {
        printf("RS usage:\n"
                        "\t-f rs[:<k>:<n>]\n"
                        "\n"
                        "\t\t<k> - block length (default %d, max %d)\n"
                        "\t\t<n> - length of block + parity (default %d, max %d)\n\t\t\tmust be > <k>\n"
                        "\n",
                        DEFAULT_K, MAX_K, DEFAULT_N, MAX_N);
}

