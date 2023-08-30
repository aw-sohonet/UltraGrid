//
// Created by sohonet on 8/29/23.
//

#ifndef ULTRAGRID_CMPTO_J2K_H
#define ULTRAGRID_CMPTO_J2K_H

#include <cmpto_j2k_dec.h>

#include "debug.h"
#include "host.h"
#include "lib_common.h"
#include "utils/misc.h"
#include "utils/synchronized_queue.h"
#include "video.h"
#include "video_decompress.h"

#include <mutex>
#include <queue>
#include <utility>

struct state_decompress_j2k {
    state_decompress_j2k(unsigned int mqs, unsigned int mif) : max_queue_size(mqs), max_in_frames(mif) {}
    ~state_decompress_j2k() {
        cmpto_j2k_dec_ctx_stop(this->decoder);
        pthread_join(this->thread_id, NULL);
        log_msg(LOG_LEVEL_VERBOSE, "[J2K dec.] Decoder stopped.\n");

        cmpto_j2k_dec_cfg_destroy(this->settings);
        cmpto_j2k_dec_ctx_destroy(this->decoder);

        while (this->decompressed_frames.size() > 0) {
            auto decoded = this->decompressed_frames.pop();
            free(decoded.first);
        }

        // Push a poison pill
        this->decompressed_frames.push(std::make_pair<char*, int>(nullptr, 0));
    }

    cmpto_j2k_dec_ctx *decoder{};
    cmpto_j2k_dec_cfg *settings{};

    struct video_desc desc{};
    codec_t out_codec{};

    std::mutex lock;

    synchronized_queue<std::pair<char *, size_t>> decompressed_frames; ///< buffer, length
    int pitch;
    pthread_t thread_id{};
    unsigned int max_queue_size; ///< maximal length of @ref decompressed_frames
    unsigned int max_in_frames; ///< maximal frames that can be "in progress"
    unsigned int in_frames{}; ///< actual number of decompressed frames

    unsigned long long int dropped{}; ///< number of dropped frames because queue was full

    void (*convert)(unsigned char *dst_buffer,
                    unsigned char *src_buffer,
                    unsigned int width, unsigned int height){nullptr};
};

constexpr const int DEFAULT_TILE_LIMIT = 1;
/// maximal size of queue for decompressed frames
constexpr const int DEFAULT_MAX_QUEUE_SIZE = 2;
/// maximal number of concurrently decompressed frames
constexpr const int DEFAULT_MAX_IN_FRAMES = 4;
constexpr const int64_t DEFAULT_MEM_LIMIT = 1000000000LL;
constexpr const char *MOD_NAME = "[J2K dec.] ";

#endif //ULTRAGRID_CMPTO_J2K_H
