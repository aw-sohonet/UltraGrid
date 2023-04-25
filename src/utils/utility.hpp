//
// Created by Sohonet on 2/16/23.
//

#ifndef ULTRAGRID_UTILITY_H
#define ULTRAGRID_UTILITY_H

#include <vector>
#include <utility>
#include <math.h>


enum MediaType {
    MEDIA_VIDEO = 0,
    MEDIA_AUDIO = 1
};


/**
 * A wrapper around a pair. The first element represents the index in the array, the second represents the
 * the length of the block.
 */ 
using UIntPair = std::pair<unsigned int, unsigned int>;
/**
 * A wrapper around a vector of pairs to represent a list of blocks.
 */ 
using Blocks = std::vector<UIntPair>;

// Forward declaration
Blocks createBlocks(unsigned int total, unsigned int blockCount);

#endif // ULTRAGRID_UTILITY_H