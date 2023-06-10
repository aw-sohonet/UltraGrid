//
// Created by sohonet on 4/18/23.
//
#include "utility.hpp"

/**
 * @brief A helper function for creating a vector of blocks based on the size and desired block count for a list.
 *
 * @param total       The total amount of elements to split into blocks
 * @param blockCount  The total amount of desired blocks for the elements to be split into
*/
Blocks createBlocks(unsigned int total, unsigned int blockCount) {
    // Create the blocks vector
    Blocks blocks = Blocks();
    // Calculate the largest a block should be for equality amongst them
    const unsigned int avgBlockSize = floor((double)total / (double)blockCount);
    // Calculate the remaining leftover elements which don't fit neatly into the average block size
    unsigned int leftover = total % blockCount;
    // Start from the beginning of an array
    unsigned int blockBegin = 0;
    for(unsigned int i = 0; i < blockCount; i++) {
        // Attempt to set the blockSize to be the maximum block size
        unsigned int blockSize = avgBlockSize;

        // Add leftovers to the beginning set of groups for a better
        // spread for the blocks.
        if(leftover > 0) {
            blockSize++;
            leftover--;
        }

        // If by assigning the max block size we'd have an overflow, set the block size to be the difference
        // between the block beginning and the end.
        if(blockBegin + blockSize > total) {
            blockSize = total - blockBegin;
        }

        // Push the pairing onto the block listing
        blocks.emplace_back(blockBegin, blockSize);

        // Increment the block beginning by the size of the block we just pushed
        blockBegin += blockSize;
    }

    return blocks;
}