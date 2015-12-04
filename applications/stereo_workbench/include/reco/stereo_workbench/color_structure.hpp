/*
 * CSExtraction.hpp
 *
 *  Created on: Apr 22, 2014
 *      Author: Gregory Kramida
 */


#pragma GCC diagnostic ignored "-Wunused-function"


#include <cstdio>
#include <cstring>
#include <cmath>
#include <omp.h>
#include <reco/stereo_workbench/color_structure_bitcounting.hpp>

#define REGION_SIZE 256
#define BASE_QUANT_SPACE 256
#define WINDOW_SIZE 8
#define REGION_CLIP (REGION_SIZE - WINDOW_SIZE + 1)
#define REGION_NORM (REGION_CLIP * REGION_CLIP)
#define SCALE_INCREMENT_FACTOR 2.0

namespace gpxa{
/**
 * Extract a single region MPEG7 Color Structure descriptor from the pre-computed 8x8 window bit-arrays at the
 * given coordinate.
 * @param windowBitarrays - a NumPy array of bitarrays for each pixel of the original image, having shape
 * (h,w,8), where w & h are width and height of the original image, and type unsigned int. Within, the bitarray for
 * pixel (y,x) is spread over 8 uints at y,x,0-7. The bitarray for a pixel represents,
 * for 256 values, whether that value occurred within the 8x8 pixel window with top-left corner at that pixel.
 * @param x - x-coordinate for the top-left corner of the region whose descriptor to retrieve
 * @param y - y-coordinate for the top-right corner of the region whose descriptor to retrieve
 * REGION_SIZE * SCALE_INCREMENT_FACTOR^z
 * @return a 256-ushort long 1D NumPy array containing the Color Structure descriptor
 */
void precomputeTotalLevels();

inline void calculate_window_sizes(int cell_size, int& win_size, int& subsample_size);

void quantize_and_ampify(uint16_t* hist, uint8_t* histOut);
void bitstrings_to_histogram(uint64_t* arr, uint16_t* hist, int width, int x, int y);

void bitstrings_to_sliding_histogram(uint64_t* arr, uint8_t* descriptors, int width, int height);
void bitstrings_to_sliding_histogram_multithreaded(uint64_t* arr, uint8_t* descriptors, int width, int height);
void bitstrings_to_sliding_histogram_MT_function_pointers(uint64_t* arr, uint8_t* descriptors,
		void (*histAdd)(uint64_t*, uint16_t*),
		void (*histSub)(uint64_t*, uint16_t*), const int width,
		const int height);
}//end namespace gpxa
void bitstrings_to_sliding_histogram_MT_basic(uint64_t* arr, uint8_t* descriptors, int width, int height);
void bitstrings_to_sliding_histogram_MT_vector_extensions(uint64_t* arr, uint8_t* descriptors, int width, int height);
void bitstrings_to_sliding_histogram_MT_mask_LSB(uint64_t* arr, uint8_t* descriptors, int width, int height);
void bitstrings_to_sliding_histogram_MT_Matthews(uint64_t* arr, uint8_t* descriptors, int width, int height);
void bitstrings_to_sliding_histogram_MT_FFS(uint64_t* arr, uint8_t* descriptors, int width, int height);

