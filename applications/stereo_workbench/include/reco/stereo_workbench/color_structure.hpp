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


class color_structure_extractor{
	static const std::string compile_string;


};

inline void calculate_window_sizes(int cell_size, int& window_size, int& subsample_size);

void quantize_and_ampify(uint16_t* hist, uint8_t* histOut);
void bitstrings_to_histogram(uint64_t* arr, uint16_t* hist, int width, int x, int y);

void bitstrings_to_sliding_histogram(uint64_t* arr, uint8_t* descriptors, int width, int height);
void bitstrings_to_sliding_histogram_multithreaded(uint64_t* arr, uint8_t* descriptors, int width, int height);
void bitstrings_to_sliding_histogram_MT_function_pointers(uint64_t* arr, uint8_t* descriptors,
		void (*histAdd)(uint64_t*, uint16_t*),
		void (*histSub)(uint64_t*, uint16_t*), const int width,
		const int height);

void bitstrings_to_sliding_histogram_MT_basic(uint64_t* arr, uint8_t* descriptors, int width, int height);
void bitstrings_to_sliding_histogram_MT_vector_extensions(uint64_t* arr, uint8_t* descriptors, int width, int height);
void bitstrings_to_sliding_histogram_MT_mask_LSB(uint64_t* arr, uint8_t* descriptors, int width, int height);
void bitstrings_to_sliding_histogram_MT_Matthews(uint64_t* arr, uint8_t* descriptors, int width, int height);
void bitstrings_to_sliding_histogram_MT_FFS(uint64_t* arr, uint8_t* descriptors, int width, int height);
}//end namespace gpxa

