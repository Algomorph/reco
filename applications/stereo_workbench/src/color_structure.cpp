#include <reco/stereo_workbench/color_structure.hpp>

namespace gpxa {

const double amplification_thresholds[] = { 0.0, 0.000000000001, 0.037, 0.08, 0.195, 0.32 };
const int numbers_of_amplification_levels[] = { 1, 25, 20, 35, 35, 140 };
const int total_number_of_levels = []() -> int {
	const int nAmplLinearRegions = sizeof(numbers_of_amplification_levels) / sizeof(numbers_of_amplification_levels[0]);
	int num_levels = 0;
	// Calculate total levels
	for (int iQuant = 0; iQuant < nAmplLinearRegions; iQuant++) {
		num_levels += numbers_of_amplification_levels[iQuant];
	}
	return num_levels;
}();

const std::string color_structure_extractor::compile_string = "-D REGION_SIZE={0:d} -D REGION_CLIP={1:d} -D WINDOW_SIZE={2:d} -D BASE_QUANT_SPACE={3:d}";

/**
 * Routine that mimics how window & subsample size are is calculated in the libMPEG7
 * @param cell_size (lateral) size of each cell
 * @param window_size (out) size of each window
 * @param subsample_size (out) size of each subsample
 */
inline void calculate_window_sizes(int cell_size, int& window_size, int& subsample_size){
	double log_area = std::log2(static_cast<double>(cell_size*cell_size));
	//TODO: describe magic numbers
	int scale_power = std::max(static_cast<int>(std::floor(0.5*log_area - 8.0 + 0.5)), 0);
	subsample_size = 1 << scale_power;
	window_size = 8 * subsample_size;
}


void bitstrings_to_histogram(uint64_t* arr, uint16_t* hist, int width, int x,
		int y) {
	//pointing at the first relevant bitstring
	uint64_t* bitdataRowStart = arr + ((y * width + x) << 2);
	const int uint64_twidth = width << 2;
	for (int yR = 0; yR < REGION_CLIP; yR++) {
		uint64_t* rowEnd = bitdataRowStart + ((REGION_CLIP) << 2);
		for (uint64_t* cursor = bitdataRowStart; cursor < rowEnd; cursor += 4) {
			//traverse each bit of the 256-bit-long bitstring by splitting up into 4 bitsets
			std::bitset<64> a(*cursor);
			std::bitset<64> b(*(cursor + 1));
			std::bitset<64> c(*(cursor + 2));
			std::bitset<64> d(*(cursor + 3));
			for (int bit = 0; bit < 64; bit++) {
				hist[bit] += a.test(bit);
			}
			for (int bit = 0; bit < 64; bit++) {
				hist[bit + 64] += b.test(bit);
			}
			for (int bit = 0; bit < 64; bit++) {
				hist[bit + 128] += c.test(bit);
			}
			for (int bit = 0; bit < 64; bit++) {
				hist[bit + 192] += d.test(bit);
			}
		}
		bitdataRowStart += uint64_twidth;
	}
}
void quantize_and_ampify(uint16_t* hist, uint8_t* histOut) {
	unsigned long iBin, iQuant;
	const int nAmplLinearRegions = sizeof(numbers_of_amplification_levels) / sizeof(numbers_of_amplification_levels[0]);

	// Loop through bins
	for (iBin = 0; iBin < BASE_QUANT_SPACE; iBin++) {
		// Get bin amplitude
		double val = hist[iBin];

		// Normalize
		val /= REGION_NORM;

		// Find quantization boundary and base value
		int quantValue = 0;
		for (iQuant = 0; iQuant + 1 < nAmplLinearRegions; iQuant++) {
			if (val < amplification_thresholds[iQuant + 1])
				break;
			quantValue += numbers_of_amplification_levels[iQuant];
		}

		// Quantize
		double nextThresh =
				(iQuant + 1 < nAmplLinearRegions) ?
						amplification_thresholds[iQuant + 1] : 1.0;
		val = floor(
				quantValue
						+ (val - amplification_thresholds[iQuant])
								* (numbers_of_amplification_levels[iQuant]
										/ (nextThresh - amplification_thresholds[iQuant])));

		// Limit (and alert), one bin contains all of histogram
		if (val == total_number_of_levels) {
			val = total_number_of_levels - 1;
		}

		// Set value into histogram
		histOut[iBin] = (uint8_t) val;
	}
}

void bitstrings_to_sliding_histogram(uint64_t* arr, uint8_t* descriptors,
		const int width, const int height) {
	//pointing at the first relevant bitstring

	const int uint64_twidth = width << 2;
	const int stopAtY = height - REGION_SIZE + 1;
	const int stopAtX = width - REGION_SIZE + 1;
	const int histShift = (const int) log((float) BASE_QUANT_SPACE) / log(2)
			+ 1;
	const int histsWidth = stopAtX << histShift;

	for (int xR = 0; xR < stopAtX; xR++) {
		/*clean out the sliding histogram at the top of each column*/
		uint16_t slidingHist[BASE_QUANT_SPACE] = { 0 };
		uint64_t* bitdataRowStart = arr + (xR << 2);
		uint64_t* bitdataRowSubStart = bitdataRowStart;
		uint64_t* stopAtRow = bitdataRowStart + REGION_CLIP * uint64_twidth;
		/*first histogram in this column*/
		for (; bitdataRowStart < stopAtRow; bitdataRowStart += uint64_twidth) {
			uint64_t* rowEnd = bitdataRowStart + ((REGION_CLIP) << 2);
			for (uint64_t* cursor = bitdataRowStart; cursor < rowEnd; cursor +=
					4) {
				histAddFromBits(cursor, slidingHist);
			}
		}
		//determine location of descriptor
		uint8_t* descrAt = descriptors + (xR << histShift);
		quantize_and_ampify(slidingHist, descrAt);
		//determine stopping row
		stopAtRow = bitdataRowStart + (stopAtY - 1) * uint64_twidth;

		descrAt += histsWidth;
		//slide over the rest of the rows, removing the first row and adding the next one

		for (; bitdataRowStart < stopAtRow;
				bitdataRowStart += uint64_twidth, bitdataRowSubStart +=
						uint64_twidth, descrAt += histsWidth) {
			uint64_t* rowEnd = bitdataRowStart + ((REGION_CLIP) << 2);
			for (uint64_t* cursorAdd = bitdataRowStart, *cursorSub =
					bitdataRowSubStart; cursorAdd < rowEnd;
					cursorAdd += 4, cursorSub += 4) {
				histAddFromBits(cursorAdd, slidingHist);
				histSubtractFromBits(cursorSub, slidingHist);
			}
			quantize_and_ampify(slidingHist, descrAt);
		}
	}
}
void bitstrings_to_sliding_histogram_multithreaded(uint64_t* arr, uint8_t* descriptors,
		const int width, const int height) {
	//pointing at the first relevant bitstring

	int uint64_twidth = width << 2;

	const int stopAtY = height - REGION_SIZE + 1;
	const int stopAtX = width - REGION_SIZE + 1;
	const int histShift = (const int) log((float) BASE_QUANT_SPACE) / log(2)
			+ 1;
	const int histsWidth = stopAtX << histShift;
#pragma omp parallel for
	for (int xR = 0; xR < stopAtX; xR++) {

		/*clean out the sliding histogram at the top of each column*/
		uint16_t slidingHist[BASE_QUANT_SPACE] = { 0 };
		uint64_t* bitdataRowStart = arr + (xR << 2);

		uint64_t* bitdataRowSubStart = bitdataRowStart;
		uint64_t* stopAtRow = bitdataRowStart + REGION_CLIP * uint64_twidth;
		/*first histogram in this column*/
		for (; bitdataRowStart < stopAtRow; bitdataRowStart += uint64_twidth) {
			uint64_t* rowEnd = bitdataRowStart + ((REGION_CLIP) << 2);
			for (uint64_t* cursor = bitdataRowStart; cursor < rowEnd; cursor +=
					4) {
				histAddFromBits(cursor, slidingHist);
			}
		}
		//determine location of descriptor
		uint8_t* descrAt = descriptors + (xR << histShift);
		quantize_and_ampify(slidingHist, descrAt);
		//determine stopping row
		stopAtRow = bitdataRowStart + (stopAtY - 1) * uint64_twidth;

		descrAt += histsWidth;
		//slide over the rest of the rows, removing the first row and adding the next one

		for (; bitdataRowStart < stopAtRow;
				bitdataRowStart += uint64_twidth, bitdataRowSubStart +=
						uint64_twidth, descrAt += histsWidth) {
			uint64_t* rowEnd = bitdataRowStart + ((REGION_CLIP) << 2);
			for (uint64_t* cursorAdd = bitdataRowStart, *cursorSub =
					bitdataRowSubStart; cursorAdd < rowEnd;
					cursorAdd += 4, cursorSub += 4) {
				histAddFromBits(cursorAdd, slidingHist);
				histSubtractFromBits(cursorSub, slidingHist);
			}
			quantize_and_ampify(slidingHist, descrAt);
		}
	}
}

void bitstrings_to_sliding_histogram_MT_function_pointers(uint64_t* arr, uint8_t* descriptors,
		void (*histAdd)(uint64_t*, uint16_t*),
		void (*histSub)(uint64_t*, uint16_t*), const int width,
		const int height) {
	//pointing at the first relevant bitstring

	int uint64_twidth = width << 2;

	const int stopAtY = height - REGION_SIZE + 1;
	const int stopAtX = width - REGION_SIZE + 1;
	const int histShift = (const int) log((float) BASE_QUANT_SPACE) / log(2)
			+ 1;
	const int histsWidth = stopAtX << histShift;
#pragma omp parallel for
	for (int xR = 0; xR < stopAtX; xR++) {

		/*clean out the sliding histogram at the top of each column*/
		uint16_t slidingHist[BASE_QUANT_SPACE] = { 0 };
		uint64_t* bitdataRowStart = arr + (xR << 2);

		uint64_t* bitdataRowSubStart = bitdataRowStart;
		uint64_t* stopAtRow = bitdataRowStart + REGION_CLIP * uint64_twidth;
		/*first histogram in this column*/
		for (; bitdataRowStart < stopAtRow; bitdataRowStart += uint64_twidth) {
			uint64_t* rowEnd = bitdataRowStart + ((REGION_CLIP) << 2);
			for (uint64_t* cursor = bitdataRowStart; cursor < rowEnd; cursor +=
					4) {
				histAdd(cursor, slidingHist);
			}
		}
		//determine location of descriptor
		uint8_t* descrAt = descriptors + (xR << histShift);
		quantize_and_ampify(slidingHist, descrAt);
		//determine stopping row
		stopAtRow = bitdataRowStart + (stopAtY - 1) * uint64_twidth;

		descrAt += histsWidth;
		//slide over the rest of the rows, removing the first row and adding the next one

		for (; bitdataRowStart < stopAtRow;
				bitdataRowStart += uint64_twidth, bitdataRowSubStart +=
						uint64_twidth, descrAt += histsWidth) {
			uint64_t* rowEnd = bitdataRowStart + ((REGION_CLIP) << 2);
			for (uint64_t* cursorAdd = bitdataRowStart, *cursorSub =
					bitdataRowSubStart; cursorAdd < rowEnd;
					cursorAdd += 4, cursorSub += 4) {
				histAdd(cursorAdd, slidingHist);
				histSub(cursorSub, slidingHist);
			}
			quantize_and_ampify(slidingHist, descrAt);
		}
	}
}
void bitstrings_to_sliding_histogram_MT_basic(uint64_t* arr, uint8_t* descriptors,
		const int width, const int height){
	bitstrings_to_sliding_histogram_MT_function_pointers(arr, descriptors,&histAddFromBits,&histSubtractFromBits,width,height);
}
void bitstrings_to_sliding_histogram_MT_mask_LSB(uint64_t* arr, uint8_t* descriptors,
		const int width, const int height){
	bitstrings_to_sliding_histogram_MT_function_pointers(arr, descriptors,&histAddFromBitsMaskLSB,&histSubtractFromBitsMaskLSB,width,height);
}
void bitstrings_to_sliding_histogram_MT_vector_extensions(uint64_t* arr, uint8_t* descriptors,
		const int width, const int height){
	bitstrings_to_sliding_histogram_MT_function_pointers(arr, descriptors,&histAddFromBitsVecExt,&histSubtractFromBitsVecExt,width,height);
}
void bitstrings_to_sliding_histogram_MT_Matthews(uint64_t* arr, uint8_t* descriptors,
		const int width, const int height){
	bitstrings_to_sliding_histogram_MT_function_pointers(arr, descriptors,&histAddFromBitsMatthews,&histSubtractFromBitsMatthews,width,height);
}
void bitstrings_to_sliding_histogram_MT_FFS(uint64_t* arr, uint8_t* descriptors,
		const int width, const int height){
	bitstrings_to_sliding_histogram_MT_function_pointers(arr, descriptors,&histAddFromBitsFFS,&histSubtractFromBitsFFS,width,height);
}


} //end namespace gpxa

