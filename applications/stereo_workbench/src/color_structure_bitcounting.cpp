/*
 * Bitcounting.cpp
 *
 *  Created on: Apr 29, 2014
 *      Author: Gregory Kramida
 */

#include <reco/stereo_workbench/color_structure_bitcounting.hpp>


namespace gpxa{
typedef short v8hi __attribute__ ((vector_size (16)));

static v8hi table[256];
#ifndef N_CHAR_BITS
#define N_CHAR_BITS 8
#endif

#define MSB_MASK 9223372036854775807ULL
#define MSB 9223372036854775808ULL

void setupTable(){
    for(int i = 0; i < 256; i++){
        for(int j = 0; j < 8; j++){
            table[i][j] = (i >> j) & 1;
        }
    }
}

void histAddFromBitsFFS(uint64_t* cursor, uint16_t* hist){
	//traverse each bit of the 256-bit-long bitstring by splitting up into 4 uint64s
	uint64_t a = *cursor;
	uint64_t b = *(cursor+1);
	uint64_t c = *(cursor+2);
	uint64_t d = *(cursor+3);

	hist[63] += ((a & MSB) != 0);
	hist[127] += ((b & MSB) != 0);
	hist[191] += ((c & MSB) != 0);
	hist[255] += ((d & MSB) != 0);

	a &= MSB_MASK;
	b &= MSB_MASK;
	c &= MSB_MASK;
	d &= MSB_MASK;

	int next;

	for(int bit = 0; (next = __builtin_ffsll(a)) != 0; ) {
	    bit += next;
	    hist[bit - 1] += 1;
	    a >>= next;
	}
	hist += 64;
	for(int bit = 0; (next = __builtin_ffsll(b)) != 0; ) {
	    bit += next;
	    hist[bit - 1] += 1;
	    b >>= next;
	}
	hist += 64;
	for(int bit = 0; (next = __builtin_ffsll(c)) != 0;) {
	    bit += next;
	    hist[bit - 1] += 1;
	    c >>= next;
	}

	hist += 64;
	for(int bit = 0; (next = __builtin_ffsll(d)) != 0;) {
	    bit += next;
	    hist[bit - 1] += 1;
	    d >>= next;
	}

}
void histSubtractFromBitsFFS(uint64_t* cursor, uint16_t* hist){
	//traverse each bit of the 256-bit-long bitstring by splitting up into 4 uint64s
	uint64_t a = *cursor;
	uint64_t b = *(cursor+1);
	uint64_t c = *(cursor+2);
	uint64_t d = *(cursor+3);

	hist[63] -= ((a & MSB) != 0);
	hist[127] -= ((b & MSB) != 0);
	hist[191] -= ((c & MSB) != 0);
	hist[255] -= ((d & MSB) != 0);

	a &= MSB_MASK;
	b &= MSB_MASK;
	c &= MSB_MASK;
	d &= MSB_MASK;
	int next;
	for(int bit = 0; (next = __builtin_ffsll(a)) != 0; ) {
		bit += next;
		hist[bit - 1] -= 1;
		a >>= next;
	}
	hist+=64;
	for(int bit = 0; (next = __builtin_ffsll(b)) != 0; ) {
		bit += next;
		hist[bit - 1] -= 1;
		b >>= next;
	}

	hist+=64;
	for(int bit = 0; (next = __builtin_ffsll(c)) != 0; ) {
		bit += next;
		hist[bit - 1] -= 1;
		c >>= next;
	}

	hist+=64;
	for(int bit = 0; (next = __builtin_ffsll(d)) != 0; ) {
		bit += next;
		hist[bit - 1] -= 1;
		d >>= next;
	}
}

void histAddFromBitsMaskLSB(uint64_t* cursor, uint16_t* hist){
	//traverse each bit of the 256-bit-long bitstring by splitting up into 4 uint64s
	uint64_t a = *cursor;
	uint64_t b = *(cursor+1);
	uint64_t c = *(cursor+2);
	uint64_t d = *(cursor+3);
	for(int bit = 0; a != 0; bit++, a >>= 1){

	}
	for(int bit = 0; a != 0; bit++, a >>= 1) {
	    hist[bit] += (a & 1);
	}
	for(int bit = 64; b != 0; bit++, b >>= 1) {
		hist[bit] += (b & 1);
	}
	for(int bit = 128; c != 0; bit++, c >>= 1) {
		hist[bit] += (c & 1);
	}
	for(int bit = 192; d != 0; bit++, d >>= 1) {
		hist[bit] += (d & 1);
	}
}
void histSubtractFromBitsMaskLSB(uint64_t* cursor, uint16_t* hist){
	//traverse each bit of the 256-bit-long bitstring by splitting up into 4 uint64s
	uint64_t a = *cursor;
	uint64_t b = *(cursor+1);
	uint64_t c = *(cursor+2);
	uint64_t d = *(cursor+3);
	for(int bit = 0; a != 0; bit++, a >>= 1) {
	    hist[bit] -= (a & 1);
	}
	for(int bit = 64; b != 0; bit++, b >>= 1) {
		hist[bit] -= (b & 1);
	}
	for(int bit = 128; c != 0; bit++, c >>= 1) {
		hist[bit] -= (c & 1);
	}
	for(int bit = 192; d != 0; bit++, d >>= 1) {
		hist[bit] -= (d & 1);
	}
}

void histAddFromBitsVecExt(uint64_t* cursor, uint16_t* hist){
	//traverse each bit of the 256-bit-long bitstring by splitting up into 32 uint8s
	uint8_t* cursor_tmp = (uint8_t*)cursor;
	v8hi* hist_tmp = (v8hi*)hist;
	for(int i = 0; i < 32; i++, cursor_tmp++, hist_tmp++){
		*hist_tmp += table[*cursor_tmp];
	}
}
void histSubtractFromBitsVecExt(uint64_t* cursor, uint16_t* hist){
	//traverse each bit of the 256-bit-long bitstring by splitting up into 32 uint8s
	uint8_t* cursor_tmp = (uint8_t*)cursor;
	v8hi* hist_tmp = (v8hi*)hist;
	for(int i = 0; i < 32; i++, cursor_tmp++, hist_tmp++){
		*hist_tmp -= table[*cursor_tmp];
	}
}
void histAddFromBitsMatthews(uint64_t* cursor, uint16_t* hist){
	//traverse each bit of the 256-bit-long bitstring by splitting up into 4 uint64s
    register uint64_t a = *cursor++;
    register uint64_t b = *cursor++;
    register uint64_t c = *cursor++;
    register uint64_t d = *cursor++;
    register unsigned int i = 0;
    uint16_t* histA = hist;
    uint16_t* histB = hist+64;
    uint16_t* histC = hist+128;
    uint16_t* histD = hist+192;
    for (i = 0; i < (sizeof(*cursor) * N_CHAR_BITS); ++i){
    	histA[i] += a & 1;
    	histB[i] += b & 1;
    	histC[i] += c & 1;
    	histD[i] += d & 1;
    	a >>= 1;
    	b >>= 1;
    	c >>= 1;
        d >>= 1;
    }
}
void histSubtractFromBitsMatthews(uint64_t* cursor, uint16_t* hist){
	//traverse each bit of the 256-bit-long bitstring by splitting up into 4 uint64s
    register uint64_t a = *cursor++;
    register uint64_t b = *cursor++;
    register uint64_t c = *cursor++;
    register uint64_t d = *cursor++;
    register unsigned int i = 0;
    uint16_t* histA = hist;
    uint16_t* histB = hist+64;
    uint16_t* histC = hist+128;
    uint16_t* histD = hist+192;
    for (i = 0; i < (sizeof(*cursor) * N_CHAR_BITS); ++i){
    	histA[i] -= a & 1;
    	histB[i] -= b & 1;
    	histC[i] -= c & 1;
    	histD[i] -= d & 1;
    	a >>= 1;
    	b >>= 1;
    	c >>= 1;
        d >>= 1;
    }
}

void histAddFromBits(uint64_t* cursor, uint16_t* hist){
	//traverse each bit of the 256-bit-long bitstring by splitting up into 4 bitsets
	std::bitset<64> a(*cursor);
	std::bitset<64> b(*(cursor+1));
	std::bitset<64> c(*(cursor+2));
	std::bitset<64> d(*(cursor+3));
	for(int bit = 0; bit < 64; bit++){
		hist[bit] += a.test(bit);
	}
	for(int bit = 0; bit < 64; bit++){
		hist[bit+64] += b.test(bit);
	}
	for(int bit = 0; bit < 64; bit++){
		hist[bit+128] += c.test(bit);
	}
	for(int bit = 0; bit < 64; bit++){
		hist[bit+192] += d.test(bit);
	}
}
void histSubtractFromBits(uint64_t* cursor, uint16_t* hist){
	//traverse each bit of the 256-bit-long bitstring by splitting up into 4 bitsets
	std::bitset<64> a(*cursor);
	std::bitset<64> b(*(cursor+1));
	std::bitset<64> c(*(cursor+2));
	std::bitset<64> d(*(cursor+3));
	for(int bit = 0; bit < 64; bit++){
		hist[bit] -= a.test(bit);
	}
	for(int bit = 0; bit < 64; bit++){
		hist[bit+64] -= b.test(bit);
	}
	for(int bit = 0; bit < 64; bit++){
		hist[bit+128] -= c.test(bit);
	}
	for(int bit = 0; bit < 64; bit++){
		hist[bit+192] -= d.test(bit);
	}
}
}//end namespace gpxa
