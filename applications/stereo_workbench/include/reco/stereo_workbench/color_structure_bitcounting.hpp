/*
 * Bitcounting.hpp
 *
 *  Created on: Apr 29, 2014
 *      Author: Gregory Kramida
 */

#ifndef BITCOUNTING_HPP_
#define BITCOUNTING_HPP_
#include <iostream>
#include <bitset>
#include <omp.h>
namespace gpxa{
void setupTable();
void histAddFromBits(uint64_t* cursor, uint16_t* hist);
void histSubtractFromBits(uint64_t* cursor, uint16_t* hist);
void histAddFromBitsMaskLSB(uint64_t* cursor, uint16_t* hist);
void histSubtractFromBitsMaskLSB(uint64_t* cursor, uint16_t* hist);
void histAddFromBitsVecExt(uint64_t* cursor, uint16_t* hist);
void histSubtractFromBitsVecExt(uint64_t* cursor, uint16_t* hist);
void histAddFromBitsMatthews(uint64_t* cursor, uint16_t* hist);
void histSubtractFromBitsMatthews(uint64_t* cursor, uint16_t* hist);
void histAddFromBitsFFS(uint64_t* cursor, uint16_t* hist);
void histSubtractFromBitsFFS(uint64_t* cursor, uint16_t* hist);
}


#endif /* BITCOUNTING_HPP_ */
