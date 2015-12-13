/*
 * semiglobal_matcher.cpp
 *
 *  Created on: Dec 8, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 */
#include <opencv2/cvconfig.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/hal/intrin.hpp>



//#include <opencv2/core/private.hpp>

#include <opencv2/core/ocl.hpp>

#include <functional>

#include <reco/stereo_workbench/semiglobal_matcher.hpp>
#include <reco/utils/cpp_exception_util.h>
#include "pixel_cost.hpp"


namespace reco{
namespace stereo_workbench{



enum { NR = 16, NR2 = NR/2 };


using namespace cv;



struct stereo_cost_calculator{

	stereo_cost_calculator(row_cost_function compute_row_pixel_cost,
			cost_precomputation_function precompute_cost){
		this->compute_row_pixel_cost = compute_row_pixel_cost;
		this->precompute_cost = precompute_cost;
	}
	stereo_cost_calculator(pixel_cost_type type){
		switch(type){
		case BIRCHFIELD_TOMASI:
			this->compute_row_pixel_cost = calculate_row_cost_BT;
			this->precompute_cost = precompute_nothing;
			break;
		case DAISY:
			this->compute_row_pixel_cost = calculate_row_cost_DAISY;
			this->precompute_cost = precompute_DAISY;
			break;
		case NORM_L2:
			this->compute_row_pixel_cost = calculate_row_cost_L2;
			this->precompute_cost = precompute_nothing;
			break;
		default:
			err2(std::runtime_error, "Unknown semiglobal matcher cost type: " << static_cast<int>(type));
			break;
		}
	}
	stereo_cost_calculator()
		: stereo_cost_calculator(pixel_cost_type::BIRCHFIELD_TOMASI){}
	row_cost_function compute_row_pixel_cost;
	cost_precomputation_function precompute_cost;
};






/*
 computes disparity for "roi" in img1 w.r.t. img2 and write it to disp1buf.
 that is, disp1buf(x, y)=d means that img1(x+roi.x, y+roi.y) ~ img2(x+roi.x-d, y+roi.y).
 minD <= d < maxD.
 disp2full is the reverse disparity map, that is:
 disp2full(x+roi.x,y+roi.y)=d means that img2(x+roi.x, y+roi.y) ~ img1(x+roi.x+d, y+roi.y)

 note that disp1buf will have the same size as the roi and
 disp2full will have the same size as img1 (or img2).
 On exit disp2buf is not the final disparity, it is an intermediate result that becomes
 final after all the tiles are processed.

 the disparity in disp1buf is written with sub-pixel accuracy
 (4 fractional bits, see StereoSGBM::DISP_SCALE),
 using quadratic interpolation, while the disparity in disp2buf
 is written as is, without interpolation.

 disp2cost also has the same size as img1 (or img2).
 It contains the minimum current cost, used to find the best disparity, corresponding to the minimal cost.
 */
static void computeDisparitySGBM( const Mat& img1, const Mat& img2,
                                 Mat& disp1, const semiglobal_matcher_parameters& params,
                                 Mat& buffer, pixel_cost_type cost_type =
                                		 pixel_cost_type::BIRCHFIELD_TOMASI)
{
#if CV_SSE2
    static const uchar LSBTab[] =
    {
        0, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
        5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
        6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
        5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
        7, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
        5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
        6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
        5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0
    };

    volatile bool useSIMD = checkHardwareSupport(CV_CPU_SSE2);
#endif

    const int ALIGN = 16;
    const int DISP_SHIFT = StereoMatcher::DISP_SHIFT;
    const int DISP_SCALE = (1 << DISP_SHIFT);
    const CostType MAX_COST = SHRT_MAX;

    int minD = params.minDisparity, maxD = minD + params.numDisparities;
    Size SADWindowSize;
    SADWindowSize.width = SADWindowSize.height = params.block_size > 0 ? params.block_size : 5;

    int ftzero = std::max(params.preFilterCap, 15) | 1;

    int uniquenessRatio = params.uniquenessRatio >= 0 ? params.uniquenessRatio : 10;
    int disp12MaxDiff = params.disp12MaxDiff > 0 ? params.disp12MaxDiff : 1;
    int P1 = params.P1 > 0 ? params.P1 : 2, P2 = std::max(params.P2 > 0 ? params.P2 : 5, P1+1);
    int k, width = disp1.cols, height = disp1.rows;
    int minX1 = std::max(maxD, 0), maxX1 = width + std::min(minD, 0);
    int D = maxD - minD, width1 = maxX1 - minX1;
    int INVALID_DISP = minD - 1, INVALID_DISP_SCALED = INVALID_DISP*DISP_SCALE;
    int SW2 = SADWindowSize.width/2, SH2 = SADWindowSize.height/2;
    bool fullDP = params.mode == StereoSGBM::MODE_HH;
    int npasses = fullDP ? 2 : 1;

    //precomputation for faster cost computation
    Mat descriptor_img1, descriptor_img2;
    std::unique_ptr<abstract_stereo_cost_calculator> cost_calculator
		= build_stereo_cost_calculator(cost_type, img1, img2, params);

    //1024, 2304
    const int clip_table_offset = 256*4, clip_table_size = 256 + clip_table_offset*2;

    PixType clip_table[clip_table_size];

    //the first few entries will be 0, others grow to 2*ftzero
    for( k = 0; k < clip_table_size; k++ )
        clip_table[k] = (PixType)(std::min(std::max(k - clip_table_offset, -ftzero), ftzero) + ftzero);

    if( minX1 >= maxX1 )
    {
        disp1 = Scalar::all(INVALID_DISP_SCALED);
        return;
    }

    CV_Assert( D % 16 == 0 );

    // NR - the number of directions. the loop on x below that computes Lr assumes that NR == 8.
    // if you change NR, please, modify the loop as well.
    int D2 = D+16, NRD2 = NR2*D2;

    // the number of L_r(.,.) and min_k L_r(.,.) lines in the buffer:
    // for 8-way dynamic programming we need the current row and
    // the previous row, i.e. 2 rows in total
    const int NLR = 2;
    const int LrBorder = NLR - 1;

    // for each possible stereo match (img1(x,y) <=> img2(x-d,y))
    // we keep pixel difference cost (C) and the summary cost over NR directions (S).
    // we also keep all the partial costs for the previous line L_r(x,d) and also min_k L_r(x, k)
    size_t costBufSize = width1*D;
    size_t CSBufSize = costBufSize*(fullDP ? height : 1);
    size_t minLrSize = (width1 + LrBorder*2)*NR2, LrSize = minLrSize*D2;
    int hsumBufNRows = SH2*2 + 2;
    size_t totalBufSize = (LrSize + minLrSize)*NLR*sizeof(CostType) + // minLr[] and Lr[]
    costBufSize*(hsumBufNRows + 1)*sizeof(CostType) + // hsumBuf, pixdiff
    CSBufSize*2*sizeof(CostType) + // C, S
    width*16*img1.channels()*sizeof(PixType) + // temp buffer for computing per-pixel cost
    width*(sizeof(CostType) + sizeof(DispType)) + 1024; // disp2cost + disp2

    if( buffer.empty() || !buffer.isContinuous() ||
        buffer.cols*buffer.rows*buffer.elemSize() < totalBufSize )
        buffer.create(1, (int)totalBufSize, CV_8U);

    // summary cost over different (nDirs) directions
    CostType* Cbuf = (CostType*)alignPtr(buffer.ptr(), ALIGN);
    CostType* Sbuf = Cbuf + CSBufSize;
    CostType* hsumBuf = Sbuf + CSBufSize;
    CostType* pixDiff = hsumBuf + costBufSize*hsumBufNRows;

    CostType* disp2cost = pixDiff + costBufSize + (LrSize + minLrSize)*NLR;
    DispType* disp2ptr = (DispType*)(disp2cost + width);
    PixType* tempBuf = (PixType*)(disp2ptr + width);

    // add P2 to every C(x,y). it saves a few operations in the inner loops
    for( k = 0; k < width1*D; k++ )
        Cbuf[k] = (CostType)P2;

    for( int pass = 1; pass <= npasses; pass++ )
    {
        int x1, y1, x2, y2, dx, dy;

        if( pass == 1 )
        {
            y1 = 0; y2 = height; dy = 1;
            x1 = 0; x2 = width1; dx = 1;
        }
        else
        {
            y1 = height-1; y2 = -1; dy = -1;
            x1 = width1-1; x2 = -1; dx = -1;
        }

        CostType *Lr[NLR]={0}, *minLr[NLR]={0};

        for( k = 0; k < NLR; k++ )
        {
            // shift Lr[k] and minLr[k] pointers, because we allocated them with the borders,
            // and will occasionally use negative indices with the arrays
            // we need to shift Lr[k] pointers by 1, to give the space for d=-1.
            // however, then the alignment will be imperfect, i.e. bad for SSE,
            // thus we shift the pointers by 8 (8*sizeof(short) == 16 - ideal alignment)
            Lr[k] = pixDiff + costBufSize + LrSize*k + NRD2*LrBorder + 8;
            memset( Lr[k] - LrBorder*NRD2 - 8, 0, LrSize*sizeof(CostType) );
            minLr[k] = pixDiff + costBufSize + LrSize*NLR + minLrSize*k + NR2*LrBorder;
            memset( minLr[k] - LrBorder*NR2, 0, minLrSize*sizeof(CostType) );
        }

        for( int y = y1; y != y2; y += dy )
        {
            int x, d;
            DispType* disp1ptr = disp1.ptr<DispType>(y);
            CostType* C = Cbuf + (!fullDP ? 0 : y*costBufSize);
            CostType* S = Sbuf + (!fullDP ? 0 : y*costBufSize);

            if( pass == 1 ) // compute C on the first pass, and reuse it on the second pass, if any.
            {
                int dy1 = y == 0 ? 0 : y + SH2, dy2 = y == 0 ? SH2 : dy1;

                for( k = dy1; k <= dy2; k++ )
                {
                    CostType* hsumAdd = hsumBuf + (std::min(k, height-1) % hsumBufNRows)*costBufSize;

                    if( k < height )
                    {
                    	cost_calculator->compute(k,pixDiff);

                        memset(hsumAdd, 0, D*sizeof(CostType));
                        for( x = 0; x <= SW2*D; x += D )
                        {
                            int scale = x == 0 ? SW2 + 1 : 1;
                            for( d = 0; d < D; d++ )
                                hsumAdd[d] = (CostType)(hsumAdd[d] + pixDiff[x + d]*scale);
                        }

                        if( y > 0 )
                        {
                            const CostType* hsumSub = hsumBuf + (std::max(y - SH2 - 1, 0) % hsumBufNRows)*costBufSize;
                            const CostType* Cprev = !fullDP || y == 0 ? C : C - costBufSize;

                            for( x = D; x < width1*D; x += D )
                            {
                                const CostType* pixAdd = pixDiff + std::min(x + SW2*D, (width1-1)*D);
                                const CostType* pixSub = pixDiff + std::max(x - (SW2+1)*D, 0);

                            #if CV_SSE2
                                if( useSIMD )
                                {
                                    for( d = 0; d < D; d += 8 )
                                    {
                                        __m128i hv = _mm_load_si128((const __m128i*)(hsumAdd + x - D + d));
                                        __m128i Cx = _mm_load_si128((__m128i*)(Cprev + x + d));
                                        hv = _mm_adds_epi16(_mm_subs_epi16(hv,
                                                                           _mm_load_si128((const __m128i*)(pixSub + d))),
                                                            _mm_load_si128((const __m128i*)(pixAdd + d)));
                                        Cx = _mm_adds_epi16(_mm_subs_epi16(Cx,
                                                                           _mm_load_si128((const __m128i*)(hsumSub + x + d))),
                                                            hv);
                                        _mm_store_si128((__m128i*)(hsumAdd + x + d), hv);
                                        _mm_store_si128((__m128i*)(C + x + d), Cx);
                                    }
                                }
                                else
                            #endif
                                {
                                    for( d = 0; d < D; d++ )
                                    {
                                        int hv = hsumAdd[x + d] = (CostType)(hsumAdd[x - D + d] + pixAdd[d] - pixSub[d]);
                                        C[x + d] = (CostType)(Cprev[x + d] + hv - hsumSub[x + d]);
                                    }
                                }
                            }
                        }
                        else
                        {
                            for( x = D; x < width1*D; x += D )
                            {
                                const CostType* pixAdd = pixDiff + std::min(x + SW2*D, (width1-1)*D);
                                const CostType* pixSub = pixDiff + std::max(x - (SW2+1)*D, 0);

                                for( d = 0; d < D; d++ )
                                    hsumAdd[x + d] = (CostType)(hsumAdd[x - D + d] + pixAdd[d] - pixSub[d]);
                            }
                        }
                    }

                    if( y == 0 )
                    {
                        int scale = k == 0 ? SH2 + 1 : 1;
                        for( x = 0; x < width1*D; x++ )
                            C[x] = (CostType)(C[x] + hsumAdd[x]*scale);
                    }
                }

                // also, clear the S buffer
                for( k = 0; k < width1*D; k++ )
                    S[k] = 0;
            }

            // clear the left and the right borders
            memset( Lr[0] - NRD2*LrBorder - 8, 0, NRD2*LrBorder*sizeof(CostType) );
            memset( Lr[0] + width1*NRD2 - 8, 0, NRD2*LrBorder*sizeof(CostType) );
            memset( minLr[0] - NR2*LrBorder, 0, NR2*LrBorder*sizeof(CostType) );
            memset( minLr[0] + width1*NR2, 0, NR2*LrBorder*sizeof(CostType) );

            /*
             [formula 13 in the paper]
             compute L_r(p, d) = C(p, d) +
             min(L_r(p-r, d),
             L_r(p-r, d-1) + P1,
             L_r(p-r, d+1) + P1,
             min_k L_r(p-r, k) + P2) - min_k L_r(p-r, k)
             where p = (x,y), r is one of the directions.
             we process all the directions at once:
             0: r=(-dx, 0)
             1: r=(-1, -dy)
             2: r=(0, -dy)
             3: r=(1, -dy)
             4: r=(-2, -dy)
             5: r=(-1, -dy*2)
             6: r=(1, -dy*2)
             7: r=(2, -dy)
             */
            for( x = x1; x != x2; x += dx )
            {
                int xm = x*NR2, xd = xm*D2;

                int delta0 = minLr[0][xm - dx*NR2] + P2, delta1 = minLr[1][xm - NR2 + 1] + P2;
                int delta2 = minLr[1][xm + 2] + P2, delta3 = minLr[1][xm + NR2 + 3] + P2;

                CostType* Lr_p0 = Lr[0] + xd - dx*NRD2;
                CostType* Lr_p1 = Lr[1] + xd - NRD2 + D2;
                CostType* Lr_p2 = Lr[1] + xd + D2*2;
                CostType* Lr_p3 = Lr[1] + xd + NRD2 + D2*3;

                Lr_p0[-1] = Lr_p0[D] = Lr_p1[-1] = Lr_p1[D] =
                Lr_p2[-1] = Lr_p2[D] = Lr_p3[-1] = Lr_p3[D] = MAX_COST;

                CostType* Lr_p = Lr[0] + xd;
                const CostType* Cp = C + x*D;
                CostType* Sp = S + x*D;

            #if CV_SSE2
                if( useSIMD )
                {
                    __m128i _P1 = _mm_set1_epi16((short)P1);

                    __m128i _delta0 = _mm_set1_epi16((short)delta0);
                    __m128i _delta1 = _mm_set1_epi16((short)delta1);
                    __m128i _delta2 = _mm_set1_epi16((short)delta2);
                    __m128i _delta3 = _mm_set1_epi16((short)delta3);
                    __m128i _minL0 = _mm_set1_epi16((short)MAX_COST);

                    for( d = 0; d < D; d += 8 )
                    {
                        __m128i Cpd = _mm_load_si128((const __m128i*)(Cp + d));
                        __m128i L0, L1, L2, L3;

                        L0 = _mm_load_si128((const __m128i*)(Lr_p0 + d));
                        L1 = _mm_load_si128((const __m128i*)(Lr_p1 + d));
                        L2 = _mm_load_si128((const __m128i*)(Lr_p2 + d));
                        L3 = _mm_load_si128((const __m128i*)(Lr_p3 + d));

                        L0 = _mm_min_epi16(L0, _mm_adds_epi16(_mm_loadu_si128((const __m128i*)(Lr_p0 + d - 1)), _P1));
                        L0 = _mm_min_epi16(L0, _mm_adds_epi16(_mm_loadu_si128((const __m128i*)(Lr_p0 + d + 1)), _P1));

                        L1 = _mm_min_epi16(L1, _mm_adds_epi16(_mm_loadu_si128((const __m128i*)(Lr_p1 + d - 1)), _P1));
                        L1 = _mm_min_epi16(L1, _mm_adds_epi16(_mm_loadu_si128((const __m128i*)(Lr_p1 + d + 1)), _P1));

                        L2 = _mm_min_epi16(L2, _mm_adds_epi16(_mm_loadu_si128((const __m128i*)(Lr_p2 + d - 1)), _P1));
                        L2 = _mm_min_epi16(L2, _mm_adds_epi16(_mm_loadu_si128((const __m128i*)(Lr_p2 + d + 1)), _P1));

                        L3 = _mm_min_epi16(L3, _mm_adds_epi16(_mm_loadu_si128((const __m128i*)(Lr_p3 + d - 1)), _P1));
                        L3 = _mm_min_epi16(L3, _mm_adds_epi16(_mm_loadu_si128((const __m128i*)(Lr_p3 + d + 1)), _P1));

                        L0 = _mm_min_epi16(L0, _delta0);
                        L0 = _mm_adds_epi16(_mm_subs_epi16(L0, _delta0), Cpd);

                        L1 = _mm_min_epi16(L1, _delta1);
                        L1 = _mm_adds_epi16(_mm_subs_epi16(L1, _delta1), Cpd);

                        L2 = _mm_min_epi16(L2, _delta2);
                        L2 = _mm_adds_epi16(_mm_subs_epi16(L2, _delta2), Cpd);

                        L3 = _mm_min_epi16(L3, _delta3);
                        L3 = _mm_adds_epi16(_mm_subs_epi16(L3, _delta3), Cpd);

                        _mm_store_si128( (__m128i*)(Lr_p + d), L0);
                        _mm_store_si128( (__m128i*)(Lr_p + d + D2), L1);
                        _mm_store_si128( (__m128i*)(Lr_p + d + D2*2), L2);
                        _mm_store_si128( (__m128i*)(Lr_p + d + D2*3), L3);

                        __m128i t0 = _mm_min_epi16(_mm_unpacklo_epi16(L0, L2), _mm_unpackhi_epi16(L0, L2));
                        __m128i t1 = _mm_min_epi16(_mm_unpacklo_epi16(L1, L3), _mm_unpackhi_epi16(L1, L3));
                        t0 = _mm_min_epi16(_mm_unpacklo_epi16(t0, t1), _mm_unpackhi_epi16(t0, t1));
                        _minL0 = _mm_min_epi16(_minL0, t0);

                        __m128i Sval = _mm_load_si128((const __m128i*)(Sp + d));

                        L0 = _mm_adds_epi16(L0, L1);
                        L2 = _mm_adds_epi16(L2, L3);
                        Sval = _mm_adds_epi16(Sval, L0);
                        Sval = _mm_adds_epi16(Sval, L2);

                        _mm_store_si128((__m128i*)(Sp + d), Sval);
                    }

                    _minL0 = _mm_min_epi16(_minL0, _mm_srli_si128(_minL0, 8));
                    _mm_storel_epi64((__m128i*)&minLr[0][xm], _minL0);
                }
                else
            #endif
                {
                    int minL0 = MAX_COST, minL1 = MAX_COST, minL2 = MAX_COST, minL3 = MAX_COST;

                    for( d = 0; d < D; d++ )
                    {
                        int Cpd = Cp[d], L0, L1, L2, L3;

                        L0 = Cpd + std::min((int)Lr_p0[d], std::min(Lr_p0[d-1] + P1, std::min(Lr_p0[d+1] + P1, delta0))) - delta0;
                        L1 = Cpd + std::min((int)Lr_p1[d], std::min(Lr_p1[d-1] + P1, std::min(Lr_p1[d+1] + P1, delta1))) - delta1;
                        L2 = Cpd + std::min((int)Lr_p2[d], std::min(Lr_p2[d-1] + P1, std::min(Lr_p2[d+1] + P1, delta2))) - delta2;
                        L3 = Cpd + std::min((int)Lr_p3[d], std::min(Lr_p3[d-1] + P1, std::min(Lr_p3[d+1] + P1, delta3))) - delta3;

                        Lr_p[d] = (CostType)L0;
                        minL0 = std::min(minL0, L0);

                        Lr_p[d + D2] = (CostType)L1;
                        minL1 = std::min(minL1, L1);

                        Lr_p[d + D2*2] = (CostType)L2;
                        minL2 = std::min(minL2, L2);

                        Lr_p[d + D2*3] = (CostType)L3;
                        minL3 = std::min(minL3, L3);

                        Sp[d] = saturate_cast<CostType>(Sp[d] + L0 + L1 + L2 + L3);
                    }
                    minLr[0][xm] = (CostType)minL0;
                    minLr[0][xm+1] = (CostType)minL1;
                    minLr[0][xm+2] = (CostType)minL2;
                    minLr[0][xm+3] = (CostType)minL3;
                }
            }

            if( pass == npasses )
            {
                for( x = 0; x < width; x++ )
                {
                    disp1ptr[x] = disp2ptr[x] = (DispType)INVALID_DISP_SCALED;
                    disp2cost[x] = MAX_COST;
                }

                for( x = width1 - 1; x >= 0; x-- )
                {
                    CostType* Sp = S + x*D;
                    int minS = MAX_COST, bestDisp = -1;

                    if( npasses == 1 )
                    {
                        int xm = x*NR2, xd = xm*D2;

                        int minL0 = MAX_COST;
                        int delta0 = minLr[0][xm + NR2] + P2;
                        CostType* Lr_p0 = Lr[0] + xd + NRD2;
                        Lr_p0[-1] = Lr_p0[D] = MAX_COST;
                        CostType* Lr_p = Lr[0] + xd;

                        const CostType* Cp = C + x*D;

                    #if CV_SSE2
                        if( useSIMD )
                        {
                            __m128i _P1 = _mm_set1_epi16((short)P1);
                            __m128i _delta0 = _mm_set1_epi16((short)delta0);

                            __m128i _minL0 = _mm_set1_epi16((short)minL0);
                            __m128i _minS = _mm_set1_epi16(MAX_COST), _bestDisp = _mm_set1_epi16(-1);
                            __m128i _d8 = _mm_setr_epi16(0, 1, 2, 3, 4, 5, 6, 7), _8 = _mm_set1_epi16(8);

                            for( d = 0; d < D; d += 8 )
                            {
                                __m128i Cpd = _mm_load_si128((const __m128i*)(Cp + d)), L0;

                                L0 = _mm_load_si128((const __m128i*)(Lr_p0 + d));
                                L0 = _mm_min_epi16(L0, _mm_adds_epi16(_mm_loadu_si128((const __m128i*)(Lr_p0 + d - 1)), _P1));
                                L0 = _mm_min_epi16(L0, _mm_adds_epi16(_mm_loadu_si128((const __m128i*)(Lr_p0 + d + 1)), _P1));
                                L0 = _mm_min_epi16(L0, _delta0);
                                L0 = _mm_adds_epi16(_mm_subs_epi16(L0, _delta0), Cpd);

                                _mm_store_si128((__m128i*)(Lr_p + d), L0);
                                _minL0 = _mm_min_epi16(_minL0, L0);
                                L0 = _mm_adds_epi16(L0, *(__m128i*)(Sp + d));
                                _mm_store_si128((__m128i*)(Sp + d), L0);

                                __m128i mask = _mm_cmpgt_epi16(_minS, L0);
                                _minS = _mm_min_epi16(_minS, L0);
                                _bestDisp = _mm_xor_si128(_bestDisp, _mm_and_si128(_mm_xor_si128(_bestDisp,_d8), mask));
                                _d8 = _mm_adds_epi16(_d8, _8);
                            }

                            short CV_DECL_ALIGNED(16) bestDispBuf[8];
                            _mm_store_si128((__m128i*)bestDispBuf, _bestDisp);

                            _minL0 = _mm_min_epi16(_minL0, _mm_srli_si128(_minL0, 8));
                            _minL0 = _mm_min_epi16(_minL0, _mm_srli_si128(_minL0, 4));
                            _minL0 = _mm_min_epi16(_minL0, _mm_srli_si128(_minL0, 2));

                            __m128i qS = _mm_min_epi16(_minS, _mm_srli_si128(_minS, 8));
                            qS = _mm_min_epi16(qS, _mm_srli_si128(qS, 4));
                            qS = _mm_min_epi16(qS, _mm_srli_si128(qS, 2));

                            minLr[0][xm] = (CostType)_mm_cvtsi128_si32(_minL0);
                            minS = (CostType)_mm_cvtsi128_si32(qS);

                            qS = _mm_shuffle_epi32(_mm_unpacklo_epi16(qS, qS), 0);
                            qS = _mm_cmpeq_epi16(_minS, qS);
                            int idx = _mm_movemask_epi8(_mm_packs_epi16(qS, qS)) & 255;

                            bestDisp = bestDispBuf[LSBTab[idx]];
                        }
                        else
                    #endif
                        {
                            for( d = 0; d < D; d++ )
                            {
                                int L0 = Cp[d] + std::min((int)Lr_p0[d], std::min(Lr_p0[d-1] + P1, std::min(Lr_p0[d+1] + P1, delta0))) - delta0;

                                Lr_p[d] = (CostType)L0;
                                minL0 = std::min(minL0, L0);

                                int Sval = Sp[d] = saturate_cast<CostType>(Sp[d] + L0);
                                if( Sval < minS )
                                {
                                    minS = Sval;
                                    bestDisp = d;
                                }
                            }
                            minLr[0][xm] = (CostType)minL0;
                        }
                    }
                    else
                    {
                        for( d = 0; d < D; d++ )
                        {
                            int Sval = Sp[d];
                            if( Sval < minS )
                            {
                                minS = Sval;
                                bestDisp = d;
                            }
                        }
                    }

                    for( d = 0; d < D; d++ )
                    {
                        if( Sp[d]*(100 - uniquenessRatio) < minS*100 && std::abs(bestDisp - d) > 1 )
                            break;
                    }
                    if( d < D )
                        continue;
                    d = bestDisp;
                    int _x2 = x + minX1 - d - minD;
                    if( disp2cost[_x2] > minS )
                    {
                        disp2cost[_x2] = (CostType)minS;
                        disp2ptr[_x2] = (DispType)(d + minD);
                    }

                    if( 0 < d && d < D-1 )
                    {
                        // do subpixel quadratic interpolation:
                        //   fit parabola into (x1=d-1, y1=Sp[d-1]), (x2=d, y2=Sp[d]), (x3=d+1, y3=Sp[d+1])
                        //   then find minimum of the parabola.
                        int denom2 = std::max(Sp[d-1] + Sp[d+1] - 2*Sp[d], 1);
                        d = d*DISP_SCALE + ((Sp[d-1] - Sp[d+1])*DISP_SCALE + denom2)/(denom2*2);
                    }
                    else
                        d *= DISP_SCALE;
                    disp1ptr[x + minX1] = (DispType)(d + minD*DISP_SCALE);
                }

                for( x = minX1; x < maxX1; x++ )
                {
                    // we round the computed disparity both towards -inf and +inf and check
                    // if either of the corresponding disparities in disp2 is consistent.
                    // This is to give the computed disparity a chance to look valid if it is.
                    int d1 = disp1ptr[x];
                    if( d1 == INVALID_DISP_SCALED )
                        continue;
                    int _d = d1 >> DISP_SHIFT;
                    int d_ = (d1 + DISP_SCALE-1) >> DISP_SHIFT;
                    int _x = x - _d, x_ = x - d_;
                    if( 0 <= _x && _x < width && disp2ptr[_x] >= minD && std::abs(disp2ptr[_x] - _d) > disp12MaxDiff &&
                       0 <= x_ && x_ < width && disp2ptr[x_] >= minD && std::abs(disp2ptr[x_] - d_) > disp12MaxDiff )
                        disp1ptr[x] = (DispType)INVALID_DISP_SCALED;
                }
            }

            // now shift the cyclic buffers
            std::swap( Lr[0], Lr[1] );
            std::swap( minLr[0], minLr[1] );
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

void getBufferPointers(Mat& buffer, int width, int width1, int D, int num_ch, int SH2, int P2,
                       CostType*& curCostVolumeLine, CostType*& hsumBuf, CostType*& pixDiff,
                       PixType*& tmpBuf, CostType*& horPassCostVolume,
                       CostType*& vertPassCostVolume, CostType*& vertPassMin, CostType*& rightPassBuf,
                       CostType*& disp2CostBuf, short*& disp2Buf);

struct SGBM3WayMainLoop : public ParallelLoopBody{
    Mat* buffers;
    const Mat *img1, *img2;
    Mat descriptors_img1, descriptors_img2;
    Mat* dst_disp;

    int nstripes, stripe_sz;
    int stripe_overlap;

    int width,height;
    int minD, maxD, D;
    int minX1, maxX1, width1;

    int SW2, SH2;
    int P1, P2;
    int uniquenessRatio, disp12MaxDiff;

    int costBufSize, hsumBufNRows;
    int TAB_OFS, ftzero;

    PixType* clip_table;
    std::unique_ptr<abstract_stereo_cost_calculator> cost_calculator;

    SGBM3WayMainLoop(Mat *_buffers, const Mat& _img1, const Mat& _img2, Mat* _dst_disp,
    		const semiglobal_matcher_parameters& params, PixType* clip_table, int _nstripes, int _stripe_overlap,
			pixel_cost_type cost_type = pixel_cost_type::BIRCHFIELD_TOMASI);

    void getRawMatchingCost(CostType* C, CostType* hsumBuf, CostType* pixDiff, PixType* tmpBuf,
    		int y, int src_start_idx) const;
    void operator () (const Range& range) const;
};

SGBM3WayMainLoop::SGBM3WayMainLoop(Mat *_buffers, const Mat& _img1, const Mat& _img2, Mat* _dst_disp,
		const semiglobal_matcher_parameters& params, PixType* clip_table, int _nstripes, int _stripe_overlap,
		pixel_cost_type cost_type):
		buffers(_buffers), img1(&_img1), img2(&_img2), dst_disp(_dst_disp), clip_table(clip_table),
		cost_calculator(build_stereo_cost_calculator(cost_type,_img1,_img2,params)){

    nstripes = _nstripes;
    stripe_overlap = _stripe_overlap;
    stripe_sz = (int)ceil(img1->rows/(double)nstripes);

    width = img1->cols; height = img1->rows;
    minD = params.minDisparity; maxD = minD + params.numDisparities; D = maxD - minD;
    minX1 = std::max(maxD, 0); maxX1 = width + std::min(minD, 0); width1 = maxX1 - minX1;
    CV_Assert( D % 16 == 0 );

    SW2 = SH2 = params.block_size > 0 ? params.block_size/2 : 1;

    P1 = params.P1 > 0 ? params.P1 : 2; P2 = std::max(params.P2 > 0 ? params.P2 : 5, P1+1);
    uniquenessRatio = params.uniquenessRatio >= 0 ? params.uniquenessRatio : 10;
    disp12MaxDiff = params.disp12MaxDiff > 0 ? params.disp12MaxDiff : 1;

    costBufSize = width1*D;
    hsumBufNRows = SH2*2 + 2;
    TAB_OFS = 256*4;
    ftzero = std::max(params.preFilterCap, 15) | 1;

}

void getBufferPointers(Mat& buffer, int width, int width1, int D, int num_ch, int SH2, int P2,
                       CostType*& curCostVolumeLine, CostType*& hsumBuf, CostType*& pixDiff,
                       PixType*& tmpBuf, CostType*& horPassCostVolume,
                       CostType*& vertPassCostVolume, CostType*& vertPassMin, CostType*& rightPassBuf,
                       CostType*& disp2CostBuf, short*& disp2Buf){
    // allocating all the required memory:
    int costVolumeLineSize = width1*D;
    int width1_ext = width1+2;
    int costVolumeLineSize_ext = width1_ext*D;
    int hsumBufNRows = SH2*2 + 2;

    // main buffer to store matching costs for the current line:
    int curCostVolumeLineSize = costVolumeLineSize*sizeof(CostType);

    // auxiliary buffers for the raw matching cost computation:
    int hsumBufSize  = costVolumeLineSize*hsumBufNRows*sizeof(CostType);
    int pixDiffSize  = costVolumeLineSize*sizeof(CostType);
    int tmpBufSize   = width*16*num_ch*sizeof(PixType);

    // auxiliary buffers for the matching cost aggregation:
    int horPassCostVolumeSize  = costVolumeLineSize_ext*sizeof(CostType); // buffer for the 2-pass horizontal cost aggregation
    int vertPassCostVolumeSize = costVolumeLineSize_ext*sizeof(CostType); // buffer for the vertical cost aggregation
    int vertPassMinSize        = width1_ext*sizeof(CostType);             // buffer for storing minimum costs from the previous line
    int rightPassBufSize       = D*sizeof(CostType);                      // additional small buffer for the right-to-left pass

    // buffers for the pseudo-LRC check:
    int disp2CostBufSize = width*sizeof(CostType);
    int disp2BufSize     = width*sizeof(short);

    // sum up the sizes of all the buffers:
    size_t totalBufSize = curCostVolumeLineSize +
                          hsumBufSize +
                          pixDiffSize +
                          tmpBufSize  +
                          horPassCostVolumeSize +
                          vertPassCostVolumeSize +
                          vertPassMinSize +
                          rightPassBufSize +
                          disp2CostBufSize +
                          disp2BufSize +
                          16;  //to compensate for the alignPtr shifts

    if( buffer.empty() || !buffer.isContinuous() || buffer.cols*buffer.rows*buffer.elemSize() < totalBufSize )
        buffer.create(1, (int)totalBufSize, CV_8U);

    // set up all the pointers:
    curCostVolumeLine  = (CostType*)alignPtr(buffer.ptr(), 16);
    hsumBuf            = curCostVolumeLine + costVolumeLineSize;
    pixDiff            = hsumBuf + costVolumeLineSize*hsumBufNRows;
    tmpBuf             = (PixType*)(pixDiff + costVolumeLineSize);
    horPassCostVolume  = (CostType*)(tmpBuf + width*16*num_ch);
    vertPassCostVolume = horPassCostVolume + costVolumeLineSize_ext;
    rightPassBuf       = vertPassCostVolume + costVolumeLineSize_ext;
    vertPassMin        = rightPassBuf + D;
    disp2CostBuf       = vertPassMin + width1_ext;
    disp2Buf           = disp2CostBuf + width;

    // initialize memory:
    memset(buffer.ptr(),0,totalBufSize);
    for(int i=0;i<costVolumeLineSize;i++)
        curCostVolumeLine[i] = (CostType)P2; //such initialization simplifies the cost aggregation loops a bit
}

// performing block matching and building raw cost-volume for the current row
void SGBM3WayMainLoop::getRawMatchingCost(CostType* C, // target cost-volume row
                                          CostType* hsumBuf, CostType* pixDiff, PixType* tmpBuf, //buffers
                                          int y, int src_start_idx) const{
    int x, d;
    int dy1 = (y == src_start_idx) ? src_start_idx : y + SH2, dy2 = (y == src_start_idx) ? src_start_idx+SH2 : dy1;

    for(int k = dy1; k <= dy2; k++ )
    {
        CostType* hsumAdd = hsumBuf + (std::min(k, height-1) % hsumBufNRows)*costBufSize;
        if( k < height )
        {
            cost_calculator->compute(k,pixDiff);

            memset(hsumAdd, 0, D*sizeof(CostType));
            for(x = 0; x <= SW2*D; x += D )
            {
                int scale = x == 0 ? SW2 + 1 : 1;

                for( d = 0; d < D; d++ )
                    hsumAdd[d] = (CostType)(hsumAdd[d] + pixDiff[x + d]*scale);
            }

            if( y > src_start_idx )
            {
                const CostType* hsumSub = hsumBuf + (std::max(y - SH2 - 1, src_start_idx) % hsumBufNRows)*costBufSize;

                for( x = D; x < width1*D; x += D )
                {
                    const CostType* pixAdd = pixDiff + std::min(x + SW2*D, (width1-1)*D);
                    const CostType* pixSub = pixDiff + std::max(x - (SW2+1)*D, 0);

#if CV_SIMD128
                    v_int16x8 hv_reg;
                    for( d = 0; d < D; d+=8 )
                    {
                        hv_reg = v_load_aligned(hsumAdd+x-D+d) + (v_load_aligned(pixAdd+d) - v_load_aligned(pixSub+d));
                        v_store_aligned(hsumAdd+x+d,hv_reg);
                        v_store_aligned(C+x+d,v_load_aligned(C+x+d)+(hv_reg-v_load_aligned(hsumSub+x+d)));
                    }
#else
                    for( d = 0; d < D; d++ )
                    {
                        int hv = hsumAdd[x + d] = (CostType)(hsumAdd[x - D + d] + pixAdd[d] - pixSub[d]);
                        C[x + d] = (CostType)(C[x + d] + hv - hsumSub[x + d]);
                    }
#endif
                }
            }
            else
            {
                for( x = D; x < width1*D; x += D )
                {
                    const CostType* pixAdd = pixDiff + std::min(x + SW2*D, (width1-1)*D);
                    const CostType* pixSub = pixDiff + std::max(x - (SW2+1)*D, 0);

                    for( d = 0; d < D; d++ )
                        hsumAdd[x + d] = (CostType)(hsumAdd[x - D + d] + pixAdd[d] - pixSub[d]);
                }
            }
        }

        if( y == src_start_idx )
        {
            int scale = k == src_start_idx ? SH2 + 1 : 1;
            for( x = 0; x < width1*D; x++ )
                C[x] = (CostType)(C[x] + hsumAdd[x]*scale);
        }
    }
}

#if CV_SIMD128
// define some additional reduce operations:
inline short min(const v_int16x8& a)
{
    short CV_DECL_ALIGNED(16) buf[8];
    v_store_aligned(buf, a);
    short s0 = std::min(buf[0], buf[1]);
    short s1 = std::min(buf[2], buf[3]);
    short s2 = std::min(buf[4], buf[5]);
    short s3 = std::min(buf[6], buf[7]);
    return std::min(std::min(s0, s1),std::min(s2, s3));
}

inline short min_pos(const v_int16x8& val,const v_int16x8& pos)
{
    short CV_DECL_ALIGNED(16) val_buf[8];
    v_store_aligned(val_buf, val);
    short CV_DECL_ALIGNED(16) pos_buf[8];
    v_store_aligned(pos_buf, pos);
    short res_pos = 0;
    short min_val = SHRT_MAX;
    if(val_buf[0]<min_val) {min_val=val_buf[0]; res_pos=pos_buf[0];}
    if(val_buf[1]<min_val) {min_val=val_buf[1]; res_pos=pos_buf[1];}
    if(val_buf[2]<min_val) {min_val=val_buf[2]; res_pos=pos_buf[2];}
    if(val_buf[3]<min_val) {min_val=val_buf[3]; res_pos=pos_buf[3];}
    if(val_buf[4]<min_val) {min_val=val_buf[4]; res_pos=pos_buf[4];}
    if(val_buf[5]<min_val) {min_val=val_buf[5]; res_pos=pos_buf[5];}
    if(val_buf[6]<min_val) {min_val=val_buf[6]; res_pos=pos_buf[6];}
    if(val_buf[7]<min_val) {min_val=val_buf[7]; res_pos=pos_buf[7];}
    return res_pos;
}
#endif

// performing SGM cost accumulation from left to right (result is stored in leftBuf) and
// in-place cost accumulation from top to bottom (result is stored in topBuf)
inline void accumulateCostsLeftTop(CostType* leftBuf, CostType* leftBuf_prev, CostType* topBuf, CostType* costs,
                                   CostType& leftMinCost, CostType& topMinCost, int D, int P1, int P2)
{
#if CV_SIMD128
    v_int16x8 P1_reg = v_setall_s16(cv::saturate_cast<CostType>(P1));

    v_int16x8 leftMinCostP2_reg   = v_setall_s16(cv::saturate_cast<CostType>(leftMinCost+P2));
    v_int16x8 leftMinCost_new_reg = v_setall_s16(SHRT_MAX);
    v_int16x8 src0_leftBuf        = v_setall_s16(SHRT_MAX);
    v_int16x8 src1_leftBuf        = v_load_aligned(leftBuf_prev);

    v_int16x8 topMinCostP2_reg   = v_setall_s16(cv::saturate_cast<CostType>(topMinCost+P2));
    v_int16x8 topMinCost_new_reg = v_setall_s16(SHRT_MAX);
    v_int16x8 src0_topBuf        = v_setall_s16(SHRT_MAX);
    v_int16x8 src1_topBuf        = v_load_aligned(topBuf);

    v_int16x8 src2;
    v_int16x8 src_shifted_left,src_shifted_right;
    v_int16x8 res;

    for(int i=0;i<D-8;i+=8)
    {
        //process leftBuf:
        //lookahead load:
        src2 = v_load_aligned(leftBuf_prev+i+8);

        //get shifted versions of the current block and add P1:
        src_shifted_left  = v_extract<7> (src0_leftBuf,src1_leftBuf) + P1_reg;
        src_shifted_right = v_extract<1> (src1_leftBuf,src2        ) + P1_reg;

        // process and save current block:
        res = v_load_aligned(costs+i) + (v_min(v_min(src_shifted_left,src_shifted_right),v_min(src1_leftBuf,leftMinCostP2_reg))-leftMinCostP2_reg);
        leftMinCost_new_reg = v_min(leftMinCost_new_reg,res);
        v_store_aligned(leftBuf+i, res);

        //update src buffers:
        src0_leftBuf = src1_leftBuf;
        src1_leftBuf = src2;

        //process topBuf:
        //lookahead load:
        src2 = v_load_aligned(topBuf+i+8);

        //get shifted versions of the current block and add P1:
        src_shifted_left  = v_extract<7> (src0_topBuf,src1_topBuf) + P1_reg;
        src_shifted_right = v_extract<1> (src1_topBuf,src2       ) + P1_reg;

        // process and save current block:
        res = v_load_aligned(costs+i) + (v_min(v_min(src_shifted_left,src_shifted_right),v_min(src1_topBuf,topMinCostP2_reg))-topMinCostP2_reg);
        topMinCost_new_reg = v_min(topMinCost_new_reg,res);
        v_store_aligned(topBuf+i, res);

        //update src buffers:
        src0_topBuf = src1_topBuf;
        src1_topBuf = src2;
    }

    // a bit different processing for the last cycle of the loop:
    //process leftBuf:
    src2 = v_setall_s16(SHRT_MAX);
    src_shifted_left  = v_extract<7> (src0_leftBuf,src1_leftBuf) + P1_reg;
    src_shifted_right = v_extract<1> (src1_leftBuf,src2        ) + P1_reg;

    res = v_load_aligned(costs+D-8) + (v_min(v_min(src_shifted_left,src_shifted_right),v_min(src1_leftBuf,leftMinCostP2_reg))-leftMinCostP2_reg);
    leftMinCost = min(v_min(leftMinCost_new_reg,res));
    v_store_aligned(leftBuf+D-8, res);

    //process topBuf:
    src2 = v_setall_s16(SHRT_MAX);
    src_shifted_left  = v_extract<7> (src0_topBuf,src1_topBuf) + P1_reg;
    src_shifted_right = v_extract<1> (src1_topBuf,src2       ) + P1_reg;

    res = v_load_aligned(costs+D-8) + (v_min(v_min(src_shifted_left,src_shifted_right),v_min(src1_topBuf,topMinCostP2_reg))-topMinCostP2_reg);
    topMinCost = min(v_min(topMinCost_new_reg,res));
    v_store_aligned(topBuf+D-8, res);
#else
    CostType leftMinCost_new = SHRT_MAX;
    CostType topMinCost_new  = SHRT_MAX;
    int leftMinCost_P2  = leftMinCost + P2;
    int topMinCost_P2   = topMinCost  + P2;
    CostType leftBuf_prev_i_minus_1 = SHRT_MAX;
    CostType topBuf_i_minus_1       = SHRT_MAX;
    CostType tmp;

    for(int i=0;i<D-1;i++)
    {
        leftBuf[i] = cv::saturate_cast<CostType>(costs[i] + std::min(std::min(leftBuf_prev_i_minus_1+P1,leftBuf_prev[i+1]+P1),std::min((int)leftBuf_prev[i],leftMinCost_P2))-leftMinCost_P2);
        leftBuf_prev_i_minus_1 = leftBuf_prev[i];
        leftMinCost_new = std::min(leftMinCost_new,leftBuf[i]);

        tmp = topBuf[i];
        topBuf[i]  = cv::saturate_cast<CostType>(costs[i] + std::min(std::min(topBuf_i_minus_1+P1,topBuf[i+1]+P1),std::min((int)topBuf[i],topMinCost_P2))-topMinCost_P2);
        topBuf_i_minus_1 = tmp;
        topMinCost_new  = std::min(topMinCost_new,topBuf[i]);
    }

    leftBuf[D-1] = cv::saturate_cast<CostType>(costs[D-1] + std::min(leftBuf_prev_i_minus_1+P1,std::min((int)leftBuf_prev[D-1],leftMinCost_P2))-leftMinCost_P2);
    leftMinCost = std::min(leftMinCost_new,leftBuf[D-1]);

    topBuf[D-1]  = cv::saturate_cast<CostType>(costs[D-1] + std::min(topBuf_i_minus_1+P1,std::min((int)topBuf[D-1],topMinCost_P2))-topMinCost_P2);
    topMinCost  = std::min(topMinCost_new,topBuf[D-1]);
#endif
}

// performing in-place SGM cost accumulation from right to left (the result is stored in rightBuf) and
// summing rightBuf, topBuf, leftBuf together (the result is stored in leftBuf), as well as finding the
// optimal disparity value with minimum accumulated cost
inline void accumulateCostsRight(CostType* rightBuf, CostType* topBuf, CostType* leftBuf, CostType* costs,
                                 CostType& rightMinCost, int D, int P1, int P2, int& optimal_disp, CostType& min_cost)
{
#if CV_SIMD128
    v_int16x8 P1_reg = v_setall_s16(cv::saturate_cast<CostType>(P1));

    v_int16x8 rightMinCostP2_reg   = v_setall_s16(cv::saturate_cast<CostType>(rightMinCost+P2));
    v_int16x8 rightMinCost_new_reg = v_setall_s16(SHRT_MAX);
    v_int16x8 src0_rightBuf        = v_setall_s16(SHRT_MAX);
    v_int16x8 src1_rightBuf        = v_load(rightBuf);

    v_int16x8 src2;
    v_int16x8 src_shifted_left,src_shifted_right;
    v_int16x8 res;

    v_int16x8 min_sum_cost_reg = v_setall_s16(SHRT_MAX);
    v_int16x8 min_sum_pos_reg  = v_setall_s16(0);
    v_int16x8 loop_idx(0,1,2,3,4,5,6,7);
    v_int16x8 eight_reg = v_setall_s16(8);

    for(int i=0;i<D-8;i+=8)
    {
        //lookahead load:
        src2 = v_load_aligned(rightBuf+i+8);

        //get shifted versions of the current block and add P1:
        src_shifted_left  = v_extract<7> (src0_rightBuf,src1_rightBuf) + P1_reg;
        src_shifted_right = v_extract<1> (src1_rightBuf,src2         ) + P1_reg;

        // process and save current block:
        res = v_load_aligned(costs+i) + (v_min(v_min(src_shifted_left,src_shifted_right),v_min(src1_rightBuf,rightMinCostP2_reg))-rightMinCostP2_reg);
        rightMinCost_new_reg = v_min(rightMinCost_new_reg,res);
        v_store_aligned(rightBuf+i, res);

        // compute and save total cost:
        res = res + v_load_aligned(leftBuf+i) + v_load_aligned(topBuf+i);
        v_store_aligned(leftBuf+i, res);

        // track disparity value with the minimum cost:
        min_sum_cost_reg = v_min(min_sum_cost_reg,res);
        min_sum_pos_reg = min_sum_pos_reg + ((min_sum_cost_reg == res) & (loop_idx - min_sum_pos_reg));
        loop_idx = loop_idx+eight_reg;

        //update src:
        src0_rightBuf    = src1_rightBuf;
        src1_rightBuf    = src2;
    }

    // a bit different processing for the last cycle of the loop:
    src2 = v_setall_s16(SHRT_MAX);
    src_shifted_left  = v_extract<7> (src0_rightBuf,src1_rightBuf) + P1_reg;
    src_shifted_right = v_extract<1> (src1_rightBuf,src2         ) + P1_reg;

    res = v_load_aligned(costs+D-8) + (v_min(v_min(src_shifted_left,src_shifted_right),v_min(src1_rightBuf,rightMinCostP2_reg))-rightMinCostP2_reg);
    rightMinCost = min(v_min(rightMinCost_new_reg,res));
    v_store_aligned(rightBuf+D-8, res);

    res = res + v_load_aligned(leftBuf+D-8) + v_load_aligned(topBuf+D-8);
    v_store_aligned(leftBuf+D-8, res);

    min_sum_cost_reg = v_min(min_sum_cost_reg,res);
    min_cost = min(min_sum_cost_reg);
    min_sum_pos_reg = min_sum_pos_reg + ((min_sum_cost_reg == res) & (loop_idx - min_sum_pos_reg));
    optimal_disp = min_pos(min_sum_cost_reg,min_sum_pos_reg);
#else
    CostType rightMinCost_new = SHRT_MAX;
    int rightMinCost_P2  = rightMinCost + P2;
    CostType rightBuf_i_minus_1 = SHRT_MAX;
    CostType tmp;
    min_cost = SHRT_MAX;

    for(int i=0;i<D-1;i++)
    {
        tmp = rightBuf[i];
        rightBuf[i]  = cv::saturate_cast<CostType>(costs[i] + std::min(std::min(rightBuf_i_minus_1+P1,rightBuf[i+1]+P1),std::min((int)rightBuf[i],rightMinCost_P2))-rightMinCost_P2);
        rightBuf_i_minus_1 = tmp;
        rightMinCost_new  = std::min(rightMinCost_new,rightBuf[i]);
        leftBuf[i] = cv::saturate_cast<CostType>((int)leftBuf[i]+rightBuf[i]+topBuf[i]);
        if(leftBuf[i]<min_cost)
        {
            optimal_disp = i;
            min_cost = leftBuf[i];
        }
    }

    rightBuf[D-1]  = cv::saturate_cast<CostType>(costs[D-1] + std::min(rightBuf_i_minus_1+P1,std::min((int)rightBuf[D-1],rightMinCost_P2))-rightMinCost_P2);
    rightMinCost  = std::min(rightMinCost_new,rightBuf[D-1]);
    leftBuf[D-1] = cv::saturate_cast<CostType>((int)leftBuf[D-1]+rightBuf[D-1]+topBuf[D-1]);
    if(leftBuf[D-1]<min_cost)
    {
        optimal_disp = D-1;
        min_cost = leftBuf[D-1];
    }
#endif
}

void SGBM3WayMainLoop::operator () (const Range& range) const
{
    // force separate processing of stripes:
    if(range.end>range.start+1)
    {
        for(int n=range.start;n<range.end;n++)
            (*this)(Range(n,n+1));
        return;
    }

    const int DISP_SCALE = (1 << StereoMatcher::DISP_SHIFT);
    int INVALID_DISP = minD - 1, INVALID_DISP_SCALED = INVALID_DISP*DISP_SCALE;

    // setting up the ranges:
    int src_start_idx = std::max(std::min(range.start * stripe_sz - stripe_overlap, height),0);
    int src_end_idx   = std::min(range.end   * stripe_sz, height);

    int dst_offset;
    if(range.start==0)
        dst_offset=stripe_overlap;
    else
        dst_offset=0;

    Mat cur_buffer = buffers [range.start];
    Mat cur_disp   = dst_disp[range.start];
    cur_disp = Scalar(INVALID_DISP_SCALED);

    // prepare buffers:
    CostType *curCostVolumeLine, *hsumBuf, *pixDiff;
    PixType* tmpBuf;
    CostType *horPassCostVolume, *vertPassCostVolume, *vertPassMin, *rightPassBuf, *disp2CostBuf;
    short* disp2Buf;
    getBufferPointers(cur_buffer,width,width1,D,img1->channels(),SH2,P2,
                      curCostVolumeLine,hsumBuf,pixDiff,tmpBuf,horPassCostVolume,
                      vertPassCostVolume,vertPassMin,rightPassBuf,disp2CostBuf,disp2Buf);

    // start real processing:
    for(int y=src_start_idx;y<src_end_idx;y++)
    {
        getRawMatchingCost(curCostVolumeLine,hsumBuf,pixDiff,tmpBuf,y,src_start_idx);

        short* disp_row = (short*)cur_disp.ptr(dst_offset+(y-src_start_idx));

        // initialize the auxiliary buffers for the pseudo left-right consistency check:
        for(int x=0;x<width;x++)
        {
            disp2Buf[x] = (short)INVALID_DISP_SCALED;
            disp2CostBuf[x] = SHRT_MAX;
        }
        CostType* C = curCostVolumeLine - D;
        CostType prev_min, min_cost;
        int d, best_d;
        d = best_d = 0;

        // forward pass
        prev_min=0;
        for (int x=D;x<(1+width1)*D;x+=D)
            accumulateCostsLeftTop(horPassCostVolume+x,horPassCostVolume+x-D,vertPassCostVolume+x,C+x,prev_min,vertPassMin[x/D],D,P1,P2);

        //backward pass
        memset(rightPassBuf,0,D*sizeof(CostType));
        prev_min=0;
        for (int x=width1*D;x>=D;x-=D)
        {
            accumulateCostsRight(rightPassBuf,vertPassCostVolume+x,horPassCostVolume+x,C+x,prev_min,D,P1,P2,best_d,min_cost);

            if(uniquenessRatio>0)
            {
#if CV_SIMD128
                horPassCostVolume+=x;
                int thresh = (100*min_cost)/(100-uniquenessRatio);
                v_int16x8 thresh_reg = v_setall_s16((short)(thresh+1));
                v_int16x8 d1 = v_setall_s16((short)(best_d-1));
                v_int16x8 d2 = v_setall_s16((short)(best_d+1));
                v_int16x8 eight_reg = v_setall_s16(8);
                v_int16x8 cur_d(0,1,2,3,4,5,6,7);
                v_int16x8 mask,cost1,cost2;

                for( d = 0; d < D; d+=16 )
                {
                    cost1 = v_load_aligned(horPassCostVolume+d);
                    cost2 = v_load_aligned(horPassCostVolume+d+8);

                    mask = cost1 < thresh_reg;
                    mask = mask & ( (cur_d<d1) | (cur_d>d2) );
                    if( v_signmask(mask) )
                        break;

                    cur_d = cur_d+eight_reg;

                    mask = cost2 < thresh_reg;
                    mask = mask & ( (cur_d<d1) | (cur_d>d2) );
                    if( v_signmask(mask) )
                        break;

                    cur_d = cur_d+eight_reg;
                }
                horPassCostVolume-=x;
#else
                for( d = 0; d < D; d++ )
                {
                    if( horPassCostVolume[x+d]*(100 - uniquenessRatio) < min_cost*100 && std::abs(d - best_d) > 1 )
                        break;
                }
#endif
                if( d < D )
                    continue;
            }
            d = best_d;

            int _x2 = x/D - 1 + minX1 - d - minD;
            if( _x2>=0 && _x2<width && disp2CostBuf[_x2] > min_cost )
            {
                disp2CostBuf[_x2] = min_cost;
                disp2Buf[_x2] = (short)(d + minD);
            }

            if( 0 < d && d < D-1 )
            {
                // do subpixel quadratic interpolation:
                //   fit parabola into (x1=d-1, y1=Sp[d-1]), (x2=d, y2=Sp[d]), (x3=d+1, y3=Sp[d+1])
                //   then find minimum of the parabola.
                int denom2 = std::max(horPassCostVolume[x+d-1] + horPassCostVolume[x+d+1] - 2*horPassCostVolume[x+d], 1);
                d = d*DISP_SCALE + ((horPassCostVolume[x+d-1] - horPassCostVolume[x+d+1])*DISP_SCALE + denom2)/(denom2*2);
            }
            else
                d *= DISP_SCALE;

            disp_row[(x/D)-1 + minX1] = (DispType)(d + minD*DISP_SCALE);
        }

        for(int x = minX1; x < maxX1; x++ )
        {
            // pseudo LRC consistency check using only one disparity map;
            // pixels with difference more than disp12MaxDiff are invalidated
            int d1 = disp_row[x];
            if( d1 == INVALID_DISP_SCALED )
                continue;
            int _d = d1 >> StereoMatcher::DISP_SHIFT;
            int d_ = (d1 + DISP_SCALE-1) >> StereoMatcher::DISP_SHIFT;
            int _x = x - _d, x_ = x - d_;
            if( 0 <= _x && _x < width && disp2Buf[_x] >= minD && std::abs(disp2Buf[_x] - _d) > disp12MaxDiff &&
                0 <= x_ && x_ < width && disp2Buf[x_] >= minD && std::abs(disp2Buf[x_] - d_) > disp12MaxDiff )
                disp_row[x] = (short)INVALID_DISP_SCALED;
        }
    }
}

static void computeDisparity3WaySGBM( const Mat& img1, const Mat& img2,
                                      Mat& disp1, const semiglobal_matcher_parameters& params,
                                      Mat* buffers, int nstripes, pixel_cost_type cost_type = pixel_cost_type::BIRCHFIELD_TOMASI)
{
    // precompute a lookup table for the raw matching cost computation:
    const int TAB_OFS = 256*4, TAB_SIZE = 256 + TAB_OFS*2;

    PixType* clip_table = new PixType[TAB_SIZE];
    int ftzero = std::max(params.preFilterCap, 15) | 1;
    //value at TAB_OFS = ftzero
    //values beyond that grow for ftzero up to 2*ftzero, then keep at 2*ftzero
    for(int k = 0; k < TAB_SIZE; k++ )
        clip_table[k] = (PixType)(std::min(std::max(k - TAB_OFS, -ftzero), ftzero) + ftzero);

    // allocate separate dst_disp arrays to avoid conflicts due to stripe overlap:
    int stripe_sz = (int)ceil(img1.rows/(double)nstripes);
    int stripe_overlap = (params.block_size/2+1) + (int)ceil(0.1*stripe_sz);
    Mat* dst_disp = new Mat[nstripes];
    for(int i=0;i<nstripes;i++)
        dst_disp[i].create(stripe_sz+stripe_overlap,img1.cols,CV_16S);

    parallel_for_(Range(0,nstripes),SGBM3WayMainLoop(buffers,img1,img2,dst_disp,params,clip_table,nstripes,stripe_overlap, cost_type));

    //assemble disp1 from dst_disp:
    short* dst_row;
    short* src_row;
    for(int i=0;i<disp1.rows;i++)
    {
        dst_row = (short*)disp1.ptr(i);
        src_row = (short*)dst_disp[i/stripe_sz].ptr(stripe_overlap+i%stripe_sz);
        memcpy(dst_row,src_row,disp1.cols*sizeof(short));
    }

    delete[] clip_table;
    delete[] dst_disp;
}

class semiglobal_matcher_implementation : public StereoSGBM
{
private:
	pixel_cost_type cost_type;

public:
    semiglobal_matcher_implementation(): cost_type(pixel_cost_type::BIRCHFIELD_TOMASI), params() {}

    semiglobal_matcher_implementation( int _minDisparity, int _numDisparities, int _SADWindowSize,
                    int _P1, int _P2, int _disp12MaxDiff, int _preFilterCap,
                    int _uniquenessRatio, int _speckleWindowSize, int _speckleRange,
                    int _mode ):cost_type(pixel_cost_type::BIRCHFIELD_TOMASI),params( _minDisparity, _numDisparities, _SADWindowSize,
                                   _P1, _P2, _disp12MaxDiff, _preFilterCap,
                                   _uniquenessRatio, _speckleWindowSize, _speckleRange,
                                   _mode ){}

    semiglobal_matcher_implementation( int _minDisparity, int _numDisparities, int _SADWindowSize,
                        int _P1, int _P2, int _disp12MaxDiff, int _preFilterCap,
                        int _uniquenessRatio, int _speckleWindowSize, int _speckleRange,
                        int _mode, pixel_cost_type cost_type):
                        	cost_type(cost_type),
                        	params( _minDisparity, _numDisparities, _SADWindowSize,
								   _P1, _P2, _disp12MaxDiff, _preFilterCap,
								   _uniquenessRatio, _speckleWindowSize, _speckleRange,
								   _mode )
						    {}

    void compute( InputArray leftarr, InputArray rightarr, OutputArray disparr ){
        Mat left = leftarr.getMat(), right = rightarr.getMat();
        CV_Assert( left.size() == right.size() && left.type() == right.type() &&
                   left.depth() == CV_8U );

        disparr.create( left.size(), CV_16S );
        Mat disp = disparr.getMat();

        if(params.mode==MODE_SGBM_3WAY){
        	computeDisparity3WaySGBM( left, right, disp, params, buffers, num_stripes, cost_type );
        }else{
        	computeDisparitySGBM( left, right, disp, params, buffer, cost_type );
        }

        medianBlur(disp, disp, 3);

        if( params.speckleWindowSize > 0 ){
            filterSpeckles(disp, (params.minDisparity - 1)*StereoMatcher::DISP_SCALE, params.speckleWindowSize,
                           StereoMatcher::DISP_SCALE*params.speckleRange, buffer);
        }
    }

    int getMinDisparity() const { return params.minDisparity; }
    void setMinDisparity(int minDisparity) { params.minDisparity = minDisparity; }

    int getNumDisparities() const { return params.numDisparities; }
    void setNumDisparities(int numDisparities) { params.numDisparities = numDisparities; }

    int getBlockSize() const { return params.block_size; }
    void setBlockSize(int blockSize) { params.block_size = blockSize; }

    int getSpeckleWindowSize() const { return params.speckleWindowSize; }
    void setSpeckleWindowSize(int speckleWindowSize) { params.speckleWindowSize = speckleWindowSize; }

    int getSpeckleRange() const { return params.speckleRange; }
    void setSpeckleRange(int speckleRange) { params.speckleRange = speckleRange; }

    int getDisp12MaxDiff() const { return params.disp12MaxDiff; }
    void setDisp12MaxDiff(int disp12MaxDiff) { params.disp12MaxDiff = disp12MaxDiff; }

    int getPreFilterCap() const { return params.preFilterCap; }
    void setPreFilterCap(int preFilterCap) { params.preFilterCap = preFilterCap; }

    int getUniquenessRatio() const { return params.uniquenessRatio; }
    void setUniquenessRatio(int uniquenessRatio) { params.uniquenessRatio = uniquenessRatio; }

    int getP1() const { return params.P1; }
    void setP1(int P1) { params.P1 = P1; }

    int getP2() const { return params.P2; }
    void setP2(int P2) { params.P2 = P2; }

    int getMode() const { return params.mode; }
    void setMode(int mode) { params.mode = mode; }

    void write(FileStorage& fs) const
    {
        fs << "name" << name_
        << "minDisparity" << params.minDisparity
        << "numDisparities" << params.numDisparities
        << "blockSize" << params.block_size
        << "speckleWindowSize" << params.speckleWindowSize
        << "speckleRange" << params.speckleRange
        << "disp12MaxDiff" << params.disp12MaxDiff
        << "preFilterCap" << params.preFilterCap
        << "uniquenessRatio" << params.uniquenessRatio
        << "P1" << params.P1
        << "P2" << params.P2
        << "mode" << params.mode;
    }

    void read(const FileNode& fn)
    {
        FileNode n = fn["name"];
        CV_Assert( n.isString() && String(n) == name_ );
        params.minDisparity = (int)fn["minDisparity"];
        params.numDisparities = (int)fn["numDisparities"];
        params.block_size = (int)fn["blockSize"];
        params.speckleWindowSize = (int)fn["speckleWindowSize"];
        params.speckleRange = (int)fn["speckleRange"];
        params.disp12MaxDiff = (int)fn["disp12MaxDiff"];
        params.preFilterCap = (int)fn["preFilterCap"];
        params.uniquenessRatio = (int)fn["uniquenessRatio"];
        params.P1 = (int)fn["P1"];
        params.P2 = (int)fn["P2"];
        params.mode = (int)fn["mode"];
    }

    semiglobal_matcher_parameters params;
    Mat buffer;

    // the number of stripes is fixed, disregarding the number of threads/processors
    // to make the results fully reproducible:
    static const int num_stripes = 4;
    Mat buffers[num_stripes];

    static const char* name_;
};

const char* semiglobal_matcher_implementation::name_ = "StereoMatcher.SGBM";


Ptr<StereoSGBM> create_semiglobal_matcher(int minDisparity, int numDisparities, int block_size,
                                 int P1, int P2, int disp12MaxDiff,
                                 int preFilterCap, int uniquenessRatio,
                                 int speckleWindowSize, int speckleRange,
                                 int mode, pixel_cost_type cost_type){
    return Ptr<StereoSGBM>(
        new reco::stereo_workbench::semiglobal_matcher_implementation(minDisparity, numDisparities, block_size,
                           P1, P2, disp12MaxDiff,
                           preFilterCap, uniquenessRatio,
                           speckleWindowSize, speckleRange,
                           mode, cost_type));
}

Rect getValidDisparityROI( Rect roi1, Rect roi2,
                          int minDisparity,
                          int numberOfDisparities,
                          int SADWindowSize )
{
    int SW2 = SADWindowSize/2;
    int minD = minDisparity, maxD = minDisparity + numberOfDisparities - 1;

    int xmin = std::max(roi1.x, roi2.x + maxD) + SW2;
    int xmax = std::min(roi1.x + roi1.width, roi2.x + roi2.width - minD) - SW2;
    int ymin = std::max(roi1.y, roi2.y) + SW2;
    int ymax = std::min(roi1.y + roi1.height, roi2.y + roi2.height) - SW2;

    Rect r(xmin, ymin, xmax - xmin, ymax - ymin);

    return r.width > 0 && r.height > 0 ? r : Rect();
}

typedef cv::Point_<short> Point2s;

template <typename T>
void filterSpecklesImpl(cv::Mat& img, int newVal, int maxSpeckleSize, int maxDiff, cv::Mat& _buf)
{
    using namespace cv;

    int width = img.cols, height = img.rows, npixels = width*height;
    size_t bufSize = npixels*(int)(sizeof(Point2s) + sizeof(int) + sizeof(uchar));
    if( !_buf.isContinuous() || _buf.empty() || _buf.cols*_buf.rows*_buf.elemSize() < bufSize )
        _buf.create(1, (int)bufSize, CV_8U);

    uchar* buf = _buf.ptr();
    int i, j, dstep = (int)(img.step/sizeof(T));
    int* labels = (int*)buf;
    buf += npixels*sizeof(labels[0]);
    Point2s* wbuf = (Point2s*)buf;
    buf += npixels*sizeof(wbuf[0]);
    uchar* rtype = (uchar*)buf;
    int curlabel = 0;

    // clear out label assignments
    memset(labels, 0, npixels*sizeof(labels[0]));

    for( i = 0; i < height; i++ )
    {
        T* ds = img.ptr<T>(i);
        int* ls = labels + width*i;

        for( j = 0; j < width; j++ )
        {
            if( ds[j] != newVal )   // not a bad disparity
            {
                if( ls[j] )     // has a label, check for bad label
                {
                    if( rtype[ls[j]] ) // small region, zero out disparity
                        ds[j] = (T)newVal;
                }
                // no label, assign and propagate
                else
                {
                    Point2s* ws = wbuf; // initialize wavefront
                    Point2s p((short)j, (short)i);  // current pixel
                    curlabel++; // next label
                    int count = 0;  // current region size
                    ls[j] = curlabel;

                    // wavefront propagation
                    while( ws >= wbuf ) // wavefront not empty
                    {
                        count++;
                        // put neighbors onto wavefront
                        T* dpp = &img.at<T>(p.y, p.x);
                        T dp = *dpp;
                        int* lpp = labels + width*p.y + p.x;

                        if( p.y < height-1 && !lpp[+width] && dpp[+dstep] != newVal && std::abs(dp - dpp[+dstep]) <= maxDiff )
                        {
                            lpp[+width] = curlabel;
                            *ws++ = Point2s(p.x, p.y+1);
                        }

                        if( p.y > 0 && !lpp[-width] && dpp[-dstep] != newVal && std::abs(dp - dpp[-dstep]) <= maxDiff )
                        {
                            lpp[-width] = curlabel;
                            *ws++ = Point2s(p.x, p.y-1);
                        }

                        if( p.x < width-1 && !lpp[+1] && dpp[+1] != newVal && std::abs(dp - dpp[+1]) <= maxDiff )
                        {
                            lpp[+1] = curlabel;
                            *ws++ = Point2s(p.x+1, p.y);
                        }

                        if( p.x > 0 && !lpp[-1] && dpp[-1] != newVal && std::abs(dp - dpp[-1]) <= maxDiff )
                        {
                            lpp[-1] = curlabel;
                            *ws++ = Point2s(p.x-1, p.y);
                        }

                        // pop most recent and propagate
                        // NB: could try least recent, maybe better convergence
                        p = *--ws;
                    }

                    // assign label type
                    if( count <= maxSpeckleSize )   // speckle region
                    {
                        rtype[ls[j]] = 1;   // small region label
                        ds[j] = (T)newVal;
                    }
                    else
                        rtype[ls[j]] = 0;   // large region label
                }
            }
        }
    }
}

#ifdef HAVE_IPP
static bool ipp_filterSpeckles(Mat &img, int maxSpeckleSize, int newVal, int maxDiff)
{
#if IPP_VERSION_X100 >= 810
    int type = img.type();
    Ipp32s bufsize = 0;
    IppiSize roisize = { img.cols, img.rows };
    IppDataType datatype = type == CV_8UC1 ? ipp8u : ipp16s;
    Ipp8u *pBuffer = NULL;
    IppStatus status = ippStsNoErr;

    if(ippiMarkSpecklesGetBufferSize(roisize, datatype, CV_MAT_CN(type), &bufsize) < 0)
        return false;

    pBuffer = (Ipp8u*)ippMalloc(bufsize);
    if(!pBuffer && bufsize)
        return false;

    if (type == CV_8UC1)
    {
        status = ippiMarkSpeckles_8u_C1IR(img.ptr<Ipp8u>(), (int)img.step, roisize,
                                            (Ipp8u)newVal, maxSpeckleSize, (Ipp8u)maxDiff, ippiNormL1, pBuffer);
    }
    else
    {
        status = ippiMarkSpeckles_16s_C1IR(img.ptr<Ipp16s>(), (int)img.step, roisize,
                                            (Ipp16s)newVal, maxSpeckleSize, (Ipp16s)maxDiff, ippiNormL1, pBuffer);
    }
    if(pBuffer) ippFree(pBuffer);

    if (status >= 0)
        return true;
#else
    CV_UNUSED(img); CV_UNUSED(maxSpeckleSize); CV_UNUSED(newVal); CV_UNUSED(maxDiff);
#endif
    return false;
}
#endif
} //stereo_workbench
} //reco
