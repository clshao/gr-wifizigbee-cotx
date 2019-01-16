/* -*- c++ -*- */
/* 
 * Copyright 2017 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_WIFI_ZIGBEE_PREAMBLE_SYNC_IMPL_H
#define INCLUDED_WIFI_ZIGBEE_PREAMBLE_SYNC_IMPL_H

#include <wifi_zigbee/preamble_sync.h>
#include <gnuradio/filter/fir_filter.h>
#include <gnuradio/fft/fft.h>
#include <string>
#include <list>
#include <tuple>

using namespace std;

namespace gr {
  namespace wifi_zigbee {

  static const int SYNC_LENGTH = 120;
  static const int PREAMBLE_PEAK_CNT = 15;

    class preamble_sync_impl : public preamble_sync
    {
     private:
      bool d_debug;

      unsigned int d_preamble_spl_cnt;
      unsigned int d_threshold;
      unsigned int d_preamble_peak_cnt;
      //unsigned int d_head_cnt;
      //unsigned int d_residual_padding;
      unsigned long d_pkt_index;
      unsigned d_offset;
      unsigned int d_spl_index_real;
      unsigned int d_spl_index_img;
      unsigned int d_frame_start;
      unsigned int d_reset_cnt;
      unsigned int d_copy;
      float d_preamble_buf_real[80];
      float d_preamble_buf_img[80];
      //float d_cor_head_buf[PREAMBLE_PEAK_CNT];
      float d_fine_sync_real[SYNC_LENGTH];
      float d_fine_sync_img[SYNC_LENGTH];
      float d_register_real[80];
      float d_register_img[80];
      float *d_fir_real_result;
      float *d_fir_img_result;
      enum {SEARCH, SYNC, RELEASE, FIND, COPY} d_state;
      enum {EOP_NOT_FOUND, EOP_FOUND} d_eop_state;
      static const std::vector<float> COARSE_REAL_CORRELATION;
      static const std::vector<float> COARSE_IMG_CORRELATION;
      static const std::vector<float> FINE_REAL_CORRELATION;
      static const std::vector<float> FINE_IMG_CORRELATION;

      gr::filter::kernel::fir_filter_fff d_fir_real;
      gr::filter::kernel::fir_filter_fff d_fir_img;
      list<pair<float, int> > d_cor_real;
      list<pair<float, int> > d_cor_img;

     public:

	  #define dout d_debug && std::cout
      preamble_sync_impl(bool debug, unsigned int threshold);
      ~preamble_sync_impl();

      void forecast (int noutput_items, gr_vector_int &ninput_items_required);
      int make_correlation(float *buf_real, float *buf_img);
      void search_frame_start();
      void insert_tag(unsigned int out_index, uint64_t item, long pkt_index, string str);
      int find_zero_padding(float *buf_real, float *buf_img);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace wifi_zigbee
} // namespace gr

#endif /* INCLUDED_WIFI_ZIGBEE_PREAMBLE_SYNC_IMPL_H */

