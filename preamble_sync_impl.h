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
#include <string>

using namespace std;

namespace gr {
  namespace wifi_zigbee {

    class preamble_sync_impl : public preamble_sync
    {
     private:
      bool d_debug;
      unsigned int d_threshold;
      unsigned int d_preamble_peak_cnt;
      float d_correlation;
      unsigned long d_pkt_index;
      unsigned int d_cnt;
      unsigned d_offset;
      unsigned int d_copy;
      float d_register_real[80];
      float d_register_img[80];
      enum {SEARCH, FIND, COPY} d_state;
      enum {EOP_NOT_FOUND, EOP_FOUND} d_eop_state;
      static const std::vector<float> HALF_SIN;

     public:

	  #define dout d_debug && std::cout
      preamble_sync_impl(bool debug, unsigned int threshold);
      ~preamble_sync_impl();

      void forecast (int noutput_items, gr_vector_int &ninput_items_required);
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

