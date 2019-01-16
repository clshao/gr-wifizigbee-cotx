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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "zero_padding_add_impl.h"

using namespace gr::wifi_zigbee;
using namespace std;

zero_padding_add::sptr
zero_padding_add::make(int vlen)
{
  return gnuradio::get_initial_sptr
	(new zero_padding_add_impl(vlen));
}

/*
 * The private constructor
 */
zero_padding_add_impl::zero_padding_add_impl(int vlen)
  : gr::block("zero_padding_add",
		  gr::io_signature::make(2, 2, sizeof(gr_complex)),
		  gr::io_signature::make(1, 1, sizeof(gr_complex))),
		  d_vlen(vlen)
{
	set_tag_propagation_policy(block::TPP_DONT);
}

zero_padding_add_impl::~zero_padding_add_impl()
{
}

void
zero_padding_add_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
	ninput_items_required[0] = 1;
	ninput_items_required[1] = 1;
}

int
zero_padding_add_impl::general_work (int noutput_items,
				   gr_vector_int &ninput_items,
				   gr_vector_const_void_star &input_items,
				   gr_vector_void_star &output_items)
{
	const gr_complex *in1 = (const gr_complex *) input_items[0];
	const gr_complex *in2 = (const gr_complex *) input_items[1];
	gr_complex *out = (gr_complex *) output_items[0];

	int noutput = noutput_items;

	int i = 0;

	while (i < noutput) {
		if (i >= ninput_items[0] && i < ninput_items[1]) {
			out[i] = in2[i];
		}
		else if (i >= ninput_items[0] && i < ninput_items[1]) {
			out[i] = in1[i];
		}
		else if (i < ninput_items[0] && i < ninput_items[1]) {
			out[i] = in1[i] + in2[i];
		}
		else {
			out[i] = gr_complex(0, 0);
		}

		i++;
	}

	consume_each(i);
	return i;
}


