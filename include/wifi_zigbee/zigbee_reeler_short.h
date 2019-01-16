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


#ifndef INCLUDED_WIFI_ZIGBEE_ZIGBEE_REELER_SHORT_H
#define INCLUDED_WIFI_ZIGBEE_ZIGBEE_REELER_SHORT_H

#include <wifi_zigbee/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace wifi_zigbee {

    /*!
     * \brief <+description of block+>
     * \ingroup wifi_zigbee
     *
     */
    class WIFI_ZIGBEE_API zigbee_reeler_short : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<zigbee_reeler_short> sptr;
      static sptr make(bool debug);
    };

  } // namespace wifi_zigbee
} // namespace gr

#endif /* INCLUDED_WIFI_ZIGBEE_ZIGBEE_REELER_SHORT_H */

