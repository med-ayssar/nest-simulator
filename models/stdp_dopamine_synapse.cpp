/*
 *  stdp_dopamine_synapse.cpp
 *
 *  This file is part of NEST.
 *
 *  Copyright (C) 2004 The NEST Initiative
 *
 *  NEST is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  NEST is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with NEST.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "stdp_dopamine_synapse.h"

// Includes from nestkernel:
#include "common_synapse_properties.h"
#include "connector_model.h"
#include "event.h"
#include "kernel_manager.h"
#include "nest_impl.h"


void
nest::register_stdp_dopamine_synapse( const std::string& name )
{
  register_connection_model< stdp_dopamine_synapse >( name );
}

namespace nest
{
//
// Implementation of class STDPDopaCommonProperties.
//

STDPDopaCommonProperties::STDPDopaCommonProperties()
  : CommonSynapseProperties()
  , volume_transmitter_( nullptr )
  , A_plus_( 1.0 )
  , A_minus_( 1.5 )
  , tau_plus_( 20.0 )
  , tau_c_( 1000.0 )
  , tau_n_( 200.0 )
  , b_( 0.0 )
  , Wmin_( 0.0 )
  , Wmax_( 200.0 )
{
}

void
STDPDopaCommonProperties::get_status( dictionary& d ) const
{
  CommonSynapseProperties::get_status( d );

  d[ names::A_minus ] = A_minus_;
  d[ names::A_plus ] = A_plus_;
  d[ names::Wmax ] = Wmax_;
  d[ names::Wmin ] = Wmin_;
  d[ names::b ] = b_;
  d[ names::tau_c ] = tau_c_;
  d[ names::tau_n ] = tau_n_;
  d[ names::tau_plus ] = tau_plus_;
  d[ names::volume_transmitter ] = NodeCollection::create( volume_transmitter_ );
}

void
STDPDopaCommonProperties::set_status( const dictionary& d, ConnectorModel& cm )
{
  CommonSynapseProperties::set_status( d, cm );

  NodeCollectionPTR vt_nc;
  if ( d.update_value( names::volume_transmitter, vt_nc ) )
  {
    if ( vt_nc->size() != 1 )
    {
      throw BadProperty( "Property volume_transmitter must be a single element NodeCollection" );
    }

    const size_t tid = kernel().vp_manager.get_thread_id();
    Node* vt_node = kernel().node_manager.get_node_or_proxy( ( *vt_nc )[ 0 ], tid );
    volume_transmitter* vt = dynamic_cast< volume_transmitter* >( vt_node );
    if ( not vt )
    {
      throw BadProperty( "Property volume_transmitter must be set to a node of type volume_transmitter" );
    }

    volume_transmitter_ = vt;
  }

  d.update_value( names::A_minus, A_minus_ );
  d.update_value( names::A_plus, A_plus_ );
  d.update_value( names::Wmax, Wmax_ );
  d.update_value( names::Wmin, Wmin_ );
  d.update_value( names::b, b_ );
  d.update_value( names::tau_c, tau_c_ );
  d.update_value( names::tau_n, tau_n_ );
  d.update_value( names::tau_plus, tau_plus_ );
}

} // of namespace nest
