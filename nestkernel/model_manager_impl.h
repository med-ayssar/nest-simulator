/*
 *  model_manager_impl.h
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

#ifndef MODEL_MANAGER_IMPL_H
#define MODEL_MANAGER_IMPL_H

#include "model_manager.h"

// Includes from libnestutil:
#include "compose.hpp"
#include "string_utils.h"

// Includes from nestkernel:
#include "connection_label.h"
#include "kernel_manager.h"
#include "target_identifier.h"


namespace nest
{

template < class ModelT >
index
ModelManager::register_node_model( const Name& name, bool private_model, std::string deprecation_info )
{
  if ( not private_model and modeldict_.known( name.toString() ) )
  {
    std::string msg = String::compose(
      "A model called '%1' already exists.\n"
      "Please choose a different name!",
      name );
    throw NamingConflict( msg );
  }

  Model* model = new GenericModel< ModelT >( name.toString(), deprecation_info );
  return register_node_model_( model, private_model );
}


template < template < typename targetidentifierT > class ConnectionT >
void
ModelManager::register_connection_model( const std::string& name, const RegisterConnectionModelFlags flags )
{
  // register normal version of the synapse
  ConnectorModel* cf = new GenericConnectorModel< ConnectionT< TargetIdentifierPtrRport > >( name,
    enumFlagSet( flags, RegisterConnectionModelFlags::IS_PRIMARY ),
    enumFlagSet( flags, RegisterConnectionModelFlags::HAS_DELAY ),
    enumFlagSet( flags, RegisterConnectionModelFlags::REQUIRES_SYMMETRIC ),
    enumFlagSet( flags, RegisterConnectionModelFlags::SUPPORTS_WFR ),
    enumFlagSet( flags, RegisterConnectionModelFlags::REQUIRES_CLOPATH_ARCHIVING ),
    enumFlagSet( flags, RegisterConnectionModelFlags::REQUIRES_URBANCZIK_ARCHIVING ) );
  register_connection_model_( cf );

  // register the "hpc" version with the same parameters but a different target
  // identifier
  if ( enumFlagSet( flags, RegisterConnectionModelFlags::REGISTER_HPC ) )
  {
    cf = new GenericConnectorModel< ConnectionT< TargetIdentifierIndex > >( name + "_hpc",
      enumFlagSet( flags, RegisterConnectionModelFlags::IS_PRIMARY ),
      enumFlagSet( flags, RegisterConnectionModelFlags::HAS_DELAY ),
      enumFlagSet( flags, RegisterConnectionModelFlags::REQUIRES_SYMMETRIC ),
      enumFlagSet( flags, RegisterConnectionModelFlags::SUPPORTS_WFR ),
      enumFlagSet( flags, RegisterConnectionModelFlags::REQUIRES_CLOPATH_ARCHIVING ),
      enumFlagSet( flags, RegisterConnectionModelFlags::REQUIRES_URBANCZIK_ARCHIVING ) );
    register_connection_model_( cf );
  }

  // register the "lbl" (labeled) version with the same parameters but a
  // different connection type
  if ( enumFlagSet( flags, RegisterConnectionModelFlags::REGISTER_LBL ) )
  {
    cf = new GenericConnectorModel< ConnectionLabel< ConnectionT< TargetIdentifierPtrRport > > >( name + "_lbl",
      enumFlagSet( flags, RegisterConnectionModelFlags::IS_PRIMARY ),
      enumFlagSet( flags, RegisterConnectionModelFlags::HAS_DELAY ),
      enumFlagSet( flags, RegisterConnectionModelFlags::REQUIRES_SYMMETRIC ),
      enumFlagSet( flags, RegisterConnectionModelFlags::SUPPORTS_WFR ),
      enumFlagSet( flags, RegisterConnectionModelFlags::REQUIRES_CLOPATH_ARCHIVING ),
      enumFlagSet( flags, RegisterConnectionModelFlags::REQUIRES_URBANCZIK_ARCHIVING ) );
    register_connection_model_( cf );
  }
}

/**
 * Register a synape with default Connector and without any common properties.
 */
template < template < typename targetidentifierT > class ConnectionT >
void
ModelManager::register_secondary_connection_model( const std::string& name, const RegisterConnectionModelFlags flags )
{
  ConnectorModel* cm = new GenericSecondaryConnectorModel< ConnectionT< TargetIdentifierPtrRport > >( name,
    enumFlagSet( flags, RegisterConnectionModelFlags::HAS_DELAY ),
    enumFlagSet( flags, RegisterConnectionModelFlags::REQUIRES_SYMMETRIC ),
    enumFlagSet( flags, RegisterConnectionModelFlags::SUPPORTS_WFR ) );

  synindex syn_id = register_connection_model_( cm );

  // idea: save *cm in data structure
  // otherwise when number of threads is increased no way to get further
  // elements
  if ( secondary_connector_models_.size() < syn_id + ( unsigned int ) 1 )
  {
    secondary_connector_models_.resize( syn_id + 1, NULL );
  }

  secondary_connector_models_[ syn_id ] = cm;

  ConnectionT< TargetIdentifierPtrRport >::EventType::set_syn_id( syn_id );

  // create labeled secondary event connection model
  cm = new GenericSecondaryConnectorModel< ConnectionLabel< ConnectionT< TargetIdentifierPtrRport > > >( name + "_lbl",
    enumFlagSet( flags, RegisterConnectionModelFlags::HAS_DELAY ),
    enumFlagSet( flags, RegisterConnectionModelFlags::REQUIRES_SYMMETRIC ),
    enumFlagSet( flags, RegisterConnectionModelFlags::SUPPORTS_WFR ) );

  syn_id = register_connection_model_( cm );

  // idea: save *cm in data structure
  // otherwise when number of threads is increased no way to get further
  // elements
  if ( secondary_connector_models_.size() < syn_id + ( unsigned int ) 1 )
  {
    secondary_connector_models_.resize( syn_id + 1, NULL );
  }

  secondary_connector_models_[ syn_id ] = cm;

  ConnectionT< TargetIdentifierPtrRport >::EventType::set_syn_id( syn_id );
}

inline Node*
ModelManager::get_proxy_node( thread tid, index node_id )
{
  const int model_id = kernel().modelrange_manager.get_model_id( node_id );
  Node* proxy = proxy_nodes_[ tid ].at( model_id );
  proxy->set_node_id_( node_id );
  proxy->set_vp( kernel().vp_manager.node_id_to_vp( node_id ) );
  return proxy;
}


inline bool
ModelManager::is_model_in_use( index i )
{
  return kernel().modelrange_manager.model_in_use( i );
}


} // namespace nest

#endif // #ifndef MODEL_MANAGER_IMPL_H
