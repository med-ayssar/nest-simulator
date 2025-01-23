/*
 *  layer_impl.h
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

#ifndef LAYER_IMPL_H
#define LAYER_IMPL_H

#include "layer.h"

// Includes from nestkernel:
#include "node_collection.h"

// Includes from spatial:
#include "grid_layer.h"
#include "grid_mask.h"

namespace nest
{

template < int D >
std::shared_ptr< Ntree< D, size_t > > Layer< D >::cached_ntree_;

template < int D >
std::vector< std::pair< Position< D >, size_t > >* Layer< D >::cached_vector_ = 0;

template < int D >
Position< D >
Layer< D >::compute_displacement( const Position< D >& from_pos, const Position< D >& to_pos ) const
{
  Position< D > displ = to_pos;
  for ( int i = 0; i < D; ++i )
  {
    displ[ i ] -= from_pos[ i ];
    if ( periodic_[ i ] )
    {
      displ[ i ] = -0.5 * extent_[ i ] + std::fmod( displ[ i ] + 0.5 * extent_[ i ], extent_[ i ] );
      if ( displ[ i ] < -0.5 * extent_[ i ] )
      {
        displ[ i ] += extent_[ i ];
      }
    }
  }
  return displ;
}

template < int D >
double
Layer< D >::compute_displacement( const std::vector< double >& from_pos,
  const std::vector< double >& to_pos,
  const unsigned int dimension ) const
{
  double displacement = to_pos[ dimension ] - from_pos[ dimension ];
  if ( periodic_[ dimension ] )
  {
    displacement -= extent_[ dimension ] * std::round( displacement * ( 1 / extent_[ dimension ] ) );
  }
  return displacement;
}

template < int D >
void
Layer< D >::set_status( const dictionary& d )
{
  if ( d.known( names::edge_wrap ) )
  {
    if ( d.get< bool >( names::edge_wrap ) )
    {
      periodic_ = ( 1 << D ) - 1; // All dimensions periodic
    }
  }
}

template < int D >
void
Layer< D >::get_status( dictionary& d, NodeCollection const* const nc ) const
{
  d[ names::extent ] = std::vector< double >( extent_.get_vector() );
  d[ names::center ] = std::vector< double >( ( lower_left_ + extent_ / 2 ).get_vector() );

  if ( periodic_.none() )
  {
    d[ names::edge_wrap ] = false;
  }
  else if ( periodic_.count() == D )
  {
    d[ names::edge_wrap ] = true;
  }

  if ( nc )
  {
    // This is for backward compatibility with some tests and scripts
    // TODO: Rename parameter
    d[ names::network_size ] = nc->size();
  }
}

template < int D >
void
Layer< D >::connect( NodeCollectionPTR source_nc,
  AbstractLayerPTR target_layer,
  NodeCollectionPTR target_nc,
  ConnectionCreator& connector )
{
  // We need to extract the real pointer here to be able to cast to the
  // dimension-specific subclass.
  AbstractLayer* target_abs = target_layer.get();
  assert( target_abs );

  try
  {
    Layer< D >& tgt = dynamic_cast< Layer< D >& >( *target_abs );
    connector.connect( *this, source_nc, tgt, target_nc );
  }
  catch ( std::bad_cast& e )
  {
    throw BadProperty( "Target layer must have same number of dimensions as source layer." );
  }
}

template < int D >
std::shared_ptr< Ntree< D, size_t > >
Layer< D >::get_global_positions_ntree( NodeCollectionPTR node_collection )
{
  if ( cached_ntree_md_ == node_collection->get_metadata() )
  {
    assert( cached_ntree_.get() );
    return cached_ntree_;
  }

  clear_ntree_cache_();

  cached_ntree_ = std::shared_ptr< Ntree< D, size_t > >(
    new Ntree< D, size_t >( this->lower_left_, this->extent_, this->periodic_ ) );

  return do_get_global_positions_ntree_( node_collection );
}

template < int D >
std::shared_ptr< Ntree< D, size_t > >
Layer< D >::get_global_positions_ntree( std::bitset< D > periodic,
  Position< D > lower_left,
  Position< D > extent,
  NodeCollectionPTR node_collection )
{
  clear_ntree_cache_();
  clear_vector_cache_();

  // Keep layer geometry for non-periodic dimensions
  for ( int i = 0; i < D; ++i )
  {
    if ( not periodic[ i ] )
    {
      extent[ i ] = extent_[ i ];
      lower_left[ i ] = lower_left_[ i ];
    }
  }

  cached_ntree_ =
    std::shared_ptr< Ntree< D, size_t > >( new Ntree< D, size_t >( this->lower_left_, extent, periodic ) );

  do_get_global_positions_ntree_( node_collection );

  // Do not use cache since the periodic bits and extents were altered.
  cached_ntree_md_ = NodeCollectionMetadataPTR( nullptr );

  return cached_ntree_;
}

template < int D >
std::shared_ptr< Ntree< D, size_t > >
Layer< D >::do_get_global_positions_ntree_( NodeCollectionPTR node_collection )
{
  if ( cached_vector_md_ == node_collection->get_metadata() )
  {
    // Convert from vector to Ntree
    // PYNEST-NG: Why different from Master?
    typename std::back_insert_iterator< Ntree< D, size_t > > to = std::back_inserter( *cached_ntree_ );

    for ( typename std::vector< std::pair< Position< D >, size_t > >::iterator from = cached_vector_->begin();
          from != cached_vector_->end();
          ++from )
    {
      *to = *from;
    }
  }
  else
  {

    insert_global_positions_ntree_( *cached_ntree_, node_collection );
  }

  clear_vector_cache_();

  cached_ntree_md_ = node_collection->get_metadata();

  return cached_ntree_;
}

template < int D >
std::vector< std::pair< Position< D >, size_t > >*
Layer< D >::get_global_positions_vector( NodeCollectionPTR node_collection )
{
  if ( cached_vector_md_ == node_collection->get_metadata() )
  {
    assert( cached_vector_ );
    return cached_vector_;
  }

  clear_vector_cache_();

  cached_vector_ = new std::vector< std::pair< Position< D >, size_t > >;

  if ( cached_ntree_md_ == node_collection->get_metadata() )
  {
    // Convert from NTree to vector

    typename std::back_insert_iterator< std::vector< std::pair< Position< D >, size_t > > > to =
      std::back_inserter( *cached_vector_ );

    for ( typename Ntree< D, size_t >::iterator from = cached_ntree_->begin(); from != cached_ntree_->end(); ++from )
    {
      *to = *from;
    }
  }
  else
  {
    insert_global_positions_vector_( *cached_vector_, node_collection );
  }

  clear_ntree_cache_();

  cached_vector_md_ = node_collection->get_metadata();

  return cached_vector_;
}

template < int D >
std::vector< std::pair< Position< D >, size_t > >
Layer< D >::get_global_positions_vector( const MaskPTR mask,
  const Position< D >& anchor,
  bool allow_oversized,
  NodeCollectionPTR node_collection )
{
  MaskedLayer< D > masked_layer( *this, mask, allow_oversized, node_collection );
  std::vector< std::pair< Position< D >, size_t > > positions;

  for ( typename Ntree< D, size_t >::masked_iterator iter = masked_layer.begin( anchor ); iter != masked_layer.end();
        ++iter )
  {
    positions.push_back( *iter );
  }

  return positions;
}

template < int D >
std::vector< size_t >
Layer< D >::get_global_nodes( const MaskPTR mask,
  const std::vector< double >& anchor,
  bool allow_oversized,
  NodeCollectionPTR node_collection )
{
  MaskedLayer< D > masked_layer( *this, mask, allow_oversized, node_collection );
  std::vector< size_t > nodes;
  for ( typename Ntree< D, size_t >::masked_iterator i = masked_layer.begin( anchor ); i != masked_layer.end(); ++i )
  {
    nodes.push_back( i->second );
  }
  return nodes;
}

template < int D >
void
Layer< D >::dump_nodes( std::ostream& out ) const
{
  for ( NodeCollection::const_iterator it = this->node_collection_->rank_local_begin();
        it < this->node_collection_->end();
        ++it )
  {
    out << ( *it ).node_id << ' ';
    get_position( ( *it ).nc_index ).print( out );
    out << std::endl;
  }
}

template < int D >
void
Layer< D >::dump_connections( std::ostream& out,
  NodeCollectionPTR node_collection,
  AbstractLayerPTR target_layer,
  const std::string& syn_model )
{
  // Find all connections for given sources, targets and synapse model
  dictionary conn_filter;
  conn_filter[ names::source ] = node_collection;
  conn_filter[ names::target ] = NodeCollectionPTR( target_layer->get_node_collection() );
  conn_filter[ names::synapse_model ] = syn_model;

  const auto& connectome = kernel().connection_manager.get_connections( conn_filter );

  // Get positions of remote nodes
  std::vector< std::pair< Position< D >, size_t > >* src_vec = get_global_positions_vector( node_collection );

  // Iterate over connectome and write every connection, looking up source position only if source neuron changes
  size_t previous_source_node_id = 0; // dummy initial value, cannot be node_id of any node
  Position< D > source_pos;           // dummy value
  for ( const auto& conn : connectome )
  {
    const size_t source_node_id = conn.get_source_node_id();

    // Search source_pos for source node only if it is a different node
    if ( source_node_id != previous_source_node_id )
    {
      const auto it = std::find_if( src_vec->begin(),
        src_vec->end(),
        [ source_node_id ]( const std::pair< Position< D >, size_t >& p ) { return p.second == source_node_id; } );
      assert( it != src_vec->end() ); // internal error if node not found

      source_pos = it->first;
      previous_source_node_id = source_node_id;
    }

    const dictionary result_dict = kernel().connection_manager.get_synapse_status( source_node_id,
      conn.get_target_node_id(),
      conn.get_target_thread(),
      conn.get_synapse_model_id(),
      conn.get_port() );

    const auto target_node_id = result_dict.get< size_t >( names::target );
    const auto weight = result_dict.get< double >( names::weight );
    const auto delay = result_dict.get< double >( names::delay );

    const Layer< D >* const tgt_layer = dynamic_cast< Layer< D >* >( target_layer.get() );
    const long tnode_lid = tgt_layer->node_collection_->get_nc_index( target_node_id );
    assert( tnode_lid >= 0 );

    // Print source, target, weight, delay, rports
    out << source_node_id << ' ' << target_node_id << ' ' << weight << ' ' << delay << ' ';
    tgt_layer->compute_displacement( source_pos, tnode_lid ).print( out );
    out << '\n';
  }
}

template < int D >
void
MaskedLayer< D >::check_mask_( Layer< D >& layer, bool allow_oversized )
{
  if ( not mask_.get() )
  {
    mask_ = MaskPTR( new AllMask< D >() );
    return;
  }

  try // Try to cast to GridMask
  {
    const GridMask< D >& grid_mask = dynamic_cast< const GridMask< D >& >( *mask_ );

    // If the above cast succeeds, then this is a grid mask

    GridLayer< D >* grid_layer = dynamic_cast< GridLayer< D >* >( &layer );
    if ( grid_layer == 0 )
    {
      throw BadProperty( "Grid masks can only be used with grid layers." );
    }

    Position< D > ext = grid_layer->get_extent();
    Position< D, size_t > dims = grid_layer->get_dims();

    if ( not allow_oversized )
    {
      bool oversize = false;
      for ( int i = 0; i < D; ++i )
      {
        oversize |= layer.get_periodic_mask()[ i ]
          and ( grid_mask.get_lower_right()[ i ] - grid_mask.get_upper_left()[ i ] ) > static_cast< int >( dims[ i ] );
      }
      if ( oversize )
      {
        throw BadProperty(
          "Mask size must not exceed layer size; set allow_oversized_mask to "
          "override." );
      }
    }

    Position< D > lower_left = ext / dims * grid_mask.get_upper_left() - ext / dims * 0.5;
    Position< D > upper_right = ext / dims * grid_mask.get_lower_right() - ext / dims * 0.5;

    const double y = lower_left[ 1 ];
    lower_left[ 1 ] = -upper_right[ 1 ];
    upper_right[ 1 ] = -y;

    mask_ = MaskPTR( new BoxMask< D >( lower_left, upper_right ) );
  }
  catch ( std::bad_cast& )
  {

    // Not a grid mask

    try // Try to cast to correct dimension Mask
    {
      const Mask< D >& mask = dynamic_cast< const Mask< D >& >( *mask_ );

      if ( not allow_oversized )
      {
        const Box< D > bb = mask.get_bbox();
        bool oversize = false;
        for ( int i = 0; i < D; ++i )
        {
          oversize |=
            layer.get_periodic_mask()[ i ] and bb.upper_right[ i ] - bb.lower_left[ i ] > layer.get_extent()[ i ];
        }
        if ( oversize )
        {
          throw BadProperty(
            "Mask size must not exceed layer size; set allow_oversized_mask to "
            "override." );
        }
      }
    }
    catch ( std::bad_cast& )
    {
      throw BadProperty( "Mask is incompatible with layer." );
    }
  }
}

} // namespace nest

#endif
