/*
 *  connection_manager.cpp
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

#include "connection_manager.h"

// Generated includes:
#include "config.h"

// C++ includes:
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iomanip>
#include <limits>
#include <set>
#include <vector>

// Includes from libnestutil:
#include "compose.hpp"
#include "logging.h"

// Includes from nestkernel:
#include "clopath_archiving_node.h"
#include "conn_builder.h"
#include "conn_builder_conngen.h"
#include "conn_builder_factory.h"
#include "connection_label.h"
#include "connection_manager_impl.h"
#include "connector_base.h"
#include "connector_model.h"
#include "delay_checker.h"
#include "eprop_archiving_node.h"
#include "eprop_archiving_node_readout.h"
#include "eprop_archiving_node_recurrent.h"
#include "exceptions.h"
#include "kernel_manager.h"
#include "mpi_manager_impl.h"
#include "nest_names.h"
#include "node.h"
#include "sonata_connector.h"
#include "stopwatch_impl.h"
#include "target_table_devices_impl.h"
#include "vp_manager_impl.h"

// Includes from sli:
#include "dictutils.h"
#include "sliexceptions.h"
#include "token.h"
#include "tokenutils.h"


nest::ConnectionManager::ConnectionManager()
  : connruledict_( new Dictionary() )
  , connbuilder_factories_()
  , thirdconnruledict_( new Dictionary() )
  , thirdconnbuilder_factories_()
  , min_delay_( 1 )
  , max_delay_( 1 )
  , keep_source_table_( true )
  , connections_have_changed_( false )
  , get_connections_has_been_called_( false )
  , use_compressed_spikes_( true )
  , has_primary_connections_( false )
  , check_primary_connections_()
  , secondary_connections_exist_( false )
  , check_secondary_connections_()
  , stdp_eps_( 1.0e-6 )
{
}

nest::ConnectionManager::~ConnectionManager()
{
  // Memory leak on purpose!
  // The ConnectionManager is deleted, when the network is deleted, and
  // this happens only, when main() is finished and we give the allocated memory
  // back to the system anyway. Hence, why bother cleaning up our highly
  // scattered connection infrastructure? They do not have any open files, which
  // need to be closed or similar.
}

void
nest::ConnectionManager::initialize( const bool adjust_number_of_threads_or_rng_only )
{
  if ( not adjust_number_of_threads_or_rng_only )
  {
    // Add connection rules
    register_conn_builder< OneToOneBuilder >( "one_to_one" );
    register_conn_builder< AllToAllBuilder >( "all_to_all" );
    register_conn_builder< FixedInDegreeBuilder >( "fixed_indegree" );
    register_conn_builder< FixedOutDegreeBuilder >( "fixed_outdegree" );
    register_conn_builder< BernoulliBuilder >( "pairwise_bernoulli" );
    register_conn_builder< PoissonBuilder >( "pairwise_poisson" );
    register_conn_builder< SymmetricBernoulliBuilder >( "symmetric_pairwise_bernoulli" );
    register_conn_builder< FixedTotalNumberBuilder >( "fixed_total_number" );
    register_third_conn_builder< ThirdBernoulliWithPoolBuilder >( "third_factor_bernoulli_with_pool" );
#ifdef HAVE_LIBNEUROSIM
    register_conn_builder< ConnectionGeneratorBuilder >( "conngen" );
#endif

    keep_source_table_ = true;
    connections_have_changed_ = false;
    get_connections_has_been_called_ = false;
    use_compressed_spikes_ = true;
    stdp_eps_ = 1.0e-6;
    min_delay_ = max_delay_ = 1;
    sw_construction_connect.reset();
  }

  const size_t num_threads = kernel().vp_manager.get_num_threads();
  connections_.resize( num_threads );
  secondary_recv_buffer_pos_.resize( num_threads );
  compressed_spike_data_.resize( 0 );

  has_primary_connections_ = false;
  check_primary_connections_.initialize( num_threads, false );
  secondary_connections_exist_ = false;
  check_secondary_connections_.initialize( num_threads, false );

  // We need to obtain this while in serial context to avoid problems when
  // increasing the number of threads.
  const size_t num_conn_models = kernel().model_manager.get_num_connection_models();

#pragma omp parallel
  {
    const size_t tid = kernel().vp_manager.get_thread_id();
    connections_.at( tid ) = std::vector< ConnectorBase* >( num_conn_models );
    secondary_recv_buffer_pos_.at( tid ) = std::vector< std::vector< size_t > >();
  } // of omp parallel

  source_table_.initialize();
  target_table_.initialize();
  target_table_devices_.initialize();

  std::vector< DelayChecker > tmp( kernel().vp_manager.get_num_threads() );
  delay_checkers_.swap( tmp );

  std::vector< std::vector< size_t > > tmp2( kernel().vp_manager.get_num_threads(), std::vector< size_t >() );
  num_connections_.swap( tmp2 );
}

void
nest::ConnectionManager::finalize( const bool adjust_number_of_threads_or_rng_only )
{
  source_table_.finalize();
  target_table_.finalize();
  target_table_devices_.finalize();
  delete_connections_();
  std::vector< std::vector< ConnectorBase* > >().swap( connections_ );
  std::vector< std::vector< std::vector< size_t > > >().swap( secondary_recv_buffer_pos_ );
  compressed_spike_data_.clear();

  if ( not adjust_number_of_threads_or_rng_only )
  {
    for ( auto cbf : connbuilder_factories_ )
    {
      delete cbf;
    }
    connbuilder_factories_.clear();
    connruledict_->clear();

    for ( auto tcbf : thirdconnbuilder_factories_ )
    {
      delete tcbf;
    }
    thirdconnbuilder_factories_.clear();
    thirdconnruledict_->clear();
  }
}

void
nest::ConnectionManager::set_status( const DictionaryDatum& d )
{
  for ( size_t i = 0; i < delay_checkers_.size(); ++i )
  {
    delay_checkers_[ i ].set_status( d );
  }

  updateValue< bool >( d, names::keep_source_table, keep_source_table_ );
  if ( not keep_source_table_ and kernel().sp_manager.is_structural_plasticity_enabled() )
  {
    throw KernelException(
      "If structural plasticity is enabled, keep_source_table can not be set "
      "to false." );
  }

  updateValue< bool >( d, names::use_compressed_spikes, use_compressed_spikes_ );

  //  Need to update the saved values if we have changed the delay bounds.
  if ( d->known( names::min_delay ) or d->known( names::max_delay ) )
  {
    update_delay_extrema_();
  }
}

nest::DelayChecker&
nest::ConnectionManager::get_delay_checker()
{
  return delay_checkers_[ kernel().vp_manager.get_thread_id() ];
}

void
nest::ConnectionManager::get_status( DictionaryDatum& dict )
{
  update_delay_extrema_();
  def< double >( dict, names::min_delay, Time( Time::step( min_delay_ ) ).get_ms() );
  def< double >( dict, names::max_delay, Time( Time::step( max_delay_ ) ).get_ms() );

  const size_t n = get_num_connections();
  def< long >( dict, names::num_connections, n );
  def< bool >( dict, names::keep_source_table, keep_source_table_ );
  def< bool >( dict, names::use_compressed_spikes, use_compressed_spikes_ );

  sw_construction_connect.get_status( dict, names::time_construction_connect, names::time_construction_connect_cpu );

  ArrayDatum connection_rules;
  for ( auto const& element : *connruledict_ )
  {
    connection_rules.push_back( new LiteralDatum( element.first ) );
  }
  def< ArrayDatum >( dict, names::connection_rules, connection_rules );
}

DictionaryDatum
nest::ConnectionManager::get_synapse_status( const size_t source_node_id,
  const size_t target_node_id,
  const size_t tid,
  const synindex syn_id,
  const size_t lcid ) const
{
  kernel().model_manager.assert_valid_syn_id( syn_id, kernel().vp_manager.get_thread_id() );

  DictionaryDatum dict( new Dictionary );
  ( *dict )[ names::source ] = source_node_id;
  ( *dict )[ names::synapse_model ] =
    LiteralDatum( kernel().model_manager.get_connection_model( syn_id, /* thread */ 0 ).get_name() );
  ( *dict )[ names::target_thread ] = tid;
  ( *dict )[ names::synapse_id ] = syn_id;
  ( *dict )[ names::port ] = lcid;

  const Node* source = kernel().node_manager.get_node_or_proxy( source_node_id, tid );
  const Node* target = kernel().node_manager.get_node_or_proxy( target_node_id, tid );

  // synapses from neurons to neurons and from neurons to globally
  // receiving devices
  if ( ( source->has_proxies() and target->has_proxies() and connections_[ tid ][ syn_id ] )
    or ( ( source->has_proxies() and not target->has_proxies() and not target->local_receiver()
      and connections_[ tid ][ syn_id ] ) ) )
  {
    connections_[ tid ][ syn_id ]->get_synapse_status( tid, lcid, dict );
  }
  else if ( source->has_proxies() and not target->has_proxies() and target->local_receiver() )
  {
    target_table_devices_.get_synapse_status_to_device( tid, source_node_id, syn_id, dict, lcid );
  }
  else if ( not source->has_proxies() )
  {
    const size_t ldid = source->get_local_device_id();
    target_table_devices_.get_synapse_status_from_device( tid, ldid, syn_id, dict, lcid );
  }
  else
  {
    assert( false );
  }

  return dict;
}

void
nest::ConnectionManager::set_synapse_status( const size_t source_node_id,
  const size_t target_node_id,
  const size_t tid,
  const synindex syn_id,
  const size_t lcid,
  const DictionaryDatum& dict )
{
  kernel().model_manager.assert_valid_syn_id( syn_id, kernel().vp_manager.get_thread_id() );

  const Node* source = kernel().node_manager.get_node_or_proxy( source_node_id, tid );
  const Node* target = kernel().node_manager.get_node_or_proxy( target_node_id, tid );

  try
  {
    ConnectorModel& cm = kernel().model_manager.get_connection_model( syn_id, tid );
    // synapses from neurons to neurons and from neurons to globally
    // receiving devices
    if ( ( source->has_proxies() and target->has_proxies() and connections_[ tid ][ syn_id ] )
      or ( ( source->has_proxies() and not target->has_proxies() and not target->local_receiver()
        and connections_[ tid ][ syn_id ] ) ) )
    {
      connections_[ tid ][ syn_id ]->set_synapse_status( lcid, dict, cm );
    }
    else if ( source->has_proxies() and not target->has_proxies() and target->local_receiver() )
    {
      target_table_devices_.set_synapse_status_to_device( tid, source_node_id, syn_id, cm, dict, lcid );
    }
    else if ( not source->has_proxies() )
    {
      const size_t ldid = source->get_local_device_id();
      target_table_devices_.set_synapse_status_from_device( tid, ldid, syn_id, cm, dict, lcid );
    }
    else
    {
      assert( false );
    }
  }
  catch ( BadProperty& e )
  {
    throw BadProperty(
      String::compose( "Setting status of '%1' connecting from node ID %2 to node ID %3 via port %4: %5",
        kernel().model_manager.get_connection_model( syn_id, tid ).get_name(),
        source_node_id,
        target_node_id,
        lcid,
        e.message() ) );
  }
}

void
nest::ConnectionManager::delete_connections_()
{
  for ( size_t tid = 0; tid < connections_.size(); ++tid )
  {
    for ( auto conn = connections_[ tid ].begin(); conn != connections_[ tid ].end(); ++conn )
    {
      delete *conn;
    }
  }
}

const nest::Time
nest::ConnectionManager::get_min_delay_time_() const
{
  Time min_delay = Time::pos_inf();

  std::vector< DelayChecker >::const_iterator it;
  for ( it = delay_checkers_.begin(); it != delay_checkers_.end(); ++it )
  {
    min_delay = std::min( min_delay, it->get_min_delay() );
  }

  return min_delay;
}

const nest::Time
nest::ConnectionManager::get_max_delay_time_() const
{
  Time max_delay = Time::get_resolution();

  std::vector< DelayChecker >::const_iterator it;
  for ( it = delay_checkers_.begin(); it != delay_checkers_.end(); ++it )
  {
    max_delay = std::max( max_delay, it->get_max_delay() );
  }

  return max_delay;
}

bool
nest::ConnectionManager::get_user_set_delay_extrema() const
{
  bool user_set_delay_extrema = false;

  std::vector< DelayChecker >::const_iterator it;
  for ( it = delay_checkers_.begin(); it != delay_checkers_.end(); ++it )
  {
    user_set_delay_extrema |= it->get_user_set_delay_extrema();
  }

  return user_set_delay_extrema;
}

nest::BipartiteConnBuilder*
nest::ConnectionManager::get_conn_builder( const std::string& name,
  NodeCollectionPTR sources,
  NodeCollectionPTR targets,
  ThirdOutBuilder* third_out,
  const DictionaryDatum& conn_spec,
  const std::vector< DictionaryDatum >& syn_specs )
{
  if ( not connruledict_->known( name ) )
  {
    throw IllegalConnection( String::compose( "Unknown connection rule '%1'.", name ) );
  }

  const size_t rule_id = connruledict_->lookup( name );
  BipartiteConnBuilder* cb =
    connbuilder_factories_.at( rule_id )->create( sources, targets, third_out, conn_spec, syn_specs );
  assert( cb );
  return cb;
}

nest::ThirdOutBuilder*
nest::ConnectionManager::get_third_conn_builder( const std::string& name,
  NodeCollectionPTR sources,
  NodeCollectionPTR targets,
  ThirdInBuilder* third_in,
  const DictionaryDatum& conn_spec,
  const std::vector< DictionaryDatum >& syn_specs )
{
  if ( not thirdconnruledict_->known( name ) )
  {
    throw IllegalConnection( String::compose( "Unknown third-factor connection rule '%1'.", name ) );
  }

  const size_t rule_id = thirdconnruledict_->lookup( name );
  ThirdOutBuilder* cb =
    thirdconnbuilder_factories_.at( rule_id )->create( sources, targets, third_in, conn_spec, syn_specs );
  assert( cb );
  return cb;
}

void
nest::ConnectionManager::calibrate( const TimeConverter& tc )
{
  for ( size_t tid = 0; tid < kernel().vp_manager.get_num_threads(); ++tid )
  {
    delay_checkers_[ tid ].calibrate( tc );
  }
}

void
nest::ConnectionManager::connect( NodeCollectionPTR sources,
  NodeCollectionPTR targets,
  const DictionaryDatum& conn_spec,
  const std::vector< DictionaryDatum >& syn_specs )
{
  if ( sources->empty() )
  {
    throw IllegalConnection( "Presynaptic nodes cannot be an empty NodeCollection" );
  }
  if ( targets->empty() )
  {
    throw IllegalConnection( "Postsynaptic nodes cannot be an empty NodeCollection" );
  }

  conn_spec->clear_access_flags();

  for ( auto syn_params : syn_specs )
  {
    syn_params->clear_access_flags();
  }

  if ( not conn_spec->known( names::rule ) )
  {
    throw BadProperty( "The connection specification must contain a connection rule." );
  }
  const std::string rule = static_cast< const std::string >( ( *conn_spec )[ names::rule ] );

  if ( not connruledict_->known( rule ) )
  {
    throw BadProperty( String::compose( "Unknown connection rule: %1", rule ) );
  }

  ConnBuilder cb( rule, sources, targets, conn_spec, syn_specs );

  // at this point, all entries in conn_spec and syn_spec have been checked
  ALL_ENTRIES_ACCESSED( *conn_spec, "Connect", "Unread dictionary entries in conn_spec: " );
  for ( auto syn_params : syn_specs )
  {
    ALL_ENTRIES_ACCESSED( *syn_params, "Connect", "Unread dictionary entries in syn_spec: " );
  }

  // Set flag before calling cb->connect() in case exception is thrown after some connections have been created.
  set_connections_have_changed();

  cb.connect();
}


void
nest::ConnectionManager::connect( TokenArray sources, TokenArray targets, const DictionaryDatum& syn_spec )
{
  // Get synapse id
  size_t syn_id = 0;
  auto synmodel = syn_spec->lookup( names::model );
  if ( not synmodel.empty() )
  {
    const std::string synmodel_name = getValue< std::string >( synmodel );
    // The following throws UnknownSynapseType for invalid synmodel_name
    syn_id = kernel().model_manager.get_synapse_model_id( synmodel_name );
  }
  // Connect all sources to all targets
  for ( auto&& source : sources )
  {
    auto source_node = kernel().node_manager.get_node_or_proxy( source );
    for ( auto&& target : targets )
    {
      auto target_node = kernel().node_manager.get_node_or_proxy( target );
      auto target_thread = target_node->get_thread();
      connect_( *source_node, *target_node, source, target_thread, syn_id, syn_spec );
    }
  }
}


void
nest::ConnectionManager::update_delay_extrema_()
{
  if ( kernel().simulation_manager.has_been_simulated() )
  {
    // Once simulation has started, min/max_delay can no longer change,
    // so there is nothing to update.
    return;
  }

  min_delay_ = get_min_delay_time_().get_steps();
  max_delay_ = get_max_delay_time_().get_steps();

  if ( not get_user_set_delay_extrema() )
  {
    // If no min/max_delay is set explicitly, then the default delay used by the
    // SPBuilders have to be respected for min/max_delay.
    min_delay_ = std::min( min_delay_, kernel().sp_manager.builder_min_delay() );
    max_delay_ = std::max( max_delay_, kernel().sp_manager.builder_max_delay() );
  }

  // If the user explicitly set min/max_delay, this happend on all MPI ranks,
  // so all ranks are up to date already. Also, once the user has set min/max_delay
  // explicitly, Connect() cannot induce new extrema. Thuse, we only need to communicate
  // with other ranks if the user has not set the extrema and connections may have
  // been created.
  if ( not kernel().connection_manager.get_user_set_delay_extrema()
    and kernel().connection_manager.connections_have_changed() and kernel().mpi_manager.get_num_processes() > 1 )
  {
    std::vector< long > min_delays( kernel().mpi_manager.get_num_processes() );
    min_delays[ kernel().mpi_manager.get_rank() ] = min_delay_;
    kernel().mpi_manager.communicate( min_delays );
    min_delay_ = *std::min_element( min_delays.begin(), min_delays.end() );

    std::vector< long > max_delays( kernel().mpi_manager.get_num_processes() );
    max_delays[ kernel().mpi_manager.get_rank() ] = max_delay_;
    kernel().mpi_manager.communicate( max_delays );
    max_delay_ = *std::max_element( max_delays.begin(), max_delays.end() );
  }

  if ( min_delay_ == Time::pos_inf().get_steps() )
  {
    min_delay_ = Time::get_resolution().get_steps();
  }
}

// node ID node thread syn_id dict delay weight
void
nest::ConnectionManager::connect( const size_t snode_id,
  Node* target,
  size_t target_thread,
  const synindex syn_id,
  const DictionaryDatum& params,
  const double delay,
  const double weight )
{
  kernel().model_manager.assert_valid_syn_id( syn_id, kernel().vp_manager.get_thread_id() );

  Node* source = kernel().node_manager.get_node_or_proxy( snode_id, target_thread );

  ConnectionType connection_type = connection_required( source, target, target_thread );

  switch ( connection_type )
  {
  case CONNECT:
    connect_( *source, *target, snode_id, target_thread, syn_id, params, delay, weight );
    break;
  case CONNECT_FROM_DEVICE:
    connect_from_device_( *source, *target, target_thread, syn_id, params, delay, weight );
    break;
  case CONNECT_TO_DEVICE:
    connect_to_device_( *source, *target, snode_id, target_thread, syn_id, params, delay, weight );
    break;
  case NO_CONNECTION:
    return;
  }
}

// node_id node_id dict syn_id
bool
nest::ConnectionManager::connect( const size_t snode_id,
  const size_t tnode_id,
  const DictionaryDatum& params,
  const synindex syn_id )
{
  kernel().model_manager.assert_valid_syn_id( syn_id, kernel().vp_manager.get_thread_id() );

  const size_t tid = kernel().vp_manager.get_thread_id();

  if ( not kernel().node_manager.is_local_node_id( tnode_id ) )
  {
    return false;
  }

  Node* target = kernel().node_manager.get_node_or_proxy( tnode_id, tid );
  const size_t target_thread = target->get_thread();
  Node* source = kernel().node_manager.get_node_or_proxy( snode_id, target_thread );

  ConnectionType connection_type = connection_required( source, target, target_thread );
  bool connected = true;

  switch ( connection_type )
  {
  case CONNECT:
    connect_( *source, *target, snode_id, target_thread, syn_id, params );
    break;
  case CONNECT_FROM_DEVICE:
    connect_from_device_( *source, *target, target_thread, syn_id, params );
    break;
  case CONNECT_TO_DEVICE:
    connect_to_device_( *source, *target, snode_id, target_thread, syn_id, params );
    break;
  case NO_CONNECTION:
    connected = false;
    break;
  }

  return connected;
}

void
nest::ConnectionManager::connect_arrays( long* sources,
  long* targets,
  double* weights,
  double* delays,
  std::vector< std::string >& p_keys,
  double* p_values,
  size_t n,
  std::string syn_model )
{
  // only place, where stopwatch sw_construction_connect is needed in addition to nestmodule.cpp
  sw_construction_connect.start();

  // Mapping pointers to the first parameter value of each parameter to their respective names.
  // The bool indicates whether the value is an integer or not, and is determined at a later point.
  std::map< Name, std::pair< double*, bool > > param_pointers;
  if ( p_keys.size() != 0 )
  {
    size_t i = 0;
    for ( auto& key : p_keys )
    {
      // Shifting the pointer to the first value of the parameter.
      param_pointers[ key ] = std::make_pair( p_values + i * n, false );
      ++i;
    }
  }

  const auto synapse_model_id = kernel().model_manager.get_synapse_model_id( syn_model );
  const auto syn_model_defaults = kernel().model_manager.get_connector_defaults( synapse_model_id );

  // Dictionary holding additional synapse parameters, passed to the connect call.
  std::vector< DictionaryDatum > param_dicts;
  param_dicts.reserve( kernel().vp_manager.get_num_threads() );
  for ( size_t i = 0; i < kernel().vp_manager.get_num_threads(); ++i )
  {
    param_dicts.emplace_back( new Dictionary );
    for ( auto& param_key : p_keys )
    {
      const Name param_name = param_key; // Convert string to Name
      // Check that the parameter exists for the synapse model.
      const auto syn_model_default_it = syn_model_defaults->find( param_name );
      if ( syn_model_default_it == syn_model_defaults->end() )
      {
        throw BadParameter( syn_model + " does not have parameter " + param_key );
      }

      // If the default value is an integer, the synapse parameter must also be an integer.
      if ( dynamic_cast< IntegerDatum* >( syn_model_default_it->second.datum() ) )
      {
        param_pointers[ param_key ].second = true;
        ( *param_dicts[ i ] )[ param_key ] = Token( new IntegerDatum( 0 ) );
      }
      else
      {
        ( *param_dicts[ i ] )[ param_key ] = Token( new DoubleDatum( 0.0 ) );
      }
    }
  }

  // Increments pointers to weight and delay, if they are specified.
  auto increment_wd = [ weights, delays ]( decltype( weights ) & w, decltype( delays ) & d )
  {
    if ( weights )
    {
      ++w;
    }
    if ( delays )
    {
      ++d;
    }
  };

  // Set flag before entering parallel section in case we have fewer connections than ranks.
  set_connections_have_changed();

  // Vector for storing exceptions raised by threads.
  std::vector< std::shared_ptr< WrappedThreadException > > exceptions_raised( kernel().vp_manager.get_num_threads() );

#pragma omp parallel
  {
    const auto tid = kernel().vp_manager.get_thread_id();
    try
    {
      auto s = sources;
      auto t = targets;
      auto w = weights;
      auto d = delays;
      double weight_buffer = numerics::nan;
      double delay_buffer = numerics::nan;
      int index_counter = 0; // Index of the current connection, for connection parameters

      for ( ; s != sources + n; ++s, ++t, ++index_counter )
      {
        if ( 0 >= *s or static_cast< size_t >( *s ) > kernel().node_manager.size() )
        {
          throw UnknownNode( *s );
        }
        if ( 0 >= *t or static_cast< size_t >( *t ) > kernel().node_manager.size() )
        {
          throw UnknownNode( *t );
        }
        auto target_node = kernel().node_manager.get_node_or_proxy( *t, tid );
        if ( target_node->is_proxy() )
        {
          increment_wd( w, d );
          continue;
        }

        // If weights or delays are specified, the buffers are replaced with the values.
        // If not, the buffers will be NaN and replaced by a default value by the connect function.
        if ( weights )
        {
          weight_buffer = *w;
        }
        if ( delays )
        {
          delay_buffer = *d;
        }

        // Store the key-value pair of each parameter in the Dictionary.
        for ( auto& param_pointer_pair : param_pointers )
        {
          // Increment the pointer to the parameter value.
          const auto param_pointer = param_pointer_pair.second.first;
          const auto is_int = param_pointer_pair.second.second;
          auto* param = param_pointer + index_counter;

          // Integer parameters are stored as IntegerDatums.
          if ( is_int )
          {
            const auto rtype_as_long = static_cast< long >( *param );

            if ( *param > 1L << 31 or std::abs( *param - rtype_as_long ) > 0 ) // To avoid rounding errors
            {
              const auto msg = std::string( "Expected integer value for " ) + param_pointer_pair.first.toString()
                + ", but got double.";
              throw BadParameter( msg );
            }

            // Change value of dictionary entry without allocating new datum.
            auto id = static_cast< IntegerDatum* >( ( ( *param_dicts[ tid ] )[ param_pointer_pair.first ] ).datum() );
            ( *id ) = rtype_as_long;
          }
          else
          {
            auto dd = static_cast< DoubleDatum* >( ( ( *param_dicts[ tid ] )[ param_pointer_pair.first ] ).datum() );
            ( *dd ) = *param;
          }
        }

        connect( *s, target_node, tid, synapse_model_id, param_dicts[ tid ], delay_buffer, weight_buffer );

        ALL_ENTRIES_ACCESSED( *param_dicts[ tid ], "connect_arrays", "Unread dictionary entries: " );

        increment_wd( w, d );
      }
    }
    catch ( std::exception& err )
    {
      // We must create a new exception here, err's lifetime ends at the end of the catch block.
      exceptions_raised.at( tid ) = std::shared_ptr< WrappedThreadException >( new WrappedThreadException( err ) );
    }
  }
  // check if any exceptions have been raised
  for ( size_t tid = 0; tid < kernel().vp_manager.get_num_threads(); ++tid )
  {
    if ( exceptions_raised.at( tid ).get() )
    {
      throw WrappedThreadException( *( exceptions_raised.at( tid ) ) );
    }
  }

  sw_construction_connect.stop();
}

void
nest::ConnectionManager::connect_sonata( const DictionaryDatum& graph_specs, const long hyberslab_size )
{
#ifdef HAVE_HDF5
  SonataConnector sonata_connector( graph_specs, hyberslab_size );

  // Set flag before calling sonata_connector.connect() in case exception is thrown after some connections have been
  // created.
  set_connections_have_changed();
  sonata_connector.connect();
#else
  throw KernelException( "Cannot use connect_sonata because NEST was compiled without HDF5 support" );
#endif
}

void
nest::ConnectionManager::connect_tripartite( NodeCollectionPTR sources,
  NodeCollectionPTR targets,
  NodeCollectionPTR third,
  const DictionaryDatum& conn_spec,
  const DictionaryDatum& third_conn_spec,
  const std::map< Name, std::vector< DictionaryDatum > >& syn_specs )
{
  if ( sources->empty() )
  {
    throw IllegalConnection( "Presynaptic nodes cannot be an empty NodeCollection" );
  }
  if ( targets->empty() )
  {
    throw IllegalConnection( "Postsynaptic nodes cannot be an empty NodeCollection" );
  }
  if ( third->empty() )
  {
    throw IllegalConnection( "Third-factor nodes cannot be an empty NodeCollection" );
  }

  conn_spec->clear_access_flags();
  for ( auto& [ key, syn_spec_array ] : syn_specs )
  {
    for ( auto& syn_spec : syn_spec_array )
    {
      syn_spec->clear_access_flags();
    }
  }

  if ( not conn_spec->known( names::rule ) )
  {
    throw BadProperty( "The connection specification must contain a connection rule." );
  }
  if ( not third_conn_spec->known( names::rule ) )
  {
    throw BadProperty( "The third-factor connection specification must contain a connection rule." );
  }

  const std::string primary_rule = static_cast< const std::string >( ( *conn_spec )[ names::rule ] );
  const std::string third_rule = static_cast< const std::string >( ( *third_conn_spec )[ names::rule ] );

  ConnBuilder cb( primary_rule, third_rule, sources, targets, third, conn_spec, third_conn_spec, syn_specs );

  // at this point, all entries in conn_spec and syn_spec have been checked
  ALL_ENTRIES_ACCESSED( *conn_spec, "Connect", "Unread dictionary entries in conn_spec: " );
  for ( auto& [ key, syn_spec_array ] : syn_specs )
  {
    for ( auto& syn_spec : syn_spec_array )
    {
      ALL_ENTRIES_ACCESSED( *syn_spec, "Connect", "Unread dictionary entries in syn_specs: " );
    }
  }

  // Set flag before calling cb->connect() in case exception is thrown after some connections have been created.
  set_connections_have_changed();

  cb.connect();
}


void
nest::ConnectionManager::connect_( Node& source,
  Node& target,
  const size_t s_node_id,
  const size_t tid,
  const synindex syn_id,
  const DictionaryDatum& params,
  const double delay,
  const double weight )
{
  ConnectorModel& conn_model = kernel().model_manager.get_connection_model( syn_id, tid );

  const bool clopath_archiving = conn_model.has_property( ConnectionModelProperties::REQUIRES_CLOPATH_ARCHIVING );
  if ( clopath_archiving and not dynamic_cast< ClopathArchivingNode* >( &target ) )
  {
    throw NotImplemented( "This synapse model is not supported by the neuron model of at least one connection." );
  }

  const bool urbanczik_archiving = conn_model.has_property( ConnectionModelProperties::REQUIRES_URBANCZIK_ARCHIVING );
  if ( urbanczik_archiving and not target.supports_urbanczik_archiving() )
  {
    throw NotImplemented( "This synapse model is not supported by the neuron model of at least one  connection." );
  }

  const bool eprop_archiving = conn_model.has_property( ConnectionModelProperties::REQUIRES_EPROP_ARCHIVING );
  if ( eprop_archiving
    and not( dynamic_cast< EpropArchivingNodeRecurrent< false >* >( &target )
      or dynamic_cast< EpropArchivingNodeRecurrent< true >* >( &target )
      or dynamic_cast< EpropArchivingNodeReadout< false >* >( &target )
      or dynamic_cast< EpropArchivingNodeReadout< true >* >( &target ) ) )
  {
    throw NotImplemented( "This synapse model is not supported by the neuron model of at least one connection." );
  }

  const bool is_primary = conn_model.has_property( ConnectionModelProperties::IS_PRIMARY );
  conn_model.add_connection( source, target, connections_[ tid ], syn_id, params, delay, weight );
  source_table_.add_source( tid, syn_id, s_node_id, is_primary );

  increase_connection_count( tid, syn_id );

  // We do not check has_primary_connections_ and secondary_connections_exist_
  // directly as this led to worse performance on the supercomputer Piz Daint.
  if ( check_primary_connections_[ tid ].is_false() and is_primary )
  {
#pragma omp atomic write
    has_primary_connections_ = true;
    check_primary_connections_.set_true( tid );
  }
  else if ( check_secondary_connections_[ tid ].is_false() and not is_primary )
  {
#pragma omp atomic write
    secondary_connections_exist_ = true;
    check_secondary_connections_.set_true( tid );
  }
}

void
nest::ConnectionManager::connect_to_device_( Node& source,
  Node& target,
  const size_t s_node_id,
  const size_t tid,
  const synindex syn_id,
  const DictionaryDatum& params,
  const double delay,
  const double weight )
{
  // create entries in connection structure for connections to devices
  target_table_devices_.add_connection_to_device( source, target, s_node_id, tid, syn_id, params, delay, weight );

  increase_connection_count( tid, syn_id );
}

void
nest::ConnectionManager::connect_from_device_( Node& source,
  Node& target,
  const size_t tid,
  const synindex syn_id,
  const DictionaryDatum& params,
  const double delay,
  const double weight )
{
  // create entries in connections vector of devices
  target_table_devices_.add_connection_from_device( source, target, tid, syn_id, params, delay, weight );

  increase_connection_count( tid, syn_id );
}

void
nest::ConnectionManager::increase_connection_count( const size_t tid, const synindex syn_id )
{
  if ( num_connections_[ tid ].size() <= syn_id )
  {
    num_connections_[ tid ].resize( syn_id + 1 );
  }
  ++num_connections_[ tid ][ syn_id ];
  if ( num_connections_[ tid ][ syn_id ] > MAX_LCID - 1 )
  {
    // MAX_LCID is used as invalid marker an can therefore not be used as a proper value
    throw KernelException(
      String::compose( "Too many connections: at most %1 connections supported per virtual "
                       "process and synapse model.",
        MAX_LCID - 1 ) );
  }
}

size_t
nest::ConnectionManager::find_connection( const size_t tid,
  const synindex syn_id,
  const size_t snode_id,
  const size_t tnode_id )
{
  // lcid will hold the position of the /first/ connection from node
  // snode_id to any local node, or be invalid
  size_t lcid = source_table_.find_first_source( tid, syn_id, snode_id );
  if ( lcid == invalid_index )
  {
    return invalid_index;
  }

  // lcid will hold the position of the /first/ connection from node
  // snode_id to node tnode_id, or be invalid
  lcid = connections_[ tid ][ syn_id ]->find_first_target( tid, lcid, tnode_id );
  if ( lcid != invalid_index )
  {
    return lcid;
  }

  return lcid;
}

void
nest::ConnectionManager::disconnect( const size_t tid,
  const synindex syn_id,
  const size_t snode_id,
  const size_t tnode_id )
{
  assert( syn_id != invalid_synindex );

  const size_t lcid = find_connection( tid, syn_id, snode_id, tnode_id );

  if ( lcid == invalid_index ) // this function should only be called
                               // with a valid connection
  {
    throw InexistentConnection();
  }

  connections_[ tid ][ syn_id ]->disable_connection( lcid );
  source_table_.disable_connection( tid, syn_id, lcid );

  --num_connections_[ tid ][ syn_id ];
}

void
nest::ConnectionManager::trigger_update_weight( const long vt_id,
  const std::vector< spikecounter >& dopa_spikes,
  const double t_trig )
{
  const size_t tid = kernel().vp_manager.get_thread_id();

  for ( std::vector< ConnectorBase* >::iterator it = connections_[ tid ].begin(); it != connections_[ tid ].end();
        ++it )
  {
    if ( *it )
    {
      ( *it )->trigger_update_weight(
        vt_id, tid, dopa_spikes, t_trig, kernel().model_manager.get_connection_models( tid ) );
    }
  }
}

size_t
nest::ConnectionManager::get_num_target_data( const size_t tid ) const
{
  size_t num_connections = 0;
  for ( synindex syn_id = 0; syn_id < connections_[ tid ].size(); ++syn_id )
  {
    if ( connections_[ tid ][ syn_id ] )
    {
      num_connections += source_table_.num_unique_sources( tid, syn_id );
    }
  }
  return num_connections;
}

size_t
nest::ConnectionManager::get_num_connections() const
{
  size_t num_connections = 0;
  for ( size_t t = 0; t < num_connections_.size(); ++t )
  {
    for ( size_t s = 0; s < num_connections_[ t ].size(); ++s )
    {
      num_connections += num_connections_[ t ][ s ];
    }
  }

  return num_connections;
}

size_t
nest::ConnectionManager::get_num_connections( const synindex syn_id ) const
{
  size_t num_connections = 0;
  for ( size_t t = 0; t < num_connections_.size(); ++t )
  {
    if ( num_connections_[ t ].size() > syn_id )
    {
      num_connections += num_connections_[ t ][ syn_id ];
    }
  }

  return num_connections;
}

ArrayDatum
nest::ConnectionManager::get_connections( const DictionaryDatum& params )
{
  std::deque< ConnectionID > connectome;
  const Token& source_t = params->lookup( names::source );
  const Token& target_t = params->lookup( names::target );
  const Token& syn_model_t = params->lookup( names::synapse_model );
  NodeCollectionPTR source_a = NodeCollectionPTR( nullptr );
  NodeCollectionPTR target_a = NodeCollectionPTR( nullptr );

  long synapse_label = UNLABELED_CONNECTION;
  updateValue< long >( params, names::synapse_label, synapse_label );

  if ( not source_t.empty() )
  {
    source_a = getValue< NodeCollectionDatum >( source_t );
    if ( not source_a->valid() )
    {
      throw KernelException( "GetConnection requires valid source NodeCollection." );
    }
  }
  if ( not target_t.empty() )
  {
    target_a = getValue< NodeCollectionDatum >( target_t );
    if ( not target_a->valid() )
    {
      throw KernelException( "GetConnection requires valid target NodeCollection." );
    }
  }

  // If connections have changed, (re-)build presynaptic infrastructure,
  // as this may involve sorting connections by source node IDs.
  if ( connections_have_changed() )
  {
    // We need to update min_delay because it is used by check_wfr_use() below
    // to set secondary event data size.
    update_delay_extrema_();

    // Check whether waveform relaxation is used on any MPI process;
    // needs to be called before update_connection_infrastructure since
    // it resizes coefficient arrays for secondary events
    kernel().node_manager.check_wfr_use();

#pragma omp parallel
    {
      const size_t tid = kernel().vp_manager.get_thread_id();
      kernel().simulation_manager.update_connection_infrastructure( tid );
    }
  }

  // We check, whether a synapse model is given. If not, we will iterate all.
  size_t syn_id = 0;
  if ( not syn_model_t.empty() )
  {
    const std::string synmodel_name = getValue< std::string >( syn_model_t );
    // The following throws UnknownSynapseType for invalid synmodel_name
    syn_id = kernel().model_manager.get_synapse_model_id( synmodel_name );
    get_connections( connectome, source_a, target_a, syn_id, synapse_label );
  }
  else
  {
    for ( syn_id = 0; syn_id < kernel().model_manager.get_num_connection_models(); ++syn_id )
    {
      get_connections( connectome, source_a, target_a, syn_id, synapse_label );
    }
  }

  ArrayDatum result;
  result.reserve( connectome.size() );

  while ( not connectome.empty() )
  {
    result.push_back( ConnectionDatum( connectome.front() ) );
    connectome.pop_front();
  }

  get_connections_has_been_called_ = true;

  return result;
}

// Helper method which removes ConnectionIDs from input deque and
// appends them to output deque.
static inline std::deque< nest::ConnectionID >&
extend_connectome( std::deque< nest::ConnectionID >& out, std::deque< nest::ConnectionID >& in )
{
  while ( not in.empty() )
  {
    out.push_back( in.front() );
    in.pop_front();
  }

  return out;
}

void
nest::ConnectionManager::split_to_neuron_device_vectors_( const size_t tid,
  NodeCollectionPTR nodecollection,
  std::vector< size_t >& neuron_node_ids,
  std::vector< size_t >& device_node_ids ) const
{
  NodeCollection::const_iterator t_id = nodecollection->begin();
  for ( ; t_id < nodecollection->end(); ++t_id )
  {
    const size_t node_id = ( *t_id ).node_id;
    const auto node = kernel().node_manager.get_node_or_proxy( node_id, tid );
    // Normal neuron nodes have proxies. Globally receiving devices, e.g. volume transmitter, don't have a local
    // receiver, but are connected in the same way as normal neuron nodes. Therefore they have to be treated as such
    // here.
    if ( node->has_proxies() or not node->local_receiver() )
    {
      neuron_node_ids.push_back( node_id );
    }
    else
    {
      device_node_ids.push_back( node_id );
    }
  }
}

void
nest::ConnectionManager::get_connections_( const size_t tid,
  std::deque< ConnectionID >& conns_in_thread,
  NodeCollectionPTR,
  NodeCollectionPTR,
  synindex syn_id,
  long synapse_label ) const
{
  ConnectorBase* connections = connections_[ tid ][ syn_id ];
  if ( connections )
  {
    // Passing target_node_id = 0 ignores target_node_id while getting connections.
    const size_t num_connections_in_thread = connections->size();
    for ( size_t lcid = 0; lcid < num_connections_in_thread; ++lcid )
    {
      const size_t source_node_id = source_table_.get_node_id( tid, syn_id, lcid );
      connections->get_connection( source_node_id, 0, tid, lcid, synapse_label, conns_in_thread );
    }
  }

  target_table_devices_.get_connections( 0, 0, tid, syn_id, synapse_label, conns_in_thread );
}

void
nest::ConnectionManager::get_connections_to_targets_( const size_t tid,
  std::deque< ConnectionID >& conns_in_thread,
  NodeCollectionPTR,
  NodeCollectionPTR target,
  synindex syn_id,
  long synapse_label ) const
{
  // Split targets into neuron- and device-vectors.
  std::vector< size_t > target_neuron_node_ids;
  std::vector< size_t > target_device_node_ids;
  split_to_neuron_device_vectors_( tid, target, target_neuron_node_ids, target_device_node_ids );

  // Getting regular connections, if they exist.
  ConnectorBase* connections = connections_[ tid ][ syn_id ];
  if ( connections )
  {
    const size_t num_connections_in_thread = connections->size();
    for ( size_t lcid = 0; lcid < num_connections_in_thread; ++lcid )
    {
      const size_t source_node_id = source_table_.get_node_id( tid, syn_id, lcid );
      connections->get_connection_with_specified_targets(
        source_node_id, target_neuron_node_ids, tid, lcid, synapse_label, conns_in_thread );
    }
  }

  // Getting connections from devices.
  for ( auto t_node_id : target_neuron_node_ids )
  {
    target_table_devices_.get_connections_from_devices_( 0, t_node_id, tid, syn_id, synapse_label, conns_in_thread );
  }

  // Getting connections to devices.
  for ( auto t_device_id : target_device_node_ids )
  {
    target_table_devices_.get_connections_to_devices_( 0, t_device_id, tid, syn_id, synapse_label, conns_in_thread );
  }
}

void
nest::ConnectionManager::get_connections_from_sources_( const size_t tid,
  std::deque< ConnectionID >& conns_in_thread,
  NodeCollectionPTR source,
  NodeCollectionPTR target,
  synindex syn_id,
  long synapse_label ) const
{
  // Split targets into neuron- and device-vectors.
  std::vector< size_t > target_neuron_node_ids;
  std::vector< size_t > target_device_node_ids;
  if ( target.get() )
  {
    split_to_neuron_device_vectors_( tid, target, target_neuron_node_ids, target_device_node_ids );
  }

  const ConnectorBase* connections = connections_[ tid ][ syn_id ];
  if ( connections )
  {
    const size_t num_connections_in_thread = connections->size();
    for ( size_t lcid = 0; lcid < num_connections_in_thread; ++lcid )
    {
      const size_t source_node_id = source_table_.get_node_id( tid, syn_id, lcid );
      if ( source->contains( source_node_id ) )
      {
        if ( not target.get() )
        {
          // Passing target_node_id = 0 ignores target_node_id while getting
          // connections.
          connections->get_connection( source_node_id, 0, tid, lcid, synapse_label, conns_in_thread );
        }
        else
        {
          connections->get_connection_with_specified_targets(
            source_node_id, target_neuron_node_ids, tid, lcid, synapse_label, conns_in_thread );
        }
      }
    }
  }

  NodeCollection::const_iterator s_id = source->begin();
  for ( ; s_id < source->end(); ++s_id )
  {
    const size_t source_node_id = ( *s_id ).node_id;
    if ( not target.get() )
    {
      target_table_devices_.get_connections( source_node_id, 0, tid, syn_id, synapse_label, conns_in_thread );
    }
    else
    {
      for ( std::vector< size_t >::const_iterator t_node_id = target_neuron_node_ids.begin();
            t_node_id != target_neuron_node_ids.end();
            ++t_node_id )
      {
        // target_table_devices_ contains connections both to and from
        // devices. First we get connections from devices.
        target_table_devices_.get_connections_from_devices_(
          source_node_id, *t_node_id, tid, syn_id, synapse_label, conns_in_thread );
      }
      for ( std::vector< size_t >::const_iterator t_node_id = target_device_node_ids.begin();
            t_node_id != target_device_node_ids.end();
            ++t_node_id )
      {
        // Then, we get connections to devices.
        target_table_devices_.get_connections_to_devices_(
          source_node_id, *t_node_id, tid, syn_id, synapse_label, conns_in_thread );
      }
    }
  }
}

void
nest::ConnectionManager::get_connections( std::deque< ConnectionID >& connectome,
  NodeCollectionPTR source,
  NodeCollectionPTR target,
  synindex syn_id,
  long synapse_label ) const
{
  if ( get_num_connections( syn_id ) == 0 )
  {
    return;
  }

#pragma omp parallel
  {
    if ( is_source_table_cleared() )
    {
      throw KernelException( "Invalid attempt to access connection information: source table was cleared." );
    }

    size_t tid = kernel().vp_manager.get_thread_id();

    std::deque< ConnectionID > conns_in_thread;

    if ( not source.get() and not target.get() )
    {
      get_connections_( tid, conns_in_thread, source, target, syn_id, synapse_label );
    }
    else if ( not source.get() and target.get() )
    {
      get_connections_to_targets_( tid, conns_in_thread, source, target, syn_id, synapse_label );
    }
    else if ( source.get() )
    {
      get_connections_from_sources_( tid, conns_in_thread, source, target, syn_id, synapse_label );
    }

    if ( conns_in_thread.size() > 0 )
    {
#pragma omp critical( get_connections )
      {
        extend_connectome( connectome, conns_in_thread );
      }
    }
  }
}

void
nest::ConnectionManager::get_source_node_ids_( const size_t tid,
  const synindex syn_id,
  const size_t tnode_id,
  std::vector< size_t >& sources )
{
  std::vector< size_t > source_lcids;
  if ( connections_[ tid ][ syn_id ] )
  {
    connections_[ tid ][ syn_id ]->get_source_lcids( tid, tnode_id, source_lcids );
    source_table_.get_source_node_ids( tid, syn_id, source_lcids, sources );
  }
}

void
nest::ConnectionManager::get_sources( const std::vector< size_t >& targets,
  const size_t syn_id,
  std::vector< std::vector< size_t > >& sources )
{
  sources.resize( targets.size() );
  for ( std::vector< std::vector< size_t > >::iterator i = sources.begin(); i != sources.end(); ++i )
  {
    ( *i ).clear();
  }

  for ( size_t tid = 0; tid < kernel().vp_manager.get_num_threads(); ++tid )
  {
    for ( size_t i = 0; i < targets.size(); ++i )
    {
      get_source_node_ids_( tid, syn_id, targets[ i ], sources[ i ] );
    }
  }
}

void
nest::ConnectionManager::get_targets( const std::vector< size_t >& sources,
  const size_t syn_id,
  const std::string& post_synaptic_element,
  std::vector< std::vector< size_t > >& targets )
{
  targets.resize( sources.size() );
  for ( std::vector< std::vector< size_t > >::iterator i = targets.begin(); i != targets.end(); ++i )
  {
    ( *i ).clear();
  }

  for ( size_t tid = 0; tid < kernel().vp_manager.get_num_threads(); ++tid )
  {
    for ( size_t i = 0; i < sources.size(); ++i )
    {
      const size_t start_lcid = source_table_.find_first_source( tid, syn_id, sources[ i ] );
      if ( start_lcid != invalid_index )
      {
        connections_[ tid ][ syn_id ]->get_target_node_ids( tid, start_lcid, post_synaptic_element, targets[ i ] );
      }
    }
  }
}

void
nest::ConnectionManager::sort_connections( const size_t tid )
{
  assert( not source_table_.is_cleared() );
  if ( use_compressed_spikes_ )
  {
    for ( synindex syn_id = 0; syn_id < connections_[ tid ].size(); ++syn_id )
    {
      if ( connections_[ tid ][ syn_id ] )
      {
        connections_[ tid ][ syn_id ]->sort_connections( source_table_.get_thread_local_sources( tid )[ syn_id ] );
      }
    }
    remove_disabled_connections( tid );
  }
}

void
nest::ConnectionManager::compute_target_data_buffer_size()
{
  // Determine number of target data on this rank. Since each thread
  // has its own data structures, we need to count connections on every
  // thread separately to compute the total number of sources.
  size_t num_target_data = 0;
  for ( size_t tid = 0; tid < kernel().vp_manager.get_num_threads(); ++tid )
  {
    num_target_data += get_num_target_data( tid );
  }

  // Determine maximum number of target data across all ranks, because
  // all ranks need identically sized buffers.
  std::vector< long > global_num_target_data( kernel().mpi_manager.get_num_processes() );
  global_num_target_data[ kernel().mpi_manager.get_rank() ] = num_target_data;
  kernel().mpi_manager.communicate( global_num_target_data );
  const size_t max_num_target_data = *std::max_element( global_num_target_data.begin(), global_num_target_data.end() );

  // MPI buffers should have at least two entries per process
  const size_t min_num_target_data = 2 * kernel().mpi_manager.get_num_processes();

  // Adjust target data buffers accordingly
  kernel().mpi_manager.set_buffer_size_target_data( std::max( min_num_target_data, max_num_target_data ) );
}

void
nest::ConnectionManager::compute_compressed_secondary_recv_buffer_positions( const size_t tid )
{
#pragma omp single
  {
    buffer_pos_of_source_node_id_syn_id_.clear();
  }

  source_table_.compute_buffer_pos_for_unique_secondary_sources( tid, buffer_pos_of_source_node_id_syn_id_ );
  secondary_recv_buffer_pos_[ tid ].resize( connections_[ tid ].size() );

  const synindex syn_id_end = connections_[ tid ].size();
  for ( synindex syn_id = 0; syn_id < syn_id_end; ++syn_id )
  {
    std::vector< size_t >& positions = secondary_recv_buffer_pos_[ tid ][ syn_id ];

    if ( connections_[ tid ][ syn_id ] )
    {
      ConnectorModel& conn_model = kernel().model_manager.get_connection_model( syn_id, tid );
      const bool is_primary = conn_model.has_property( ConnectionModelProperties::IS_PRIMARY );

      if ( not is_primary )
      {
        positions.clear();
        const size_t lcid_end = get_num_connections_( tid, syn_id );
        positions.resize( lcid_end, 0 );

        // Compute and store the buffer position from which this connection
        // should read secondary events.
        for ( size_t lcid = 0; lcid < lcid_end; ++lcid )
        {
          const size_t source_node_id = source_table_.get_node_id( tid, syn_id, lcid );
          const size_t sg_s_id = source_table_.pack_source_node_id_and_syn_id( source_node_id, syn_id );
          const size_t source_rank = kernel().mpi_manager.get_process_id_of_node_id( source_node_id );

          positions[ lcid ] = buffer_pos_of_source_node_id_syn_id_[ sg_s_id ]
            + kernel().mpi_manager.get_recv_displacement_secondary_events_in_int( source_rank );
        }
      }
    }
  }
}

nest::ConnectionManager::ConnectionType
nest::ConnectionManager::connection_required( Node*& source, Node*& target, size_t tid )
{
  // The caller has to check and guarantee that the target is not a
  // proxy and that it is on thread tid.
  assert( not target->is_proxy() );
  size_t target_vp = target->get_vp();
  assert( kernel().vp_manager.is_local_vp( target_vp ) );
  assert( kernel().vp_manager.vp_to_thread( target_vp ) == tid );

  // Connections to nodes with proxies (neurons or devices with
  // proxies) which are local to tid have always to be
  // established, independently of where and what type the source node
  // is.
  if ( target->has_proxies() )
  {
    if ( source->has_proxies() )
    {
      return CONNECT;
    }
    else
    {
      return CONNECT_FROM_DEVICE;
    }
  }

  // Local receivers are all devices that collect data only from
  // thread-local nodes.
  if ( target->local_receiver() )
  {
    // Connections to nodes with one node per process (MUSIC proxies
    // or similar devices) have to be established by the thread of the
    // target if the source is on the local process even though the
    // source may be a proxy on tid.
    if ( target->one_node_per_process() )
    {
      if ( kernel().node_manager.is_local_node( source ) )
      {
        return CONNECT_TO_DEVICE;
      }
      else
      {
        return NO_CONNECTION;
      }
    }

    // Connections from nodes with proxies (neurons or devices with
    // proxies) to devices are only created if source is not a proxy
    // and source and target are both on thread tid
    const size_t source_thread = source->get_thread();
    const bool source_is_proxy = source->is_proxy();
    if ( source->has_proxies() and source_thread == tid and not source_is_proxy )
    {
      return CONNECT_TO_DEVICE;
    }

    // Connections from devices to devices are established only on the
    // vp that is suggested for the target node. In this case, we also
    // set the pointer to the source node on the target's thread.
    if ( not source->has_proxies() )
    {
      const size_t target_node_id = target->get_node_id();
      target_vp = kernel().vp_manager.node_id_to_vp( target_node_id );
      const bool target_vp_local = kernel().vp_manager.is_local_vp( target_vp );
      const size_t target_thread = kernel().vp_manager.vp_to_thread( target_vp );

      if ( target_vp_local and target_thread == tid )
      {
        const size_t source_node_id = source->get_node_id();
        source = kernel().node_manager.get_node_or_proxy( source_node_id, target_thread );
        return CONNECT_FROM_DEVICE;
      }
    }
  }

  // Globally receiving nodes (e.g. the volume transmitter) have to be
  // connected regardless of where the source is. However, we
  // currently prohibit connections from devices to global receivers.
  else
  {
    if ( source->has_proxies() )
    {
      target = kernel().node_manager.get_node_or_proxy( target->get_node_id(), tid );
      return CONNECT;
    }

    throw IllegalConnection( "We do not allow connection of a device to a global receiver at the moment." );
  }

  return NO_CONNECTION;
}

void
nest::ConnectionManager::set_stdp_eps( const double stdp_eps )
{
  if ( not( stdp_eps < Time::get_resolution().get_ms() ) )
  {
    throw KernelException(
      "The epsilon used for spike-time comparison in STDP must be less "
      "than the simulation resolution." );
  }
  else if ( stdp_eps < 0 )
  {
    throw KernelException(
      "The epsilon used for spike-time comparison in STDP must not be "
      "negative." );
  }
  else
  {
    stdp_eps_ = stdp_eps;

    std::ostringstream os;
    os << "Epsilon for spike-time comparison in STDP was set to "
       << std::setprecision( std::numeric_limits< long double >::digits10 ) << stdp_eps_ << ".";

    LOG( M_INFO, "ConnectionManager::set_stdp_eps", os.str() );
  }
}

// recv_buffer can not be a const reference as iterators used in
// secondary events must not be const
bool
nest::ConnectionManager::deliver_secondary_events( const size_t tid,
  const bool called_from_wfr_update,
  std::vector< unsigned int >& recv_buffer )
{
  const std::vector< ConnectorModel* >& cm = kernel().model_manager.get_connection_models( tid );
  const Time stamp =
    kernel().simulation_manager.get_slice_origin() + Time::step( 1 - kernel().connection_manager.get_min_delay() );
  const std::vector< std::vector< size_t > >& positions_tid = secondary_recv_buffer_pos_[ tid ];

  const synindex syn_id_end = positions_tid.size();
  for ( synindex syn_id = 0; syn_id < syn_id_end; ++syn_id )
  {
    const ConnectorModel& conn_model = kernel().model_manager.get_connection_model( syn_id, tid );
    const bool supports_wfr = conn_model.has_property( ConnectionModelProperties::SUPPORTS_WFR );
    if ( not called_from_wfr_update or supports_wfr )
    {
      if ( positions_tid[ syn_id ].size() > 0 )
      {
        SecondaryEvent& prototype = kernel().model_manager.get_secondary_event_prototype( syn_id, tid );

        size_t lcid = 0;
        const size_t lcid_end = positions_tid[ syn_id ].size();
        while ( lcid < lcid_end )
        {
          std::vector< unsigned int >::iterator readpos = recv_buffer.begin() + positions_tid[ syn_id ][ lcid ];
          prototype << readpos;
          prototype.set_stamp( stamp );

          // send delivers event to all targets with the same source
          // and returns how many targets this event was delivered to
          lcid += connections_[ tid ][ syn_id ]->send( tid, lcid, cm, prototype );
        }
      }
    }
  }

  // Read waveform relaxation done marker from last position in every
  // chunk
  bool done = true;
  for ( size_t rank = 0; rank < kernel().mpi_manager.get_num_processes(); ++rank )
  {
    done =
      done and recv_buffer[ kernel().mpi_manager.get_done_marker_position_in_secondary_events_recv_buffer( rank ) ];
  }
  return done;
}

void
nest::ConnectionManager::compress_secondary_send_buffer_pos( const size_t tid )
{
  target_table_.compress_secondary_send_buffer_pos( tid );
}

void
nest::ConnectionManager::remove_disabled_connections( const size_t tid )
{
  std::vector< ConnectorBase* >& connectors = connections_[ tid ];

  for ( synindex syn_id = 0; syn_id < connectors.size(); ++syn_id )
  {
    if ( not connectors[ syn_id ] )
    {
      continue;
    }

    // Source table and connectors are sorted synchronously. All invalid connections have
    // been sorted to end of source_table_. We find them there, then remove corresponding
    // elements from connectors.
    const size_t first_disabled_index = source_table_.remove_disabled_sources( tid, syn_id );

    if ( first_disabled_index != invalid_index )
    {
      connectors[ syn_id ]->remove_disabled_connections( first_disabled_index );
    }
  }
}

void
nest::ConnectionManager::resize_connections()
{
  kernel().vp_manager.assert_thread_parallel();

  connections_.at( kernel().vp_manager.get_thread_id() ).resize( kernel().model_manager.get_num_connection_models() );

  source_table_.resize_sources();
  target_table_devices_.resize_to_number_of_synapse_types();
}

void
nest::ConnectionManager::sync_has_primary_connections()
{
  has_primary_connections_ = kernel().mpi_manager.any_true( has_primary_connections_ );
}

void
nest::ConnectionManager::check_secondary_connections_exist()
{
  secondary_connections_exist_ = kernel().mpi_manager.any_true( secondary_connections_exist_ );
}

void
nest::ConnectionManager::set_connections_have_changed()
{
  assert( kernel().vp_manager.get_thread_id() == 0 );

  if ( get_connections_has_been_called_ )
  {
    std::string msg =
      "New connections created, connection descriptors previously obtained using 'GetConnections' are now invalid.";
    LOG( M_WARNING, "ConnectionManager", msg );
    // Reset the get_connections_has_been_called_ flag because we have updated connections.
    get_connections_has_been_called_ = false;
  }

  connections_have_changed_ = true;
}

void
nest::ConnectionManager::unset_connections_have_changed()
{
  connections_have_changed_ = false;
}


void
nest::ConnectionManager::collect_compressed_spike_data( const size_t tid )
{
  if ( use_compressed_spikes_ )
  {

#pragma omp single
    {
      source_table_.resize_compressible_sources();
    } // of omp single; implicit barrier

    source_table_.collect_compressible_sources( tid );
    kernel().get_omp_synchronization_construction_stopwatch().start();
#pragma omp barrier
    kernel().get_omp_synchronization_construction_stopwatch().stop();
#pragma omp single
    {
      source_table_.fill_compressed_spike_data( compressed_spike_data_ );
    } // of omp single; implicit barrier
  }
}

bool
nest::ConnectionManager::fill_target_buffer( const size_t tid,
  const size_t rank_start,
  const size_t rank_end,
  std::vector< TargetData >& send_buffer_target_data,
  TargetSendBufferPosition& send_buffer_position )
{
  // At this point, NEST has at least one synapse type (because we can only get here if at least
  // one connection has been created) and we know that iteration_state_ for each thread
  // contains a valid entry.
  const auto& csd_maps = source_table_.compressed_spike_data_map_;
  auto syn_id = iteration_state_.at( tid ).first;
  auto source_2_idx = iteration_state_.at( tid ).second;

  if ( syn_id >= csd_maps.size() )
  {
    return true; // this thread has previously written all its targets
  }

  do
  {
    const auto& conn_model = kernel().model_manager.get_connection_model( syn_id, tid );
    const bool is_primary = conn_model.has_property( ConnectionModelProperties::IS_PRIMARY );

    while ( source_2_idx != csd_maps.at( syn_id ).end() )
    {
      const auto source_gid = source_2_idx->first;
      const auto source_rank = kernel().mpi_manager.get_process_id_of_node_id( source_gid );
      if ( not( rank_start <= source_rank and source_rank < rank_end ) )
      {
        // We are not responsible for this source.
        ++source_2_idx;
        continue;
      }

      if ( send_buffer_position.is_chunk_filled( source_rank ) )
      {
        // When the we have filled the buffer space for one rank, we stop. If we continued for other ranks,
        // we would need to introduce "processed" markers to avoid multiple insertion (similar to base case).
        // Since sources should be evenly distributed, this should not matter very much.
        //
        // We store where we need to continue and stop iteration for now.
        iteration_state_.at( tid ) =
          std::pair< size_t, std::map< size_t, CSDMapEntry >::const_iterator >( syn_id, source_2_idx );

        return false; // there is data left to communicate
      }

      TargetData next_target_data;
      next_target_data.set_is_primary( is_primary );
      next_target_data.reset_marker();
      next_target_data.set_source_tid(
        kernel().vp_manager.vp_to_thread( kernel().vp_manager.node_id_to_vp( source_gid ) ) );
      next_target_data.set_source_lid( kernel().vp_manager.node_id_to_lid( source_gid ) );

      if ( is_primary )
      {
        TargetDataFields& target_fields = next_target_data.target_data;
        target_fields.set_syn_id( syn_id );
        target_fields.set_tid( 0 ); // meaningless, use 0 as fill
        target_fields.set_lcid( source_2_idx->second.get_source_index() );
      }
      else
      {
        const auto target_thread = source_2_idx->second.get_target_thread();
        const SpikeData& conn_info =
          compressed_spike_data_[ syn_id ][ source_2_idx->second.get_source_index() ][ target_thread ];
        assert( target_thread == static_cast< unsigned long >( conn_info.get_tid() ) );
        const size_t relative_recv_buffer_pos =
          get_secondary_recv_buffer_position( target_thread, syn_id, conn_info.get_lcid() )
          - kernel().mpi_manager.get_recv_displacement_secondary_events_in_int( source_rank );

        SecondaryTargetDataFields& secondary_fields = next_target_data.secondary_data;
        secondary_fields.set_recv_buffer_pos( relative_recv_buffer_pos );
        secondary_fields.set_syn_id( syn_id );
      }

      send_buffer_target_data.at( send_buffer_position.idx( source_rank ) ) = next_target_data;
      send_buffer_position.increase( source_rank );

      ++source_2_idx;
    } // end while

    ++syn_id;
    if ( syn_id < csd_maps.size() )
    {
      source_2_idx = csd_maps.at( syn_id ).begin();
    }
  } while ( syn_id < csd_maps.size() );

  // Store iteration state for this thread. If we get here, ther is nothing more to do for
  // this thread so we store a non-existing syn_id with a meaningless iterator to inform that
  // this thread has nothing to do in the next round.
  iteration_state_.at( tid ) =
    std::pair< size_t, std::map< size_t, CSDMapEntry >::const_iterator >( syn_id, source_2_idx );

  // Mark end of data for this round
  for ( size_t rank = rank_start; rank < rank_end; ++rank )
  {
    if ( send_buffer_position.idx( rank ) > send_buffer_position.begin( rank ) )
    {
      // We have written data for the rank, mark last written entry with END marker
      send_buffer_target_data.at( send_buffer_position.idx( rank ) - 1 ).set_end_marker();
    }
    else
    {
      // We have not written anything, mark beginning of chunk with INVALID marker
      send_buffer_target_data.at( send_buffer_position.begin( rank ) ).set_invalid_marker();
    }
  }

  // If we get here, this thread has written everything.
  return true;
}

void
nest::ConnectionManager::initialize_iteration_state()
{
  const size_t num_threads = kernel().vp_manager.get_num_threads();
  iteration_state_.clear();
  iteration_state_.reserve( num_threads );

  // This method only runs if at least one connection has been created,
  // so we must have at least one synapse model and we can start iteration
  // at the beginning of its compressed spike data map.
  auto begin = source_table_.compressed_spike_data_map_.at( 0 ).cbegin();
  for ( size_t t = 0; t < num_threads; ++t )
  {
    iteration_state_.push_back( std::pair< size_t, std::map< size_t, CSDMapEntry >::const_iterator >( 0, begin ) );
  }
}
