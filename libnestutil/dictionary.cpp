/*
 *  dictionary.cpp
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

#include <algorithm>
#include <boost/any.hpp>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "dictionary.h"
#include "kernel_manager.h"
#include "nest_datums.h"
#include "parameter.h"
#include "streamers.h"


// debug
std::string
debug_type( const boost::any& operand )
{
  return operand.type().name();
}

std::string
debug_dict_types( const dictionary& dict )
{
  std::string s = "[dictionary]\n";

  for ( auto& kv : dict )
  {
    s += kv.first + ": ";
    s += debug_type( kv.second ) + "\n";
  }
  return s;
}

std::ostream&
operator<<( std::ostream& os, const dictionary& dict )
{
  const auto max_key_length = std::max_element(
    dict.begin(), dict.end(), []( const dictionary::value_type s1, const dictionary::value_type s2 ) {
      return s1.first.length() < s2.first.length();
    } )->first.length();
  const std::string pre_padding = "    ";
  os << "dictionary{\n";
  for ( auto& kv : dict )
  {
    std::string type;
    std::stringstream value_stream;
    if ( is_int( kv.second ) )
    {
      type = "int";
      value_stream << boost::any_cast< int >( kv.second ) << '\n';
    }
    else if ( is_uint( kv.second ) )
    {
      type = "unsigned int";
      value_stream << boost::any_cast< unsigned int >( kv.second ) << '\n';
    }
    else if ( is_long( kv.second ) )
    {
      type = "long";
      value_stream << boost::any_cast< long >( kv.second ) << '\n';
    }
    else if ( is_size_t( kv.second ) )
    {
      type = "size_t";
      value_stream << boost::any_cast< size_t >( kv.second ) << '\n';
    }
    else if ( is_double( kv.second ) )
    {
      type = "double";
      value_stream << boost::any_cast< double >( kv.second ) << '\n';
    }
    else if ( is_bool( kv.second ) )
    {
      type = "bool";
      const auto value = boost::any_cast< bool >( kv.second );
      value_stream << ( value ? "true" : "false" ) << '\n';
    }
    else if ( is_string( kv.second ) )
    {
      type = "std::string";
      value_stream << "\"" << boost::any_cast< std::string >( kv.second ) << "\"\n";
    }
    else if ( is_int_vector( kv.second ) )
    {
      type = "std::vector<int>";
      value_stream << boost::any_cast< std::vector< int > >( kv.second ) << '\n';
    }
    else if ( is_double_vector( kv.second ) )
    {
      type = "std::vector<double>";
      value_stream << boost::any_cast< std::vector< double > >( kv.second ) << '\n';
    }
    else if ( is_double_vector_vector( kv.second ) )
    {
      type = "vector<vector<double>>";
      value_stream << "vector<vector<double>>" << '\n';
    }
    else if ( is_string_vector( kv.second ) )
    {
      type = "std::vector<std::string>";
      value_stream << boost::any_cast< std::vector< std::string > >( kv.second ) << '\n';
    }
    else if ( is_any_vector( kv.second ) )
    {
      type = "vector<boost::any>";
      value_stream << "vector<any>" << '\n';
    }
    else if ( is_dict( kv.second ) )
    {
      type = "dictionary";
      value_stream << "dictionary" << '\n';
    }
    else if ( is_parameter( kv.second ) )
    {
      type = "parameter";
      value_stream << "parameter" << '\n';
    }
    else if ( is_nc( kv.second ) )
    {
      type = "NodeCollection";
      value_stream << "NodeCollection" << '\n';
    }
    else
    {
      throw TypeMismatch( "Type is not known" );
    }
    const auto s = value_stream.str();
    const auto post_padding = max_key_length - kv.first.length() + 5;
    os << pre_padding << kv.first << std::setw( post_padding ) << "(" << type << ")"
       << " " << std::setw( 25 - type.length() ) << s;
  }
  return os << "}";
}

// int
bool
is_int( const boost::any& operand )
{
  return operand.type() == typeid( int );
}

bool
is_uint( const boost::any& operand )
{
  return operand.type() == typeid( unsigned int );
}

// long
bool
is_long( const boost::any& operand )
{
  return operand.type() == typeid( long );
}

bool
is_size_t( const boost::any& operand )
{
  return operand.type() == typeid( size_t );
}

// double
bool
is_double( const boost::any& operand )
{
  return operand.type() == typeid( double );
}

// bool
bool
is_bool( const boost::any& operand )
{
  return operand.type() == typeid( bool );
}

// string
bool
is_string( const boost::any& operand )
{
  return operand.type() == typeid( std::string );
}

// vector of ints
bool
is_int_vector( const boost::any& operand )
{
  return operand.type() == typeid( std::vector< int > );
}

// vector of doubles
bool
is_double_vector( const boost::any& operand )
{
  return operand.type() == typeid( std::vector< double > );
}

// vector of vector of doubles
bool
is_double_vector_vector( const boost::any& operand )
{
  return operand.type() == typeid( std::vector< std::vector< double > > );
}

// vector of strings
bool
is_string_vector( const boost::any& operand )
{
  return operand.type() == typeid( std::vector< std::string > );
}

// vector of boost::any
bool
is_any_vector( const boost::any& operand )
{
  return operand.type() == typeid( std::vector< boost::any > );
}

// dict
bool
is_dict( const boost::any& operand )
{
  return operand.type() == typeid( dictionary );
}

// parameter
bool
is_parameter( const boost::any& operand )
{
  return operand.type() == typeid( std::shared_ptr< nest::Parameter > );
}

// NodeCollection
bool
is_nc( const boost::any& operand )
{
  return operand.type() == typeid( NodeCollectionDatum );
}

bool
value_equal( const boost::any first, const boost::any second )
{
  if ( is_int( first ) )
  {
    if ( not is_int( second ) )
    {
      return false;
    }
    const auto this_value = boost::any_cast< int >( first );
    const auto other_value = boost::any_cast< int >( second );
    if ( this_value != other_value )
    {
      return false;
    }
  }
  else if ( is_long( first ) )
  {
    if ( not is_long( second ) )
    {
      return false;
    }
    const auto this_value = boost::any_cast< long >( first );
    const auto other_value = boost::any_cast< long >( second );
    if ( this_value != other_value )
    {
      return false;
    }
  }
  else if ( is_size_t( first ) )
  {
    if ( not is_size_t( second ) )
    {
      return false;
    }
    const auto this_value = boost::any_cast< size_t >( first );
    const auto other_value = boost::any_cast< size_t >( second );
    if ( this_value != other_value )
    {
      return false;
    }
  }
  else if ( is_double( first ) )
  {
    if ( not is_double( second ) )
    {
      return false;
    }
    const auto this_value = boost::any_cast< double >( first );
    const auto other_value = boost::any_cast< double >( second );
    if ( this_value != other_value )
    {
      return false;
    }
  }
  else if ( is_bool( first ) )
  {
    if ( not is_bool( second ) )
    {
      return false;
    }
    const auto this_value = boost::any_cast< bool >( first );
    const auto other_value = boost::any_cast< bool >( second );
    if ( this_value != other_value )
    {
      return false;
    }
  }
  else if ( is_string( first ) )
  {
    if ( not is_string( second ) )
    {
      return false;
    }
    const auto this_value = boost::any_cast< std::string >( first );
    const auto other_value = boost::any_cast< std::string >( second );
    if ( this_value != other_value )
    {
      return false;
    }
  }
  else if ( is_int_vector( first ) )
  {
    if ( not is_int_vector( second ) )
    {
      return false;
    }
    const auto this_value = boost::any_cast< std::vector< int > >( first );
    const auto other_value = boost::any_cast< std::vector< int > >( second );
    if ( this_value != other_value )
    {
      return false;
    }
  }
  else if ( is_double_vector( first ) )
  {
    if ( not is_double_vector( second ) )
    {
      return false;
    }
    const auto this_value = boost::any_cast< std::vector< double > >( first );
    const auto other_value = boost::any_cast< std::vector< double > >( second );
    if ( this_value != other_value )
    {
      return false;
    }
  }
  else if ( is_double_vector_vector( first ) )
  {
    if ( not is_double_vector_vector( second ) )
    {
      return false;
    }
    const auto this_value = boost::any_cast< std::vector< std::vector< double > > >( first );
    const auto other_value = boost::any_cast< std::vector< std::vector< double > > >( second );
    if ( this_value != other_value )
    {
      return false;
    }
  }
  else if ( is_string_vector( first ) )
  {
    if ( not is_string_vector( second ) )
    {
      return false;
    }
    const auto this_value = boost::any_cast< std::vector< std::string > >( first );
    const auto other_value = boost::any_cast< std::vector< std::string > >( second );
    if ( this_value != other_value )
    {
      return false;
    }
  }
  else if ( is_dict( first ) )
  {
    if ( not is_dict( second ) )
    {
      return false;
    }
    const auto this_value = boost::any_cast< dictionary >( first );
    const auto other_value = boost::any_cast< dictionary >( second );
    if ( this_value != other_value )
    {
      return false;
    }
  }
  else if ( is_parameter( first ) )
  {
    if ( not is_parameter( second ) )
    {
      return false;
    }
    const auto this_value = boost::any_cast< std::shared_ptr< nest::Parameter > >( first );
    const auto other_value = boost::any_cast< std::shared_ptr< nest::Parameter > >( second );
    if ( this_value != other_value )
    {
      return false;
    }
  }
  else
  {
    // TODO-PYNEST-NG: raise error
    assert( false );
  }
  return true;
}


bool
dictionary::operator==( const dictionary& other ) const
{
  if ( size() != other.size() )
  {
    return false;
  }
  // Iterate elements in the other dictionary
  for ( auto& kv_pair : other )
  {
    // Check if it exists in this dictionary
    if ( not known( kv_pair.first ) )
    {
      return false;
    }
    // Check for equality
    const auto value = maptype_::at( kv_pair.first );
    if ( not value_equal( value, kv_pair.second ) )
    {
      return false;
    }
  }
  // All elements are equal
  return true;
}


boost::any& dictionary::operator[]( const std::string& key )
{
  nest::kernel().get_dict_access_flag_manager().register_access( *this, key );
  return maptype_::operator[]( key );
}

boost::any& dictionary::operator[]( std::string&& key )
{
  nest::kernel().get_dict_access_flag_manager().register_access( *this, key );
  return maptype_::operator[]( key );
}

boost::any&
dictionary::at( const std::string& key )
{
  nest::kernel().get_dict_access_flag_manager().register_access( *this, key );
  return maptype_::at( key );
}

const boost::any&
dictionary::at( const std::string& key ) const
{
  nest::kernel().get_dict_access_flag_manager().register_access( *this, key );
  return maptype_::at( key );
}

dictionary::iterator
dictionary::find( const std::string& key )
{
  nest::kernel().get_dict_access_flag_manager().register_access( *this, key );
  return maptype_::find( key );
}

dictionary::const_iterator
dictionary::find( const std::string& key ) const
{
  nest::kernel().get_dict_access_flag_manager().register_access( *this, key );
  return maptype_::find( key );
}

void
dictionary::init_access_flags() const
{
  nest::kernel().get_dict_access_flag_manager().init_access_flags( *this );
}

void
dictionary::all_entries_accessed( const std::string where, const std::string what ) const
{
  nest::kernel().get_dict_access_flag_manager().all_accessed( *this, where, what );
}

// TODO-PYNEST-NG: Convenience function for accessed()?
