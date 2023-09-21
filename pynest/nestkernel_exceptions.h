/*
 *  nestkernel_exceptions.h
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

#include <Python.h>
#include <exception>
#include <ios>
#include <stdexcept>

#include "exceptions.h"
#include "nest_exception_names.h"

static struct PyModuleDef
  nest_module = { PyModuleDef_HEAD_INIT, "NESTErrors", nullptr, -1, nullptr, nullptr, nullptr, nullptr, nullptr };
PyObject* nest_error_module = PyModule_Create( &nest_module );

std::map< std::string, PyObject* > nest_exceptions_map;

void
create_exceptions()
{
  std::string kernel_exn_name = "KernelException";
  PyObject* base_class = nullptr; // will be nullptr for KernelException
  nest_exceptions.insert( nest_exceptions.begin(), kernel_exn_name );

  for ( auto name : nest_exceptions )
  {
    if ( name != kernel_exn_name )
    {
      base_class = nest_exceptions_map[ kernel_exn_name ];
    }
    PyObject* exception = PyErr_NewException( ( "NESTErrors." + name ).c_str(), base_class, nullptr );
    nest_exceptions_map[ name ] = exception;
    Py_INCREF( exception );
    PyModule_AddObject( nest_error_module, name.c_str(), exception );
  }
}

void
custom_exception_handler()
{
  try
  {
    if ( PyErr_Occurred() )
    {
      ; // let the latest Python exn pass through and ignore the current one
    }
    else
    {
      throw;
    }
  }
  catch ( nest::KernelException& exn )
  {
    PyErr_SetString( nest_exceptions_map[ exn.exception_name() ], exn.what() );
  }
  catch ( std::exception& exc )
  {
    PyErr_SetString( PyExc_RuntimeError, ( std::string( "C++ exception: " ) + exc.what() ).c_str() );
  }
  catch ( ... )
  {
    PyErr_SetString( PyExc_RuntimeError, "Unexpected C++ exception" );
  }
}
