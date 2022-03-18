# -*- coding: utf-8 -*-
#
# ll_api.pyx
#
# This file is part of NEST.
#
# Copyright (C) 2004 The NEST Initiative
#
# NEST is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# NEST is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with NEST.  If not, see <http://www.gnu.org/licenses/>.
#
# distutils: language = c++
#

import cython

# from libc.stdlib cimport malloc, free
# from libc.string cimport memcpy

from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.memory cimport shared_ptr

from cython.operator cimport dereference as deref
from cython.operator cimport preincrement as inc

# from cpython cimport array

# from cpython.ref cimport PyObject
# from cpython.object cimport Py_LT, Py_LE, Py_EQ, Py_NE, Py_GT, Py_GE

import nest
from nest.lib.hl_api_exceptions import NESTMappedException, NESTErrors, NESTError
from libcpp.memory cimport shared_ptr

import numpy


cdef class NodeCollectionObject:

    cdef NodeCollectionPTR thisptr

    def __repr__(self):
        return "<NodeCollectionObject>"

    cdef _set_nc(self, NodeCollectionPTR nc):
        self.thisptr = nc

cdef class ParameterObject:

    cdef shared_ptr[Parameter] thisptr

    def __repr__(self):
        return "<ParameterObject>"

    cdef _set_parameter(self, shared_ptr[Parameter] parameter_ptr):
        self.thisptr = parameter_ptr


cdef object any_vector_to_list(vector[any] cvec):
    cdef tmp = []
    cdef vector[any].iterator it = cvec.begin()
    while it != cvec.end():
        tmp.append(any_to_pyobj(deref(it)))
        inc(it)
    return tmp


cdef object any_to_pyobj(any operand):
    if is_int(operand):
        return any_cast[int](operand)
    if is_long(operand):
        return any_cast[long](operand)
    if is_size_t(operand):
        return any_cast[size_t](operand)
    if is_double(operand):
        return any_cast[double](operand)
    if is_bool(operand):
        return any_cast[cbool](operand)
    if is_string(operand):
        return any_cast[string](operand)
    if is_int_vector(operand):
        return any_cast[vector[int]](operand)
    if is_double_vector(operand):
        return any_cast[vector[double]](operand)
    if is_double_vector_vector(operand):
        return any_cast[vector[vector[double]]](operand)
    if is_string_vector(operand):
        return any_cast[vector[string]](operand)
    if is_any_vector(operand):
        return tuple(any_vector_to_list(any_cast[vector[any]](operand)))
    if is_dict(operand):
        return dictionary_to_pydict(any_cast[dictionary](operand))

cdef object dictionary_to_pydict(dictionary cdict):
    cdef tmp = {}

    cdef dictionary.const_iterator it = cdict.begin()
    while it != cdict.end():
        tmp[deref(it).first.decode('utf8')] = any_to_pyobj(deref(it).second)
        if tmp[deref(it).first.decode('utf8')] is None:
            raise RuntimeError('Could not convert: ' + deref(it).first.decode('utf8') + ' of type ' + debug_type(deref(it).second).decode('utf8'))
        inc(it)
    return tmp

cdef dictionary pydict_to_dictionary(object py_dict) except *:
    cdef dictionary cdict = dictionary()
    for key, value in py_dict.items():
        if type(value) is int:
            cdict[key.encode('utf-8')] = <long>value
        elif type(value) is float:
            cdict[key.encode('utf-8')] = <double>value
        elif type(value) is bool:
            cdict[key.encode('utf-8')] = <cbool>value
        elif type(value) is str:
            cdict[key.encode('utf-8')] = <string>value.encode('utf-8')
        elif type(value) is dict:
            cdict[key.encode('utf-8')] = pydict_to_dictionary(value)
        elif type(value) is ParameterObject:
            cdict[key.encode('utf-8')] = (<ParameterObject>value).thisptr
        else:
            raise AttributeError(f'value of key ({key}) is not a known type, got {type(value)}')
    return cdict

# cdef object vec_of_dict_to_list(vector[dictionary] cvec):
#     cdef tmp = []
#     cdef vector[dictionary].iterator it = cvec.begin()
#     while it != cvec.end():
#         tmp.append(dictionary_to_pydict(deref(it)))
#         inc(it)
#     return tmp

def llapi_reset_kernel():
    reset_kernel()

def llapi_create(string model, long n):
    cdef NodeCollectionPTR gids
    try:
        gids = create(model, n)
    except RuntimeError as e:
        exceptionCls = getattr(NESTErrors, str(e))
        raise exceptionCls('llapi_create', '') from None
    obj = NodeCollectionObject()
    obj._set_nc(gids)
    return nest.NodeCollection(obj)

def llapi_create_spatial(object layer_params):
    cdef NodeCollectionPTR gids
    try:
        gids = create_spatial(pydict_to_dictionary(layer_params))
    except RuntimeError as e:
        exceptionCls = getattr(NESTErrors, str(e))
        raise exceptionCls('llapi_create_spatial', '') from None
    obj = NodeCollectionObject()
    obj._set_nc(gids)
    return nest.NodeCollection(obj)

def llapi_make_nodecollection(object node_ids):
    cdef NodeCollectionPTR gids
    try:
        # node_ids list is automatically converted to an std::vector
        gids = make_nodecollection(node_ids)
    except RuntimeError as e:
        exceptionCls = getattr(NESTErrors, str(e))
        raise exceptionCls('llapi_make_nodecollection', '') from None
    obj = NodeCollectionObject()
    obj._set_nc(gids)
    return nest.NodeCollection(obj)

def llapi_connect(NodeCollectionObject pre, NodeCollectionObject post, object conn_params, object synapse_params):
    cdef vector[dictionary] syn_param_vec
    if synapse_params is not None:
        syn_param_vec.push_back(pydict_to_dictionary(synapse_params))

    connect(pre.thisptr, post.thisptr,
            pydict_to_dictionary(conn_params),
            syn_param_vec)

def llapi_slice(NodeCollectionObject nc, long start, long stop, long step):
    cdef NodeCollectionPTR nc_ptr
    try:
        nc_ptr = slice_nc(nc.thisptr, start, stop, step)
    except RuntimeError as e:
        exceptionCls = getattr(NESTErrors, str(e))
        raise exceptionCls('llapi_slice', '') from None
    obj = NodeCollectionObject()
    obj._set_nc(nc_ptr)
    return nest.NodeCollection(obj)

def llapi_nc_size(NodeCollectionObject nc):
    return nc_size(nc.thisptr)

def llapi_to_string(NodeCollectionObject nc):
    return pprint_to_string(nc.thisptr)

def llapi_get_kernel_status():
    cdef dictionary cdict = get_kernel_status()
    return dictionary_to_pydict(cdict)

def llapi_set_kernel_status(object params):
    cdef dictionary params_dict = pydict_to_dictionary(params)
    try:
        set_kernel_status(params_dict)
    except RuntimeError as e:
        exceptionCls = getattr(NESTErrors, str(e))
        raise exceptionCls('llapi_set_kernel_status', '') from None

def llapi_simulate(float t):
    simulate(t)

def llapi_prepare():
    prepare()

def llapi_run(float t):
    run(t)

def llapi_cleanup():
    cleanup()

def llapi_get_nc_status(NodeCollectionObject nc, object key=None):
    cdef dictionary statuses = get_nc_status(nc.thisptr)
    if key is None:
        return dictionary_to_pydict(statuses)
    elif isinstance(key, str):
        value = any_to_pyobj(statuses[key.encode('utf-8')])
        return value[0] if len(value) == 1 else value
    else:
        raise TypeError(f'key must be a string, got {type(key)}')

def llapi_set_nc_status(NodeCollectionObject nc, object params):
    cdef dictionary params_dict = pydict_to_dictionary(params)
    try:
        set_nc_status(nc.thisptr, params_dict)
    except RuntimeError as e:
        exceptionCls = getattr(NESTErrors, str(e))
        raise exceptionCls('llapi_set_nc_status', '') from None

def llapi_join_nc(NodeCollectionObject lhs, NodeCollectionObject rhs):
    cdef NodeCollectionPTR result
    try:
        # Using operator+() directly
        result = lhs.thisptr + rhs.thisptr
    except RuntimeError as e:
        exceptionCls = getattr(NESTErrors, str(e))
        raise exceptionCls('llapi_join_nc', '') from None
    obj = NodeCollectionObject()
    obj._set_nc(result)
    return nest.NodeCollection(obj)


def llapi_eq_nc(NodeCollectionObject lhs, NodeCollectionObject rhs):
    try:
        return equal(lhs.thisptr, rhs.thisptr)
    except RuntimeError as e:
        exceptionCls = getattr(NESTErrors, str(e))
        raise exceptionCls('llapi_eq_nc', '') from None


def llapi_nc_contains(NodeCollectionObject nc, long node_id):
    try:
        return contains(nc.thisptr, node_id)
    except RuntimeError as e:
        exceptionCls = getattr(NESTErrors, str(e))
        raise exceptionCls('llapi_nc_contains', '') from None

def llapi_nc_find(NodeCollectionObject nc, long node_id):
    try:
        return find(nc.thisptr, node_id)
    except RuntimeError as e:
        exceptionCls = getattr(NESTErrors, str(e))
        raise exceptionCls('llapi_nc_find', '') from None

# TODO-PYNEST-NG: decorator for exception handling?
def llapi_get_nc_metadata(NodeCollectionObject nc):
    try:
        return dictionary_to_pydict(get_metadata(nc.thisptr))
    except RuntimeError as e:
        exceptionCls = getattr(NESTErrors, str(e))
        raise exceptionCls('llapi_get_nc_metadata', '') from None


def llapi_take_array_index(NodeCollectionObject node_collection, object array):
    if not isinstance(array, numpy.ndarray):
        raise TypeError('array must be a 1-dimensional NumPy array of ints or bools, got {}'.format(type(array)))
    if not array.ndim == 1:
        raise TypeError('array must be a 1-dimensional NumPy array, got {}-dimensional NumPy array'.format(array.ndim))

    # Get pointers to the first element in the Numpy array
    cdef long[:] array_long_mv
    cdef long* array_long_ptr

    cdef cbool[:] array_bool_mv
    cdef cbool* array_bool_ptr

    cdef NodeCollectionPTR new_nc_ptr

    try:
        if array.dtype == numpy.bool:
            # Boolean C-type arrays are not supported in NumPy, so we use an 8-bit integer array
            array_bool_mv = numpy.ascontiguousarray(array, dtype=numpy.uint8)
            array_bool_ptr = &array_bool_mv[0]
            new_nc_ptr = node_collection_array_index(node_collection.thisptr, array_bool_ptr, len(array))
        elif numpy.issubdtype(array.dtype, numpy.integer):
            array_long_mv = numpy.ascontiguousarray(array, dtype=numpy.long)
            array_long_ptr = &array_long_mv[0]
            new_nc_ptr = node_collection_array_index(node_collection.thisptr, array_long_ptr, len(array))
        else:
            raise TypeError('array must be a NumPy array of ints or bools, got {}'.format(array.dtype))
        obj = NodeCollectionObject()
        obj._set_nc(new_nc_ptr)
        return nest.NodeCollection(obj)
    except RuntimeError as e:
        exceptionCls = getattr(NESTErrors, str(e))
        raise exceptionCls('take_array_index', '') from None

def llapi_create_parameter(object specs):
    cdef dictionary specs_dictionary = pydict_to_dictionary(specs)
    cdef shared_ptr[Parameter] parameter
    try:
        parameter = create_parameter(specs_dictionary)
    except RuntimeError as e:
        exceptionCls = getattr(NESTErrors, str(e))
        raise exceptionCls('llapi_create_parameter', '') from None
    obj = ParameterObject()
    obj._set_parameter(parameter)
    return nest.Parameter(obj)

def llapi_dimension_parameter(object list_of_pos_params):
    cdef shared_ptr[Parameter] dim_parameter
    cdef ParameterObject x, y, z
    if len(list_of_pos_params) == 2:
        x, y = list_of_pos_params
        dim_parameter = dimension_parameter(x.thisptr, y.thisptr)
    if len(list_of_pos_params) == 3:
        x, y, z = list_of_pos_params
        dim_parameter = dimension_parameter(x.thisptr, y.thisptr, z.thisptr)
    obj = ParameterObject()
    obj._set_parameter(dim_parameter)
    return nest.Parameter(obj)
