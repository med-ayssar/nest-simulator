# -*- coding: utf-8 -*-
#
# hl_api_simulation.py
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

"""
Functions for simulation control
"""

import warnings
from contextlib import contextmanager

from .. import nestkernel_api as nestkernel
from .hl_api_helper import is_iterable

__all__ = [
    "Cleanup",
    "DisableStructuralPlasticity",
    "EnableStructuralPlasticity",
    "GetKernelStatus",
    "Install",
    "Prepare",
    "ResetKernel",
    "Run",
    "RunManager",
    "SetKernelStatus",
    "Simulate",
]


def Simulate(t):
    """Simulate the network for `t` milliseconds.

    `Simulate(t)` runs `Prepare()`, `Run(t)`, and `Cleanup()` in this order.

    Parameters
    ----------
    t : float
        Time to simulate in ms

    See Also
    --------
    RunManager, Prepare, Run, Cleanup

    """

    nestkernel.llapi_simulate(t)


def Run(t):
    """Simulate the network for `t` milliseconds.

    Parameters
    ----------
    t : float
        Time to simulate in ms

    Notes
    ------

    Call between `Prepare` and `Cleanup` calls, or within a
    ``with RunManager`` clause.  `Run(t)` is called once by each call to `Simulate(t)`.

    `Prepare` must be called before `Run` to calibrate the system, and
    `Cleanup` must be called after `Run` to close files, cleanup handles, and
    so on. After `Cleanup`, `Prepare` can and must be called before more `Run`
    calls.

    Be careful about modifying the network or neurons between `Prepare` and `Cleanup`
    calls. In particular, do not call `Create`, `Connect`, or `SetKernelStatus`.
    Changing the membrane potential `V_m` of neurons or synaptic weights (but not delays!)
    will in most cases work as expected, while changing membrane or synaptic times
    constants will not work correctly. If in doubt, assume that changes may cause
    undefined behavior and check these thoroughly.

    Also note that `local_spike_counter` is reset each time you call `Run`.

    See Also
    --------
    Prepare, Cleanup, RunManager, Simulate

    """
    nestkernel.llapi_run(t)


def Prepare():
    """Calibrate the system before a `Run` call.

    `Prepare` is automatically called by `Simulate` and `RunManager`.

    See Also
    --------
    Run, Cleanup, Simulate, RunManager

    """
    nestkernel.llapi_prepare()


def Cleanup():
    """Cleans up resources after a `Run` calls.

    `Cleanup` is automatically called by `Simulate` and `RunManager`.

    Closes state for a series of runs, such as flushing and closing files.
    A `Prepare` is needed after a `Cleanup` before any more calls to `Run`.

    See Also
    --------
    Run, Prepare, Simulate, RunManager

    """
    nestkernel.llapi_cleanup()


@contextmanager
def RunManager():
    """ContextManager for `Run`

    Calls `Prepare` before a series of `Run` calls, and calls `Cleanup` at end.

    For example:

    ::

        with RunManager():
            for _ in range(10):
                Run(100)
                # extract results

    Notes
    -----
    Be careful about modifying the network or neurons between `Prepare` and `Cleanup`
    calls. In particular, do not call `Create`, `Connect`, or `SetKernelStatus`.
    Changing the membrane potential `V_m` of neurons or synaptic weights (but not delays!)
    will in most cases work as expected, while changing membrane or synaptic times
    constants will not work correctly. If in doubt, assume that changes may cause
    undefined behavior and check these thoroughly.

    See Also
    --------
    Prepare, Run, Cleanup, Simulate

    """

    Prepare()
    try:
        yield
    finally:
        Cleanup()


def ResetKernel():
    """Reset the simulation kernel.

    This will destroy the network as well as all custom models created with
    :py:func:`.CopyModel`. Calling this function is equivalent to restarting NEST.

    In particular,

    * all network nodes
    * all connections
    * all user-defined neuron and synapse models

    are deleted, and

    * time
    * random generators

    are reset. All dynamically loaded modules (via :py:func:`.Install()`) are unloaded.

    """
    nestkernel.llapi_reset_kernel()


def SetKernelStatus(params):
    """Set parameters for the simulation kernel.

    See the documentation of :ref:`sec_kernel_attributes` for a valid
    list of params.

    Parameters
    ----------

    params : dict
        Dictionary of parameters to set.

    See Also
    --------

    GetKernelStatus

    """
    # We need the nest module to be fully initialized in order to access the
    # _kernel_attr_names and _readonly_kernel_attrs. As hl_api_simulation is
    # imported during nest module initialization, we can't put the import on
    # the module level, but have to have it on the function level.
    import nest  # noqa

    raise_errors = params.get("dict_miss_is_error", nest.dict_miss_is_error)
    valids = nest._kernel_attr_names
    readonly = nest._readonly_kernel_attrs
    keys = list(params.keys())
    for key in keys:
        msg = None
        if key not in valids:
            msg = (
                f"`{key}` is not a valid kernel parameter, "
                + "valid parameters are: "
                + ", ".join(f"'{p}'" for p in sorted(valids))
            )
        elif key in readonly:
            msg = f"`{key}` is a readonly kernel parameter"
        if msg is not None:
            if raise_errors:
                raise ValueError(msg)
            else:
                warnings.warn(msg + f" \n`{key}` has been ignored")
                del params[key]

    nestkernel.llapi_set_kernel_status(params)


def GetKernelStatus(keys=None):
    """Obtain parameters of the simulation kernel.

    Parameters
    ----------

    keys : str or list, optional
        Single parameter name or `list` of parameter names

    Returns
    -------

    dict:
        Parameter dictionary, if called without argument
    type:
        Single parameter value, if called with single parameter name
    list:
        List of parameter values, if called with list of parameter names

    Raises
    ------

    TypeError
        If `keys` are of the wrong type.

    Notes
    -----
    See SetKernelStatus for documentation on each parameter key.

    See Also
    --------
    SetKernelStatus

    """

    status_root = nestkernel.llapi_get_kernel_status()

    if keys is None:
        return status_root
    elif isinstance(keys, str):
        return status_root[keys]
    elif is_iterable(keys):
        return tuple(status_root[k] for k in keys)
    else:
        raise TypeError("keys should be either a string or an iterable")


def Install(module_name):
    """Load a dynamically linked NEST module.

    Parameters
    ----------
    module_name : str
        Name of the dynamically linked module

    Returns
    -------
    handle
        NEST module identifier, required for unloading

    Notes
    -----
    Dynamically linked modules are searched in the NEST library
    directory (``<prefix>/lib/nest``) and in ``LD_LIBRARY_PATH`` (on
    Linux) or ``DYLD_LIBRARY_PATH`` (on OSX).

    **Example**
    ::

        nest.Install("mymodule")

    """

    raise NotImplementedError


def EnableStructuralPlasticity():
    """Enable structural plasticity for the network simulation

    See Also
    --------
    DisableStructuralPlasticity

    """

    nestkernel.llapi_enable_structural_plasticity()


def DisableStructuralPlasticity():
    """Disable structural plasticity for the network simulation

    See Also
    --------
    EnableStructuralPlasticity

    """
    nestkernel.llapi_disable_structural_plasticity()
