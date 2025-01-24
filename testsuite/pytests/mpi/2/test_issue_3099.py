# -*- coding: utf-8 -*-
#
# test_issue_3099.py
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


import nest
import pytest


@pytest.fixture
def conns():
    nest.ResetKernel()
    nrn = nest.Create("parrot_neuron")
    nest.Connect(nrn, nrn)
    return nest.GetConnections()


def test_conn_weight(conns):
    """Test that operation does not cause MPI deadlock."""

    if conns:
        conns.weight = 2.5


def test_set_weight(conns):
    """Test that operation does not cause MPI deadlock."""

    if conns:
        conns.set({"weight": 2.5})


def test_set_status_weight(conns):
    """Test that operation does not cause MPI deadlock."""

    if conns:
        conns.weight = 2.5
