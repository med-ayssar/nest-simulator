# -*- coding: utf-8 -*-
#
# test_free_mask_circ_anchor_00.py
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


""""""
import nest
import numpy as np
import pytest


@pytest.fixture
def network():
    nest.ResetKernel()

    nest.set(use_compressed_spikes=False, sort_connections_by_source=False)
    positions = nest.spatial.free([[x, y] for x in np.linspace(-0.5, 0.5, 5) for y in np.linspace(0.5, -0.5, 5)],
                                  edge_wrap=False, extent=[1.25, 1.25])
    population_type = 'iaf_psc_alpha'

    conns = {'rule': 'pairwise_bernoulli',
             'mask': {'circular': {'radius': 0.25}, 'anchor': [0., 0.]}}

    src_layer = nest.Create(population_type, positions=positions)
    target_layer = nest.Create(population_type, positions=positions)

    nest.Connect(src_layer, target_layer, conns)
    return src_layer, target_layer


@pytest.fixture
def src_layer_ref():
    return np.array(
        [[1., -0.5, 0.5], [2., -0.5, 0.25], [3., -0.5, 0.], [4., -0.5, -0.25], [5., -0.5, -0.5], [6., -0.25, 0.5],
         [7., -0.25, 0.25], [8., -0.25, 0.], [9., -0.25, -0.25], [10., -0.25, -0.5], [11., 0., 0.5], [12., 0., 0.25],
         [13., 0., 0.], [14., 0., -0.25], [15., 0., -0.5], [16., 0.25, 0.5], [17., 0.25, 0.25], [18., 0.25, 0.],
         [19., 0.25, -0.25], [20., 0.25, -0.5], [21., 0.5, 0.5], [22., 0.5, 0.25], [23., 0.5, 0.], [24., 0.5, -0.25],
         [25., 0.5, -0.5]])


@pytest.fixture
def target_layer_ref():
    return np.array(
        [[26.0, -0.5, 0.5], [27, -0.5, 0.25], [28, -0.5, 0], [29, -0.5, -0.25], [30, -0.5, -0.5], [31, -0.25, 0.5],
         [32, -0.25, 0.25], [33, -0.25, 0], [34, -0.25, -0.25], [35, -0.25, -0.5], [36, 0, 0.5], [37, 0, 0.25],
         [38, 0, 0], [39, 0, -0.25], [40, 0, -0.5], [41, 0.25, 0.5], [42, 0.25, 0.25], [43, 0.25, 0], [44, 0.25, -0.25],
         [45, 0.25, -0.5], [46, 0.5, 0.5], [47, 0.5, 0.25], [48, 0.5, 0], [49, 0.5, -0.25], [50, 0.5, -0.5]])


@pytest.fixture
def connections_ref():
    return np.array([[1, 26, 1, 1, 0, 0], [1, 27, 1, 1, 0, -0.25], [1, 31, 1, 1, 0.25, 0], [2, 26, 1, 1, 0, 0.25],
                     [2, 27, 1, 1, 0, 0], [2, 28, 1, 1, 0, -0.25], [2, 32, 1, 1, 0.25, 0], [3, 27, 1, 1, 0, 0.25],
                     [3, 28, 1, 1, 0, 0], [3, 29, 1, 1, 0, -0.25], [3, 33, 1, 1, 0.25, 0], [4, 28, 1, 1, 0, 0.25],
                     [4, 29, 1, 1, 0, 0], [4, 30, 1, 1, 0, -0.25], [4, 34, 1, 1, 0.25, 0], [5, 29, 1, 1, 0, 0.25],
                     [5, 30, 1, 1, 0, 0], [5, 35, 1, 1, 0.25, 0], [6, 26, 1, 1, -0.25, 0], [6, 31, 1, 1, 0, 0],
                     [6, 32, 1, 1, 0, -0.25], [6, 36, 1, 1, 0.25, 0], [7, 27, 1, 1, -0.25, 0], [7, 31, 1, 1, 0, 0.25],
                     [7, 32, 1, 1, 0, 0], [7, 33, 1, 1, 0, -0.25], [7, 37, 1, 1, 0.25, 0], [8, 28, 1, 1, -0.25, 0],
                     [8, 32, 1, 1, 0, 0.25], [8, 33, 1, 1, 0, 0], [8, 34, 1, 1, 0, -0.25], [8, 38, 1, 1, 0.25, 0],
                     [9, 29, 1, 1, -0.25, 0], [9, 33, 1, 1, 0, 0.25], [9, 34, 1, 1, 0, 0], [9, 35, 1, 1, 0, -0.25],
                     [9, 39, 1, 1, 0.25, 0], [10, 30, 1, 1, -0.25, 0], [10, 34, 1, 1, 0, 0.25], [10, 35, 1, 1, 0, 0],
                     [10, 40, 1, 1, 0.25, 0], [11, 31, 1, 1, -0.25, 0], [11, 36, 1, 1, 0, 0], [11, 37, 1, 1, 0, -0.25],
                     [11, 41, 1, 1, 0.25, 0], [12, 32, 1, 1, -0.25, 0], [12, 36, 1, 1, 0, 0.25], [12, 37, 1, 1, 0, 0],
                     [12, 38, 1, 1, 0, -0.25], [12, 42, 1, 1, 0.25, 0], [13, 33, 1, 1, -0.25, 0],
                     [13, 37, 1, 1, 0, 0.25], [13, 38, 1, 1, 0, 0], [13, 39, 1, 1, 0, -0.25], [13, 43, 1, 1, 0.25, 0],
                     [14, 34, 1, 1, -0.25, 0], [14, 38, 1, 1, 0, 0.25], [14, 39, 1, 1, 0, 0], [14, 40, 1, 1, 0, -0.25],
                     [14, 44, 1, 1, 0.25, 0], [15, 35, 1, 1, -0.25, 0], [15, 39, 1, 1, 0, 0.25], [15, 40, 1, 1, 0, 0],
                     [15, 45, 1, 1, 0.25, 0], [16, 36, 1, 1, -0.25, 0], [16, 41, 1, 1, 0, 0], [16, 42, 1, 1, 0, -0.25],
                     [16, 46, 1, 1, 0.25, 0], [17, 37, 1, 1, -0.25, 0], [17, 41, 1, 1, 0, 0.25], [17, 42, 1, 1, 0, 0],
                     [17, 43, 1, 1, 0, -0.25], [17, 47, 1, 1, 0.25, 0], [18, 38, 1, 1, -0.25, 0],
                     [18, 42, 1, 1, 0, 0.25], [18, 43, 1, 1, 0, 0], [18, 44, 1, 1, 0, -0.25], [18, 48, 1, 1, 0.25, 0],
                     [19, 39, 1, 1, -0.25, 0], [19, 43, 1, 1, 0, 0.25], [19, 44, 1, 1, 0, 0], [19, 45, 1, 1, 0, -0.25],
                     [19, 49, 1, 1, 0.25, 0], [20, 40, 1, 1, -0.25, 0], [20, 44, 1, 1, 0, 0.25], [20, 45, 1, 1, 0, 0],
                     [20, 50, 1, 1, 0.25, 0], [21, 41, 1, 1, -0.25, 0], [21, 46, 1, 1, 0, 0], [21, 47, 1, 1, 0, -0.25],
                     [22, 42, 1, 1, -0.25, 0], [22, 46, 1, 1, 0, 0.25], [22, 47, 1, 1, 0, 0], [22, 48, 1, 1, 0, -0.25],
                     [23, 43, 1, 1, -0.25, 0], [23, 47, 1, 1, 0, 0.25], [23, 48, 1, 1, 0, 0], [23, 49, 1, 1, 0, -0.25],
                     [24, 44, 1, 1, -0.25, 0], [24, 48, 1, 1, 0, 0.25], [24, 49, 1, 1, 0, 0], [24, 50, 1, 1, 0, -0.25],
                     [25, 45, 1, 1, -0.25, 0], [25, 49, 1, 1, 0, 0.25], [25, 50, 1, 1, 0, 0]])


def test_source_layer_matches(tmp_path, network, src_layer_ref):
    src_layer, _ = network

    src_path = str(tmp_path) + "/src_layer.txt"

    nest.DumpLayerNodes(src_layer, src_path)
    stored_src = np.genfromtxt(src_path)

    assert np.all(stored_src == src_layer_ref)


def test_target_layer_matches(tmp_path, network, target_layer_ref):
    _, target_layer = network

    target_path = str(tmp_path) + "/target_layer.txt"

    nest.DumpLayerNodes(target_layer, target_path)
    stored_target = np.genfromtxt(target_path)

    assert np.all(stored_target == target_layer_ref)


def test_connections_match(tmp_path, network, connections_ref):
    src_layer, target_layer = network

    connections_path = str(tmp_path) + "/connections.txt"

    nest.DumpLayerConnections(src_layer, target_layer, 'static_synapse', connections_path)
    stored_connections = np.genfromtxt(connections_path)

    assert np.all(stored_connections == connections_ref)
