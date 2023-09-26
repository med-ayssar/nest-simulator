# -*- coding: utf-8 -*-
#
# test_hh_cond_beta_gap_traub.py
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
test_hh_cond_beta_gap_traub.sli is an overall test of the hh_cond_beta_gap_traub model connected
by gap_junction connection.

Two neurons of whom one receives an constant input current of 200 pA are connected
by gap_junction with an (unrealistic) high gap weight. The accurate functionality
of the gap_junction feature is tested by measuring the membrane potential of the
neuron without input current.

Although 0.1 cannot be represented in the IEEE double data type, it
is safe to simulate with a resolution (computation step size) of 0.1
ms because by default nest is built with a timebase enabling exact
representation of 0.1 ms.
"""

import nest
import pytest


@pytest.fixture(autouse=True)
def prepare():
    nest.ResetKernel()
    nest.set(
        local_num_threads=1,
        resolution=0.1,
        use_wfr=True,
        wfr_tol=0.0001,
        wfr_interpolation_order=3,
        wfr_max_iterations=10,
        wfr_comm_interval=1.0,
    )


@pytest.fixture()
def prepare_voltmeter():
    n1, n2 = nest.Create("hh_cond_beta_gap_traub", 2)
    n1.set(I_e=200.0)

    vm = nest.Create("voltmeter")
    vm.set(time_in_steps=True, interval=nest.resolution)

    conn_rule = {"rule": "one_to_one", "make_symmetric": True}
    syn_dict = {"synapse_model": "gap_junction", "weight": 20.0}

    nest.Connect(n1, n2, conn_rule, syn_dict)
    nest.Connect(vm, n2, syn_spec={"weight": 1.0, "delay": nest.resolution})

    nest.Simulate(20)

    return vm


@pytest.fixture()
def reference_data():
    return [
        [1, -60],
        [2, -5.999800e01],
        [3, -5.999600e01],
        [4, -5.999200e01],
        [5, -5.998800e01],
        [6, -5.998300e01],
        [7, -5.997700e01],
        [8, -5.997000e01],
        [9, -5.996300e01],
        [10, -5.995500e01],
        [11, -5.994600e01],
        [12, -5.993600e01],
        [13, -5.992600e01],
        [14, -5.991400e01],
        [15, -5.990300e01],
        [117, -5.746100e01],
        [118, -5.743600e01],
        [119, -5.741100e01],
        [120, -5.738600e01],
        [121, -5.736100e01],
        [122, -5.733600e01],
        [123, -5.731100e01],
        [124, -5.728600e01],
        [125, -5.726100e01],
        [126, -5.723700e01],
        [127, -5.721200e01],
        [128, -5.718800e01],
        [129, -5.716300e01],
        [130, -5.713900e01],
        [131, -5.711500e01],
        [190, -5.583700e01],
        [191, -5.581800e01],
        [192, -5.579900e01],
        [193, -5.578000e01],
        [194, -5.576100e01],
        [195, -5.574200e01],
        [196, -5.572400e01],
        [197, -5.570500e01],
        [198, -5.568600e01],
        [199, -5.566800e01],
    ]


@pytest.mark.skipif_missing_gsl()
def test_hh_cond_beta_gap_traub(prepare_voltmeter, reference_data):
    vm = prepare_voltmeter
    reference_data = dict(reference_data)

    events = vm.events
    recorded_times = events["times"]
    recorded_vm = events["V_m"]

    actual_data = dict(zip(recorded_times, recorded_vm))
    actual_data = {k: actual_data[k] for k in reference_data.keys()}

    actual_data = list(actual_data.values())
    reference_data = list(reference_data.values())

    assert pytest.approx(actual_data, rel=1e-5) == reference_data
