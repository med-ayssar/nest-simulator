# -*- coding: utf-8 -*-
#
# test_stdp_nn_synapses.py
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

# This script tests the stdp_nn_symm_synapse, stdp_nn_pre_centered_synapse,
# and stdp_nn_restr_synapse in NEST.

from math import exp

import nest
import numpy as np
import pytest


@nest.ll_api.check_stack
class TestSTDPNNSynapses:
    """
    Test the weight change by STDP
    with three nearest-neighbour spike pairing schemes.
    The test is performed by generating two Poisson spike trains,
    feeding them to NEST as presynaptic and postsynaptic,
    then reconstructing the expected weight change outside of NEST
    and comparing the actual weight change to the expected one.

    The outside-of-NEST STDP implementation made here is not very beautiful,
    but deliberately made so in order to be as independent from NEST as
    possible: it does not involve additional variables for eligibility traces.
    Instead, it directly iterates through the spike history.
    """

    @pytest.fixture(autouse=True)
    def setUp(self):
        self.resolution = 0.1  # [ms]
        self.presynaptic_firing_rate = 20.0  # [spks/s]
        self.postsynaptic_firing_rate = 20.0  # [spks/s]
        self.simulation_duration = 1e4  # [ms]
        self.synapse_parameters = {
            "receptor_type": 1,
            "delay": self.resolution,
            # STDP constants
            "lambda": 0.01,
            "alpha": 0.85,
            "mu_plus": 0.0,
            "mu_minus": 0.0,
            "tau_plus": 16.8,
            "Wmax": 1.0,
            # initial weight
            "weight": 0.5,
        }
        self.neuron_parameters = {"tau_minus": 33.7}

        # While the random sequences, fairly long, would supposedly
        # reveal small differences in the weight change between NEST
        # and ours, some low-probability events (say, coinciding
        # spikes) can well not have occurred. To generate and
        # test every possible combination of pre/post order, we
        # append some hardcoded spike sequences:
        # pre: 1       5 6 7   9    11 12 13
        # post:  2 3 4       8 9 10    12
        self.hardcoded_pre_times = np.array([1, 5, 6, 7, 9, 11, 12, 13], dtype=float)
        self.hardcoded_post_times = np.array([2, 3, 4, 8, 9, 10, 12], dtype=float)
        self.hardcoded_trains_length = 2.0 + max(np.amax(self.hardcoded_pre_times), np.amax(self.hardcoded_post_times))

    def do_nest_simulation_and_compare_to_reproduced_weight(self, pairing_scheme):
        synapse_model = "stdp_" + pairing_scheme + "_synapse"
        self.synapse_parameters["synapse_model"] = synapse_model

        pre_spikes, post_spikes, weight_by_nest = self.do_the_nest_simulation()
        weight_reproduced_independently = self.reproduce_weight_drift(
            pre_spikes, post_spikes, self.synapse_parameters["weight"]
        )
        np.testing.assert_almost_equal(
            weight_reproduced_independently,
            weight_by_nest,
            err_msg=synapse_model + " test: "
            "Resulting synaptic weight %e "
            "differs from expected %e" % (weight_by_nest, weight_reproduced_independently),
        )

    def do_the_nest_simulation(self):
        """
        This function is where calls to NEST reside.
        Returns the generated pre- and post spike sequences
        and the resulting weight established by STDP.
        """
        nest.set_verbosity("M_WARNING")
        nest.ResetKernel()
        nest.resolution = self.resolution

        presynaptic_neuron, postsynaptic_neuron = nest.Create("parrot_neuron", 2, params=self.neuron_parameters)

        generators = nest.Create(
            "poisson_generator",
            2,
            params=(
                {
                    "rate": self.presynaptic_firing_rate,
                    "stop": (self.simulation_duration - self.hardcoded_trains_length),
                },
                {
                    "rate": self.postsynaptic_firing_rate,
                    "stop": (self.simulation_duration - self.hardcoded_trains_length),
                },
            ),
        )
        presynaptic_generator = generators[0]
        postsynaptic_generator = generators[1]

        spike_senders = nest.Create(
            "spike_generator",
            2,
            params=(
                {"spike_times": self.hardcoded_pre_times + self.simulation_duration - self.hardcoded_trains_length},
                {"spike_times": self.hardcoded_post_times + self.simulation_duration - self.hardcoded_trains_length},
            ),
        )
        pre_spike_generator = spike_senders[0]
        post_spike_generator = spike_senders[1]

        # The recorder is to save the randomly generated spike trains.
        spike_recorder = nest.Create("spike_recorder")

        nest.Connect(
            presynaptic_generator + pre_spike_generator,
            presynaptic_neuron,
            syn_spec={"synapse_model": "static_synapse"},
        )
        nest.Connect(
            postsynaptic_generator + post_spike_generator,
            postsynaptic_neuron,
            syn_spec={"synapse_model": "static_synapse"},
        )
        nest.Connect(
            presynaptic_neuron + postsynaptic_neuron, spike_recorder, syn_spec={"synapse_model": "static_synapse"}
        )
        # The synapse of interest itself
        nest.Connect(presynaptic_neuron, postsynaptic_neuron, syn_spec=self.synapse_parameters)
        plastic_synapse_of_interest = nest.GetConnections(synapse_model=self.synapse_parameters["synapse_model"])

        nest.Simulate(self.simulation_duration)

        all_spikes = nest.GetStatus(spike_recorder, keys="events")[0]
        pre_spikes = all_spikes["times"][all_spikes["senders"] == presynaptic_neuron.tolist()[0]]
        post_spikes = all_spikes["times"][all_spikes["senders"] == postsynaptic_neuron.tolist()[0]]
        weight = nest.GetStatus(plastic_synapse_of_interest, keys="weight")[0]
        return (pre_spikes, post_spikes, weight)

    def reproduce_weight_drift(self, _pre_spikes, _post_spikes, _initial_weight):
        """
        Returns the total weight change of the synapse
        computed outside of NEST.
        The implementation imitates a step-based simulation: evolving time, we
        trigger a weight update when the time equals one of the spike moments.
        """

        # These are defined just for convenience,
        # STDP is evaluated on exact times nonetheless
        pre_spikes_forced_to_grid = [int(t / self.resolution) for t in _pre_spikes]
        post_spikes_forced_to_grid = [int(t / self.resolution) for t in _post_spikes]

        t_previous_pre = -1
        t_previous_post = -1
        w = _initial_weight
        n_steps = int(self.simulation_duration / self.resolution)
        for time_in_simulation_steps in range(n_steps):
            if time_in_simulation_steps in pre_spikes_forced_to_grid:
                # A presynaptic spike occurred now.

                # Adjusting the current time to make it exact.
                t = _pre_spikes[pre_spikes_forced_to_grid.index(time_in_simulation_steps)]

                # Evaluating the depression rule.
                # For the first two rules below, simply pair current pre-spike with nearest post-spike.
                # For "nn_restr" and `post < pre`, the previous post has already been paired, thus due
                #   to the restricted pairing scheme, we do not account it.
                current_nearest_neighbour_pair_is_suitable = self.synapse_parameters["synapse_model"] in [
                    "stdp_nn_symm_synapse",
                    "stdp_nn_pre_centered_synapse",
                ] or (
                    self.synapse_parameters["synapse_model"] == "stdp_nn_restr_synapse"
                    and t_previous_post > t_previous_pre
                )

                if current_nearest_neighbour_pair_is_suitable and t_previous_post != -1:
                    # Otherwise, if == -1, there have been
                    # no post-spikes yet, and nothing to pair with.
                    w = self.depress(t_previous_post - t, w)

                # Memorizing the current pre-spike to account it
                # further with the next post-spike.
                t_previous_pre = t

            if time_in_simulation_steps in post_spikes_forced_to_grid:
                # A postsynaptic spike occurred now.

                # Adjusting the current time to make it exact.
                t = _post_spikes[post_spikes_forced_to_grid.index(time_in_simulation_steps)]

                # A post-spike is actually accounted in STDP only after
                # it backpropagates through the dendrite.
                t += self.synapse_parameters["delay"]

                # Evaluating the facilitation rule.
                if self.synapse_parameters["synapse_model"] == "stdp_nn_symm_synapse":
                    if t_previous_pre != -1:  # otherwise nothing to pair with
                        w = self.facilitate(t - t_previous_pre, w)

                if self.synapse_parameters["synapse_model"] == "stdp_nn_restr_synapse":
                    if t_previous_pre != -1:  # if == -1, nothing to pair with
                        if t_previous_pre > t_previous_post:
                            # if '<', t_previous_pre has already been
                            # paired with t_previous_post, thus due to the
                            # restricted pairing scheme we do not account it.
                            w = self.facilitate(t - t_previous_pre, w)

                if self.synapse_parameters["synapse_model"] == "stdp_nn_pre_centered_synapse":
                    if t_previous_pre != -1:  # if == -1, nothing to pair with
                        # Going through all preceding presynaptic spikes
                        # that are later than the previous post.
                        i = list(_pre_spikes).index(t_previous_pre)
                        while i >= 0:
                            if _pre_spikes[i] <= t_previous_post:
                                # _pre_spikes[i] has already been
                                # paired with t_previous_post,
                                # thus we do not account it.
                                break
                            w = self.facilitate(t - _pre_spikes[i], w)
                            i -= 1

                # Memorizing the current post-spike to account it further
                # with the next pre-spike.
                t_previous_post = t

        return w

    def facilitate(self, _delta_t, w):
        if _delta_t == 0:
            return w
        w += (
            self.synapse_parameters["lambda"]
            * ((1 - w / self.synapse_parameters["Wmax"]) ** self.synapse_parameters["mu_plus"])
            * exp(-1 * _delta_t / self.synapse_parameters["tau_plus"])
        )
        w = min(w, self.synapse_parameters["Wmax"])
        return w

    def depress(self, _delta_t, w):
        if _delta_t == 0:
            return w
        w -= (
            self.synapse_parameters["lambda"]
            * self.synapse_parameters["alpha"]
            * ((w / self.synapse_parameters["Wmax"]) ** self.synapse_parameters["mu_minus"])
            * exp(_delta_t / self.neuron_parameters["tau_minus"])
        )
        w = max(0, w)
        return w

    def test_nn_symm_synapse(self):
        self.do_nest_simulation_and_compare_to_reproduced_weight("nn_symm")

    def test_nn_pre_centered_synapse(self):
        self.do_nest_simulation_and_compare_to_reproduced_weight("nn_pre_centered")

    def test_nn_restr_synapse(self):
        self.do_nest_simulation_and_compare_to_reproduced_weight("nn_restr")
