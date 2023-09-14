# -*- coding: utf-8 -*-
#
# test_sic_connection.py
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
Test functionality of the SIC connection
"""

import unittest
import nest
import numpy as np

HAVE_GSL = nest.ll_api.sli_func("statusdict/have_gsl ::")


@nest.ll_api.check_stack
@unittest.skipIf(not HAVE_GSL, "GSL is not available")
class SICConnectionTestCase(unittest.TestCase):
    """Test SIC connection"""

    def test_ConnectNeuronsWithSICConnection(self):
        """Ensures that the restriction to supported neuron models works."""

        nest.set_verbosity("M_WARNING")

        supported_sources = [
            "astrocyte_lr_1994",
        ]
        supported_targets = [
            "aeif_cond_alpha_astro",
        ]

        # Ensure that connecting not supported models fails
        for smodel in [x for x in nest.node_models]:
            for tmodel in [y for y in nest.node_models]:
                nest.ResetKernel()

                source = nest.Create(smodel)
                target = nest.Create(tmodel)

                if smodel in supported_sources and tmodel in supported_targets:
                    nest.Connect(source, target, syn_spec={"synapse_model": "sic_connection"})
                    continue

                # try to connect with sic_connection
                with self.assertRaises(nest.kernel.NESTError):
                    nest.Connect(source, target, syn_spec={"synapse_model": "sic_connection"})

    def test_SynapseFunctionWithAeifModel(self):
        """Ensure that SICEvent is properly processed"""

        nest.set_verbosity("M_WARNING")
        nest.ResetKernel()

        # Create neurons and devices
        astrocyte = nest.Create("astrocyte_lr_1994", {"Ca": 0.2})  # a calcium value which produces SIC
        neuron = nest.Create("aeif_cond_alpha_astro")

        test_resol = 1.0  # test resolution
        mm_neuron = nest.Create("multimeter", params={"record_from": ["I_SIC"], "interval": test_resol})
        mm_astro = nest.Create("multimeter", params={"record_from": ["Ca"], "interval": test_resol})

        nest.Connect(astrocyte, neuron, syn_spec={"synapse_model": "sic_connection"})
        nest.Connect(mm_neuron, neuron)
        nest.Connect(mm_astro, astrocyte)

        # Simulation
        nest.Simulate(100.0)

        # Evaluation
        # The expected SIC values are calculated based on the astrocyte dynamics
        # implemented in astrocyte_lr_1994.cpp.
        actual_sic_values = mm_neuron.events["I_SIC"]
        Ca = mm_astro.events["Ca"]
        f_v = np.vectorize(lambda x: np.log(x * 1000.0 - 196.69) if x * 1000.0 - 196.69 > 1.0 else 0.0)
        expected_sic_values = f_v(Ca)

        # The sic_connection has a default delay (1 ms), thus the values after
        # the number of steps of delay are compared with the expected values.
        sic_delay = nest.GetDefaults("sic_connection")["delay"]
        n_step_delay = int(sic_delay / test_resol)
        self.assertTrue(
            np.allclose(
                actual_sic_values[n_step_delay:],
                expected_sic_values[:-n_step_delay],
                rtol=1e-5,
            )
        )


def suite():
    suite = unittest.makeSuite(SICConnectionTestCase, "test")
    return suite


def run():
    runner = unittest.TextTestRunner(verbosity=2)
    runner.run(suite())


if __name__ == "__main__":
    run()
