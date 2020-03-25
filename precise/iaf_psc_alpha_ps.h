/*
 *  iaf_psc_alpha_ps.h
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

#ifndef IAF_PSC_ALPHA_PS_H
#define IAF_PSC_ALPHA_PS_H

// C++ includes:
#include <vector>

// Generated includes:
#include "config.h"

// Includes from nestkernel:
#include "archiving_node.h"
#include "connection.h"
#include "event.h"
#include "nest_types.h"
#include "ring_buffer.h"
#include "universal_data_logger.h"

// Includes from precise:
#include "slice_ring_buffer.h"

namespace nest
{

/** @BeginDocumentation
Name: iaf_psc_alpha_ps - Leaky integrate-and-fire neuron
with alpha-shape postsynaptic currents and bisectioning method for
approximation of threshold crossing.

.. versionadded:: 2.18

Description:

iaf_psc_alpha_ps is the "canonical" implementation of the leaky
integrate-and-fire model neuron with alpha-shaped postsynaptic
currents in the sense of [1]. This is the most exact implementation
available.

PSCs are normalized to an amplitude of 1pA.

The precise implementation handles neuronal dynamics in a locally
event-based manner with in coarse time grid defined by the minimum
delay in the network, see [1]. Incoming spikes are applied at the
precise moment of their arrival, while the precise time of outgoing
spikes is determined by a bisectioning method to approximate the timing
of a threshold crossing [1,3]. Return from refractoriness occurs precisly
at spike time plus refractory period.

This implementation is more complex than the plain iaf_psc_alpha
neuron, but achieves much higher precision. In particular, it does not
suffer any binning of spike times to grid points. Depending on your
application, the canonical application may provide superior overall
performance given an accuracy goal; see [1] for details. Subthreshold
dynamics are integrated using exact integration between events [2].


Parameters:

The following parameters can be set in the status dictionary.

V_m          double - Membrane potential in mV
E_L          double - Resting membrane potential in mV.
V_min        double - Absolute lower value for the membrane potential.
C_m          double - Capacity of the membrane in pF
tau_m        double - Membrane time constant in ms.
t_ref        double - Duration of refractory period in ms.
V_th         double - Spike threshold in mV.
V_reset      double - Reset potential of the membrane in mV.
tau_syn_ex   double - Rise time of the excitatory synaptic function in ms.
tau_syn_in   double - Rise time of the inhibitory synaptic function in ms.
I_e          double - Constant external input current in pA.

Remarks:

This model transmits precise spike times to target nodes (on-grid spike
time and offset). If this node is connected to a spike_detector, the
property "precise_times" of the spike_detector has to be set to true in
order to record the offsets in addition to the on-grid spike times.

The iaf_psc_alpha_ps neuron accepts connections transmitting
CurrentEvents. These events transmit stepwise-constant currents which
can only change at on-grid times.

If tau_m is very close to tau_syn_ex/in, the model will numerically behave as
if tau_m is equal to tau_syn_ex/in, to avoid numerical instabilities.
For details, please see doc/model_details/IAF_neurons_singularity.ipynb.


References:

[1] Morrison A, Straube S, Plesser H E, & Diesmann M (2006) Exact Subthreshold
    Integration with Continuous Spike Times in Discrete Time Neural Network
    Simulations. To appear in Neural Computation.
[2] Rotter S & Diesmann M (1999) Exact simulation of time-invariant linear
    systems with applications to neuronal modeling. Biologial Cybernetics
    81:381-402.
[3] Hanuschkin A, Kunkel S, Helias M, Morrison A & Diesmann M (2010)
    A general and efficient method for incorporating exact spike times in
    globally time-driven simulations Front Neuroinformatics, 4:113

Author: Tanguy Fardet (based on Diesmann, Eppler, Morrison, Plesser, Straube)

Sends: SpikeEvent

Receives: SpikeEvent, CurrentEvent, DataLoggingRequest

SeeAlso: iaf_psc_alpha, iaf_psc_exp_ps
*/
class iaf_psc_alpha_ps : public Archiving_Node
{
public:
  /** Basic constructor.
      This constructor should only be used by GenericModel to create
      model prototype instances.
  */
  iaf_psc_alpha_ps();

  /** Copy constructor.
      GenericModel::allocate_() uses the copy constructor to clone
      actual model instances from the prototype instance.

      @note The copy constructor MUST NOT be used to create nodes based
      on nodes that have been placed in the network.
  */
  iaf_psc_alpha_ps( const iaf_psc_alpha_ps& );

  /**
   * Import sets of overloaded virtual functions.
   * @see Technical Issues / Virtual Functions: Overriding, Overloading, and
   * Hiding
   */
  using Node::handle;
  using Node::handles_test_event;

  port send_test_event( Node&, rport, synindex, bool );

  port handles_test_event( SpikeEvent&, rport );
  port handles_test_event( CurrentEvent&, rport );
  port handles_test_event( DataLoggingRequest&, rport );

  void handle( SpikeEvent& );
  void handle( CurrentEvent& );
  void handle( DataLoggingRequest& );

  bool
  is_off_grid() const
  {
    return true;
  } // uses off_grid events

  void get_status( DictionaryDatum& ) const;
  void set_status( const DictionaryDatum& );

private:
  /** @name Interface functions
   * @note These functions are private, so that they can be accessed
   * only through a Node*.
   */
  //@{
  void init_state_( const Node& proto );
  void init_buffers_();
  void calibrate();

  bool get_next_event_( const long T, double& ev_offset, double& ev_weight, bool& end_of_refract );

  /**
   * Time Evolution Operator.
   *
   * update() promotes the state of the neuron from origin+from to origin+to.
   * It does so in steps of the resolution h.  Within each step, time is
   * advanced from event to event, as retrieved from the spike queue.
   *
   * Return from refractoriness is handled as a special event in the
   * queue, which is marked by a weight that is GSL_NAN.  This greatly
   * simplifies the code.
   *
   * For steps, during which no events occur, the precomputed propagator matrix
   * is used.  For other steps, the propagator matrix is computed as needed.
   *
   * While the neuron is refractory, membrane potential (y3_) is
   * clamped to U_reset_.
   */
  void update( Time const& origin, const long from, const long to );

  //@}

  /**
   * Propagate neuron state.
   * Propagate the neuron's state by dt.
   * @param dt Interval over which to propagate
   */
  void propagate_( const double dt );

  /**
   * Trigger interpolation method to find the precise spike time
   * within the mini-timestep (t0,t0+dt] assuming that the membrane
   * potential was below threshold at t0 and above at t0+dt. Emit
   * the spike and reset the neuron.
   *
   * @param origin  Time stamp at beginning of slice
   * @param lag     Time step within slice
   * @param t0      Beginning of mini-timestep
   * @param dt      Duration of mini-timestep
   */
  void emit_spike_( Time const& origin, const long lag, const double t0, const double dt );

  /**
   * Instantaneously emit a spike at the precise time defined by
   * origin, lag and spike_offset and reset the neuron.
   *
   * @param origin        Time stamp at beginning of slice
   * @param lag           Time step within slice
   * @param spike_offset  Time offset for spike
   */
  void emit_instant_spike_( Time const& origin, const long lag, const double spike_offset );

  /** @name Threshold-crossing interpolation
   * These functions determine the time of threshold crossing using
   * interpolation, one function per interpolation
   * order. thresh_find() is the driver function and the only one to
   * be called directly.
   */
  //@{

  /** Interpolation orders. */
  enum interpOrder
  {
    NO_INTERPOL,
    LINEAR,
    QUADRATIC,
    CUBIC,
    END_INTERP_ORDER
  };


  /**
   * Difference between threshold and membrane potential for given time step.
   * @param   double time step
   * @returns difference between updated membrane potential and threshold
   */
  double threshold_distance_( double t_step ) const;

  /**
   * Localize threshold crossing by using Illinois algorithm of regula falsi method.
   *
   * See https://en.wikipedia.org/wiki/Regula_falsi#The_Illinois_algorithm for details on the algorithm.
   *
   * @param   double length of interval since previous event
   * @returns time from previous event to threshold crossing
   */
  double regula_falsi_( const double dt ) const;


  // The next two classes need to be friends to access the State_ class/member
  friend class RecordablesMap< iaf_psc_alpha_ps >;
  friend class UniversalDataLogger< iaf_psc_alpha_ps >;

  // ----------------------------------------------------------------

  /**
   * Independent parameters of the model.
   */
  struct Parameters_
  {

    /** Membrane time constant in ms. */
    double tau_m_;

    /** Time constant of synaptic current in ms. */
    double tau_syn_ex_;
    double tau_syn_in_;

    /** Membrane capacitance in pF. */
    double c_m_;

    /** Refractory period in ms. */
    double t_ref_;

    /** Resting potential in mV. */
    double E_L_;

    /** External DC current [pA] */
    double I_e_;

    /** Threshold, RELATIVE TO RESTING POTENTAIL(!).
        I.e. the real threshold is U_th_ + E_L_. */
    double U_th_;

    /** Lower bound, RELATIVE TO RESTING POTENTAIL(!).
        I.e. the real lower bound is U_min_+E_L_. */
    double U_min_;

    /** Reset potential.
              At threshold crossing, the membrane potential is reset to this
              value. Relative to resting potential.
     */
    double U_reset_;

    Parameters_(); //!< Sets default parameter values

    void get( DictionaryDatum& ) const; //!< Store current values in dictionary

    /** Set values from dictionary.
     * @returns Change in reversal potential E_L, to be passed to State_::set()
     */
    double set( const DictionaryDatum& );
  };

  // ----------------------------------------------------------------

  /**
   * State variables of the model.
   */
  struct State_
  {
    double y_input_;           //!< external input current
    double I_ex_;              //!< alpha current, first component
    double dI_ex_;             //!< alpha current, second component
    double I_in_;              //!< alpha current, first component
    double dI_in_;             //!< alpha current, second component
    double V_m_;               //!< Membrane pot. rel. to resting pot. E_L_.
    bool is_refractory_;       //!< true while refractory
    long last_spike_step_;     //!< time stamp of most recent spike
    double last_spike_offset_; //!< offset of most recent spike

    State_(); //!< Default initialization

    void get( DictionaryDatum&, const Parameters_& ) const;

    /** Set values from dictionary.
     * @param dictionary to take data from
     * @param current parameters
     * @param Change in reversal potential E_L specified by this dict
     */
    void set( const DictionaryDatum&, const Parameters_&, double );
  };

  // ----------------------------------------------------------------

  /**
   * Buffers of the model.
   */
  struct Buffers_
  {
    Buffers_( iaf_psc_alpha_ps& );
    Buffers_( const Buffers_&, iaf_psc_alpha_ps& );

    /**
     * Queue for incoming events.
     * @note Handles also pseudo-events marking return from refractoriness.
     */
    SliceRingBuffer events_;
    RingBuffer currents_;

    //! Logger for all analog data
    UniversalDataLogger< iaf_psc_alpha_ps > logger_;
  };

  // ----------------------------------------------------------------

  /**
   * Internal variables of the model.
   */
  struct Variables_
  {
    double h_ms_;           //!< time resolution in ms
    double psc_norm_ex_;    //!< e / tau_syn_ex
    double psc_norm_in_;    //!< e / tau_syn_in
    long refractory_steps_; //!< refractory time in steps
    double expm1_tau_m_;    //!< exp(-h/tau_m) - 1
    double exp_tau_syn_ex_; //!< exp(-h/tau_syn_ex)
    double exp_tau_syn_in_; //!< exp(-h/tau_syn_in)
    double P30_;            //!< progagator matrix elem, 3rd row
    double P31_ex_;         //!< progagator matrix elem, 3rd row (ex)
    double P32_ex_;         //!< progagator matrix elem, 3rd row (ex)
    double P31_in_;         //!< progagator matrix elem, 3rd row (in)
    double P32_in_;         //!< progagator matrix elem, 3rd row (in)
    double y_input_before_; //!< at beginning of mini-step, for interpolation
    double I_ex_before_;    //!< at beginning of mini-step, for interpolation
    double I_in_before_;    //!< at beginning of mini-step, for interpolation
    double dI_ex_before_;   //!< at beginning of mini-step, for interpolation
    double dI_in_before_;   //!< at beginning of mini-step, for interpolation
    double V_m_before_;     //!< at beginning of mini-step, for interpolation
  };

  // Access functions for UniversalDataLogger -------------------------------

  //! Read out the real membrane potential
  double
  get_V_m_() const
  {
    return S_.V_m_ + P_.E_L_;
  }

  //! Read out state variable I_ex
  double
  get_I_ex_() const
  {
    return S_.I_ex_;
  }

  //! Read out state variable derivative of I_ex
  double
  get_dI_ex_() const
  {
    return S_.dI_ex_;
  }

  //! Read out state variable I_in
  double
  get_I_in_() const
  {
    return S_.I_in_;
  }

  //! Read out state variable derivative of I_ex
  double
  get_dI_in_() const
  {
    return S_.dI_in_;
  }

  // ----------------------------------------------------------------

  /**
   * @defgroup iaf_psc_alpha_data
   * Instances of private data structures for the different types
   * of data pertaining to the model.
   * @note The order of definitions is important for speed.
   * @{
   */
  Parameters_ P_;
  State_ S_;
  Variables_ V_;
  Buffers_ B_;
  /** @} */

  //! Mapping of recordables names to access functions
  static RecordablesMap< iaf_psc_alpha_ps > recordablesMap_;
};

inline port
nest::iaf_psc_alpha_ps::send_test_event( Node& target, rport receptor_type, synindex, bool )
{
  SpikeEvent e;
  e.set_sender( *this );
  return target.handles_test_event( e, receptor_type );
}

inline port
iaf_psc_alpha_ps::handles_test_event( SpikeEvent&, rport receptor_type )
{
  if ( receptor_type != 0 )
  {
    throw UnknownReceptorType( receptor_type, get_name() );
  }
  return 0;
}

inline port
iaf_psc_alpha_ps::handles_test_event( CurrentEvent&, rport receptor_type )
{
  if ( receptor_type != 0 )
  {
    throw UnknownReceptorType( receptor_type, get_name() );
  }
  return 0;
}

inline port
iaf_psc_alpha_ps::handles_test_event( DataLoggingRequest& dlr, rport receptor_type )
{
  if ( receptor_type != 0 )
  {
    throw UnknownReceptorType( receptor_type, get_name() );
  }
  return B_.logger_.connect_logging_device( dlr, recordablesMap_ );
}

inline void
iaf_psc_alpha_ps::get_status( DictionaryDatum& d ) const
{
  P_.get( d );
  S_.get( d, P_ );
  Archiving_Node::get_status( d );

  ( *d )[ names::recordables ] = recordablesMap_.get_list();
}

inline void
iaf_psc_alpha_ps::set_status( const DictionaryDatum& d )
{
  Parameters_ ptmp = P_;                 // temporary copy in case of errors
  const double delta_EL = ptmp.set( d ); // throws if BadProperty
  State_ stmp = S_;                      // temporary copy in case of errors
  stmp.set( d, ptmp, delta_EL );         // throws if BadProperty

  // We now know that (ptmp, stmp) are consistent. We do not
  // write them back to (P_, S_) before we are also sure that
  // the properties to be set in the parent class are internally
  // consistent.
  Archiving_Node::set_status( d );

  // if we get here, temporaries contain consistent set of properties
  P_ = ptmp;
  S_ = stmp;
}

} // namespace

#endif // IAF_PSC_ALPHA_PS_H
