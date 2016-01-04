#MultiLane.jl
#auth: Christopher Ho
#date: 12/3/2015
#affil: Stanford University
#desc: an updated model for the multilane highway driving problem that can support multiple env cars
#	and is grounded in reality with physical values and uses the IDM and MOBIL model for env cars

using POMDPs
using Distributions
using POMDPToolbox
using StatsBase

import POMDPs: n_states, n_actions, n_observations # space sizes for discrete problems
import POMDPs: discount, states, actions, observations # model functions
import POMDPs: transition, observation, reward, isterminal # model functions
import POMDPs: create_state, create_action, create_observation # s,a,o initialization
import POMDPs: length, index, weight, pdf # discrete distribution functions
import POMDPs: rand!, pdf # common distribution functions
import POMDPs: domain # space functions
import POMDPs: create_transition_distribution, create_observation_distribution, create_belief, initial_belief 
import POMDPs: update, updater

import Base.convert
import Base.==
import Base.hash

import Iterators.product
using Distributions

##TODO: implement with two cars? (else no lane changing will occur)
##TODO: move MOBIL and IDM to another file(s)

include("IDM.jl")
include("MOBIL.jl")
include("encounter.jl")
include("observation.jl")
include("crash.jl")
include("ML_types.jl")
include("ML_model.jl")
include("visualization.jl")
include("simulator.jl")

##TODO: export....

#################
##Velocity Bins##
#################
"""
This is derived from where a cautious car will accelerate from the slowest speed (27 m/s) to the highest speed
(35 m/s, with a true desired speed of 36, but lower just for sanity), and this is the velocity it has at each
0.75 second time slice.

I'm thinking it'll transition to the next velocity state to the nearest as predicted by IDM with prob. 
0.9*rationality = (1-fuzz)*rationality, and with 0.05*rationality transition to the next closest states on
either side = (fuzz/2)*rationality (or would something based on the normal distribution be more sensible)

w/ prob 1-r = 1-rationality, it'll lane change or choose a speed update according to a different set of idm 
parameters
"""
"""
Might need a different set of velocity bins for self-car for the sake of state-space sanity.
Assuming 150 Hp = 111855 Watts of power available, 1500 kg car,
then at 35 m/s, the acceleration it can get is 2.13 m/s2 => 1.598 m/s delta v over dt = 0.75 seconds
--so it's probably not unreasonable to have a own-velocity that's coarser than the IDM based environment cars
"""

#################
##Position Bins##
#################
"""
Let's say the minimum relative speed to have appreciable change in relative position is dv = 1.385/4, or
a quarter of human walking speed. This means that at the finest grain of difference in relative velocity
that is permissible to allow a relative position change, corresponds to approximately dx = 0.25m = dv*dt

So for about 5 car lengths of state space, we have about 80 points in the x direction.
"""