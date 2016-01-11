#ML_types.jl
#a separate file for all the type definitions and convenience functions so they're out of the way.


###############
##POMDP Model##
###############

#2 lane case
"""
| | |  19 12 5
| | |  18 11 4
| | |  17 10 3
"""
#12-11-10 is the lane divider--an intermediate state during a lane change


#It's more effective to consider each environment car individually.
#The break even point is when (2*nb_lanes-1)*lane_intervals > (6^4)/5 (assuming complete congestion

#TODO: define hash,== for each type
#TODO: how to handle order invariance for state space?
#TODO: how to handle continuing a lane change?

type MLState <: State
	agent_pos::Int #row
	agent_vel::Int
	env_cars::Array{CarState,1}
end #MLState
==(a::MLState,b::MLState) = (a.agent_pos==b.agent_pos) && (a.agent_vel==b.agent_vel) &&(a.env_cars == b.env_cars)
Base.hash(a::MLState,h::UInt64=zero(UInt64)) = hash(a.agent_vel,hash(a.agent_pos,hash(a.env_cars,h)))


type MLAction <:Action
	vel::Int #-1,0 or +1, corresponding to desired velocities of v_fast,v_slow or v_nom
	lane_change::Int #-1,0, or +1, corresponding to to the right lane, no lane change, or to the left lane
end
==(a::MLAction,b::MLAction) = (a.vel==b.vel) && (a.lane_change==b.lane_change)
Base.hash(a::MLAction,h::UInt64=zero(UInt64)) = hash(a.vel,hash(a.lane_change,h))

type CarStateObs
	pos::Tuple{Int,Int}
	vel::Int #index of
	lane_change::Int #-1,0, or +1, corresponding to to the right lane, no lane change, or to the left lane
end
==(a::CarStateObs,b::CarStateObs) = (a.pos==b.pos) && (a.vel==b.vel) &&(a.lane_change == b.lane_change)
Base.hash(a::CarStateObs,h::UInt64=zero(UInt64)) = hash(a.pos,hash(a.vel,hash(a.lane_change,h)))

##Observe: Car positions, self position, speed, noisy realization of other car's speed (gaussian?)
type MLObs <: Observation
	agent_pos::Int #col
	agent_vel::Int #index in velocity vector?
	env_cars::Array{CarStateObs,1}
end
==(a::MLObs,b::MLObs) = (a.agent_pos==b.agent_pos) && (a.agent_vel==b.agent_vel) &&(a.env_cars == b.env_cars)
Base.hash(a::MLObs,h::UInt64=zero(UInt64)) = hash(a.agent_pos,hash(a.agent_vel,hash(a.env_cars,h)))

type MLPOMDP <: POMDP
	nb_col::Int
	col_length::Int
	nb_cars::Int
	behaviors::Array{BehaviorModel,1} #instantiate each behavior phenotype as a static object in inner constructor
	r_crash::Float64
	accel_cost::Float64
	decel_cost::Float64
	lanechange_cost::Float64
	fuzz::Float64
	discount::Float64
	phys_param::PhysicalParam
	BEHAVIORS::Array{BehaviorModel,1}
	NB_PHENOTYPES::Int
	o_vel_sig::Float64
	encounter_prob::Float64
	function MLPOMDP(;nb_lanes::Int=2,
					nb_cars::Int=1,
					discount::Float64=0.99,
					r_crash::Float64=-100000.,
					accel_cost::Float64=-1.,
					decel_cost::Float64=-0.5,
					lanechange_cost::Float64=-2.,
					fuzz::Float64=0.1,
					o_vel_sig::Float64=2.,
					encounter_prob::Float64=0.5,
					phys_param::PhysicalParam=PhysicalParam(nb_lanes))
		assert((discount >= 0.) && (discount <= 1.))
		assert((fuzz >= 0.) && (fuzz <= 1.))

		self = new()
		self.nb_col = convert(Int,round((phys_param.w_lane/phys_param.y_interval)*nb_lanes-1))
		self.col_length = length(phys_param.POSITIONS)
		self.nb_cars = nb_cars
		self.o_vel_sig = o_vel_sig
		self.encounter_prob = encounter_prob
		#behaviors...
		self.r_crash = r_crash
		self.accel_cost = accel_cost
		self.decel_cost = decel_cost
		self.lanechange_cost = lanechange_cost
		self.discount = discount
		self.fuzz = fuzz
		self.phys_param = phys_param
		self.BEHAVIORS = BehaviorModel[BehaviorModel(x[1],x[2],x[3]) for x in product(["cautious","normal","aggressive"],[phys_param.v_slow;phys_param.v_med;phys_param.v_fast],[phys_param.l_car])]
		self.NB_PHENOTYPES = length(self.BEHAVIORS)

		return self
	end
	#physical param type holder?
	#rewards
end #100000.

n_states(p::MLPOMDP) = p.nb_col*p.phys_param.nb_vel_bins*(p.col_length*p.nb_col*p.phys_param.NB_DIR*p.phys_param.nb_vel_bins*p.NB_PHENOTYPES+1)^p.nb_cars
n_actions(::MLPOMDP) = 9
n_observations(p::MLPOMDP) = p.nb_col*p.phys_param.nb_vel_bins*(p.col_length*p.nb_col*p.phys_param.NB_DIR*p.phys_param.nb_vel_bins+1)^p.nb_cars

type StateSpace <: AbstractSpace
	states::Vector{MLState}
end

type ActionSpace <: AbstractSpace
	actions::Vector{MLAction}
end

type ObsSpace <: AbstractSpace
	obs::Vector{MLObs}
end

domain(space::StateSpace) = space.states
domain(space::ActionSpace) = space.actions
domain(space::ObsSpace) = space.obs
length(space::StateSpace) = length(space.states)
length(space::ActionSpace) = length(space.actions)
length(space::ObsSpace) = length(space.obs)
