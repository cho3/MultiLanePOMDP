#ML_types.jl
#a separate file for all the type definitions and convenience functions so they're out of the way.


###############
##POMDP Model##
###############

# agent pos:
# 3 lanes: 1, 3, 5 in lanes
#          2, 4, in between lanes

type MLState
	agent_pos::Int #row
	agent_vel::Float64
	# sensor_failed::Bool # getting rid of this for the base mdp
	env_cars::Array{CarState,1}
end #MLState

==(a::MLState,b::MLState) = (a.agent_pos==b.agent_pos) && (a.agent_vel==b.agent_vel) &&(a.env_cars == b.env_cars)
Base.hash(a::MLState,h::UInt64=zero(UInt64)) = hash(a.agent_vel,hash(a.agent_pos,hash(a.env_cars,h)))

type MLAction
	vel::Float64 #-1,0 or +1, corresponding to desired velocities of v_fast,v_slow or v_nom
	lane_change::Int #-1,0, or +1, corresponding to to the right lane, no lane change, or to the left lane
end
MLAction() = MLAction(0,0)
==(a::MLAction,b::MLAction) = (a.vel==b.vel) && (a.lane_change==b.lane_change)
Base.hash(a::MLAction,h::UInt64=zero(UInt64)) = hash(a.vel,hash(a.lane_change,h))
function MLAction(x::Array{Float64,1})
	assert(length(x)==2)
	lane_change = abs(x[2]) <= 0.3? 0: sign(x[2])
	return MLAction(x[1],lane_change)
end
vec(a::MLAction) = Float64[a.vel;a.lane_change]

type CarStateObs
	pos::Tuple{Float64,Int}
	vel::Float64 #index of
	lane_change::Int #-1,0, or +1, corresponding to to the right lane, no lane change, or to the left lane
end
==(a::CarStateObs,b::CarStateObs) = (a.pos==b.pos) && (a.vel==b.vel) &&(a.lane_change == b.lane_change)
Base.hash(a::CarStateObs,h::UInt64=zero(UInt64)) = hash(a.pos,hash(a.vel,hash(a.lane_change,h)))

##Observe: Car positions, self position, speed, noisy realization of other car's speed (gaussian?)
abstract MLObservation # XXX: rolling all the information into a single concrete type would probably be more efficient

type MLObs <: MLObservation
	agent_pos::Int #col
	agent_vel::Float64 #index in velocity vector?
	sensor_failed::Bool
	env_cars::Array{CarStateObs,1}
end
MLObs(p::Int,v::Float64,cars::Array{CarStateObs,1}) = MLObs(p,v,false,cars)
==(a::MLObs,b::MLObs) = (a.agent_pos==b.agent_pos) && (a.agent_vel==b.agent_vel) &&(a.env_cars == b.env_cars) && (a.sensor_failed == b.sensor_failed)
Base.hash(a::MLObs,h::UInt64=zero(UInt64)) = hash(a.agent_pos,hash(a.agent_vel,hash(a.env_cars,hash(a.sensor_failed,h))))

type PartialFailObs <: MLObservation
	agent_pos::Int
	agent_vel::Float64
end
==(a::PartialFailObs,b::PartialFailObs) = (a.agent_pos==b.agent_pos) && (a.agent_vel == b.agent_vel)
Base.hash(a::PartialFailObs,h::UInt64=zero(UInt64)) = hash(a.agent_pos,hash(a.agent_vel,h))

type CompleteFailObs <: MLObservation
end

type MLPOMDP <: POMDP{MLState, MLAction, MLObservation}
	nb_col::Int
	col_length::Int
	nb_cars::Int
	behaviors::Array{BehaviorModel,1} #instantiate each behavior phenotype as a static object in inner constructor
	r_crash::Float64
	accel_cost::Float64
	decel_cost::Float64
	invalid_cost::Float64
	lineride_cost::Float64
	lanechange_cost::Float64
	fuzz::Float64
	discount::Float64
	phys_param::PhysicalParam
	BEHAVIORS::Array{BehaviorModel,1}
	NB_PHENOTYPES::Int
	o_vel_sig::Float64
	o_pos_sig::Float64
	o_lane_sig::Float64
	o_lanechange_sig::Float64
	encounter_prob::Float64
	p_fail_enter::Float64
	p_fail_persist::Float64
	accels::Array{Int,1}
	complete_failure::Bool #do you have access to own position and velocity?
	function MLPOMDP(;nb_lanes::Int=2,
					nb_cars::Int=1,
					discount::Float64=0.99,
					r_crash::Float64=-100000.,
					accel_cost::Float64=-1.,
					decel_cost::Float64=-0.5,
					invalid_cost::Float64 = -1.,
					lineride_cost::Float64 = -1.,
					lanechange_cost::Float64=-2.,
					fuzz::Float64=0.1,
					o_vel_sig::Float64=0.05, #meter of noise/meter of distance (should be <1)
					o_pos_sig::Float64=0.05, #saw ~0.05 RMSE at all distances in some paper
					o_lane_sig::Float64=0.05,
					o_lanechange_sig::Float64=0.025, #sign is more robust?
					encounter_prob::Float64=0.5,
					phys_param::PhysicalParam=PhysicalParam(nb_lanes),
					p_fail_enter::Float64=0.05,
					p_fail_persist::Float64=0.5,
					accels::Array{Int,1}=[-3,-2,-1,0,1],
					complete_failure::Bool=false)
		assert((discount >= 0.) && (discount <= 1.))
		assert((fuzz >= 0.) && (fuzz <= 1.))

		self = new()
		# self.nb_col = convert(Int,round((phys_param.w_lane/phys_param.y_interval)*nb_lanes-1))
		self.col_length = length(phys_param.POSITIONS)
		self.nb_cars = nb_cars
		self.o_vel_sig = o_vel_sig
		self.o_pos_sig = o_pos_sig
		self.o_lane_sig = o_lane_sig
		self.o_lanechange_sig = o_lanechange_sig
		self.encounter_prob = encounter_prob
		#behaviors...
		self.r_crash = r_crash
		self.accel_cost = accel_cost
		self.decel_cost = decel_cost
		self.lanechange_cost = lanechange_cost
		self.invalid_cost = invalid_cost
		self.lineride_cost = lineride_cost
		self.discount = discount
		self.fuzz = fuzz
		self.phys_param = phys_param
		self.BEHAVIORS = BehaviorModel[BehaviorModel(x[1],x[2],x[3],idx) for (idx,x) in enumerate(product(["cautious","normal","aggressive"],[phys_param.v_slow+0.5;phys_param.v_med;phys_param.v_fast],[phys_param.l_car]))]
		self.NB_PHENOTYPES = length(self.BEHAVIORS)
		self.p_fail_enter = p_fail_enter
		self.p_fail_persist = p_fail_persist
		self.complete_failure = complete_failure
		self.accels = accels

		return self
	end
	#physical param type holder?
	#rewards
end #100000.

create_state(p::MLPOMDP) = MLState(1,pomdp.phys_param.v_med,false,CarState[CarState((-1.,1,),1.,0,p.BEHAVIORS[1]) for _ = 1:p.nb_cars])
create_action(p::MLPOMDP) = MLAction()

n_states(p::MLPOMDP) = p.nb_col*p.phys_param.nb_vel_bins*(p.col_length*p.nb_col*p.phys_param.NB_DIR*p.phys_param.nb_vel_bins*p.NB_PHENOTYPES+1)^p.nb_cars
n_actions(p::MLPOMDP) = p.phys_param.NB_DIR*length(p.accels)
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
iterator(A::ActionSpace)=domain(A)
