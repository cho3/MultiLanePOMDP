# Things we want in common
#   visualization
#       for this, we need road parameters
# Things we want in common, but configurable
#   reward model
#   states
#   action
#   dynamics model


abstract AbstractMLRewardModel
abstract AbstractMLDynamicsModel

type MLMDP{S, A, DModel<:AbstractMLDynamicsModel, RModel<:AbstractMLRewardModel}  <: MDP{S, A}
    rmodel::RModel
    dmodel::DModel
	discount::Float64
end

type OriginalRewardModel <: AbstractMLRewardModel
	r_crash::Float64
	accel_cost::Float64
	decel_cost::Float64
	invalid_cost::Float64
	lineride_cost::Float64
	lanechange_cost::Float64
end

type NoCrashRewardModel <: AbstractMLRewardModel
    cost_close_call::Float64
    cost_emergency_brake::Float64
    reward_in_desired_lane::Float64
end

type IDMMobilModel <: AbstractMLDynamicsModel
	nb_cars::Int # ??? number of cars (what happens when they leave)
	phys_param::PhysicalParam # ??? what should really be in physical parameters

	BEHAVIORS::Array{BehaviorModel,1} # ??? will we always need this
	NB_PHENOTYPES::Int # 

	encounter_prob::Float64 # ??? What is this
	accels::Array{Int,1}
end

function IDMMobilModel(nb_cars, phys_param; encounter_prob=0.5, accels=Int[-3,-2,-1,0,1])
    BEHAVIORS = BehaviorModel[BehaviorModel(x[1],x[2],x[3],idx) for (idx,x) in enumerate(product(["cautious","normal","aggressive"],[phys_param.v_slow+0.5;phys_param.v_med;phys_param.v_fast],[phys_param.l_car]))]
    return IDMMobilModel(nb_cars, phys_param, BEHAVIORS, length(BEHAVIORS), encounter_prob, accels)
end

typealias OriginalMDP MLMDP{MLState, MLAction, IDMMobilModel, OriginalRewardModel}
