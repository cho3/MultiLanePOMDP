#ML_model.jl
#a separate file for hte ML pomdp formulation
##TODO: fix long dictionary comprehension on lines 224, 257 and 316
function observations(pomdp::MLPOMDP)

	#generate all partial states for a single environment car
	env_car_states = CarStateObs[]

	for x in 0:length(pomdp.phys_param.POSITIONS)
		for y in 1:pomdp.nb_col
			for v in 1:length(pomdp.phys_param.VELOCITIES)
				for lane_change in [-1; 0; 1]
					#collapse not in the environment into a single state
					if x != 0
						push!(env_car_states,CarStateObs((x,y,),v,lane_change))
					elseif (x == 0) && (y == 1) && (v == 1) && (lane_change == 0)
						push!(env_car_states,CarStateObs((x,y,),v,lane_change))
					end
					"""
					if (x == 0) && (y != 1) && (v != 1) && (lane_change != 0)
						break
					end
					push!(env_car_states,CarStateObs((x,y,),v,lane_change))
					"""
				end
			end
		end
	end
	car_states = product([env_car_states for i = 1:pomdp.nb_cars]...)

	###WARNING!!!! THIS IS SUPER FRAGILE ^^^VVV
	O = MLObs[]
	#include iterators.product
	#filter that thing
	for env_state in car_states
		#loop over own car stuff
		for self_pos in 1:pomdp.nb_col
			for self_vel in 1:length(pomdp.phys_param.VELOCITIES)
				push!(O,MLObs(self_pos,self_vel,CarStateObs[e_s for e_s in env_state]))
			end
		end
	end
	if(length(O) != n_observations(pomdp))
		println("Size of generated observation space: $(length(O))")
		println("Calculated observation space size: $(n_observations(pomdp))")
		error("Error: analytical number of states does not agree with generated number of observations")
	end

	return O
end

function states(pomdp::POMDP)
#generate all partial states for a single environment car
	env_car_states = CarState[]
	#behaviors = BehaviorModel[BehaviorModel(x[1],x[2]) for x in product(["cautious","normal","aggressive"],[v_slow;v_med;v_fast])]

	for x in 0:length(pomdp.phys_param.POSITIONS)
		for y in 1:pomdp.nb_col
			for v in 1:length(pomdp.phys_param.VELOCITIES)
				for lane_change in [-1; 0; 1]
					for behavior in pomdp.BEHAVIORS
						#collapse not in the environment into a single state
						if x != 0
							push!(env_car_states,CarState((x,y,),v,lane_change,behavior))
						elseif (x== 0) && (y == 1) && (v == 1) && (lane_change == 0) && (behavior == pomdp.BEHAVIORS[1])
							push!(env_car_states,CarState((x,y,),v,lane_change,behavior))
						end
						"""
						if (x == 0) && (y != 1) && (v != 1) && (lane_change != 0) && (behavior != pomdp.BEHAVIORS[1])
							continue
						end
						push!(env_car_states,CarState((x,y,),v,lane_change,behavior))
						"""
					end
				end
			end
		end
	end
	car_states = product([env_car_states for i = 1:pomdp.nb_cars]...)

	###WARNING!!!! THIS IS SUPER FRAGILE ^^^VVV
	S = MLState[]
	#include iterators.product
	#filter that thing
	for env_state in car_states
		#loop over own car stuff
		for self_pos in 1:pomdp.nb_col
			for self_vel in 1:length(pomdp.phys_param.VELOCITIES)
				push!(S,MLState(self_pos,self_vel,CarState[e_s for e_s in env_state]))
			end
		end
	end
	if(length(S) != n_states(pomdp))
		println("Size of generated state space: $(length(S))")
		println("Calculated state space size: $(n_states(pomdp))")
		error("Error: analytical number of states does not agree with generated number of states")
	end

	return S
end

actions(pomdp::MLPOMDP) = ActionSpace([MLAction(x[1],x[2]) for x in product(pomdp.accels,[-1,0,1])])
actions(pomdp::MLPOMDP,s::MLState,A::ActionSpace=actions(pomdp)) = A #SARSOP does not support partially available actions

function __reward(pomdp::MLPOMDP,s::MLState,a::MLAction)

	pos = s.agent_pos
	vel = a.vel
	lane_change = a.lane_change

	cost = 0.
	if a.vel > 0
		cost += pomdp.accel_cost*a.vel
	elseif a.vel < 0
		cost -= pomdp.decel_cost*a.vel
	end
	if abs(a.lane_change) != 0
		cost += pomdp.lanechange_cost
		if (a.lane_change == -1) && (s.agent_pos == 1)
			cost += pomdp.invalid_cost
		elseif (a.lane_change == 1) && (s.agent_pos == pomdp.nb_col)
			cost += pomdp.invalid_cost
		end
	end
	if mod(s.agent_pos,2) == 0 #on a lane divider--an
		cost += pomdp.lineride_cost
	end


	return cost #
end

function reward(pomdp::MLPOMDP,s::MLState,a::MLAction)
	#assume environment cars don't crash with one another?
	if is_crash(pomdp,s,a)
		return pomdp.r_crash
	end
	#penalty for accelerating/decelerating/lane change

	return __reward(pomdp,s,a) #
end

function reward(pomdp::MLPOMDP,s::MLState,a::MLAction,sp::MLState)
	if is_crash(pomdp,s,a) || isterminal(pomdp,sp)
		return pomdp.r_crash
	end
	return __reward(pomdp,s,a)
end

type MLStateDistr <: AbstractDistribution
	d::Dict{MLState,Float64} #e.g. sparse vector
end
pdf(d::MLStateDistr,s::MLState) = get(d.d,s,0.)
support(d::MLStateDistr) = keys(d.d)
create_transition_distribution(::MLPOMDP) = MLStateDistr(Dict{MLState,Float64}())

function rev_1d_interp(arr::Array{Float64,1},x::Float64,fx::Float64)
	#NOTE! This is not actually how interpolation works
	rng = searchsorted(arr,x)
	start = min(rng.start,length(arr))
	stop = max(rng.stop,1)
	if start <= stop
		#on the off chance it actually lands squarely on a division
		return Dict{Int,Float64}(start=>fx) #0.4 syntax
	end
	#stop is the smaller bin, start is hte larger bin
	d = Dict{Int,Float64}(stop=> fx*(arr[start]-x)/(arr[start]-arr[stop]),
								start=>fx*(x-arr[stop])/(arr[start]-arr[stop]))
	assert(abs(sum(values(d))-fx),0.000001, <=)
	return d
end

type MLObsDistr <:AbstractDistribution
	d::Dict{MLObs,Float64} #e.g. sparse vector
end
pdf(d::MLObsDistr,o::MLObs) = get(d.d,o,0.)
support(d::MLObsDistr) = keys(d.d)
create_observation_distribution(::MLPOMDP) = MLObsDistr(Dict{MLObs,Float64}())


function pdf(VELOCITIES::Array{Float64,1},vel_ind::Int,vel_true::Int,sig::Float64=2.)
	vu = vel_ind < length(VELOCITIES)? (VELOCITIES[vel_ind+1]+VELOCITIES[vel_ind])/2. : Inf
	vl = vel_ind > 1? (VELOCITIES[vel_ind-1]+VELOCITIES[vel_ind])/2. : -Inf
	zu = (vu-VELOCITIES[vel_true])/sig
	zl = (vl-VELOCITIES[vel_true])/sig
	return cdf(Normal(),zu)-cdf(Normal(),zl)
end

observation(p::MLPOMDP,s::MLState,a::MLAction,sp::MLState,d::MLObsDistr=create_observation_distribution(p)) = observation(p,s,a,d)
function observation(pomdp::MLPOMDP,s::MLState,a::MLAction,d::MLObsDistr=create_observation_distribution(pomdp))
	#or should i be operating on sp?
	distr = Dict{MLObs,Float64}
	##agent position fully observed
	agent_pos = s.agent_pos
	agent_vel = s.agent_vel
	##environment positions fully observed
	env_pos = Tuple{Int,Int}[car.pos for car in s.env_cars]
	carstate_probs = Dict{CarStateObs,Float64}[Dict{CarStateObs,Float64}() for i = 1:length(env_pos)]
	##fuzz on true velocity
	for (j,vel) in enumerate(pomdp.phys_param.VELOCITIES)
		for i in 1:length(env_pos)
			"""
			NOTE!!! You can probably add sensor failure here VVV
			"""
			#a car is observed to not be in the scene with probability 1.
			if env_pos[i][1] == 0
				carstate_probs[i][CarStateObs(env_pos[i],1,s.env_cars[i].lane_change)] = 1.
				continue
			end
			p_vel = pdf(pomdp.phys_param.VELOCITIES,j,s.env_cars[i].vel,pomdp.o_vel_sig)
			#println(p_vel)
			if p_vel < 0.00001
				break
			end
			##PLACEHOLDER vvv this should be something else, sometimes observe wrong lane change
			lane_change = s.env_cars[i].lane_change
			carstate_probs[i][CarStateObs(env_pos[i],j,lane_change)] = p_vel
			#do normal cdf
			#if below threshold: break
			#multiply prob by relevant probability
		end

	end
	##behavior model unobserved
	#println(carstate_probs)
	#distr = Dict{MLObs,Float64}()
	carstate_probs_ = product(carstate_probs...)
	distr = Dict{MLObs,Float64}([MLObs(agent_pos,agent_vel,CarStateObs[y[1] for y in env]) => prod([x[2] for x in env]) for env in carstate_probs_])

	if abs(sum(values(distr))-1.) > 0.0001
		println(d.d)
		println(sum(values(d.d)))
		error("Invalid observation distribution")
	end
	d.d = distr
	return d

end

discount(pomdp::MLPOMDP) = pomdp.discount
isterminal(pomdp::MLPOMDP,s::MLState,a::MLAction=MLAction()) = is_crash(pomdp,s,a) #placeholder
