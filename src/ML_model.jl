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

actions(pomdp::MLPOMDP) = ActionSpace([MLAction(x[1],x[2]) for x in product(p.accels,[-1,0,1])])
actions(pomdp::MLPOMDP,s::MLState,A::ActionSpace=actions(pomdp)) = A #SARSOP does not support partially available actions

function reward(pomdp::MLPOMDP,s::MLState,a::MLAction)
	#assume environment cars don't crash with one another?
	pos = s.agent_pos
	vel = a.vel
	lane_change = a.lane_change
	if is_crash(pomdp,s,a)
		return pomdp.r_crash
	end
	"""
	agent_pos_ind = (1+pomdp.col_length)/2
	if round(agent_pos_ind) != agent_pos_ind
		agent_pos = (POSITIONS[convert(Int,round(ceil(agent_pos_ind)))]+POSITIONS[convert(Int,round(floor(agent_pos_ind)))])/2
	else
		agent_pos = POSITIONS[convert(Int,round(agent_pos_ind))]
	end

	for env_car in s.env_cars
		p = env_car.pos
		v = env_car.vel
		l = env_car.lane_change
		#TODO: TRAJECTORIY INTERSECTION
		crash_flag = false
		if abs(p[1]-agent_pos) < l_car
			crash_flag = true
		elseif abs(pos-p[2]) (times) y_interval < w_car
			crash_flag = true
		end
		#check crash conditions
		if crash_flag
			return pomdp.r_crash
		end
	end
	"""
	#penalty for accelerating/decelerating/lane change
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
reward(pomdp::MLPOMDP,s::MLState,a::MLAction,sp::MLState) = reward(pomdp,sp,a)

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

function transition(pomdp::MLPOMDP,s::MLState,a::MLAction,d::MLStateDistr=create_transition_distribution(pomdp))

	dt = pomdp.phys_param.dt
	POSITIONS = pomdp.phys_param.POSITIONS
	VELOCITIES = pomdp.phys_param.VELOCITIES
	##agent position always updates deterministically
	agent_lane_ = s.agent_pos + a.lane_change
	agent_lane_ = max(1,min(agent_lane_,pomdp.nb_col)) #can't leave the grid

	v_interval = (pomdp.phys_param.v_fast-pomdp.phys_param.v_slow)/(pomdp.phys_param.nb_vel_bins-1)
	agent_vel_ = s.agent_vel + convert(Int,ceil(a.vel*dt/(v_interval)))
	agent_vel_ = max(1,min(agent_vel_,pomdp.phys_param.nb_vel_bins))
	##check if an encounter is happening/cant happen
	#if no environment cars are around, an encounter will always happen?
	##encounter depends on configuration of environment? so nest the above into the dynamic update loop
	#add

	##TODO: check each car's location to update
	##		need to estimate next position for each car first, and then check
	#TODO: update these things, do a second pass for encounter model
	valid_col_top = collect(1:pomdp.nb_col)
	valid_col_bot = collect(1:pomdp.nb_col)

	env_car_next_states = Array{Dict{CarState,Float64},1}[]
	for (i,env_car) in enumerate(s.env_cars)
		pos = env_car.pos
		vel = env_car.vel
		lane_change = env_car.lane_change
		behavior = env_car.behavior
		lane_ = max(1,min(pos[2]+lane_change,pomdp.nb_col))

		#get the leaders and followers for itself and all adjacent lanes somehow
		if pos[1] > 0

			neighborhood = get_adj_cars(pomdp.phys_param,s.env_cars,i)

			dvel_ms = get_idm_dv(behavior.p_idm,dt,VELOCITIES[vel],get(neighborhood.ahead_dv,0,0.),get(neighborhood.ahead_dist,0,1000.)) #call idm model
			vel_inds = rev_1d_interp(VELOCITIES,VELOCITIES[vel]+dvel_ms,behavior.rationality)
			pos_m = POSITIONS[pos[1]] + dt*(VELOCITIES[vel]-VELOCITIES[s.agent_vel])#+0.5*dt*dvel_ms #x+vt+1/2at2
			if (pos_m > POSITIONS[end]) || (pos_m < POSITIONS[1])
				#if out of scene, move to out of scene state
				next_state_probs = Dict{CarState,Float64}(CarState((0,1),1,0,pomdp.BEHAVIORS[1]) => 1.0)
				env_car_next_states = [env_car_next_states;next_state_probs]
				#append!(env_car_next_states,[next_state_probs])
				#push!(env_car_next_states,next_state_probs)
				continue
			end
			pos_inds = rev_1d_interp(POSITIONS,pos_m,1.0) #1.0 corresponds to total probability of 1.
			#TODO: LANE CHANGE STUFF
			#TODO: add probability of doing something else--e.g. lane change, speeding up more than expected or braking

			pos_probs = pos_inds#Dict{Int,Float64}() #placeholder, built off pos_inds
			lane_probs = Dict{Int,Float64}(lane_=>1.)
			vel_probs = deepcopy(vel_inds)
			#equal probability of doing nothing or the opposite
			if !(vel in keys(vel_probs))
				vel1 = collect(keys(vel_probs))[1]
				vel_ = vel - sign(vel1-vel) #if going faster, then maybe go slower for whatever reason and vv
				vel_probs[vel] = (1.-behavior.rationality)/2.
				if (vel_ < 1) || (vel_ > length(VELOCITIES))
					#oob--just dump the rest of the probabilty to the current bin
					vel_probs[vel] = 1.-behavior.rationality
				else
					#normal case: dump to the opposite bin
					vel_probs[vel_] = (1.-behavior.rationality)/2.
				end
				#TODO:something
			else
				vmax = maximum(keys(vel_probs))
				vmin = minimum(keys(vel_probs))
				#check against both to get the sign for the other case where you do something else
				#NOTE: there are only two keys so this should work
				if vel == vmax
					#then we should be going up in spd when being irrationality
					vel_ = vel + 1
				else #if vel == vmin
					#then we should be going down in spd when being irrationality
					vel_ = vel - 1
				end

				#check edge cases
				if (vel_ < 1) || (vel_ > length(VELOCITIES))
					vel_probs[vel] += 1-behavior.rationality
				else
					vel_probs[vel_] = 1-behavior.rationality
				end

				#TODO:the something, but with full irrationality probability
			end
			assert(abs(sum(values(vel_probs))-1.),0.0000001,<)
			#placeholder
			if mod(lane_,2) == 0
				#force a lane change if in between lanes, equal prob either way
				if lane_change == 0
					lanechange_probs = Dict{Int,Float64}([-1=>0.5,1=>0.5])
				else
					lanechange_probs = Dict{Int,Float64}([lanechange=>behavior.rationality,-1*lanechange=>1-behavior.rationality])

				end
			else
				lanechange = get_mobil_lane_change(pomdp.phys_param,env_car,neighborhood)
				lane_change_other = setdiff([-1;0;1],[lanechange])
				#safety criterion is hard
        if (get(neighborhood.behind_dist,1,1000.) < 0.) || (get(neighborhood.ahead_dist,1,1000.) < 0.)
          lane_change_other = setdiff(lane_change_other,[1])
        end
        if (get(neighborhood.behind_dist,-1,1000.) < 0.) || (get(neighborhood.ahead_dist,-1,1000.) < 0.)
          lane_change_other = setdiff(lane_change_other,[-1])
        end
				lanechange_probs = Dict{Int,Float64}()
				lanechange_probs[lanechange] = length(lane_change_other) != 0?behavior.rationality:1.
				for lanechange_ in lane_change_other
					lanechange_probs[lanechange_] = (1.-behavior.rationality)/(length(lane_change_other))
				end
				assert(abs(sum(values(lanechange_probs))-1.),0.0000001,<,"lanechange_probs")
				assert(abs(sum(values(pos_probs))-1.),0.0000001,<,"pos_probs")
			end
			comp_probs = product(pos_probs,lane_probs,vel_probs,lanechange_probs)
			#position, velocity, and lane changing are uncoupled
			#next_state_probs = Dict{CarState,Float64}()
			#TODO
			next_state_probs = Dict{CarState,Float64}([CarState((x[1][1],x[2][1],),x[3][1],x[4][1],behavior) => x[1][2]*x[2][2]*x[3][2]*x[4][2] for x in comp_probs])
		else #if pos[1] <= 0
			###NOTE:ENCOUNTER MODEL
			##position and velocities are coupled
			#TODO: check if this is even consistent
			pos_inds1 = [(x[1],x[2],) for x in product([pomdp.col_length],valid_col_top)] #SLOW CARS
			pos_vel1 = [(x[1],x[2],x[3],x[4]) for x in product(pos_inds1,collect(1:agent_vel_),[-1;0;1],pomdp.BEHAVIORS)]
			pos_inds2 = [(x[1],x[2],) for x in product([1],valid_col_bot)] #FAST CARS
			pos_vel2 = [(x[1],x[2],x[3],x[4]) for x in product(pos_inds2,collect(agent_vel_:pomdp.phys_param.nb_vel_bins),[-1;0;1],pomdp.BEHAVIORS)]
			#TODO: prune pos_inds based on location of other cars

			next_state_probs = Dict{CarState,Float64}()
			for x in pos_vel1
				next_state_probs[CarState(x[1],x[2],x[3],x[4])] = (pomdp.encounter_prob)/(length(pos_vel1)+length(pos_vel2))
			end
			for x in pos_vel2
				next_state_probs[CarState(x[1],x[2],x[3],x[4])] = (pomdp.encounter_prob)/(length(pos_vel1)+length(pos_vel2))
			end

			#for now, assume no one is changing lanes while they come into your horizon
			#lane_change_probs = Dict{Int,Float64}(0=>1.)
			#lane_change_probs = Dict{Int,Float64}(x=>(1/3.) for x in [-1;0;1])
			next_state_probs[CarState((0,1),1,0,behavior)] = 1-pomdp.encounter_prob
		end

		env_car_next_states = [env_car_next_states;next_state_probs]
		#push!(env_car_next_states,next_state_probs)
		#create dictionary of all these composite probabilities
	end

	##TODO: enumerate over agent car stuff
	#there is either one or two possible next agent velocities if we allow the agent car to follow IDM--the other option is to just have it increment its velocity in reasonable, deterministic increments
	#d = Dict{MLState,Float64}()
	env_car_next_states_ = product(env_car_next_states...)
	d.d = Dict{MLState,Float64}([MLState(agent_lane_,agent_vel_,CarState[y[1] for y in env]) => prod([x[2] for x in env]) for env in env_car_next_states_])

	#for each dictionary, calculate composite probability, and then iterate over all possibilities, and calculate the product of probabilities and add to dictionary

	##for each environment car: calculate position update according to true velocity
	##							calculate possible next possible velocities; assign probabilities (including irrationalities and 'fuzz')
	##							behavior model persists

	if abs(sum(values(d.d))-1.) > 0.00001
		println(d.d)
		println(sum(values(d.d)))
		error("Invalid distribution")
	end
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
isterminal(pomdp::MLPOMDP,s::MLState,a::MLAction=create_action(pomdp)) = is_crash(pomdp,s,a) #placeholder
