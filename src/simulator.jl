#simulator.jl
#basically a place to hold next, and to call a sequence, get relevant statistics, and visualize.

typealias MLObservation Union{MLObs,PartialFailObs,CompleteFailObs}

function rand!(rng::AbstractRNG,s::MLState,d::MLStateDistr)
  states = MLState[]
  probs = Float64[]
  for (state,prob) in d.d
    push!(states,state)
    push!(probs,prob)
  end
  s = sample(rng,states,WeightVec(probs))
  return s
end

function rand!(rng::AbstractRNG,o::MLObs,d::MLObsDistr)
  obss = MLState[]
  probs = Float64[]
  for (obs,prob) in d.d
    push!(obss,obs)
    push!(probs,prob)
  end
  o = sample(rng,obss,WeightVec(probs))
  return o
end

function next(rng::AbstractRNG,pomdp::MLPOMDP,s::MLState,a::MLAction)
  dt = pomdp.phys_param.dt
  POSITIONS = pomdp.phys_param.POSITIONS
  VELOCITIES = pomdp.phys_param.VELOCITIES
  BEHAVIORS = pomdp.BEHAVIORS
  ##agent position always updates deterministically
  agent_lane_ = s.agent_pos + a.lane_change
  agent_lane_ = max(1,min(agent_lane_,pomdp.nb_col)) #can't leave the grid

  v_interval = (pomdp.phys_param.v_fast-pomdp.phys_param.v_slow)/(pomdp.phys_param.nb_vel_bins-1)
  agent_vel_ = s.agent_vel + a.vel*convert(Int,ceil(1.*dt/(v_interval)))
  agent_vel_ = max(1,min(agent_vel_,pomdp.phys_param.nb_vel_bins))

  car_states = CarState[]
  valid_col_top = collect(1:pomdp.nb_col)
	valid_col_bot = collect(1:pomdp.nb_col)

  #sensor failure
  r = rand(rng)
  if s.sensor_failed
    if r < pomdp.p_fail_persist #so it doesnt persist
        sensor_failed_ = !s.sensor_failed
    else
      sensor_failed_ = s.sensor_failed
    end
  else
    if r < pomdp.p_fail_enter
      sensor_failed_ = true
    else
      sensor_failed_ = false
    end
  end

  pos_enter = vcat([(1,y) for y in valid_col_bot],[(pomdp.col_length,y) for y in valid_col_top])

  for (i,car) in enumerate(s.env_cars)
    if car.pos[1] > 0
      pos = car.pos
      vel = car.vel
      lane_change = car.lane_change
      lane_ = max(1,min(pos[2]+lane_change,pomdp.nb_col))

      neighborhood = get_adj_cars(pomdp.phys_param,s.env_cars,i)

      dvel_ms = get_idm_dv(car.behavior.p_idm,dt,VELOCITIES[vel],get(neighborhood.ahead_dv,0,0.),get(neighborhood.ahead_dist,0,1000.)) #call idm model
      vel_inds = rev_1d_interp(VELOCITIES,vel+dvel_ms,car.behavior.rationality)
      pos_m = POSITIONS[pos[1]] + dt*(VELOCITIES[vel]-VELOCITIES[s.agent_vel])+0.5*dt*dvel_ms #x+vt+1/2at2
      if (pos_m > POSITIONS[end]) || (pos_m < POSITIONS[1])
        push!(car_states,CarState((0,1),1,0,BEHAVIORS[1]))
        continue
      end
      pos_inds = rev_1d_interp(POSITIONS,pos_m,1.0)
      #sample position
      poss = collect(keys(pos_inds))
      if length(poss) == 1
        pos = poss[1]
      else
        r = rand(rng)
        pos = r > pos_inds[poss[1]] ? poss[1]: poss[2]
      end

      #sample velocity
      #not using rng because its a lot of work
      vels = collect(keys(vel_inds))
      probs = [vel_inds[v] for v in vels]
      if !(vel in vels)
        vel_other = vel - sign(vels[1]-vel)
        if (vel_other < 1) || (vel_other > length(VELOCITIES))
          push!(vels,vel)
          push!(probs,1.-car.behavior.rationality)
        else
          push!(vels,vel)
          push!(probs,(1.-car.behavior.rationality)/2.)
          push!(vels,vel_other)
          push!(probs,(1.-car.behavior.rationality)/2.)
        end
      else
        vmax = maximum(vels)
        vmin = minimum(vels)
        if vel == vmax
          vel_other = vel + 1
        else
          vel_other = vel - 1
        end

        if (vel_other < 1) || (vel_other > length(VELOCITIES))
            #this actually works--sample doesn't care about objects being unique
            push!(vels,vel)
            push!(probs,1.-car.behavior.rationality)
				else
          push!(vels,vel_other)
          push!(probs,1.-car.behavior.rationality)
				end
      end
      vel_ = sample(rng,vels,WeightVec(probs))
      #sample lanechange
      #if in between lanes, continue lanechange with prob behavior.rationality, else go other direction
      if mod(lane_,2) == 0 #in between lanes
        r = rand(rng)
        #if on the off chance its not changing lanes, make it, the jerk
        if lane_change == 0
          lane_change = rand(rng,-1:2:1)
        end
        lanechange = r < car.behavior.rationality ? lane_change : -1*lane_change
      else
        #sample normally
        lanechange_ = get_mobil_lane_change(pomdp.phys_param,car,neighborhood)
        lane_change_other = setdiff([-1;0;1],[lanechange_])
        lanechange_other_probs = ((1-car.behavior.rationality)/length(lane_change_other))*ones(length(lane_change_other))
        lanechange_probs = WeightVec([car.behavior.rationality;lanechange_other_probs])
        lanechange = sample(rng,[lanechange_;lane_change_other],lanechange_probs)
      end

      push!(car_states,CarState((pos,lane_),vel_,lanechange,car.behavior))
    else
      #TODO: push this to a second loop after this loop
      r = rand(rng)
      if r < 1.-car.behavior.rationality
        push!(car_states,car)
        continue
      end
      r = rand(rng,1:length(pos_enter))
      pos = splice!(pos_enter,r)
      if pos[1] > 1
        vels = collect(1:agent_vel_)
      else
        vels = collect(agent_vel_:pomdp.phys_param.nb_vel_bins)
      end
      vel = vels[rand(rng,1:length(vels))]
      if mod(pos[1],2) == 0
        #in between lanes, so he must be lane changing
        lanechanges = [-1;1]
      else
        lanechanges = [-1;0;1]
      end
      lanechange = lanechanges[rand(rng,1:length(lanechanges))]
      behavior = BEHAVIORS[rand(rng,1:length(BEHAVIORS))]

      push!(car_states,CarState(pos,vel,lanechange,behavior))
    end
  end

  return MLState(agent_lane_,agent_vel_,sensor_failed_,car_states)

end

function observe(rng::AbstractRNG,pomdp::MLPOMDP,s::MLState,a::MLAction)
  agent_pos = s.agent_pos
	agent_vel = s.agent_vel
	##environment positions fully observed

  #sensor failure
	car_obs = CarStateObs[]
  if s.sensor_failed
    if pomdp.complete_failure
      return CompleteFailObs()
    else
      return PartialFailObs(agent_pos,agent_vel)
    end
  end

  Z = randn(rng,pomdp.nb_cars)
  for (i,car) in enumerate(s.env_cars)
    if car.pos[1] == 0
      #nothing to observe
      push!(car_obs,CarStateObs(car.pos,car.vel,car.lane_change))
      continue
    end

    v = pomdp.phys_param.VELOCITIES[car.vel]
    v_ = v + Z[i]*pomdp.o_vel_sig

    vel_rng = searchsorted(pomdp.phys_param.VELOCITIES,v_)

    ###NOTE: the following might not actually be correct
    #set it to whatever is closer
    start = min(vel_rng.start,length(pomdp.phys_param.VELOCITIES))
    stop = max(vel_rng.stop,1)
    if start <= stop
      vel_ = start
    else
      dstart = abs(v_ - pomdp.phys_param.VELOCITIES[start])
      dstop = abs(v_ - pomdp.phys_param.VELOCITIES[stop])
      vel_ = dstart < dstop ? start: stop
    end

    push!(car_obs,CarStateObs(car.pos,vel_,car.lane_change))
  end

  #i'm going to be relying on the feature function to sort this out for me
  return MLObs(agent_pos,agent_vel,s,car_obs)
end

function to_tilecode(s::MLObs)
  pos = s.agent_pos
  vel = s.agent_vel
  phi = zeros(length(s.env_cars)*4) #x,y, velocity, lanechange
  offset = 0
  for car in s.env_cars
    phi[1 + offset] = car.pos[1] #already wrt to relative frame
    phi[2 + offset] = pos - car.pos[2]
    phi[3 + offset] = vel - car.vel
    phi[4 + offset] = car.lane_change
    offset += 4
  end

  return phi
end

function to_tilecode(s::MLState)
  pos = s.agent_pos
  vel = s.agent_vel
  phi = zeros(length(s.env_cars)*4) #x,y, velocity, lanechange, behavior <-todo
  offset = 0
  for car in s.env_cars
    phi[1 + offset] = car.pos[1] #already wrt to relative frame
    phi[2 + offset] = pos - car.pos[2]
    phi[3 + offset] = vel - car.vel
    phi[4 + offset] = car.lane_change
    #phi[5 + offset] = 0.
    offset += 4
  end

  return phi
end

function init(pomdp::MLPOMDP,rng::AbstractRNG=MersenneTwister(34985))
  return MLState(1,convert(Int,round(length(pomdp.phys_param.nb_vel_bins))),
                CarState[CarState((0,1),1,0,pomdp.BEHAVIORS[1]) for i = 1:pomdp.nb_cars])
end

function simulate(pomdp::MLPOMDP,policy::Function;
          discount::Float64=0.99,
          nb_timesteps::Int=20,
          simRNG::AbstractRNG=MersenneTwister(1336),
          polRNG::AbstractRNG=MersenneTwister(53487523))
    #init
    state_hist = Array{MLState,1}[]
    action_hist = Array{MLAction,1}[]
    gamma = 1.
    R = 0.
    s = init(pomdp,rng=simRNG) #function does not yet exist
    for t = 1:nb_timesteps
      a = policy(s,rng=polRNG)
      state_hist = MLState[state_hist; s]
      action_hist = MLAction[action_hist; a]
      if isterminal(pomdp,s,a)
        break
      end
      R += gamma*reward(pomdp,s,a)
      d = transition(pomdp,s,a)
      s = rand!(simRNG,s,d)
      #s = next(simRNG,s,a)
      gamma *= discount
    end
    return R,state_hist,action_hist
end

function display_sim(pomdp::MLPOMDP,S::Array{MLState,1},A::Array{MLAction,1})
  warn("This should be run in a Jupyter Notebook")
  assert(length(S) == length(A))
  f = figure()
  @manipulate for i = 1:length(S); withfig(f) do
    visualize(pomdp,S[i],A[i]) end
  end
end
