#simulator.jl
#basically a place to hold next, and to call a sequence, get relevant statistics, and visualize.

typealias MLObservation Union{MLObs,PartialFailObs,CompleteFailObs}

function rand!(rng::AbstractRNG,d::MLStateDistr)
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

function rand(rng::AbstractRNG,action_space::ActionSpace,::Union{MLState,MLAction})
  r = rand(1:length(action_space))
  return domain(action_space)[r]
end


function next(rng::AbstractRNG,pomdp::MLPOMDP,s::MLState,a::MLAction)
  dt = pomdp.phys_param.dt
  #POSITIONS = pomdp.phys_param.POSITIONS
  #VELOCITIES = pomdp.phys_param.VELOCITIES
  BEHAVIORS = pomdp.BEHAVIORS
  ##agent position always updates deterministically
  agent_lane_ = s.agent_pos + a.lane_change
  agent_lane_ = max(1,min(agent_lane_,pomdp.nb_col)) #can't leave the grid

  #v_interval = (pomdp.phys_param.v_fast-pomdp.phys_param.v_slow)/(pomdp.phys_param.nb_vel_bins-1)
  agent_vel_ = s.agent_vel + a.vel*dt#convert(Int,ceil(a.vel*dt/(v_interval)))
  #underactuating
  agent_vel_ = max(pomdp.phys_param.v_slow,min(agent_vel_,pomdp.phys_param.v_fast))

  car_states = CarState[]
  valid_col_top = collect(1:2:pomdp.nb_col)
	valid_col_bot = collect(1:2:pomdp.nb_col)

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

  encounter_inds = Int[] #which cars need to be updated in the second loop

  for (i,car) in enumerate(s.env_cars)
    if car.pos[1] >= 0.
      pos = car.pos
      vel = car.vel
      lane_change = car.lane_change
      lane_ = max(1,min(pos[2]+lane_change,pomdp.nb_col))

      neighborhood = get_adj_cars(pomdp.phys_param,s.env_cars,i)

      dvel_ms = get_idm_dv(car.behavior.p_idm,dt,vel,get(neighborhood.ahead_dv,0,0.),get(neighborhood.ahead_dist,0,1000.)) #call idm model
      #bound by the acceleration/braking terms in idm models
      #NOTE restricting available velocities
      dvel_ms = min(max(dvel_ms/dt,-car.behavior.p_idm.b),car.behavior.p_idm.a)*dt
      vel_ = vel + dvel_ms
      pos_ = pos[1] + dt*(vel-s.agent_vel)
      #vel_inds = rev_1d_interp(VELOCITIES[3:end-2],VELOCITIES[vel]+dvel_ms,car.behavior.rationality)
      #pos_m = POSITIONS[pos[1]] + dt*(VELOCITIES[vel]-VELOCITIES[s.agent_vel])#+0.5*dt*dvel_ms #x+vt+1/2at2
      if (pos_ > pomdp.phys_param.lane_length) || (pos_ < 0.)
        push!(car_states,CarState((-1.,1),1.,0,BEHAVIORS[1]))
        continue
      end
      #TODO add noise to position transition

      #sample velocity
      #accelerate normally or dont accelerate
      if rand(rng) < 1- car.behavior.rationality
        vel_ = vel
      end
      vel_ = max(min(vel_,pomdp.phys_param.v_max),pomdp.phys_param.v_min)
      #sample lanechange
      #if in between lanes, continue lanechange with prob behavior.rationality, else go other direction
      #TODO add safety check here
      if mod(lane_,2) == 0 #in between lanes
        r = rand(rng)
        #if on the off chance its not changing lanes, make it, the jerk
        if lane_change == 0
          lane_change = rand(rng,-1:2:1)
        end
        lanechange = r < car.behavior.rationality ? lane_change : -1*lane_change

        if is_lanechange_dangerous(neighborhood,dt,pomdp.phys_param.l_car,lanechange)
          lanechange *= -1
        end

      else
        #sample normally
        lanechange_ = get_mobil_lane_change(pomdp.phys_param,car,neighborhood)
        lane_change_other = setdiff([-1;0;1],[lanechange_])
        #safety criterion is hard
        if is_lanechange_dangerous(neighborhood,dt,pomdp.phys_param.l_car,1)
          lane_change_other = setdiff(lane_change_other,[1])
        end
        if is_lanechange_dangerous(neighborhood,dt,pomdp.phys_param.l_car,-1)
          lane_change_other = setdiff(lane_change_other,[-1])
        end

        lanechange_other_probs = ((1-car.behavior.rationality)/length(lane_change_other))*ones(length(lane_change_other))
        lanechange_probs = WeightVec([car.behavior.rationality;lanechange_other_probs])
        lanechange = sample(rng,[lanechange_;lane_change_other],lanechange_probs)
        #NO LANECHANGING
        lanechange = 0
      end

      #if near top, remove from valid_col_top
      if pomdp.phys_param.lane_length - pos_ <= pomdp.phys_param.l_car*1.5
        remove_set = [lane_;lane_+1;lane_-1]
        for idx in remove_set
          idy = findfirst(valid_col_top,idx)
          if idy > 0
            splice!(valid_col_top,idy)
          end
        end
      #elseif near bot, remove from valid_col_bot
      elseif pos_ < 1.5*pomdp.phys_param.l_car
        remove_set = [lane_;lane_+1;lane_-1]
        for idx in remove_set
          idy = findfirst(valid_col_bot,idx)
          if idy > 0
            splice!(valid_col_bot,idy)
          end
        end
      end

      push!(car_states,CarState((pos_,lane_),vel_,lanechange,car.behavior))
    else
      #TODO: push this to a second loop after this loop
      r = rand(rng)
      if r < 1.-pomdp.encounter_prob
        push!(encounter_inds,i)
        #continue
      end
      push!(car_states,car)
    end
  end

  pos_enter = vcat([(0.,y) for y in valid_col_bot],
                  [(pomdp.phys_param.lane_length,y) for y in valid_col_top])
  for j in encounter_inds
    if length(pos_enter) <= 0
      #this should work find since that just means they're unencountered by default
      break
    end
    r = rand(rng,1:length(pos_enter))
    pos = splice!(pos_enter,r)
    if pos[1] > 0.
      vels = (pomdp.phys_param.v_min,agent_vel_)#collect(1:agent_vel_)
    else
      vels = (agent_vel_,pomdp.phys_param.v_max)#collect(agent_vel_:pomdp.phys_param.nb_vel_bins)
    end
    vel = (vels[2]-vels[1])*rand(rng)+vels[1]#vels[rand(rng,1:length(vels))]
    if mod(pos[2],2) == 0
      #in between lanes, so he must be lane changing
      lanechanges = [-1;1]
    else
      lanechanges = [-1;0;1]
    end
    lanechange = lanechanges[rand(rng,1:length(lanechanges))]
    behavior = deepcopy(BEHAVIORS[rand(rng,1:length(BEHAVIORS))])
    behavior.p_idm.v0 = (min(behavior.p_idm.v0+1,pomdp.phys_param.v_max)-
                          max(behavior.p_idm.v0-1,pomdp.phys_param.v_min))*
                          rand(rng) + behavior.p_idm.v0

    car_states[j] = CarState(pos,vel,lanechange,behavior)
    #push!(car_states,CarState(pos,vel,lanechange,behavior))
  end

  return MLState(agent_lane_,agent_vel_,sensor_failed_,car_states)

end


function bin(arr::AbstractArray,val::Union{Float64,Int})
  rng = searchsorted(arr,val)

  ###NOTE: the following might not actually be correct
  #set it to whatever is closer
  start = min(rng.start,length(arr))
  stop = max(rng.stop,1)
  if start <= stop
    idx = start
  else
    dstart = abs(val - arr[start])
    dstop = abs(val - arr[stop])
    idx = dstart < dstop ? start: stop
  end

  return idx
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

  x_ctr = pomdp.phys_param.lane_length/2.
  y_ctr = s.agent_pos*pomdp.phys_param.y_interval

  Z = randn(rng,pomdp.nb_cars,3)
  for (i,car) in enumerate(s.env_cars)
    if car.pos[1] == 0
      #nothing to observe
      push!(car_obs,CarStateObs(car.pos,car.vel,car.lane_change))
      continue
    end

    #TODO: adjust noise level based on distance (longitudinal) from ego-car
    dist = norm([car.pos[1]-x_ctr;
                  car.pos[2]*pomdp.phys_param.y_interval-y_ctr])
    #VELOCITY NOISE
    v = car.vel
    vel_ = v + Z[i,1]*pomdp.o_vel_sig*dist

    #LONGITUDINAL NOISE
    p = car.pos[1]
    pos_ = p + Z[i,2]*pomdp.o_pos_sig*dist

    #LANE NOISE
    #TODO: figure out how to truncate distributions so as dist->inf, we don't get all mass at extremes
    lane = car.pos[2]
    lane_sig = pomdp.o_lane_sig*dist/pomdp.phys_param.y_interval
    lane_sig = min(lane_sig,Inf)
    l_ = lane + Z[i,3]*lane_sig

    lane_ = bin(collect(1:pomdp.nb_col),l_)

    #LANECHANGE NOISE
    lc = lanechange
    lc_sig = pomdp.o_lanechange_sig*dist
    lc_sig = min(lane_sig,1.) #at sigma = 1.-> approximately uniform over [-1,0,1]
    lc_ = lc + Z[i,3]*lc_sig

    lane_change_ = bin(collect(-1:1),lx_)

    push!(car_obs,CarStateObs((pos_,lane_),vel_,car.lane_change_))
  end

  #i'm going to be relying on the feature function to sort this out for me
  return MLObs(agent_pos,agent_vel,s.sensor_failed,car_obs)
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
  return MLState(1,pomdp.phys_param.v_med,
                CarState[CarState((-1.,1),1.,0,pomdp.BEHAVIORS[1]) for i = 1:pomdp.nb_cars])
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

function display_sim(pomdp::MLPOMDP,S::Array{MLState,1},A::Array{MLAction,1};debug::Bool=false)
  warn("This should be run in a Jupyter Notebook")
  assert(length(S) == length(A))
  f = figure()
  @manipulate for i = 1:length(S); withfig(f) do
    visualize(pomdp,S[i],A[i],debug=debug) end
  end
end

function abs_display_sim(pomdp::MLPOMDP,S::Array{MLState,1},A::Array{MLAction,1};debug::Bool=false)
  assert(length(S) == length(A))
  #First pass: get total distance traveled
  tot_dist = pomdp.phys_param.lane_length/2.
  dists = zeros(length(S))
  for (i,s) in enumerate(S)
    dists[i] = tot_dist - pomdp.phys_param.lane_length/2.
    tot_dist += s.agent_vel*pomdp.phys_param.dt
  end
  nb_rows = ceil(Integer,tot_dist/pomdp.phys_param.lane_length)
  #second pass: draw the things
  f = figure()
  #pos = pomdp.phys_param.lane_length/2.
  @manipulate for i = 1:length(S); withfig(f) do
    #calculation
    visualize(pomdp,S[i],A[i],debug=debug,frame=dists[i],nb_rows=nb_rows)
    #pos += S[i].agent_vel*pomdp.phys_param.dt end
    end
  end

end


#to make partial observability generator work in MCTS
type MLStateObs
  o::MLObservation
  s::MLState
end
==(a::MLStateObs,b::MLStateObs) = a.o==b.o
hash(a::MLStateObs,h::UInt64=zero(UInt64)) = hash(a.o,h)

isterminal(pomdp::MLPOMDP,s::MLStateObs,a::MLAction=create_action(pomdp)) = isterminal(pomdp,s.s,a)
