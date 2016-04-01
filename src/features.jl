#features.jl
#just features for use with function approximation reinforcement learning

function bin(i::Int,rng::Int,nb_bins::Int)
  #NOTE: assume i \in [1...rng]
  if rng < nb_bins
    error("Number of bins is larger than domain")
  end
  if rng == nb_bins
    return i
  end
  inc = ceil(rng/nb_bins)
  bin = floor(Integer,i/inc)
  return 1 + bin #julia is 0 indexed
end


function ordered_cars(p::MLPOMDP,s::MLState)

  x = p.phys_param.lane_length/2.
  y = s.agent_pos*p.phys_param.y_interval
  dist = Inf*ones(length(s.env_cars))

  for (i,car) in enumerate(s.env_cars)
    x_ = car.pos[1]
    if x_ < 0.
      continue
    end
    y_ = car.pos[2]*p.phys_param.y_interval
    dist[i] = norm([x_-x;y_-y])
  end

  inds = sortperm(dist)
  return [(i,s.env_cars[ind]) for(i,ind) in enumerate(inds)]
  #[(i,car) for ...]
end

function generate_state_actionexplicit_featurefunction{T}(ff::Function,A::Array{T,1},nb_feat::Int)
  #ff returns a set of active indices,
  A_indices = Dict{T,Int}([a=>i for (i,a) in enumerate(A)])
  function feature_function{S}(s::S,a::T)
    active_indices = ff(s)
    active_indices += nb_feat*(A_indices[a]-1)
    return sparsevec(active_indices,ones(length(active_indices)),nb_feat*length(A_indices))
  end
  return feature_function
end

function generate_joint_featurefunction(p::MLPOMDP)

  nb_feat = 0
  nb_y_bins = p.nb_col
  nb_lanechange_bins = 3
  nb_x_bins = 0
  nb_vel_bins = 0
  nb_feat_per_car = nb_y_bins*nb_x_bins*nb_vel_bins*nb_lanechange_bins



  if pomdp.complete_failure
    Obs = Union{MLObs,CompleteFailObs}
  else
    Obs = Union{MLObs,PartialFailObs}
  end

  function feature_function(s::MLState,a::MLAction)
    phi = spzeros(nb_feat,1)
  end

  function feature_function(o::Obs,a::MLAction)

  end

  return feature_function
end

function generate_disjoint_featurefunction(p::MLPOMDP;
                                              nb_x_bins::Int=20,nb_vel_bins::Int=20)
  nb_y_bins = p.nb_col
  nb_lanechange_bins = p.phys_param.NB_DIR

  #second term for own car
  nb_feat = (nb_y_bins + nb_lanechange_bins + nb_x_bins + nb_vel_bins + p.NB_PHENOTYPES)*(p.nb_cars)
  nb_feat += nb_y_bins + nb_vel_bins
  #nb_feat += 9 #nb_actions
  function feature_function(s::Union{MLState,MLObs})
    active_indices = zeros(Int,5*p.nb_cars + 2)
    active_indices[1] = bin(s.agent_pos,1,p.nb_col,nb_y_bins)
    feat_offset = nb_y_bins
    active_indices[2] = feat_offset + bin(s.agent_vel,p.phys_param.v_slow,p.phys_param.v_fast,nb_vel_bins)
    feat_offset += nb_vel_bins
    offset = 2
    oob_set = Int[]
    for (i,car) in ordered_cars(p,s)#enumerate(s.env_cars)
      if car.pos[1] <= 0
        feat_offset += nb_x_bins+nb_y_bins+nb_vel_bins+nb_lanechange_bins+p.NB_PHENOTYPES
        offset += 5
        append!(oob_set,collect(offset+1:offset+5))
        continue
      end
      active_indices[offset + 1] = feat_offset + bin(car.pos[1],0,p.phys_param.lane_length,nb_x_bins)
      feat_offset += nb_x_bins
      active_indices[offset + 2] = feat_offset + bin(car.pos[2],1,p.nb_col,nb_y_bins)
      feat_offset += nb_y_bins
      active_indices[offset + 3] = feat_offset + bin(car.vel,p.phys_param.v_slow-1.,p.phys_param.v_fast+1.,nb_vel_bins)
      feat_offset += nb_vel_bins
      active_indices[offset + 4] = feat_offset + bin(car.lane_change,-1,1,nb_lanechange_bins)
      feat_offset += nb_lanechange_bins
      ##Adding explicit behaviormodel bit
      active_indices[offset + 5] = feat_offset + car.behavior.idx#find(car.behavior .== p.BEHAVIORS)[1] #potentially brittle
      feat_offset += p.NB_PHENOTYPES
      offset += 5#4
    end
    #remove elements from oob set--should be zero
    active_indices = [i for i in filter(x->x>0,active_indices)]

    assert(!(0 in active_indices))
    return sparsevec(active_indices,ones(length(active_indices)),nb_feat)
  end

  return feature_function
  #return generate_state_actionexplicit_featurefunction(feature_function,nb_feat,A)

end

function generate_partial_disjoint_featurefunction(p::MLPOMDP;
                                              nb_x_bins::Int=20,nb_vel_bins::Int=20)
  nb_y_bins = p.nb_col
  nb_lanechange_bins = p.phys_param.NB_DIR

  #second term for own car
  nb_feat = (nb_y_bins + nb_lanechange_bins + nb_x_bins + nb_vel_bins)*(p.nb_cars)
  nb_feat += nb_y_bins + nb_vel_bins
  #nb_feat += 9 #nb_actions
  function feature_function(s::Union{MLState,MLObs})
    active_indices = zeros(Int,4*p.nb_cars + 2)
    active_indices[1] = bin(s.agent_pos,1,p.nb_col,nb_y_bins)
    feat_offset = nb_y_bins
    active_indices[2] = feat_offset + bin(s.agent_vel,p.phys_param.v_slow,p.phys_param.v_fast,nb_vel_bins)
    feat_offset += nb_vel_bins
    offset = 2
    oob_set = Int[]
    for (i,car) in ordered_cars(p,s)#enumerate(s.env_cars)
      if car.pos[1] <= 0
        feat_offset += nb_x_bins+nb_y_bins+nb_vel_bins+nb_lanechange_bins
        offset += 4
        append!(oob_set,collect(offset+1:offset+4))
        continue
      end
      active_indices[offset + 1] = feat_offset + bin(car.pos[1],0,p.phys_param.lane_length,nb_x_bins)
      feat_offset += nb_x_bins
      active_indices[offset + 2] = feat_offset + bin(car.pos[2],1,p.nb_col,nb_y_bins)
      feat_offset += nb_y_bins
      active_indices[offset + 3] = feat_offset + bin(car.vel,p.phys_param.v_slow-1.,p.phys_param.v_fast+1.,nb_vel_bins)
      feat_offset += nb_vel_bins
      active_indices[offset + 4] = feat_offset + bin(car.lane_change,-1,1,nb_lanechange_bins)
      feat_offset += nb_lanechange_bins
      offset += 4#4
    end
    #remove elements from oob set--should be zero
    active_indices = [i for i in filter(x->x>0,active_indices)]

    assert(!(0 in active_indices))
    return sparsevec(active_indices,ones(length(active_indices)),nb_feat)
  end

  return feature_function
  #return generate_state_actionexplicit_featurefunction(feature_function,nb_feat,A)

end



type AvoidPolicy <: Policy
  p::MLPOMDP
  jerk::Bool
  lanechange::Int
  exp::TrueNullFeatureExpander
end
AvoidPolicy(p::MLPOMDP;jerk::Bool=false) = AvoidPolicy(p,jerk,0)

function get_closest_car(pomdp::MLPOMDP,s::Union{MLState,MLObs},lookahead_only::Bool=false)
  v = s.agent_vel
  x = pomdp.phys_param.lane_length/2. #row
  p = s.agent_pos #col
  d = Inf
  closest_car = Union{CarState,CarStateObs}[] #this is so we can exploit length() == 0 insead of defining null
  for car in s.env_cars
    if lookahead_only
      if x-car.pos[1] > 0
        continue
      end
    end
    _d = norm([x-car.pos[1];p-car.pos[2]])
    if _d < d
      d = _d
      if length(closest_car) <= 0
        push!(closest_car,car)
      else
        closest_car[1] = car
      end
    end
  end
  return closest_car
end


function ReinforcementLearning.action(p::AvoidPolicy,s::Union{MLState,MLObs})
  cs = get_closest_car(p.p,s,p.jerk)
  if length(cs) <= 0
    #return empty action
    return MLAction(0,0)
  end
  cs = cs[1]
  x = p.p.phys_param.lane_length/2.#(length(p.p.phys_param.POSITIONS) + 1)/2 #row
  #accelerate opposite of the sign of him - me
  if p.jerk
    accel = 1
  else
    accel = -sign(cs.pos[1] - x)
  end
  #lanechange opposite of sign of him - me
  lanechange = -sign(cs.pos[2] - s.agent_pos)
  #if same lane, go in direction with more room
  if lanechange == 0
    if p.p.nb_col - s.agent_pos >= s.agent_pos #more room to left >= biases to left
      lanechange = 1
    else
      lanechange = -1
    end
  end
  if lanechange > 0 && s.agent_pos >= p.p.nb_col
		lanechange = 0
	elseif lanechange < 0 && s.agent_pos <= 1
		lanechange = 0
	end
  #when initiating a lanechange, set in policy, else continue to do what they said
  # and reset lanechange bit
  return MLAction(accel,lanechange)
end
