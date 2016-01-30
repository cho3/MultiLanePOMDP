#features.jl
#just features for use with function approximation reinforcement learning


#TODO: make things as functions of MLObs
function is_fastcarahead(s::MLState,a::MLAction)

end

function is_slowcarahead(s::MLState,a::MLAction)

end

"""
Potential features:
is_{slow,fast}{left,right,null}{ahead,behind}
is_clear{left,right}
"""

function tilecode(s::MLState,a::MLAction)

end

function bin(i::Int,rng::Int,nb_bins::Int)
  #NOTE: assume i \in [1...rng]
  if rng < nb_bins
    error("Number of bins is larger than domain")
  end
  if rng == nb_bins
    return i
  end
  inc = ceil(rng/nb_bins)
  bin = ifloor(i/inc)
  return 1 + bin #julia is 0 indexed
end

function generate_state_actionexplicit_featurefunction{T}(ff::Function,A::Array{T,1},nb_feat::Int)
  #ff returns a set of active indices,
  A_indices = Dict{T,Int}(a=>i for (i,a) in enumerate(A))
  function feature_function{S}(s::S,a::T)
    active_indices = ff(s)
    active_indices += nb_feat*(A_indices[a]-1)
    return sparsevec(active_indices,ones(length(active_indices),nb_feat*length(A_indices))
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

function generate_disjoint_featurefunction{T}(p::MLPOMDP,A::Array{T,1})
  nb_y_bins = p.nb_col
  nb_lanechange_bins = 3
  nb_x_bins = 0
  nb_vel_bins = 0

  #include +1 for own car
  nb_feat = (nb_y_bins + nb_lane_change_bins + nb_x_bins + nb_vel_bins)*(nb_cars + 1)
  #nb_feat += 9 #nb_actions
  function feature_function(s::MLState)
    active_indices = zeros(Int,4*p.nb_cars + 2)
    active_indices[1] = bin(s.agent_pos,p.col_length,nb_y_bins)
    feat_offset = nb_y_bins
    active_indices[2] = feat_offset + bin(s.agent_vel,p.pp.nb_vel_bins,nb_vel_bins)
    feat_offset += nb_vel_bins
    offset = 2
    for (i,car) in enumerate(s.env_cars)

      active_indices[offset + 1] = feat_offset + bin(car.pos[1],p.col_length,nb_x_bins)
      feat_offset += nb_x_bins
      active_indices[offset + 2] = feat_offset + bin(car.pos[2],p.nb_col,nb_y_bins)
      feat_offset += nb_y_bins
      active_indices[offset + 3] = feat_offset + bin(car.vel,p.pp.nb_vel_bins,nb_vel_bins)
      feat_offset += nb_vel_bins
      active_indices[offset + 4] = feat_offset + bin(car.lanechange,p.pp.NB_DIR,nb_lanechange_bins)
      feat_offset += nb_lanechange_bins
      offset += 4
    end

    assert(!(0 in active_indices))
    return active_indices
  end

  return generate_state_actionexplicit_featurefunction(feature_function,nb_feat,A)

end




"""
features
  sensor failed AND/OR
  agent_state = col, vel OR
  for each car:
    is_oob(car) OR
    car_state(car) = x,y,vel,lanechange

number of param:
  for ea. car:
    lanechange: 3 values TIMES
    x: |X| values TIMES
    y: |Y| values TIMES
    vel: |VEL| values PLUS
    is_oob: 1 bit
  agent_pos: |X| values TIMES
  agent_vel: |VEL| values
  action: |A| values
"""
