#simulator.jl
#basically a place to hold next, and to call a sequence, get relevant statistics, and visualize.

import StatsBase: WeightVec, sample

function rand!(rng::AbstractRNG,s::MLState,d::MLStateDistribution)
  states = MLState[]
  probs = Float64[]
  for (state,prob) in d.d
    push!(states,state)
    push!(probs,prob)
  end
  s = sample(states,WeightVec(probs))
  return s
end

function rand!(rng::AbstractRNG,o::MLObs,d::MLObsDistribution)
  obss = MLState[]
  probs = Float64[]
  for (obs,prob) in d.d
    push!(obss,obs)
    push!(probs,prob)
  end
  o = sample(obss,WeightVec(probs))
  return o
end

function next(rng::AbstractRNG,s::MLState,a::MLAction)

end

function observe(rng::AbstractRNG,s::MLState,a::MLAction)

end

function init(pomdp::MLPOMDP;rng::AbstractRNG=MersenneTwister(34985))
  return MLState(1,convert(Int,round(length(pomdp.phys_parma.nb_vel_bins))),
                CarState[CarState((0,1),1,0,pomdp.BEHAVIORs[1]) for i = 1:pomdp.nb_cars])
end

function simulate(pomdp::MLPOMDP,policy::Function;
          discount::Float64=0.99,
          nb_timesteps::Int=20,
          simRNG::AbstractRNG=MersenneTwister(1336),
          polRNG::AbstractRNG=MersenneTwisteR(53487523))
    #init
    state_hist = Array{MLState,1}[]
    action_hist = Array{MLAction,1}[]
    gamma = 1.
    R = 0.
    s = init(pomdp,rng=simRNG) #function does not yet exist
    for t = 1:nb_timesteps
      push!(state_hist,s)
      push!(action_hist,a)
      a = policy(s,rng=polRNG)
      R += gamma*reward(s,a)
      d = transition(pomdp,s,a)
      s = rand!(pomdp,s,d)
      #s = next(simRNG,s,a)
      if isterminal(pomdp,s,a)
        break
      end
      gamma *= discount
    end
    return R,state_hist,action_hist
end

using Interact
function display_sim(pomdp::MLPOMDP,S::Array{MLState,1},A::Array{MLAction,1})
  warn("This should be run in a Jupyter Notebook")
  assert(length(S) == length(A))

  @manipulate for i = 1:length(S)
    visualize(pomdp,S[i],A[i])
  end
end
