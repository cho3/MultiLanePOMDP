#simulator.jl
#basically a place to hold next, and to call a sequence, get relevant statistics, and visualize.

function rand!(rng::AbstractRNG,s::MLState,d::MLStateDistribution)

end

function rand!(rng::AbstractRNG,o::MLObs,d::MLObsDistribution)

end

function next(rng::AbstractRNG,s::MLState,a::MLAction)

end

function observe(rng::AbstractRNG,s::MLState,a::MLAction)

end

function simulate()

end