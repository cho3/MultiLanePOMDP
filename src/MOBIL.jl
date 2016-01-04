#MOBIL.jl
#A separate file for all the MOBIL stuff

###############
##MOBIL Model##
###############

immutable MOBILParam
	p::Float64 #politeness factor
	b_safe::Float64 #safe braking value
	a_thr::Float64 #minimum accel
	#db::Float64 #lane bias #we follow the symmetric/USA lane change rule
	function MOBILParam(;p::Float64=0.25,b_safe::Float64=4.,a_thr::Float64=0.2)
		self = new()
		self.p = p
		self.b_safe = b_safe
		self.a_thr = a_thr
		return self
	end
end #MOBILParam
==(a::MOBILParam,b::MOBILParam) = (a.p==b.p) && (a.b_safe==b.b_safe) &&(a.a_thr == b.a_thr)
Base.hash(a::MOBILParam,h::Uint64=zero(Uint64)) = hash(a.p,hash(a.b_safe,hash(a.a_thr,h)))

function MOBILParam(s::AbstractString)
	typedict = ["cautious"=>0.5,"normal"=>0.25,"aggressive"=>0.0]
	p = get(typedict,s,-1.)
	assert(p >= 0.)
	return MOBILParam(p=p)
end


##No better place to put this :(
immutable BehaviorModel
	p_idm::IDMParam
	p_mobil::MOBILParam
	rationality::Float64 
end
==(a::BehaviorModel,b::BehaviorModel) = (a.p_idm==b.p_idm) && (a.p_mobil==b.p_mobil) &&(a.rationality == b.rationality)
Base.hash(a::BehaviorModel,h::Uint64=zero(Uint64)) = hash(a.p_idm,hash(a.p_mobil,hash(a.rationality,h)))

function BehaviorModel(s::AbstractString,v0::Float64,s0::Float64)
	typedict = ["cautious"=>0.95,"normal"=>0.9,"aggressive"=>0.85] #rationality
	return BehaviorModel(IDMParam(s,v0,s0),MOBILParam(s),typedict[s])
end

type CarState
	pos::Tuple{Int,Int} #row, col
	vel::Int
	lane_change::Int #-1,0, or +1, corresponding to to the right lane, no lane change, or to the left lane
	behavior::BehaviorModel
end #carstate
==(a::CarState,b::CarState) = (a.pos==b.pos) && (a.vel==b.vel) &&(a.lane_change == b.lane_change)&&(a.behavior==b.behavior)
Base.hash(a::CarState,h::Uint64=zero(Uint64)) = hash(a.vel,hash(a.pos,hash(a.lane_change,hash(a.behavior,h))))

type CarNeighborhood
	ahead_dist::Dict{Int,Float64} #(-1,0,1) for right,self, and left lane
	behind_dist::Dict{Int,Float64}
	ahead_dv::Dict{Int,Float64}
	behind_dv::Dict{Int,Float64}
	ahead_idm::Dict{Int,IDMParam}
	behind_idm::Dict{Int,IDMParam}
end
CarNeighborhood(x,y) = CarNeighborhood(x,x,x,x,y,y)
CarNeighborhood() = CarNeighborhood(Dict{Int,Float64}(),Dict{Int,IDMParam}())


function get_adj_cars(p::MLPOMDP,arr::Array{CarState,1},x::CarState)
	##TODO: update CarNeighborhood with stuff for the IDM parameters
	neighborhood = CarNeighborhood()

	for car in arr
		pos = car.pos
		dlane = pos[2]-x.pos[2]
		if abs(dlane) > 1 #not in or adjacent to current lane
			break
		end
		dist = POSITIONS[pos[1]]-POSITIONS[x.pos[1]]
		dv = VELOCITIES[vel]-VELOCITIES[x.vel] #derivative of dist
		if dist > 0 && dist - l_car < get(neighborhood.ahead_dist,dlane,1000.)
			neighborhood.ahead_dist[dlane] = dist - l_car
			neighborhood.ahead_dv[dlane] = dv
			neighborhood.ahead_idm[dlane] = car.behavior.p_idm
		elseif dist < 0 && -1*dist - l_car < get(neighborhood.behind_dist,dlane,1000.)
			neighborhood.behind_dist[dlane] = -1*dist - l_car
			neighborhood.behind_dv[dlane] = -1*dv
			neighborhood.behind_idm[dlane] = car.behavior.p_idm
		end
	end
	
	return neighborhood #PLACEHOLDER
	
end

function get_mobil_lane_change(p::PhysicalParam,state::CarState,neighorhood::CarNeighborhood)

	#need 6 distances: distance to person behind me, ahead of me
	#				potential distance to person behind me, ahead of me
	#				in other lane(s)
	#need sets of idm parameters
	dt = p.dt
	p_idm_self = state.behavior.p_idm
	p_mobil = state.behavior.p_mobil
	
	if isempty(neighborhood.ahead_dv) && isempty(neighborhood.behind_dv)
		return 0 #no reason to change lanes if you're all alone
	end
	
	##if between lanes, return +1 if moving left, -1 if moving right
	if state.pos[2] % 2 == 0 #even is between lanes
		return state.lane_change #continue going in the direction you're going
	end
	
	v = VELOCITIES[state.vel]
	#get_idm_dv(param,velocity,dv,s)
	#get predicted and potential accelerations
	a_self = get_idm_dv(p_idm_self,dt,v,get(neighborhood.ahead_dv,0,0.),get(neighborhood.ahead_dist,0,1000.))/dt
	a_self_left = get_idm_dv(p_idm_self,dt,v,get(neighborhood.ahead_dv,1,0.),get(neighborhood.ahead_dist,1,1000.))/dt
	a_self_right = get_idm_dv(p_idm_self,dt,v,get(neighborhood.ahead_dv,-1,0.),get(neighborhood.ahead_dist,-1,1000.))/dt
	
	a_follower = get_idm_dv(neighborhood.rear_idm[0],dt,v,get(neighborhood.rear_dv,0,0.),get(neighborhood.rear_dist,0,-1000.))/dt #distance behind is a negative number
	a_follower_left = get_idm_dv(neighborhood.rear_idm[1],dt,v,get(neighborhood.rear_dv,1,0.)+get(neighborhood.ahead_dv,1,0.),get(neighborhood.rear_dist,1,1000.)+get(neighborhood.ahead_dist,1,1000.)+l_car)/dt
	a_follower_right = get_idm_dv(neighborhood.rear_idm[-1],dt,v,get(neighborhood.rear_dv,-1,0.)+get(neighborhood.ahead_dv,-1,0.),get(neighborhood.rear_dist,-1,1000.)+get(neighborhood.ahead_dist,-1,1000.)+l_car)/dt
	
	a_follower_ = get_idm_dv(neighborhood.rear_idm[0],dt,v,get(neighborhood.rear_dv,0,0.)+get(neighborhood.ahead_dv,0,0.),get(neighborhood.rear_dist,0,-1000.)+get(neighborhood.ahead_dist,0,1000.)+l_car)/dt
	a_follower_left_ = get_idm_dv(neighborhood.rear_idm[1],dt,v,get(neighborhood.rear_dv,1,0.),get(neighborhood.rear_dist,1,1000.))/dt
	a_follower_right_ = get_idm_dv(neighborhood.rear_idm[-1],dt,v,get(neighborhood.rear_dv,-1,0.),get(neighborhood.rear_dist,-1,1000.))/dt
	
	#calculate incentives
	left_crit = a_self_left-a_self+p_mobil.p*(a_follower_left_-a_follower_left+a_follower_-a_follower)
	right_crit = a_self_right-a_self+p_mobil.p*(a_follower_right_-a_follower_right+a_follower_-a_follower)
	
	#check safety criterion, also check if there is physically space
	if (a_follower_right_ < -p_mobil.b_safe) && (a_follower_left_ < -p_mobil.b_safe)
		return 0 #neither safe
	elseif (a_follower_left_ < -p_mobil.b_safe) || (neighorhood.rear_dist[1] < 0.) || (neighorhood.ahead_dist[1] < 0.)
		left_crit = -100.
	elseif (a_follower_right_ < -p_mobil.b_safe) || (neighorhood.rear_dist[-1] < 0.) || (neighorhood.ahead_dist[-1] < 0.)
		right_crit = -100.
	end
	
	#check if going left or right is preferable
	dir_flag = left_crit >= right_crit ? 1:-1
	#check incentive criterion
	if max(left_crit,right_crit) > p_mobil.a_thr
		return dir_flag
	end
	return 0
end

