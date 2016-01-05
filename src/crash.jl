#crash.jl
#a separate file for all the crashing stuff

function cross2d(x::Array{Float64,1},y::Array{Float64,1})
	assert(length(x) == length(y))
	if length(x) == 3
		return cross(x,y)
	elseif length(x) != 2
		error("Incorrect length for cross product")
	end
	
	return x[1]*y[2]-x[2]*y[1]
end

function line_segment_intersect(p::Array{Float64,1},pr::Array{Float64,1},q::Array{Float64,1},qs::Array{Float64,1})
	if (length(p) != length(q)) || (length(p) != length(pr)) || (length(q) != length(qs))
		error("Error: inconsistent dimensionality")
	end
	
	s = qs - q
	r = pr - p
	
	rxs = cross2d(r,s)
	q_pxr = cross2d(q-p,r)
	if rxs == 0
		if q_pxr == 0
			#collinear
			t0 = dot(q-p,r)/dot(r,r)
			t1 = t0 + dot(s,r)/dot(r,r)
			if dot(r,s) > 0
				if max(0.,min(t1,1.)-max(0.,t0)) > 0
					return true
				end
				return false
			else
				if max(0.,min(t0,1.)-max(0.,t1)) > 0
					return true
				end
				return false
			end
		else
			return false
		end
	end
	
	t = cross2d(q-p,s)/cross2d(r,s)
	u = cross2d(q-p,r)/cross2d(r,s)
	
	if (max(0.,min(1.,t)) == t) && (max(0.,min(1.,u)) == u)
		return true
	end
		
	return false
end

function poly_intersect(X::Array{Array{Float64,2},1},Y::Array{Array{Float64,2},1})
	#each row corresponds to x, y, z....
	#need i,i+1 to be each line segment
	#does pairwise comparison of line segments in O(n2) time
	##TODO: still need to check the cases of being completely inside of one another	
	
	for x in X
		for y in Y
			if size(x) != size(y)
				error("Error: ill formed input1")
			end
			if size(x)[2] > 2
				error("Error: ill formed input2")
			end
			if line_segment_intersect(x[:,1],x[:,2],y[:,1],y[:,2])
				return true
			end
		end
	end
	
	"""
	for i in 1:(size(X)[2]-1)
		for k in 1:(size(Y)[2]-1)
			j = (i == size(X)[2])? 1: i+1
			l = (k == size(Y)[2])? 1: k+1
			if line_segment_intersect(X[:,i],X[:,j],Y[:,k],Y[:,l])
				return true
			end
		end
	end
	"""
	return false
end

function is_crash(pomdp::MLPOMDP,s::MLState,a::MLAction)
	#calculate current position, next position, convert to metric space
	#convert to polyhedron based on car size
	#do collision check between agent car and all environment cars
	
	agent_pos_ind = (1+pomdp.col_length)/2
	if round(agent_pos_ind) != agent_pos_ind
		agent_pos = (POSITIONS[convert(Int,round(ceil(agent_pos_ind)))]+POSITIONS[convert(Int,round(floor(agent_pos_ind)))])/2
	else
		agent_pos = POSITIONS[convert(Int,round(agent_pos_ind))]
	end
	agent_y = s.agent_lane*y_interval
	
	#treat agent_pos, agent_y as (0,0)
	#X = Array{Float64,2}[agent_pos agent_pos+l_car agent_pos+l_car agent_pos agent_pos; agent_y agent_y agent_y+w_car agent_y+w_car agent_y]
	X = Array{Float64,2}[[agent_pos agent_pos; agent_y agent_y+w_car];[agent_pos+l_car agent_pos+l_car; agent_y agent_y+w_car];
						[agent_pos agent_pos+l_car; agent_y agent_y];[agent_pos agent_pos+l_car; agent_y+w_car agent_y+w_car]]
	
	for env_car in s.env_cars
		pos = env_car.pos
		vel = env_car.vel
		lane_change = env_car.lane_change
		behavior = env_car.behavior
		lane_ = max(1,min(pos[2]+lane_change,pomdp.nb_col))
		neighborhood = get_adj_cars(pomdp,s.env_cars,env_car)
		
		dvel_ms = get_idm_dv(behavior.p_idm,VELOCITIES[vel],neighborhood.ahead_dv[0],neighborhood.ahead_dist[0]) #call idm model
		dp =  dt*(VELOCITIES[vel]-VELOCITIES[a.vel])+0.5*dt*dvel_ms #x+vt+1/2at2
		dy = (lane_-pos[2])*y_interval
		p = POSITIONS[pos]
		y = pos[2]*y_interval
		##TODO: make consistent with new formulation
		Y1 = Array{Float64,2}[[p p; y y+w_car];[p+l_car p+l_car; y y+w_car];[p p+l_car; y y];[p p+l_car; y+w_car y+w_car]]
		Y2 = Array{Float64,2}[xy + [dp dp; dy dy] for xy in Y1]
		Y3 = Array{Float64,2}[[p p+dp; y y+dy];[p+l_car p+dp+l_car; y y+dy];[p p+dp; y+w_car y+dy+w_car];[p+l_car p+dp+l_car; y+w_car y+dy+w_car]]
		
		Y = Array{Float64,2}[Y1;Y2;Y3]
		if poly_intersect(X,Y)
			return true
		end
	end
	
	return false
end