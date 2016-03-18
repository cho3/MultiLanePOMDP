#IDM.jl
#A separate file for all the IDM stuff

#######################
##Physical Parameters##
#######################
##TODO: this should probably be dumped into a single type
type PhysicalParam
	dt::Float64
	w_car::Float64
	l_car::Float64
	v_nominal::Float64
	w_lane::Float64
	y_interval::Float64
	x_interval::Float64
	v_fast::Float64
	v_slow::Float64
	v_med::Float64
	v_max::Float64
	v_min::Float64
	nb_vel_bins::Int
	nb_env_cars::Int
	lane_length::Float64
	NB_DIR::Int
	NB_POS::Int
	VELOCITIES::Array{Float64,1}
	POSITIONS::Array{Float64,1}
	function PhysicalParam(nb_lanes::Int;dt::Float64=0.75,
							w_car::Float64=2.,
							l_car::Float64=4.,
							v_nominal::Float64=31.,
							w_lane::Float64=4.,
							y_interval::Float64=2.,
							x_interval::Float64=0.25,
							v_fast::Float64=35.,
							v_slow::Float64=27.,
							v_med::Float64=31.,
							nb_vel_bins::Int=100,
							nb_env_cars::Int=1,
							lane_length::Float64=12.,
							NB_DIR::Int=3,
							v_max::Float64=v_fast+0.,
							v_min::Float64=v_slow-0.)

		assert(v_max >= v_fast)
		assert(v_fast >= v_med)
		assert(v_med >= v_slow)
		assert(v_slow >= v_min)
		self = new()
		self.dt = dt
		self.w_car = w_car
		self.l_car = l_car
		self.v_nominal = v_nominal
		self.w_lane = w_lane
		self.y_interval = y_interval
		self.x_interval = x_interval
		self.v_fast = v_fast
		self.v_slow = v_slow
		self.v_med = v_med
		self.v_max = v_max
		self.v_min = v_min
		self.nb_vel_bins = nb_vel_bins
		self.nb_env_cars = nb_env_cars
		self.lane_length = lane_length
		self.NB_DIR = NB_DIR
		self.NB_POS = convert(Int,round((lane_length/x_interval)*((w_lane/y_interval)*nb_lanes-1))) #2*nb_lane-1 ostensibly
		self.VELOCITIES = collect(linspace(v_slow,v_fast,nb_vel_bins))
		####NOTE: THIS IS GETTING CHANGED tO THE NEXT LINE I PRAY TO GOD IT DOESN'T EXPLODE
		#self.POSITIONS = collect(linspace(-lane_length/2,lane_length/2,convert(Int,round(lane_length/x_interval))))
		self.POSITIONS = collect(linspace(0.,lane_length,convert(Int,round(lane_length/x_interval))))


		return self
		end
end

#############
##IDM Model##
#############

type IDMParam
	a::Float64 #max  comfy acceleration
	b::Float64 #max comfy brake speed
	T::Float64 #desired safety time headway
	v0::Float64 #desired speed
	s0::Float64 #minimum headway (e.g. if x is less than this then you crashed)
	del::Float64 #'accel exponent'
end #IDMParam
==(a::IDMParam,b::IDMParam) = (a.a==b.a) && (a.b==b.b) &&(a.T == b.T)&&(a.v0==b.v0)&&(a.s0==b.s0)&&(a.del==b.del)
Base.hash(a::IDMParam,h::UInt64=zero(UInt64)) = hash(a.a,hash(a.b,hash(a.T,hash(a.v0,hash(a.s0,hash(a.del,h))))))


IDMParam(a::Float64,b::Float64,T::Float64,v0::Float64,s0::Float64;del::Float64=4.) = IDMParam(a,b,T,v0,s0,del)
function build_cautious_idm(v0::Float64,s0::Float64)
	T = 2.
	a = 1.
	b = 1.
	return IDMParam(a,b,T,v0,s0)
end

function build_aggressive_idm(v0::Float64,s0::Float64)
	T = 0.8
	a = 2.
	b = 2.
	return IDMParam(a,b,T,v0,s0)
end

function build_normal_idm(v0::Float64,s0::Float64)
	T = 1.4
	a = 1.5
	b = 1.5
	return IDMParam(a,b,T,v0,s0)
end

#TODO: use Enum or something to avoid misspelling errors
function IDMParam(s::AbstractString,v0::Float64,s0::Float64)
	if lowercase(s) == "cautious"
		return build_cautious_idm(v0,s0)
	elseif lowercase(s) == "normal"
		return build_normal_idm(v0,s0)
	elseif lowercase(s) == "aggressive"
		return build_aggressive_idm(v0,s0)
	else
		error("No such idm phenotype: \"$(s)\"")
	end
end

function get_idm_dv(p::IDMParam,dt::Float64,v::Float64,dv::Float64,s::Float64)
	s_ = p.s0 + max(0.,v*p.T+v*dv/(2*sqrt(p.a*p.b)))
	dvdt = p.a*(1.-(v/p.v0)^p.del - (s_/s)^2)
	#dvdt = min(max(dvdt,-p.b),p.a)
	return dvdt*dt
end #get_idm_dv
