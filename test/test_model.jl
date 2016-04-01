##POMDP Model
include(joinpath("..","src","ML_model.jl"))

function test_observations()
	##Currently generating 3672 vs 3456 calculated
	println("\t\tTesting Observation Space Creation")
	nb_lanes = 2
	pp = PhysicalParam(nb_lanes,nb_vel_bins=4,lane_length=2.) #2.=>col_length=8
	p = MLPOMDP(nb_cars=1,nb_lanes=nb_lanes,phys_param=pp)
	O = observations(p)
	assert(length(O) == n_observations(p))
	#assuming == and hash works, then everything should be distinct
	assert(length(unique(O)) == length(O))
end

function test_states()
	##Currently generating 33840 vs 31104 generated
	println("\t\tTesting State Space Creation")
	nb_lanes = 2
	pp = PhysicalParam(nb_lanes,nb_vel_bins=4,lane_length=2.) #2.=>col_length=8
	p = MLPOMDP(nb_cars=1,nb_lanes=nb_lanes,phys_param=pp)
	S = states(p)
	assert(length(S) == n_states(p))
	#assuming == and hash works, then everything should be distinct
	assert(length(unique(S)) == length(S))
end

function test_reward()
	println("\t\tTesting Reward Model")
	nb_lanes = 2
	pp = PhysicalParam(nb_lanes,nb_vel_bins=4,lane_length=12.) #2.=>col_length=8
	p = MLPOMDP(nb_cars=1,nb_lanes=nb_lanes,phys_param=pp)

	bs = BehaviorModel[BehaviorModel(x[1],x[2],x[3],idx) for (idx,x) in enumerate(product(["cautious","normal","aggressive"],[pp.v_slow;pp.v_med;pp.v_fast],[pp.l_car]))]

	#Env Car out of bounds
	cs_oob = CarState((-1.,1),27.,0,bs[1])
	#env car going slow in right lane: (41,1) corresponds to right in front of agent car area + 0.25m or 0.5m
	cs_r_slow = CarState((10.,1),27.,0,BehaviorModel("aggressive",27.,4.,1))
	#env car going fast in right lane: (7,1) corresponds to right behind the agent car area + 0.25m or 0.5m
	cs_r_fast = CarState((1.75,1),35.,0,BehaviorModel("aggressive",35.,4.,1))
	#env car going at a medium speed in the right lane heading left
	cs_lchange = CarState((6.,1),30.,1,BehaviorModel("normal",31.,4.,1))
	#env car going at a medium speed in the left lane heading right
	cs_rchange = CarState((6.,2),30.,-1,BehaviorModel("normal",31.,4.,1))
	#env car is just chilling in the right/btwn/lhigh lane
	cs_rchill = CarState((6.,1),30.,1,BehaviorModel("normal",31.,4.,1))
	cs_mchill = CarState((6.,2),30.,1,BehaviorModel("normal",31.,4.,1))
	cs_hchill = CarState((7.,1),30.,1,BehaviorModel("normal",31.,4.,1))

	#CASE: do nothing = no costs
	assert(reward(p,MLState(1,27.,CarState[cs_oob]),MLAction(0,0)) == 0.)
	#CASE: moving = cost
	assert(reward(p,MLState(1,27.,CarState[cs_oob]),MLAction(1,0)) == p.accel_cost)
	assert(reward(p,MLState(1,27.,CarState[cs_oob]),MLAction(-1,0)) == p.decel_cost)
	assert(reward(p,MLState(1,27.,CarState[cs_oob]),MLAction(0,1)) == p.lanechange_cost)
	assert(reward(p,MLState(1,27.,CarState[cs_oob]),MLAction(1,1)) == p.accel_cost + p.lanechange_cost)
	assert(reward(p,MLState(1,27.,CarState[cs_oob]),MLAction(-1,1)) == p.decel_cost + p.lanechange_cost)
	assert(reward(p,MLState(1,27.,CarState[cs_oob]),MLAction(1,-1)), p.accel_cost + p.lanechange_cost  + p.invalid_cost)
	assert(reward(p,MLState(1,27.,CarState[cs_oob]),MLAction(-1,-1)) == p.decel_cost + p.lanechange_cost+p.invalid_cost)
	assert(reward(p,MLState(1,27.,CarState[cs_oob]),MLAction(0,-1)) == p.lanechange_cost + p.invalid_cost)

	#Case: cars occupy same space
	assert(reward(p,MLState(1,30.,CarState[cs_rchill]),MLAction(0,0)) == p.r_crash)
	assert(reward(p,MLState(1,30.,CarState[cs_mchill]),MLAction(0,0)) == p.r_crash)
	assert(reward(p,MLState(1,30.,CarState[cs_hchill]),MLAction(0,0)) == p.r_crash)
	#CASE: cars intersect; vertically (gets railroaded from behind)
	assert(reward(p,MLState(1,27.,CarState[cs_r_fast]),MLAction(-1,0)) == p.r_crash)
	assert(reward(p,MLState(1,35.,CarState[cs_r_slow]),MLAction(1,0)) == p.r_crash)
	#CASE: cars intersect; horizontally (across lanes)
	assert(reward(p,MLState(1,30.,CarState[cs_rchange]),MLAction(0,1)) == p.r_crash)
	assert(reward(p,MLState(3,30.,CarState[cs_lchange]),MLAction(0,-1)) == p.r_crash)
	#CASE: diagonal intersection
	#CASE: cars will pass through one another during the next time step
end

function test_actions()
	println("\t\tTesting Action Space Creation")
	p = MLPOMDP()
	A = actions(p)
	bs = BehaviorModel[BehaviorModel(x[1],x[2],x[3],idx) for (idx,x) in enumerate(product(["cautious","normal","aggressive"],[27.,31.,35.],[4.]))]
	s = MLState(2,5.,CarState[CarState((1.,1),3.,0,bs[1])])
	A_ = actions(p,s) #check the other signature
	assert(domain(A_) == domain(A))
end

function test_rev_1d_interp()
	println("\t\tTesting rev_1d_interp")
	arr = Float64[10.;20.;30.;40.;50.]
	#case: collision
	assert(rev_1d_interp(arr,10.,1.),Dict{Int,Float64}(1=>1.))
	assert(rev_1d_interp(arr,20.,1.),Dict{Int,Float64}(2=>1.))
	assert(rev_1d_interp(arr,30.,1.),Dict{Int,Float64}(3=>1.))
	#case: oob
	assert(rev_1d_interp(arr,0.,1.),Dict{Int,Float64}(1=>1.))
	assert(rev_1d_interp(arr,60.,1.),Dict{Int,Float64}(5=>1.))
	#case: in between
	assert(rev_1d_interp(arr,15.,1.),Dict{Int,Float64}(1=>0.5,2=>0.5))
	assert(rev_1d_interp(arr,25.,1.),Dict{Int,Float64}(3=>0.5,2=>0.5))
	assert(rev_1d_interp(arr,37.,1.),Dict{Int,Float64}(4=>0.7,3=>0.3))
end

function test_transition()
	println("\t\tTesting Transition Model")

	nb_lanes = 2
	pp = PhysicalParam(nb_lanes,nb_vel_bins=5,lane_length=12.) #2.=>col_length=8
	p = MLPOMDP(nb_cars=1,nb_lanes=nb_lanes,phys_param=pp)

	bs = BehaviorModel[BehaviorModel(x[1],x[2],x[3],idx) for (idx,x) in enumerate(product(["cautious","normal","aggressive"],[pp.v_slow;pp.v_med;pp.v_fast],[pp.l_car]))]

	#Env Car out of bounds
	cs_oob = CarState((0,1),1,0,bs[1])
	#env car going slow in right lane: (41,1) corresponds to right in front of agent car area + 0.25m or 0.5m
	cs_l_slow = CarState((3,1),1,0,BehaviorModel("aggressive",27.,4.,1))
	#env car going fast in right lane: (7,1) corresponds to right behind the agent car area + 0.25m or 0.5m
	cs_r_fast = CarState((48,1),5,0,BehaviorModel("aggressive",35.,4.,1))
	#env car going at a medium speed in the right lane heading left
	cs_lchange = CarState((24,1),2,1,BehaviorModel("normal",31.,4.,1))
	#env car going at a medium speed in the left lane heading right
	cs_rchange = CarState((24,2),2,-1,BehaviorModel("normal",31.,4.,1))
	#env car is just chilling in the right/btwn/lhigh lane
	cs_rchill = CarState((24,1),3,1,BehaviorModel("normal",31.,4.,1))
	cs_mchill = CarState((24,2),2,1,BehaviorModel("normal",31.,4.,1))
	cs_hchill = CarState((28,1),2,1,BehaviorModel("normal",31.,4.,1))

	#CASE: test that env car goes out of bounds at top boundary
	s = MLState(3,1,CarState[cs_r_fast])
	a = MLAction(0,-1)
	d = transition(p,s,a)
	assert(length(d.d),1)
	s_ = collect(keys(d.d))[1]
	assert(s_.agent_pos,2)
	assert(s_.agent_vel,1)
	assert(s_.env_cars[1].pos[1],0)
	#CASE: test that env car goes out of bounds at bottom boundary
	s = MLState(1,5,CarState[cs_l_slow])
	a = MLAction(0,1)
	d = transition(p,s,a)
	assert(length(d.d),1)
	s_ = collect(keys(d.d))[1]
	assert(s_.agent_pos,2)
	assert(s_.agent_vel,5)
	assert(s_.env_cars[1].pos[1],0)
	#More test cases to check that the actions do what they're supposed to do
	#moving left when there's nothing
	s = MLState(3,3,CarState[cs_l_slow])
	a = MLAction(0,1)
	d = transition(p,s,a)
	s_ = collect(keys(d.d))[1]
	assert(s_.agent_pos,3)
	assert(s_.agent_vel,3)
	#moving right when you're in the rightmost position
	s = MLState(1,3,CarState[cs_l_slow])
	a = MLAction(0,-1)
	d = transition(p,s,a)
	s_ = collect(keys(d.d))[1]
	assert(s_.agent_pos,1)
	assert(s_.agent_vel,3)
	#speeding up
	s = MLState(2,3,CarState[cs_l_slow])
	a = MLAction(1,0)
	d = transition(p,s,a)
	s_ = collect(keys(d.d))[1]
	assert(s_.agent_pos,2)
	assert(s_.agent_vel,4)
	#slowing down
	s = MLState(2,3,CarState[cs_l_slow])
	a = MLAction(-1,0)
	d = transition(p,s,a)
	s_ = collect(keys(d.d))[1]
	assert(s_.agent_pos,2)
	assert(s_.agent_vel,3) #2
	#speeding up at fastest speed
	s = MLState(2,5,CarState[cs_l_slow])
	a = MLAction(1,0)
	d = transition(p,s,a)
	s_ = collect(keys(d.d))[1]
	assert(s_.agent_pos,2)
	assert(s_.agent_vel,5)
	#slowing down at slowest speed
	s = MLState(2,1,CarState[cs_l_slow])
	a = MLAction(-1,0)
	d = transition(p,s,a)
	s_ = collect(keys(d.d))[1]
	assert(s_.agent_pos,2)
	assert(s_.agent_vel,1)
	#CASE: test basic encounter model  for slow
	s = MLState(2,1,CarState[cs_oob])
	a = MLAction(0,0)
	d = transition(p,s,a)
	ss_ = collect(keys(d.d))
	for s in ss_
		car = s.env_cars[1]
		if car.pos[1] == p.col_length
			assert(car.vel,s.agent_vel,<=)
		elseif car.pos[1] == 1
			assert(car.vel,s.agent_vel,>=)
		elseif car.pos[1] != 0
			println(car)
			error("Transition failed: encounter model is summoning cars in the middle")
		end
	end
	#CASE: test basic encounter model for fast
	s = MLState(2,1,CarState[cs_oob])
	a = MLAction(0,0)
	d = transition(p,s,a)
	ss_ = collect(keys(d.d))
	s = MLState(2,1,CarState[cs_oob])
	a = MLAction(0,0)
	d = transition(p,s,a)
	ss_ = collect(keys(d.d))
	for s in ss_
		car = s.env_cars[1]
		if car.pos[1] == p.col_length
			assert(car.vel,s.agent_vel,<=)
		elseif car.pos[1] == 1
			assert(car.vel,s.agent_vel,>=)
		elseif car.pos[1] != 0
			println(car)
			error("Transition failed: encounter model is summoning cars in the middle")
		end
	end
	#CASE: normal evolution (call idm model)
	s = MLState(3,3,CarState[cs_rchill])
	a = MLAction(0,0)
	d = transition(p,s,a)
	#something
end

import Distributions: Normal, cdf

function test_normal_pdf()
	println("\t\tTesting normal_pdf for observation model")
	VELOCITIES = collect(linspace(27.,35.,5))
	#make sure tail behavior is right
	assert(pdf(VELOCITIES,1,1) > 0.5)
	assert(pdf(VELOCITIES,5,5) > 0.5)
	#smake sure things sum to one, more or less
	assert(abs(sum([pdf(VELOCITIES,i,3) for i=1:5])- 1.) < 0.00001)
	assert(abs(sum([pdf(VELOCITIES,i,5) for i=1:5])- 1.) < 0.00001)
	assert(abs(sum([pdf(VELOCITIES,i,1) for i=1:5])- 1.) < 0.00001)
	#normal case?
end

function test_observe()
	println("\t\tTesting Observation Model")

	nb_lanes = 2
	pp = PhysicalParam(nb_lanes,nb_vel_bins=5,lane_length=12.) #2.=>col_length=8
	p = MLPOMDP(nb_cars=1,nb_lanes=nb_lanes,phys_param=pp)

	bs = BehaviorModel[BehaviorModel(x[1],x[2],x[3],idx) for (idx,x) in enumerate(product(["cautious","normal","aggressive"],[pp.v_slow;pp.v_med;pp.v_fast],[pp.l_car]))]

	#Env Car out of bounds
	cs_oob = CarState((0,1),1,0,bs[1])
	#env car going slow in right lane: (41,1) corresponds to right in front of agent car area + 0.25m or 0.5m
	cs_r_slow = CarState((40,1),1,0,BehaviorModel("aggressive",27.,4.,1))
	#env car going fast in right lane: (7,1) corresponds to right behind the agent car area + 0.25m or 0.5m
	cs_r_fast = CarState((7,1),4,0,BehaviorModel("aggressive",35.,4.,1))
	#env car going at a medium speed in the right lane heading left
	cs_lchange = CarState((24,1),2,1,BehaviorModel("normal",31.,4.,1))
	#env car going at a medium speed in the left lane heading right
	cs_rchange = CarState((24,2),2,-1,BehaviorModel("normal",31.,4.,1))
	#env car is just chilling in the right/btwn/lhigh lane
	cs_rchill = CarState((24,1),3,1,BehaviorModel("normal",31.,4.,1))
	cs_mchill = CarState((24,2),2,1,BehaviorModel("normal",31.,4.,1))
	cs_hchill = CarState((28,1),2,1,BehaviorModel("normal",31.,4.,1))

	#CASE: no cars
	s = MLState(1,1,CarState[cs_oob])
	a = MLAction(0,0)
	d = observation(p,s,a)
	assert(length(d.d),1) #only truth
	o = collect(keys(d.d))[1]
	assert(o.agent_pos,1)
	assert(o.agent_vel,1)
	#CASE: one car doing whatever
	s = MLState(3,4,CarState[cs_rchill])
	a = MLAction(1,1)
	d = observation(p,s,a)
	assert(length(d.d),5)
	for (o,prob) in d.d
		assert(o.agent_pos,3)
		assert(o.agent_vel,4)
		assert(prob > 0.)
		assert(prob <= 1.)
	end

end

##TODO: indexing and other helper functions testing

function test_pomdp_model()
	println("\tTesting POMDP Model Units...")
	#test_states()
	#test_observations()
	test_actions()
	test_reward()
	#test_rev_1d_interp()
	#test_transition()
	#test_normal_pdf()
	#test_observe()
end
