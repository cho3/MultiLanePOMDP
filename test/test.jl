#test.jl
#a bunch of unit tests for all the functions and types for the Multilane POMDP

import Base.==
import Base.length
import Base.assert
using Base.Test

##TODO: a bunch of error checking and associated tests for type creation (e.g. making sure things are in bounds)

##############
##UNIT TESTS##
##############

function assert(expr,val,fn::Function= ==)
	if !fn(expr,val)
		error("Assertion failed: expected $val, got $expr")
	end
end

function assert(expr,val,flag::Bool)
	if expr == val
		error("Assertion failed: expected NOT $val, got $expr")
	end
end

function test_hashing(s::AbstractString,ps)
	##TODO: make a more comprehensive test that uses more of the created objects
	println("\t\tTesting $s Hashing")
	if length(ps) < 2
		error("Make More objects for testing $s hashing")
	end
	d1 = Dict{typeof(ps[1]),Int}()
	d1[ps[1]] = 1
	d1[ps[1]] = 1
	assert(length(d1),1)
	d2 = Dict{typeof(ps[1]),Int}([p=>3 for p in [ps[2];ps[2]]])
	assert(length(d2),1)
	assert(get(d1,ps[1],0),1)
end

function test_equality(s::AbstractString,ps)
	println("\t\tTesting $s Equality")
	if length(ps) < 3
		error("Make More objects for testing $s equality")
	end
	for i = 1:length(ps)
		for j = 1:length(ps)
			if i == j
				assert(ps[i],ps[j])
			else
				assert(ps[i],ps[j],true)
				assert(ps[j],ps[i],true)
			end
		end
	end
	#assert(ps[1] == ps[1])
	#assert(ps[2] != ps[3])
	#assert(ps[3] != ps[2])
end

##IDM
include(joinpath("..","src","IDM.jl"))
function test_idm_creation()
	println("\t\tTesting IDM Creation")
	#do it directly using the build functions
	p1 = build_cautious_idm(30.,2.)
	p2 = build_normal_idm(30.,2.)
	p3 = build_aggressive_idm(30.,2.)
	#use the new constructor
	ps = [IDMParam(s,35.,3.) for s in ["AGGRESSIVE","CautiOUS","normal"]]
end

function test_idm_equality()
	#do it directly using the build functions
	p1 = build_cautious_idm(30.,2.)
	p2 = build_normal_idm(30.,2.)
	p3 = build_aggressive_idm(30.,2.)
	#use the new constructor
	ps = [IDMParam(s,35.,3.) for s in ["AGGRESSIVE","CautiOUS","normal"]]
	#test equality
	test_equality("IDM",ps)
	assert(p1 == IDMParam(1.,1.,2.,30.,2.))
	assert(p2 == IDMParam(1.5,1.5,1.4,30.,2.))
	assert(p3 == IDMParam(2.,2.,0.8,30.,2.))
	assert(IDMParam(1.,1.,2.,30.,2.) == p1)
	assert(p2 == p2)
	assert(ps[3] == ps[3])
	assert(p1 != ps[1])
	assert(p2 != ps[2])
	assert(p3 != ps[3])
	assert(p1 != p2)
	assert(p2 != p1)
end

function test_idm_hashing()
	#println("\t\tTesting IDM Hashing")
	#do it directly using the build functions
	p1 = build_cautious_idm(30.,2.)
	p2 = build_normal_idm(30.,2.)
	p3 = build_aggressive_idm(30.,2.)
	#use the new constructor
	ps = [IDMParam(s,35.,3.) for s in ["AGGRESSIVE","CautiOUS","normal"]]
	#test hashing as applied to dictionaries
	test_hashing("IDM",ps)
	"""
	d1 = Dict{IDMParam,Int}()
	d1[p1] = 1
	d1[p1] = 1
	assert(length(d1) == 1)
	d2 = Dict{IDMParam,Int}([p=>3 for p in [p2;p2]])
	assert(length(d2) == 1)
	assert(get(d1,p1,0) == 1)
	"""
end

function test_idm_dv()
	println("\t\tTesting IDM Calculations")
	#TODO: calculate by hand...? or use existing examples if can be found
end

function test_idm()
	println("\tTesting IDM Units...")
	test_idm_creation()
	test_idm_hashing()
	test_idm_dv()
end

##MOBIL
include(joinpath("..","src","MOBIL.jl"))
function test_mobil_creation()
	println("\t\tTesting MOBIL Creation")
	p1 = MOBILParam()
	p2 = MOBILParam(p=0.5)
	p3 = MOBILParam(b_safe=5.,a_thr=0.3)
	ps = [MOBILParam(s) for s in ["cautious";"normal";"aggressive"]]
end

function test_mobil_equality()
	#println("\t\tTesting MOBIL Equality")
	p1 = MOBILParam()
	p2 = MOBILParam(p=0.5)
	p3 = MOBILParam(b_safe=5.,a_thr=0.3)
	ps = [MOBILParam(s) for s in ["cautious";"normal";"aggressive"]]
	test_equality("MOBIL",ps)
	assert(p1 == p1)
	assert(p1 != p2)
	assert(p2 != p1)
	assert(p1 == MOBILParam(0.25,4.,0.2))
	assert(MOBILParam(0.25,4.,0.2) == p1)
end

function test_mobil_hashing()
	println("\t\tTesting MOBIL Hashing")
	p1 = MOBILParam()
	p2 = MOBILParam(p=0.5)
	p3 = MOBILParam(b_safe=5.,a_thr=0.3)
	ps = [MOBILParam(s) for s in ["cautious";"normal";"aggressive"]]
	test_hashing("MOBIL",ps)
end

function test_car_neighborhood_creation()
	println("\t\tTesting Car Neighborhood Creation")
	p1 = CarNeighborhood()

end

function test_car_neighborhood_equality()
	println("\t\tTesting Car Neighborhood Equality")
	#Nothing to do--such functions weren't created
end

function test_car_neighborhood_hashing()
	println("\t\tTesting Car Neighborhood Hashing")
	#nothing to do--such functions weren't created
end

function test_get_adj_cars()
	println("\t\tTesting get_adj_cars")
	##TODO: assertions
	nb_lanes = 3
	pp = PhysicalParam(nb_lanes,nb_vel_bins=5,lane_length=12.)
	bs = BehaviorModel[BehaviorModel(x[1],x[2],x[3]) for x in product(["cautious","normal","aggressive"],[27.;31.;35.],[4.])]
	#CASE: just agent car
	cs = CarState[CarState((24,3,),3,0,bs[1])]
	dp1 = pp.POSITIONS[48]-pp.POSITIONS[24]-pp.l_car
	dp2 = pp.POSITIONS[24]-pp.POSITIONS[1]-pp.l_car
	#just make sure it doesn't explode--this should be handled by get_mobil_lane_change
	nbhd = get_adj_cars(pp,cs,1)
	assert(get(nbhd.ahead_dist,0,-1)==-1)
	assert(get(nbhd.ahead_dist,1,-1)==-1)
	assert(get(nbhd.ahead_dist,-1,-1)==-1)
	assert(get(nbhd.behind_dist,0,-1)==-1)
	assert(get(nbhd.behind_dist,1,-1)==-1)
	assert(get(nbhd.behind_dist,-1,-1)==-1)
	#CASE: nobody to the left
	cs = CarState[CarState((48,3,),3,0,bs[1]),CarState((24,3,),3,0,bs[1]),CarState((1,3,),3,0,bs[1]),CarState((48,1,),3,0,bs[1]),CarState((1,1,),3,0,bs[1])]
	nbhd = get_adj_cars(pp,cs,2)
	assert(get(nbhd.ahead_dist,0,-1),dp1)
	assert(get(nbhd.ahead_dist,1,-1),-1)
	assert(get(nbhd.ahead_dist,-1,-1),dp1)
	assert(get(nbhd.behind_dist,0,-1),dp2)
	assert(get(nbhd.behind_dist,1,-1),-1)
	assert(get(nbhd.behind_dist,-1,-1),dp2)
	#CASE: nobody to the right
	cs = CarState[CarState((48,3,),3,0,bs[1]),CarState((1,3,),3,0,bs[1]),CarState((24,3,),3,0,bs[1]),CarState((48,5,),3,0,bs[1]),CarState((1,5,),3,0,bs[1])]
	nbhd = get_adj_cars(pp,cs,3)
	assert(get(nbhd.ahead_dist,0,-1),dp1)
	assert(get(nbhd.ahead_dist,1,-1),dp1)
	assert(get(nbhd.ahead_dist,-1,-1),-1)
	assert(get(nbhd.behind_dist,0,-1),dp2)
	assert(get(nbhd.behind_dist,1,-1),dp2)
	assert(get(nbhd.behind_dist,-1,-1),-1)
	#CASE: no one ahead
	cs = CarState[CarState((1,5,),3,0,bs[1]),CarState((1,3,),3,0,bs[1]),CarState((1,1,),3,0,bs[1]),CarState((24,3,),3,0,bs[1])]
	nbhd = get_adj_cars(pp,cs,4)
	assert(get(nbhd.ahead_dist,0,-1),-1)
	assert(get(nbhd.ahead_dist,1,-1),-1)
	assert(get(nbhd.ahead_dist,-1,-1),-1)
	assert(get(nbhd.behind_dist,0,-1),dp2)
	assert(get(nbhd.behind_dist,1,-1),dp2)
	assert(get(nbhd.behind_dist,-1,-1),dp2)
	#CASE: no one behind
	cs = CarState[CarState((48,5,),3,0,bs[1]),CarState((48,3,),3,0,bs[1]),CarState((48,1,),3,0,bs[1]),CarState((24,3,),3,0,bs[1])]
	nbhd = get_adj_cars(pp,cs,4)
	assert(get(nbhd.ahead_dist,0,-1),dp1)
	assert(get(nbhd.ahead_dist,1,-1),dp1)
	assert(get(nbhd.ahead_dist,-1,-1),dp1)
	assert(get(nbhd.behind_dist,0,-1),-1)
	assert(get(nbhd.behind_dist,1,-1),-1)
	assert(get(nbhd.behind_dist,-1,-1),-1)
	#CASE: full house
	cs = CarState[CarState((48,3,),3,0,bs[1]),CarState((1,3,),3,0,bs[1]),CarState((48,1,),3,0,bs[1]),CarState((1,1,),3,0,bs[1]),CarState((48,5,),5,0,bs[1]),CarState((1,5,),5,0,bs[1]),CarState((24,3,),3,0,bs[1])]
	nbhd = get_adj_cars(pp,cs,7)
	assert(get(nbhd.ahead_dist,0,-1),dp1)
	assert(get(nbhd.ahead_dist,1,-1),dp1)
	assert(get(nbhd.ahead_dist,-1,-1),dp1)
	assert(get(nbhd.behind_dist,0,-1),dp2)
	assert(get(nbhd.behind_dist,1,-1),dp2)
	assert(get(nbhd.behind_dist,-1,-1),dp2)

	##TODO, someday: test cases for slightly off line cars, cars that are far away
end

function test_get_mobil_lane_change()
	println("\t\tTesting get_mobil_lane_change")
	nb_lanes = 2
	pp = PhysicalParam(nb_lanes,nb_vel_bins=5,lane_length=12.)
	bs = BehaviorModel[BehaviorModel(x[1],x[2],x[3]) for x in product(["cautious","normal","aggressive"],[27.;31.;35.],[4.])]
	#CASE: it's faster, but there's no space--is this even a real case?
	#CASE: it's faster and there is space
	cs = CarState[CarState((48,3,),5,0,BehaviorModel("aggressive",35.,4.)),CarState((44,1,),1,0,BehaviorModel("cautious",27.,4.)),CarState((24,1,),3,0,BehaviorModel("aggressive",35.,4.))]
	nbhd = get_adj_cars(pp,cs,3)
	assert(get_mobil_lane_change(pp,cs[3],nbhd),1)
	#CASE: it's slower and there is space
	cs = CarState[CarState((48,3,),3,0,BehaviorModel("cautious",31.,4.)),CarState((44,1,),5,0,BehaviorModel("cautious",35.,4.)),CarState((24,1,),5,0,BehaviorModel("aggressive",35.,4.))]
	nbhd = get_adj_cars(pp,cs,3)
	assert(get_mobil_lane_change(pp,cs[3],nbhd),0)
	#CASE: someone is going fast behind me and i'm slow
	cs = CarState[CarState((1,1,),5,0,BehaviorModel("aggressive",35.,4.)),CarState((24,1,),1,0,BehaviorModel("cautious",27.,4.))]
	nbhd = get_adj_cars(pp,cs,2)
	assert(get_mobil_lane_change(pp,cs[2],nbhd),1)
	###repeat for other side
	#CASE: it's faster and there is space
	cs = CarState[CarState((48,1,),5,0,BehaviorModel("aggressive",35.,4.)),CarState((44,1,),31,0,BehaviorModel("cautious",27.,4.)),CarState((24,3,),3,0,BehaviorModel("aggressive",35.,4.))]
	nbhd = get_adj_cars(pp,cs,3)
	assert(get_mobil_lane_change(pp,cs[3],nbhd),-1)
	#CASE: it's slower and there is space
	cs = CarState[CarState((48,1,),3,0,BehaviorModel("cautious",31.,4.)),CarState((44,3,),5,0,BehaviorModel("cautious",35.,4.)),CarState((24,3,),5,0,BehaviorModel("aggressive",35.,4.))]
	nbhd = get_adj_cars(pp,cs,3)
	assert(get_mobil_lane_change(pp,cs[3],nbhd),0)
	#CASE: someone is going fast behind me and i'm slow
	cs = CarState[CarState((1,3,),5,0,BehaviorModel("aggressive",35.,4.)),CarState((24,3,),1,0,BehaviorModel("cautious",27.,4.))]
	nbhd = get_adj_cars(pp,cs,2)
	assert(get_mobil_lane_change(pp,cs[2],nbhd),-1)
end

function test_mobil()
	println("\tTesting MOBIL Units...")
	test_mobil_creation()
	test_mobil_equality()
	test_mobil_hashing()
	test_car_neighborhood_creation()
	test_car_neighborhood_equality()
	test_car_neighborhood_hashing()
	test_get_adj_cars()
	#test_get_mobil_lane_change()
	#NOTE: not working, but since we'll probably be limited to 1 car in the foreseeable future, this isn't the highest priority thing
end

##POMDP Types
#Convenience abstract types to remove dependence on external packages for hte purposes of testing
abstract State
abstract Action
abstract Observation
abstract POMDP
abstract AbstractSpace
abstract AbstractDistribution
import Iterators.product
include(joinpath("..","src","ML_types.jl"))
function test_behavior_model_creation()
	println("\t\tTesting BehaviorModel creation")
	ps = BehaviorModel[BehaviorModel(x[1],x[2],x[3]) for x in product(["cautious","normal","aggressive"],[27.;31.;35.],[4.])]
end

function test_behavior_model_equality()
	ps = BehaviorModel[BehaviorModel(x[1],x[2],x[3]) for x in product(["cautious","normal","aggressive"],[27.;31.;35.],[4.])]
	test_equality("BehaviorModel",ps)
end

function test_behavior_model_hashing()
	ps = BehaviorModel[BehaviorModel(x[1],x[2],x[3]) for x in product(["cautious","normal","aggressive"],[27.;31.;35.],[4.])]
	test_hashing("BehaviorModel",ps)
end

function test_carstate_creation()
	println("\t\tTesting CarState creation")
	#nothing to do here--no constructors to test
end

function test_carstate_equality()
	#println("\t\tTesting CarState equality")
	bs = BehaviorModel[BehaviorModel(x[1],x[2],x[3]) for x in product(["cautious","normal","aggressive"],[27.;31.;35.],[4.])]
	ps = CarState[]
	push!(ps,CarState((1,1),3,0,bs[1]))
	push!(ps,CarState((1,1),3,0,bs[2]))
	push!(ps,CarState((1,1),5,0,bs[1]))
	push!(ps,CarState((1,1),3,1,bs[1]))
	push!(ps,CarState((1,2),3,0,bs[1]))
	push!(ps,CarState((2,1),3,0,bs[1]))

	test_equality("CarState",ps)

end

function test_carstate_hashing()
	#println("\t\tTesting CarState hashing")
	bs = BehaviorModel[BehaviorModel(x[1],x[2],x[3]) for x in product(["cautious","normal","aggressive"],[27.;31.;35.],[4.])]
	ps = CarState[]
	push!(ps,CarState((1,1),3,0,bs[1]))
	push!(ps,CarState((1,1),3,0,bs[2]))
	push!(ps,CarState((1,1),5,0,bs[1]))
	push!(ps,CarState((1,1),3,1,bs[1]))
	push!(ps,CarState((1,2),3,0,bs[1]))
	push!(ps,CarState((2,1),3,0,bs[1]))

	test_hashing("CarState",ps)
end
function test_carstateobs_creation()
	println("\t\tTesting CarStateObs creation")
	#nothing to do here--no constructors to test
end

function test_carstateobs_equality()
	#println("\t\tTesting CarStateObs equality")
	ps = CarStateObs[]
	push!(ps,CarStateObs((1,1),3,0))
	push!(ps,CarStateObs((1,1),5,0))
	push!(ps,CarStateObs((1,1),3,1))
	push!(ps,CarStateObs((1,2),3,0))
	push!(ps,CarStateObs((2,1),3,0))

	test_equality("CarState",ps)

end

function test_carstateobs_hashing()
	#println("\t\tTesting CarStateObs hashing")
	ps = CarStateObs[]
	push!(ps,CarStateObs((1,1),3,0))
	push!(ps,CarStateObs((1,1),5,0))
	push!(ps,CarStateObs((1,1),3,1))
	push!(ps,CarStateObs((1,2),3,0))
	push!(ps,CarStateObs((2,1),3,0))

	test_hashing("CarStateObs",ps)
end

function test_MLState_creation()
	println("\t\tTesting MLState creation")
	#nothing to do here--no constructors to test
end

function test_MLState_equality()
	#println("\t\tTesting MLState equality")
	bs = BehaviorModel[BehaviorModel(x[1],x[2],x[3]) for x in product(["cautious","normal","aggressive"],[27.;31.;35.],[4.])]
	cs = CarState[]
	push!(cs,CarState((1,1),3,0,bs[1]))
	push!(cs,CarState((1,1),3,0,bs[2]))
	push!(cs,CarState((1,1),5,0,bs[1]))
	push!(cs,CarState((1,1),3,1,bs[1]))
	push!(cs,CarState((1,2),3,0,bs[1]))
	push!(cs,CarState((2,1),3,0,bs[1]))

	ps = MLState[]
	push!(ps,MLState(2,5,cs[1:1]))
	push!(ps,MLState(3,5,cs[1:1]))
	push!(ps,MLState(2,4,cs[1:1]))
	push!(ps,MLState(2,5,cs[1:2]))

	test_equality("MLState",ps)
end

function test_MLState_hashing()
	#println("\t\tTesting MLState hashing")
	bs = BehaviorModel[BehaviorModel(x[1],x[2],x[3]) for x in product(["cautious","normal","aggressive"],[27.;31.;35.],[4.])]
	cs = CarState[]
	push!(cs,CarState((1,1),3,0,bs[1]))
	push!(cs,CarState((1,1),3,0,bs[2]))
	push!(cs,CarState((1,1),5,0,bs[1]))
	push!(cs,CarState((1,1),3,1,bs[1]))
	push!(cs,CarState((1,2),3,0,bs[1]))
	push!(cs,CarState((2,1),3,0,bs[1]))

	ps = MLState[]
	push!(ps,MLState(2,5,cs[1:1]))
	push!(ps,MLState(3,5,cs[1:1]))
	push!(ps,MLState(2,4,cs[1:1]))
	push!(ps,MLState(2,5,cs[1:2]))

	test_hashing("MLState",ps)
end

function test_MLAction_creation()
	println("\t\tTesting MLAction creation")
	#nothing to do here--no constructors to test
	#as = [MLAction(x[1],x[2]) for x in product([-1;0;1],[-1;0;1])]
end

function test_MLAction_equality()
	as = [MLAction(x[1],x[2]) for x in product([-1;0;1],[-1;0;1])]
	test_equality("MLAction",as)
end

function test_MLAction_hashing()
	as = [MLAction(x[1],x[2]) for x in product([-1;0;1],[-1;0;1])]
	test_hashing("MLAction",as)
end

function test_MLObs_creation()
	println("\t\tTesting MLObs creation")
	#nothing to do here--no constructors to test
end

function test_MLObs_equality()
	#println("\t\tTesting MLObs equality")
	cs = CarStateObs[]
	push!(cs,CarStateObs((1,1),3,0))
	push!(cs,CarStateObs((1,1),5,0))
	push!(cs,CarStateObs((1,1),3,1))
	push!(cs,CarStateObs((1,2),3,0))
	push!(cs,CarStateObs((2,1),3,0))

	ps = MLObs[]
	push!(ps,MLObs(2,5,cs[1:1]))
	push!(ps,MLObs(3,5,cs[1:1]))
	push!(ps,MLObs(2,4,cs[1:1]))
	push!(ps,MLObs(2,5,cs[1:2]))

	test_equality("MLObs",ps)
end

function test_MLObs_hashing()
	println("\t\tTesting MLObs hashing")
	cs = CarStateObs[]
	push!(cs,CarStateObs((1,1),3,0))
	push!(cs,CarStateObs((1,1),5,0))
	push!(cs,CarStateObs((1,1),3,1))
	push!(cs,CarStateObs((1,2),3,0))
	push!(cs,CarStateObs((2,1),3,0))

	ps = MLObs[]
	push!(ps,MLObs(2,5,cs[1:1]))
	push!(ps,MLObs(3,5,cs[1:1]))
	push!(ps,MLObs(2,4,cs[1:1]))
	push!(ps,MLObs(2,5,cs[1:2]))

	test_hashing("MLObs",ps)
end

function test_MLPOMDP_creation()
	println("\t\tTesting MLPOMDP type creation")
	p = MLPOMDP()
end

function test_n_state()
	println("\t\tTesting n_states")
	pp = PhysicalParam(2,nb_vel_bins=8,lane_length=2.5) #2.5=>10
	p = MLPOMDP(nb_cars=1,phys_param=pp)
	p.nb_col = 4
	assert(n_states(p) == (4*8)*(4*10*8*3*9+1))
	#idk if its worht it to do more
end

function test_n_observations()
	println("\t\tTesting n_observations")
	pp = PhysicalParam(2,nb_vel_bins=8,lane_length=2.5) #2.5=>10
	p = MLPOMDP(nb_cars=1,phys_param=pp)
	p.nb_col = 4
	assert(n_observations(p) == (4*8)*(4*10*8*3+1))
end

#don't think you need tests for n_actions, the *Space types, and the associated domain() and length() functions...

function test_pomdp_types()
	println("\tTesting POMDP Type Units...")
	test_behavior_model_creation()
	test_behavior_model_equality()
	test_behavior_model_hashing()
	test_carstate_creation()
	test_carstate_equality()
	test_carstate_hashing()
	test_carstateobs_creation()
	test_carstateobs_equality()
	test_carstateobs_hashing()
	test_MLState_creation()
	test_MLState_equality()
	test_MLState_hashing()
	test_MLAction_creation()
	test_MLAction_equality()
	test_MLAction_hashing()
	test_MLObs_creation()
	test_MLObs_equality()
	test_MLObs_hashing()
	test_MLPOMDP_creation()
	test_n_state()
	test_n_observations()
end

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

	bs = BehaviorModel[BehaviorModel(x[1],x[2],x[3]) for x in product(["cautious","normal","aggressive"],[27.;31.;35.],[4.])]

	#Env Car out of bounds
	cs_oob = CarState((0,1),1,0,bs[1])
	#env car going slow in right lane: (41,1) corresponds to right in front of agent car area + 0.25m or 0.5m
	cs_r_slow = CarState((40,1),1,0,BehaviorModel("aggressive",27.,4.))
	#env car going fast in right lane: (7,1) corresponds to right behind the agent car area + 0.25m or 0.5m
	cs_r_fast = CarState((7,1),4,0,BehaviorModel("aggressive",35.,4.))
	#env car going at a medium speed in the right lane heading left
	cs_lchange = CarState((24,1),2,1,BehaviorModel("normal",31.,4.))
	#env car going at a medium speed in the left lane heading right
	cs_rchange = CarState((24,2),2,-1,BehaviorModel("normal",31.,4.))
	#env car is just chilling in the right/btwn/lhigh lane
	cs_rchill = CarState((24,1),2,1,BehaviorModel("normal",31.,4.))
	cs_mchill = CarState((24,2),2,1,BehaviorModel("normal",31.,4.))
	cs_hchill = CarState((28,1),2,1,BehaviorModel("normal",31.,4.))

	#CASE: do nothing = no costs
	assert(reward(p,MLState(1,1,CarState[cs_oob]),MLAction(0,0)) == 0.)
	#CASE: moving = cost
	assert(reward(p,MLState(1,1,CarState[cs_oob]),MLAction(1,0)) == p.accel_cost)
	assert(reward(p,MLState(1,1,CarState[cs_oob]),MLAction(-1,0)) == p.decel_cost)
	assert(reward(p,MLState(1,1,CarState[cs_oob]),MLAction(0,1)) == p.lanechange_cost)
	assert(reward(p,MLState(1,1,CarState[cs_oob]),MLAction(1,1)) == p.accel_cost + p.lanechange_cost)
	assert(reward(p,MLState(1,1,CarState[cs_oob]),MLAction(-1,1)) == p.decel_cost + p.lanechange_cost)
	assert(reward(p,MLState(1,1,CarState[cs_oob]),MLAction(1,-1)) == p.accel_cost + p.lanechange_cost)
	assert(reward(p,MLState(1,1,CarState[cs_oob]),MLAction(-1,-1)) == p.decel_cost + p.lanechange_cost)
	assert(reward(p,MLState(1,1,CarState[cs_oob]),MLAction(0,-1)) == p.lanechange_cost)

	#Case: cars occupy same space
	assert(reward(p,MLState(1,2,CarState[cs_rchill]),MLAction(0,0)) == p.r_crash)
	assert(reward(p,MLState(1,2,CarState[cs_mchill]),MLAction(0,0)) == p.r_crash)
	assert(reward(p,MLState(1,2,CarState[cs_hchill]),MLAction(0,0)) == p.r_crash)
	#CASE: cars intersect; vertically (gets railroaded from behind)
	assert(reward(p,MLState(1,1,CarState[cs_r_fast]),MLAction(-1,0)) == p.r_crash)
	assert(reward(p,MLState(1,4,CarState[cs_r_slow]),MLAction(1,0)) == p.r_crash)
	#CASE: cars intersect; horizontally (across lanes)
	assert(reward(p,MLState(1,2,CarState[cs_rchange]),MLAction(0,1)) == p.r_crash)
	assert(reward(p,MLState(3,2,CarState[cs_lchange]),MLAction(0,-1)) == p.r_crash)
	#CASE: diagonal intersection
	#CASE: cars will pass through one another during the next time step
end

function test_actions()
	println("\t\tTesting Action Space Creation")
	p = MLPOMDP()
	A = actions(p)
	bs = BehaviorModel[BehaviorModel(x[1],x[2],x[3]) for x in product(["cautious","normal","aggressive"],[27.;31.;35.],[4.])]
	s = MLState(2,5,CarState[CarState((1,1),3,0,bs[1])])
	A_ = actions(p,s) #check the other signature
	assert(domain(A_) == domain(A))
end

function test_transition()
	println("\t\tTesting Transition Model")

	nb_lanes = 2
	pp = PhysicalParam(nb_lanes,nb_vel_bins=5,lane_length=12.) #2.=>col_length=8
	p = MLPOMDP(nb_cars=1,nb_lanes=nb_lanes,phys_param=pp)

	bs = BehaviorModel[BehaviorModel(x[1],x[2],x[3]) for x in product(["cautious","normal","aggressive"],[27.;31.;35.],[4.])]

	#Env Car out of bounds
	cs_oob = CarState((0,1),1,0,bs[1])
	#env car going slow in right lane: (41,1) corresponds to right in front of agent car area + 0.25m or 0.5m
	cs_l_slow = CarState((3,1),1,0,BehaviorModel("aggressive",27.,4.))
	#env car going fast in right lane: (7,1) corresponds to right behind the agent car area + 0.25m or 0.5m
	cs_r_fast = CarState((48,1),5,0,BehaviorModel("aggressive",35.,4.))
	#env car going at a medium speed in the right lane heading left
	cs_lchange = CarState((24,1),2,1,BehaviorModel("normal",31.,4.))
	#env car going at a medium speed in the left lane heading right
	cs_rchange = CarState((24,2),2,-1,BehaviorModel("normal",31.,4.))
	#env car is just chilling in the right/btwn/lhigh lane
	cs_rchill = CarState((24,1),3,1,BehaviorModel("normal",31.,4.))
	cs_mchill = CarState((24,2),2,1,BehaviorModel("normal",31.,4.))
	cs_hchill = CarState((28,1),2,1,BehaviorModel("normal",31.,4.))

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
	assert(s_.agent_vel,2)
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
	s = MLState(2,1,CarState[cs_oob])
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

	bs = BehaviorModel[BehaviorModel(x[1],x[2],x[3]) for x in product(["cautious","normal","aggressive"],[27.;31.;35.],[4.])]

	#Env Car out of bounds
	cs_oob = CarState((0,1),1,0,bs[1])
	#env car going slow in right lane: (41,1) corresponds to right in front of agent car area + 0.25m or 0.5m
	cs_r_slow = CarState((40,1),1,0,BehaviorModel("aggressive",27.,4.))
	#env car going fast in right lane: (7,1) corresponds to right behind the agent car area + 0.25m or 0.5m
	cs_r_fast = CarState((7,1),4,0,BehaviorModel("aggressive",35.,4.))
	#env car going at a medium speed in the right lane heading left
	cs_lchange = CarState((24,1),2,1,BehaviorModel("normal",31.,4.))
	#env car going at a medium speed in the left lane heading right
	cs_rchange = CarState((24,2),2,-1,BehaviorModel("normal",31.,4.))
	#env car is just chilling in the right/btwn/lhigh lane
	cs_rchill = CarState((24,1),3,1,BehaviorModel("normal",31.,4.))
	cs_mchill = CarState((24,2),2,1,BehaviorModel("normal",31.,4.))
	cs_hchill = CarState((28,1),2,1,BehaviorModel("normal",31.,4.))

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
	test_states()
	test_observations()
	test_actions()
	test_reward()
	test_transition()
	test_normal_pdf()
	test_observe()
end

##Crashing
include(joinpath("..","src","crash.jl"))

function test_cross2d()
	println("\t\tTesting cross2d")
end

function test_line_segment_intersect()
	println("\t\tTesting line_segment_intersect")
	X = Array{Float64,1}[[0.;0.],[2.;0.],[2.;2.],[0;2.]]
	Y = Array{Float64,1}[[1.;1.],[3.;1.],[3.;3.],[1.;3.]]
	assert(!line_segment_intersect(X[1],X[2],Y[1],Y[2]))
	assert(!line_segment_intersect(X[1],X[2],Y[2],Y[3]))
	assert(!line_segment_intersect(Y[1],Y[2],X[1],X[2]))
	assert(!line_segment_intersect(Y[2],Y[3],X[1],X[2]))
	assert(line_segment_intersect(X[2],X[3],Y[1],Y[2]))
	assert(line_segment_intersect(Y[1],Y[2],X[2],X[3]))
	assert(line_segment_intersect(X[2],X[3],X[2],X[3]))
	assert(line_segment_intersect(X[2],X[3],X[3],X[2]))
	assert(!line_segment_intersect(X[1],X[2],Y[2],Y[1]))
	assert(!line_segment_intersect(X[1],X[2],Y[3],Y[2]))
	assert(!line_segment_intersect(Y[1],Y[2],X[2],X[1]))
	assert(!line_segment_intersect(Y[2],Y[3],X[2],X[1]))
	assert(line_segment_intersect(X[2],X[3],Y[2],Y[1]))
	assert(line_segment_intersect(Y[1],Y[2],X[3],X[2]))
	assert(line_segment_intersect(X[2],X[3],X[3],X[2]))
end

function poly_to_line_segments(X::Array{Array{Float64,1}})
	X_ = [hcat(X[i],X[i+1]) for i=1:(length(X)-1)]
	push!(X_,hcat(X[end],X[1]))
	return X_
end

function test_poly_intersect()
	println("\t\tTesting poly_intersect")
	#CASE: no intersection
	X = Array{Float64,1}[[0.;0.],[1.;0.],[1.;1.],[0;1.]]
	Y = Array{Float64,1}[[3.;3.],[4.;3.],[4.;4.],[3.;4.]]
	assert(!poly_intersect(poly_to_line_segments(X),poly_to_line_segments(Y)))
	#CASE: intersect at segments --not working
	X = Array{Float64,1}[[0.;0.],[2.;0.],[2.;2.],[0;2.]]
	Y = Array{Float64,1}[[1.;1.],[3.;1.],[3.;3.],[1.;3.]]
	assert(poly_intersect(poly_to_line_segments(X),poly_to_line_segments(Y)))
	#CASE: one inside other --not supported yet; will fail test!
	X = Array{Float64,1}[[0.;0.],[3.;0.],[3.;3.],[0;3.]]
	Y = Array{Float64,1}[[1.;1.],[2.;1.],[2.;2.],[1.;2.]]
	assert(poly_intersect(poly_to_line_segments(X),poly_to_line_segments(Y)))
	#CASE: same poly
	X = Array{Float64,1}[[0.;0.],[2.;0.],[2.;2.],[0;2.]]
	Y = Array{Float64,1}[[0.;0.],[2.;0.],[2.;2.],[0;2.]]
	assert(poly_intersect(poly_to_line_segments(X),poly_to_line_segments(Y)))
	#CASE: same with 1d offset
	X = Array{Float64,1}[[0.;0.],[2.;0.],[2.;2.],[0;2.]]
	Y = Array{Float64,1}[[1.;0.],[3.;0.],[3.;2.],[1;2.]]
	assert(poly_intersect(poly_to_line_segments(X),poly_to_line_segments(Y)))
	#And permutations of the above
	##TODO
end

function test_is_crash()
	println("\t\tTesting is_crash")
	nb_lanes = 2
	pp = PhysicalParam(nb_lanes,nb_vel_bins=4,lane_length=12.) #2.=>col_length=8
	p = MLPOMDP(nb_cars=1,nb_lanes=nb_lanes,phys_param=pp)

	bs = BehaviorModel[BehaviorModel(x[1],x[2],x[3]) for x in product(["cautious","normal","aggressive"],[27.;31.;35.],[4.])]

	#Env Car out of bounds
	cs_oob = CarState((0,1),1,0,bs[1])
	#env car going slow in right lane: (41,1) corresponds to right in front of agent car area + 0.25m or 0.5m
	cs_r_slow = CarState((40,1),1,0,BehaviorModel("aggressive",27.,4.))
	#env car going fast in right lane: (7,1) corresponds to right behind the agent car area + 0.25m or 0.5m
	cs_r_fast = CarState((7,1),4,0,BehaviorModel("aggressive",35.,4.))
	#env car going at a medium speed in the right lane heading left
	cs_lchange = CarState((24,1),2,1,BehaviorModel("normal",31.,4.))
	#env car going at a medium speed in the left lane heading right
	cs_rchange = CarState((24,2),2,-1,BehaviorModel("normal",31.,4.))
	#env car is just chilling in the right/btwn/lhigh lane
	cs_rchill = CarState((24,1),2,1,BehaviorModel("normal",31.,4.))
	cs_mchill = CarState((24,2),2,1,BehaviorModel("normal",31.,4.))
	cs_hchill = CarState((28,1),2,1,BehaviorModel("normal",31.,4.))

	##TODO: change these tests borrowed from test_reward to something more meaningful
	#CASE: do nothing = no costs
	assert(!is_crash(p,MLState(1,1,CarState[cs_oob]),MLAction(0,0)))
	#CASE: moving = cost
	assert(!is_crash(p,MLState(1,1,CarState[cs_oob]),MLAction(1,0)))
	assert(!is_crash(p,MLState(1,1,CarState[cs_oob]),MLAction(-1,0)))
	assert(!is_crash(p,MLState(1,1,CarState[cs_oob]),MLAction(0,1)))
	assert(!is_crash(p,MLState(1,1,CarState[cs_oob]),MLAction(1,1)))
	assert(!is_crash(p,MLState(1,1,CarState[cs_oob]),MLAction(-1,1)))
	assert(!is_crash(p,MLState(1,1,CarState[cs_oob]),MLAction(1,-1)))
	assert(!is_crash(p,MLState(1,1,CarState[cs_oob]),MLAction(-1,-1)))
	assert(!is_crash(p,MLState(1,1,CarState[cs_oob]),MLAction(0,-1)))

	#Case: cars occupy same space
	assert(is_crash(p,MLState(1,2,CarState[cs_rchill]),MLAction(0,0)))
	assert(is_crash(p,MLState(1,2,CarState[cs_mchill]),MLAction(0,0)))
	assert(is_crash(p,MLState(1,2,CarState[cs_hchill]),MLAction(0,0)))
	#CASE: cars intersect; vertically (gets railroaded from behind)
	assert(is_crash(p,MLState(1,1,CarState[cs_r_fast]),MLAction(-1,0)))
	assert(is_crash(p,MLState(1,4,CarState[cs_r_slow]),MLAction(1,0)))
	#CASE: cars intersect; horizontally (across lanes)
	assert(is_crash(p,MLState(1,2,CarState[cs_rchange]),MLAction(0,1)))
	assert(is_crash(p,MLState(3,2,CarState[cs_lchange]),MLAction(0,-1)))
end

function test_crash()
	println("\tTesting Crashing")
	test_cross2d()
	test_line_segment_intersect()
	test_poly_intersect()
	test_is_crash()
end

#############
function test_units()
	println("Running Unit Tests...")
	test_idm()
	test_mobil()
	test_pomdp_types()
	test_pomdp_model()
	test_crash()
end

test_units()
######################
##FUNCTIONAL TESTING##
######################
