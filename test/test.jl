#test.jl
#a bunch of unit tests for all the functions and types for the Multilane POMDP

import Base.==
import Base.length
using Base.Test

##############
##UNIT TESTS##
##############

function test_hashing(s::AbstractString,ps)
	##TODO: make a more comprehensive test that uses more of the created objects
	println("\t\tTesting $s Hashing")
	if length(ps) < 2
		error("Make More objects for testing $s hashing")
	end
	d1 = Dict{typeof(ps[1]),Int}()
	d1[ps[1]] = 1
	d1[ps[1]] = 1
	assert(length(d1) == 1)
	d2 = Dict{typeof(ps[1]),Int}([p=>3 for p in [ps[2];ps[2]]])
	assert(length(d2) == 1)
	assert(get(d1,ps[1],0) == 1)
end

function test_equality(s::AbstractString,ps)
	println("\t\tTesting $s Equality")
	if length(ps) < 3
		error("Make More objects for testing $s equality")
	end
	for i = 1:length(ps)
		for j = 1:length(ps)
			if i == j
				assert(ps[i]==ps[j])
			else
				assert(ps[i] != ps[j])
				assert(ps[j] != ps[i])
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
end

function test_car_neighborhood_hashing()
	println("\t\tTesting Car Neighborhood Hashing")
end
function test_get_adj_cars()
	println("\t\tTesting get_adj_cars")
	#CASE: just agent car
	
	#CASE: nobody to the left
	
	#CASE: nobody to the right
	
	#CASE: no one ahead
	
	#CASE: no one behind
	
	#CASE: full house
	
end

function test_get_mobil_lane_change()
	println("\t\tTesting get_mobil_lane_change")
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
	test_get_mobil_lane_change()
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
	bs = CarState[]
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
	push!(ps,MLState(2,5,cs[1:1])
	push!(ps,MLState(3,5,cs[1:1])
	push!(ps,MLState(2,4,cs[1:1])
	push!(ps,MLState(2,5,cs[1:2])
	
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
	push!(ps,MLState(2,5,cs[1:1])
	push!(ps,MLState(3,5,cs[1:1])
	push!(ps,MLState(2,4,cs[1:1])
	push!(ps,MLState(2,5,cs[1:2])
	
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
	push!(ps,MLObs(2,5,cs[1:1])
	push!(ps,MLObs(3,5,cs[1:1])
	push!(ps,MLObs(2,4,cs[1:1])
	push!(ps,MLObs(2,5,cs[1:2])
	
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
	push!(ps,MLObs(2,5,cs[1:1])
	push!(ps,MLObs(3,5,cs[1:1])
	push!(ps,MLObs(2,4,cs[1:1])
	push!(ps,MLObs(2,5,cs[1:2])
	
	test_hashing("MLObs",ps)
end

function test_MLPOMDP_creation()
	println("\t\tTesting MLPOMDP type creation")
	p = MLPOMDP()
end

function test_n_state()
	println("\t\tTesting n_states")
	p = MLPOMDP(nb_cars=1,col_length=10)
	p.nb_col = 4
	p.phys_param.nb_vel_bins = 8
	assert(n_states(p) == (4*8)*(4*10*8*3*9))
	#idk if its worht it to do more
end

function test_n_observations()
	println("\t\tTesting n_observations")
	p = MLPOMDP(nb_cars=1,col_length=10)
	p.nb_col = 4
	p.phys_param.nb_vel_bins = 8
	assert(n_observations(p) == (4*8)*(4*10*8*3))
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
	println("\t\tTesting Observation Space Creation")
end

function test_states()
	println("\t\tTesting State Space Creation")
end

function test_reward()
	println("\t\tTesting Reward Model")
end

function test_actions()
	println("\t\tTesting Action Space Creation")
end

function test_transition()
	println("\t\tTesting Transition Model")
end

function test_observe()
	println("\t\tTesting Observation Model")
end

##TODO: indexing and other helper functions testing

function test_pomdp_model()
	println("\tTesting POMDP Model Units...")
	test_observations()
	test_states()
	test_reward()
	test_actions()
	test_transition()
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
end

function test_crash()
	println("\tTesting Crashing")
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




