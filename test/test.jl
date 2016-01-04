#test.jl
#a bunch of unit tests for all the functions and types for the Multilane POMDP

using Base.Test

##############
##UNIT TESTS##
##############

function test_hashing(s::String,ps::Array{Any,1})
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
	assert(get(d1,p1,0) == 1)
end

function test_equality(s::String,ps::Array{Any,1})
	println("\t\tTesting $s Equality")
	if length(ps) < 3
		error("Make More objects for testing $s equality")
	end
	assert(ps[1] == ps[1])
	assert(ps[2] != ps[3])
	assert(ps[3] != ps[2])
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
	ps = [MobilParam(s) for s in ["cautious";"normal";"aggressive"]
end

function test_mobil_equality()
	#println("\t\tTesting MOBIL Equality")
	p1 = MOBILParam()
	p2 = MOBILParam(p=0.5)
	p3 = MOBILParam(b_safe=5.,a_thr=0.3)
	ps = [MobilParam(s) for s in ["cautious";"normal";"aggressive"]
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
	ps = [MobilParam(s) for s in ["cautious";"normal";"aggressive"]
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

function test_get_mobil_lane_changes()
	println("\t\tTesting get_mobil_lane_changes")
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
end

function test_carstate_equality()
	println("\t\tTesting CarState equality")
end

function test_carstate_hashing()
	println("\t\tTesting CarState hashing")
end

function test_MLState_creation()
	println("\t\tTesting MLState creation")
end

function test_MLState_equality()
	println("\t\tTesting MLState equality")
end

function test_MLState_hashing()
	println("\t\tTesting MLState hashing")
end

function test_MLAction_creation()
	println("\t\tTesting MLAction creation")
	as = [MLAction(x[1],x[2]) for x in product([-1;0,1],[-1;0;1])]
end

function test_MLAction_equality()
	as = [MLAction(x[1],x[2]) for x in product([-1;0,1],[-1;0;1])]
	test_equality("MLAction",as)
end

function test_MLAction_hashing()
	as = [MLAction(x[1],x[2]) for x in product([-1;0,1],[-1;0;1])]
	test_hashing("MLAction",as)
end

function test_MLObs_creation()
	println("\t\tTesting MLObs creation")
end

function test_MLObs_equality()
	println("\t\tTesting MLObs equality")
end

function test_MLObs_hashing()
	println("\t\tTesting MLObs hashing")
end

function test_MLPOMDP_creation()
	println("\t\tTesting MLPOMDP type creation")
	p = MLPOMDP()
end

function test_n_state()
	println("\t\tTesting n_states")
	p = MLPOMDP(nb_col=4,nb_cars=1,col_length=10)
	p.phys_param.nb_vel_bins = 8
	assert(n_states(p) == (4*8)*(4*10*8*3*9))
	#idk if its worht it to do more
end

function test_n_observations()
	println("\t\tTesting n_observations")
	p = MLPOMDP(nb_col=4,nb_cars=1,col_length=10)
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

function test_pomdp()
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

function test_line_segment_intersect()
	println("\t\tTesting line_segment_intersect")
	X = [[0.;0.],[2.;0.],[2.;2.],[0;2.]]
	Y = [[1.;1.],[3.;1.],[3.;3.],[1.;3.]]
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
	push!(X_,hcat(X[end],X[1])
	return X_
end

function test_poly_intersect()
	println("\t\tTesting poly_intersect")
	#CASE: no intersection
	X = [[0.;0.],[1.;0.],[1.;1.],[0;1.]]
	Y = [[3.;3.],[4.;3.],[4.;4.],[3.;4.]]
	assert(!poly_intersect(poly_to_line_segments(X),poly_to_line_segments(Y)))
	#CASE: intersect at segments
	X = [[0.;0.],[2.;0.],[2.;2.],[0;2.]]
	Y = [[1.;1.],[3.;1.],[3.;3.],[1.;3.]]
	assert(poly_intersect(poly_to_line_segments(X),poly_to_line_segments(Y)))
	#CASE: one inside other
	X = [[0.;0.],[3.;0.],[3.;3.],[0;3.]]
	Y = [[1.;1.],[2.;1.],[2.;2.],[1.;2.]]
	assert(poly_intersect(poly_to_line_segments(X),poly_to_line_segments(Y)))
	#CASE: same poly
	X = [[0.;0.],[2.;0.],[2.;2.],[0;2.]]
	Y = [[0.;0.],[2.;0.],[2.;2.],[0;2.]]
	assert(poly_intersect(poly_to_line_segments(X),poly_to_line_segments(Y)))
	#CASE: same with 1d offset
	X = [[0.;0.],[2.;0.],[2.;2.],[0;2.]]
	Y = [[1.;0.],[3.;0.],[3.;2.],[1;2.]]
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




