##POMDP Types
#Convenience abstract types to remove dependence on external packages for hte purposes of testing
abstract POMDP{S,A,O}
abstract AbstractSpace
abstract AbstractDistribution
import Iterators.product
include(joinpath("..","src","ML_types.jl"))
function test_behavior_model_creation()
	println("\t\tTesting BehaviorModel creation")
	ps = BehaviorModel[BehaviorModel(x[1],x[2],x[3],idx) for (idx,x) in enumerate(product(["cautious","normal","aggressive"],[27.,31.,35.],[4.]))]
end

function test_behavior_model_equality()
	ps = BehaviorModel[BehaviorModel(x[1],x[2],x[3],idx) for (idx,x) in enumerate(product(["cautious","normal","aggressive"],[27.,31.,35.],[4.]))]
	test_equality("BehaviorModel",ps)
end

function test_behavior_model_hashing()
	ps = BehaviorModel[BehaviorModel(x[1],x[2],x[3],idx) for (idx,x) in enumerate(product(["cautious","normal","aggressive"],[27.,31.,35.],[4.]))]
	test_hashing("BehaviorModel",ps)
end

function test_carstate_creation()
	println("\t\tTesting CarState creation")
	#nothing to do here--no constructors to test
end

function test_carstate_equality()
	#println("\t\tTesting CarState equality")
	bs = BehaviorModel[BehaviorModel(x[1],x[2],x[3],idx) for (idx,x) in enumerate(product(["cautious","normal","aggressive"],[27.,31.,35.],[4.]))]
	ps = CarState[]
	push!(ps,CarState((1.,1),3.,0,bs[1]))
	push!(ps,CarState((1.,1),3.,0,bs[2]))
	push!(ps,CarState((1.,1),5.,0,bs[1]))
	push!(ps,CarState((1.,1),3.,1,bs[1]))
	push!(ps,CarState((1.,2),3.,0,bs[1]))
	push!(ps,CarState((2.,1),3.,0,bs[1]))

	test_equality("CarState",ps)

end

function test_carstate_hashing()
	#println("\t\tTesting CarState hashing")
	bs = BehaviorModel[BehaviorModel(x[1],x[2],x[3],idx) for (idx,x) in enumerate(product(["cautious","normal","aggressive"],[27.,31.,35.],[4.]))]
	ps = CarState[]
	push!(ps,CarState((1.,1),3.,0,bs[1]))
	push!(ps,CarState((1.,1),3.,0,bs[2]))
	push!(ps,CarState((1.,1),5.,0,bs[1]))
	push!(ps,CarState((1.,1),3.,1,bs[1]))
	push!(ps,CarState((1.,2),3.,0,bs[1]))
	push!(ps,CarState((2.,1),3.,0,bs[1]))

	test_hashing("CarState",ps)
end
function test_carstateobs_creation()
	println("\t\tTesting CarStateObs creation")
	#nothing to do here--no constructors to test
end

function test_carstateobs_equality()
	#println("\t\tTesting CarStateObs equality")
	ps = CarStateObs[]
	push!(ps,CarStateObs((1.,1),3.,0))
	push!(ps,CarStateObs((1.,1),5.,0))
	push!(ps,CarStateObs((1.,1),3.,1))
	push!(ps,CarStateObs((1.,2),3.,0))
	push!(ps,CarStateObs((2.,1),3.,0))

	test_equality("CarState",ps)

end

function test_carstateobs_hashing()
	#println("\t\tTesting CarStateObs hashing")
	ps = CarStateObs[]
	push!(ps,CarStateObs((1.,1),3.,0))
	push!(ps,CarStateObs((1.,1),5.,0))
	push!(ps,CarStateObs((1.,1),3.,1))
	push!(ps,CarStateObs((1.,2),3.,0))
	push!(ps,CarStateObs((2.,1),3.,0))

	test_hashing("CarStateObs",ps)
end

function test_MLState_creation()
	println("\t\tTesting MLState creation")
	#nothing to do here--no constructors to test
end

function test_MLState_equality()
	#println("\t\tTesting MLState equality")
	bs = BehaviorModel[BehaviorModel(x[1],x[2],x[3],idx) for (idx,x) in enumerate(product(["cautious","normal","aggressive"],[27.,31.,35.],[4.]))]
	cs = CarState[]
	push!(cs,CarState((1.,1),3.,0,bs[1]))
	push!(cs,CarState((1.,1),3.,0,bs[2]))
	push!(cs,CarState((1.,1),5.,0,bs[1]))
	push!(cs,CarState((1.,1),3.,1,bs[1]))
	push!(cs,CarState((1.,2),3.,0,bs[1]))
	push!(cs,CarState((2.,1),3.,0,bs[1]))

	ps = MLState[]
	push!(ps,MLState(2,5.,cs[1:1]))
	push!(ps,MLState(3,5.,cs[1:1]))
	push!(ps,MLState(2,4.,cs[1:1]))
	push!(ps,MLState(2,5.,cs[1:2]))

	test_equality("MLState",ps)
end

function test_MLState_hashing()
	#println("\t\tTesting MLState hashing")
	bs = BehaviorModel[BehaviorModel(x[1],x[2],x[3],idx) for (idx,x) in enumerate(product(["cautious","normal","aggressive"],[27.,31.,35.],[4.]))]
	cs = CarState[]
	push!(cs,CarState((1.,1),3.,0,bs[1]))
	push!(cs,CarState((1.,1),3.,0,bs[2]))
	push!(cs,CarState((1.,1),5.,0,bs[1]))
	push!(cs,CarState((1.,1),3.,1,bs[1]))
	push!(cs,CarState((1.,2),3.,0,bs[1]))
	push!(cs,CarState((2.,1),3.,0,bs[1]))

	ps = MLState[]
	push!(ps,MLState(2,5.,cs[1:1]))
	push!(ps,MLState(3,5.,cs[1:1]))
	push!(ps,MLState(2,4.,cs[1:1]))
	push!(ps,MLState(2,5.,cs[1:2]))

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
	push!(cs,CarStateObs((1.,1),3.,0))
	push!(cs,CarStateObs((1.,1),5.,0))
	push!(cs,CarStateObs((1.,1),3.,1))
	push!(cs,CarStateObs((1.,2),3.,0))
	push!(cs,CarStateObs((2.,1),3.,0))

	ps = MLObs[]
	push!(ps,MLObs(2,5.,cs[1:1]))
	push!(ps,MLObs(3,5.,cs[1:1]))
	push!(ps,MLObs(2,4.,cs[1:1]))
	push!(ps,MLObs(2,5.,cs[1:2]))

	test_equality("MLObs",ps)
end

function test_MLObs_hashing()
	println("\t\tTesting MLObs hashing")
	cs = CarStateObs[]
	push!(cs,CarStateObs((1.,1),3.,0))
	push!(cs,CarStateObs((1.,1),5.,0))
	push!(cs,CarStateObs((1.,1),3.,1))
	push!(cs,CarStateObs((1.,2),3.,0))
	push!(cs,CarStateObs((2.,1),3.,0))

	ps = MLObs[]
	push!(ps,MLObs(2,5.,cs[1:1]))
	push!(ps,MLObs(3,5.,cs[1:1]))
	push!(ps,MLObs(2,4.,cs[1:1]))
	push!(ps,MLObs(2,5.,cs[1:2]))

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
	# test_MLPOMDP_creation()
	# test_n_state()
	# test_n_observations()
end
