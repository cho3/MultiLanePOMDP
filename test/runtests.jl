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

function assert(expr,val,fn::Function= ==,varname::AbstractString="")
	if !fn(expr,val)
    error("Assertion failed: $varname : expected $val, got $expr")
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

include("test_idm.jl")

include("test_mobil.jl")

include("test_types.jl")

include("test_model.jl")

include("test_crash.jl")

#############
function test_units()
	println("Running Unit Tests...")
	test_idm()
	test_mobil()
	test_pomdp_types()
	# test_pomdp_model()
	test_crash()
end

test_units()

include("example_from_notebook.jl")
