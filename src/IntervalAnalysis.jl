module IntervalAnalysis
using Plots 
using Base 
using Random 
using Symbolics
using Documenter

export Interval,*,+,-,subdivide

"""
    Interval

Defines a single interval between two real numbers.

# Examples
```jldoctest
julia> Interval(-1,1)
-1.0 ←-→ 1.0
```
"""
struct Interval
	"Lower interval value"
	l::Real
	"Upper interval value"
	u::Real
	Interval(l,u) = l > u ? new(u,l) : new(l,u)
	
end

Base.show(io::IO, x::Interval) = print(io,round(x.l,digits=6)," ←-→ ",round(x.u,digits=6))

"""

	+(x₁::Interval, x₂::Interval)

Adds two intervals

# Examples
```jldoctest
julia> Interval(-1,1) + Interval(-2,4)
-3.0 ←-→ 5.0
```
"""
function Base.:+(x₁::Interval, x₂::Interval)
	Interval(x₁.l + x₂.l,x₁.u + x₂.u)
    end



function Base.:+(x₁::Real, x₂::Interval)
	Interval(x₁ + x₂.l,x₁ + x₂.u)
    end

function Base.:+(x₁::Interval, x₂::Real)
	Interval(x₁.l + x₂,x₁.u + x₂)
    end

 """

	-(x₁::Interval, x₂::Interval)

Subtracts one interval from another

# Examples
```jldoctest
julia> Interval(-1,1) - Interval(-2,4)
-5.0 ←-→ 3.0
```
"""
function Base.:-(x₁::Interval, x₂::Interval)
	Interval(x₁.l - x₂.u,x₁.u - x₂.l)
    end

function Base.:-(x::Interval)
	Interval(-x.l,-x.u)
    end

function Base.:-(x₁::Interval, x₂::Real)
	Interval(x₁.l - x₂,x₁.u - x₂)
    end

    function Base.:-(x₁::Real, x₂::Interval)
	Interval(x₁ - x₂.l,x₁ - x₂.u)
    end

"""

	*(x₁::Interval, x₂::Interval)

Multiplies two intervals

# Examples
```jldoctest
julia> Interval(-1,1) * Interval(-2,4)
-4.0 ←-→ 4.0
```
"""
function Base.:*(x₁::Interval, x₂::Interval)
	if 0 ≤ x₁.l 
		if 0 ≤ x₂.l 
			return Interval(x₁.l*x₂.l,x₁.u*x₂.u)
		elseif x₂.l < 0 < x₂.u 
			return Interval(x₁.u*x₂.l,x₁.u*x₂.u)
		elseif x₂.u ≤ 0 
			return Interval(x₁.u*x₂.l,x₁.l*x₂.u)
		end
	elseif x₁.l < 0 < x₁.u
		if 0 ≤ x₂.l 
			return Interval(x₁.l*x₂.u,x₁.u*x₂.u)
		elseif x₂.u ≤ 0 
			return Interval(x₁.u*x₂.l,x₁.l*x₂.l)
		elseif x₂.l < 0 < x₂.u 
			return Interval(min(x₁.l*x₂.u,x₁.u*x₂.l),max(x₁.l*x₂.l,x₁.u*x₂.u))
		end
	elseif x₁.u ≤ 0
		if 0 ≤ x₂.l 
			return Interval(x₁.l*x₂.u,x₁.u*x₂.l)
		elseif x₂.l < 0 < x₂.u 
			return Interval(x₁.l*x₂.u,x₁.l*x₂.l)
		elseif x₂.u ≤ 0 
			return Interval(x₁.u*x₂.u,x₁.l*x₂.l)
		end
	end
end

function Base.:*(x₁::Interval,x₂::Real)
	Interval(x₁.l*x₂,x₁.u*x₂)
end

function Base.:*(x₁::Real,x₂::Interval)
	Interval(x₁*x₂.l,x₁*x₂.u)
end

"""

	/(x₁::Interval, x₂::Interval)

Divides two intervals

# Examples
```jldoctest
julia> Interval(-1,1) / Interval(-2,4)
-0.5 ←-→ 0.5
```
"""
function Base.:/(x₁::Interval, x₂::Interval)
	x₁ * Interval(1/x₂.u,1/x₂.l)
    end

function Base.:/(x₁::Real, x₂::Interval)
	Interval(x₁/x₂.u,x₁/x₂.l)
    end

function Base.:/(x₁::Interval, x₂::Real)
	Interval(x₁.l/x₂,x₁.u/x₂)
    end

function w(x::Interval)
	x.u-x.l
end

function w(X::Vector{Interval})
	widths = w.(X)
	d,i = findmax(widths)
end

function m(x::Interval)
	(x.u+x.l)/2
end

function m(X::Vector{Interval})
	m.(X)
end


function Base.sqrt(x::Interval)
	Interval(sqrt(x.l),sqrt(x.u))
end

function Base.:^(x::Interval,p::Integer)
	if x.l > 0 || p%2 != 0 
		return Interval(x.l^p,x.u^p)
	elseif x.u < 0 && p%2 == 0 
		return Interval(x.u^p,x.l^p)
	else
		return Interval(0,abs(x)^p)
	end
end

function Base.exp(x::Interval)
	Interval(exp(x.l),exp(x.u))
end

function Base.log(x::Interval)
	Interval(log(x.l),log(x.u))
end

function Base.log10(x::Interval)
	Interval(log10(x.l),log10(x.u))
end

function Base.log2(x::Interval)
	Interval(log2(x.l),log2(x.u))
end

function Base.sin(x::Interval)
	if x.l ≥ -(π/2) && x.u ≤ π/2
			return Interval(sin(x.l),sin(x.u))
	end
	return Interval(-1,1)
end

function Base.cos(x::Interval)
	if x.l ≥ 0 && x.u ≤ π
			return Interval(cos(x.l),cos(x.u))
	end
	return Interval(-1,1)
end

function Base.abs(x::Interval)
	max(abs(x.l),abs(x.u))
end


function qdist(x₁::Interval,x₂::Interval)
	max(abs(x₁.l-x₂.l),abs(x₁.u-x₂.u))
end

"""

	subdivide(x::Interval,N::Integer)

Subdivides an interval into N intervals.

# Examples
```jldoctest
julia> subdivide(Interval(0,1),5)
[0.0 ←-→ 0.2, 0.2 ←-→ 0.4, 0.4 ←-→ 0.6, 0.6 ←-→ 0.8, 0.8 ←-→ 1.0]
```
"""
function subdivide(x::Interval,N::Integer)
	A = Array{Interval}(undef,N)
	w_x = w(x)/N
	for j ∈ 1:N 
		A[j] = Interval(x.l+(j-1)*w_x,x.l+j*w_x)
	end
	return A 
end

function subdivide_vector(X::Vector{Interval},N::Integer)
	A = Array{Vector{Interval}}(undef,length(X))
	for i ∈ 1:length(X)
		A[i] = subdivide(X[i],N)
	end
	return A
end

function bisect_box(X::Vector{Interval})
	# bisects a box over the largest axis 
	d,i = w(X)
	X_s = subdivide(X[i],2)
	X₁ = copy(X)
	X₁[i] = X_s[1]
	X₂ = copy(X)
	X₂[i] = X_s[2]
	return X₁,X₂
end


function hull(X::Vector{Interval})
	l = [x.l for x ∈ X]
	u = [x.u for x ∈ X]
	Interval(minimum(l),maximum(u))
end

function refinement(x,f,N)
	X = subdivide(x,N)
	Y = f.(X)
	hull(Y)
end

function plot_refinement(x,f,∇f,N)
	plot(f,x.l,x.u,c="black",linewidth=3,label="")
	X = subdivide(x,N)
	Y = f.(X)	
	Yₘᵥ = mean_value_form.(f,∇f,X)
	for i ∈ 1:length(X)
		if i == 1
			plot!([X[i].l,X[i].u],[Y[i].l,Y[i].l],fillrange=[Y[i].u,Y[i].u],linewidth=0,c="black",label="Interval Extension",fillalpha=0.15)
			plot!([X[i].l,X[i].u],[Yₘᵥ[i].l,Yₘᵥ[i].l],fillrange=[Yₘᵥ[i].u,Yₘᵥ[i].u],linewidth=0,c="red",label="Mean Value Form",fillalpha=0.25)
		else
			plot!([X[i].l,X[i].u],[Y[i].l,Y[i].l],fillrange=[Y[i].u,Y[i].u],linewidth=0,c="black",label="",fillalpha=0.15)
			plot!([X[i].l,X[i].u],[Yₘᵥ[i].l,Yₘᵥ[i].l],fillrange=[Yₘᵥ[i].u,Yₘᵥ[i].u],linewidth=0,c="red",label="",fillalpha=0.25)
		end

	end
	display(plot!(dpi=600))
end

function skelboe_moore(f,X)

	fᵤ = f(X).u # smallest upper bound (AKA the only upper bound at this stage)

	# creating a store of intervals, mean values, and interval values 
	L = [Dict("X"=>X,"f_m"=>f(m(X)),"F"=>f(X))]
	Lᵣ = []
	it_count = 0 

	# until tolerance...
	for it ∈ 1:30
		
		
		# initialise list of intervals to sack off 
		del_index =[]
		for i in 1:length(L)
			# if the lowest interval value is above the smallest upper bound..
			if L[i]["F"].l > fᵤ
				# remove interval, we know global minimum is not here...
				push!(del_index,i)
			end
		end

		# delete all the ones we said we would 
		for d_i ∈ del_index
			push!(Lᵣ,L[d_i])
		end
		deleteat!(L,del_index)


		# pick the remaining interval with smallest mean value 	
		s = argmin([l["f_m"] for l ∈ L])

		# divide the box into two 
		X_s = bisect_box(L[s]["X"])

		# get rid of the containing interval 
		deleteat!(L,s)

		for x ∈ X_s
			# evaluate interval values 
			F = f(x)

			# check if we find a smallest upper bound 
			if F.u < fᵤ
				fᵤ = F.u
			end
			# add new interval information to store
			push!(L,Dict("X"=>x,"f_m"=>f(m(x)),"F"=>F))

		end

	end
	return L,Lᵣ
end


function plot_boxes(L,Lᵣ)
	for l ∈ L 
		x1 = l["X"][1]
		x2 = l["X"][2]
		plot!(Shape([x1.l,x1.l,x1.u,x1.u] , [x2.u,x2.l,x2.l,x2.u]),label="",linewidth=1,alpha=0,linealpha=1)
	end
	for l ∈ Lᵣ
		x1 = l["X"][1]
		x2 = l["X"][2]
		plot!(Shape([x1.l,x1.l,x1.u,x1.u] , [x2.u,x2.l,x2.l,x2.u]),c="black",alpha=0.55,label="",linewidth=1)
	end
end




function mean_value_form(f,∇f,X)
	if typeof(X) == Interval
		D = ∇f(X)[1]
	else
		D = ∇f(X)
	end
	mean = m(X)
	fₘ = f(mean)
	if typeof(X) == Interval
		return fₘ + D*(X-mean)
	else
		return fₘ + sum(D[i]*(X[i]-mean[i]) for i ∈ 1:length(X))
	end

end

function build_gradient(f,len_x)
	if len_x > 1
		@variables x[1:len_x]
		∇f = Symbolics.gradient(f(x),[x[i] for i ∈ 1:len_x])
		∇f_expr = build_function(∇f, x)
		∇f = eval(∇f_expr[1])
		return ∇f
	else
		@variables x
		∇f = Symbolics.gradient(f(x),[x])
		∇f_expr = build_function(∇f, x)
		∇f = eval(∇f_expr[1])
		return ∇f
	end

end 

function g(x1,x2)
	return x1*exp(x1+x2^2)-x2^2
end

function g_vec(X)
	return g(X[1],X[2])
end


# f(x) = x^3 - x^2
# ∇f = build_gradient(f,1)
# plot_refinement(Interval(0,1),f,∇f,20)
# print(Interval(-1,1) / Interval(-2,4))

end