push!(LOAD_PATH,"../src/")
using Documenter, IntervalAnalysis


makedocs(sitename="Interval Analysis",doctest=true,modules=[IntervalAnalysis])