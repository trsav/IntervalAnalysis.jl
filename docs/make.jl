push!(LOAD_PATH,"../src/")
using Documenter, IntervalAnalysis


makedocs(sitename="Interval Analysis",doctest=true,modules=[IntervalAnalysis])
deploydocs(
    repo = "https://github.com/trsav/IntervalAnalysis.jl.git"
)