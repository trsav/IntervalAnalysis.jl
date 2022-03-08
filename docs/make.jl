push!(LOAD_PATH,"../src/")
using Documenter, IntervalAnalysis


makedocs(sitename="Interval Analysis",doctest=true,modules=[IntervalAnalysis])
deploydocs(
    repo = "github.com/trsav/interval_analysis.git",
)