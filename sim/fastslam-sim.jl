module FastSlam

# include() is unnecessary if module files are in search path 
pwd() in LOAD_PATH || push!(LOAD_PATH, pwd())

using Slam

"""
```
square(x)
```
This is a trivial function that computes x*x.
"""
square(x) = x^2

export square

end

