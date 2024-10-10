# mutable struct myPolicy(P<:robotMDP) <: Policy
#     problem::P
# end

# function action(policy::myPolicy)
#     return problem.possible_actions[1]
# end