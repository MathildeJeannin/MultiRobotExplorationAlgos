function select_sequences(r::Robot, N::Int64, roll::Bool, compute_proba::Function, next_state::Function)
    if roll
        a,_ = action_info(r.planner, r.state)
        r.in_rollout = false
    end

    index_actions_sequences = Vector{Int64}[]

    seen_action = []

    i = 1
    compteur = 1 

    while i <= N && compteur < 20
        
        one_sequence = Int64[]
        s = 1

        while length(r.planner.tree.children[s])!=0 
            potentiel_a = r.planner.tree.children[s]
            if isempty(potentiel_a)
                break
            end

            max_q = 0
            best_a = 0
            for ap in potentiel_a
                if r.planner.tree.q[ap] > max_q && ap ∉ seen_action
                    max_q = r.planner.tree.q[ap]
                    best_a = ap
                end
            end

            if best_a == 0 
                for ap in potentiel_a
                    push!(seen_action, ap)
                end
                break
            else
                push!(one_sequence, best_a)
                # s = r.planner.tree.transitions[best_a][1][1]
                s = next_state(r, best_a)
            end
        end

        if !isempty(one_sequence)

            push!(seen_action, one_sequence[end])
            seen_sequence = false
            for j in eachindex(index_actions_sequences)
                if issubset(one_sequence, index_actions_sequences[j])
                    seen_sequence = true
                end
            end

            if !seen_sequence
                push!(index_actions_sequences, one_sequence)
                i+=1
            end

        end

        compteur += 1 
    end

    actions_sequences = Vector{Vector{action_robot}}(undef, length(index_actions_sequences))
    for (k,index_sequence) in enumerate(index_actions_sequences)
        actions_sequences[k] = [r.planner.tree.a_labels[index] for index in index_sequence]
    end

    assigned_proba = compute_proba(index_actions_sequences, r)
    return actions_sequences, assigned_proba
end


function compute_q(sequences::Vector{Vector{Int64}}, r::Robot)
    Q = [[r.planner.tree.q[i] for i in sequences[j]] for j in eachindex(sequences)]
    total_Q = [Base.sum(Q[i])/length(Q[i]) for i in eachindex(Q)]
    q = [total_Q[i]/Base.sum(total_Q) for i in eachindex(total_Q)]
    return q
end


function binary(sequences::Vector{Vector{Int64}}, r::Robot)
    q = zeros(length(sequences))
    q[1] = 1.0
    return q
end


function softmax(sequences::Vector{Vector{Int64}}, r::Robot)
    Q = [[r.planner.tree.q[i] for i in sequences[j]] for j in eachindex(sequences)]
    total_Q = [Base.sum(Q[i])/length(Q[i]) for i in eachindex(Q)]
    q = [exp(total_Q[i])/Base.sum(exp.(total_Q)) for i in eachindex(total_Q)]
    return q
end


function state_best_action(r::Robot, a::Int64)
    as = filter(((sanode,spnode),) -> sanode == a, r.planner.tree.unique_transitions)
    best_q = 0
    best_s = 0
    for (a,s) in as
        for ap in r.planner.tree.children[s]
            if r.planner.tree.q[ap] > best_q
                best_s = s
                best_q = r.planner.tree.q[ap]
            end
        end
    end
    if best_s == 0
        return r.planner.tree.transitions[a][1][1]
    end
    return best_s
end


function state_best_average_action(r::Robot, a::Int64)
    as = filter(((sanode,spnode),) -> sanode == a, r.planner.tree.unique_transitions)
    dico_q = Dict(a=>0.0 for a in r.planner.mdp.possible_actions)
    dico_n = Dict(a=>0 for a in r.planner.mdp.possible_actions)
    for (a,s) in as
        for ap in r.planner.tree.children[s]
            dico_q[r.planner.tree.a_labels[ap]] += r.planner.tree.q[ap]
            dico_n[r.planner.tree.a_labels[ap]] += 1
        end
    end
    for a_label in r.planner.mdp.possible_actions 
        dico_q[a_label] = dico_q[a_label]/dico_n[a_label]
    end
    q,best_a_label = findmax(dico_q)
    best_q = 0
    best_s = 0
    for (a,s) in as
        for ap in r.planner.tree.children[s]
            if r.planner.tree.a_labels[ap] == best_a_label
                if r.planner.tree.q[ap] > best_q
                    best_s = s
                    best_q = r.planner.tree.q[ap]
                end
            end
        end
    end
    if best_s == 0
        return r.planner.tree.transitions[a][1][1]
    end
    return best_s
end


function random_state_best_action(r::Robot, a::Int64)
    as = filter(((sanode,spnode),) -> sanode == a, r.planner.tree.unique_transitions)
    return rand(as)[2]
end


function state_worst_positive_action(r::Robot, a::Int64)
    as = filter(((sanode,spnode),) -> sanode == a, r.planner.tree.unique_transitions)
    min_q = 1000000
    worst_s = 0
    for (a,s) in as
        for ap in r.planner.tree.children[s]
            if r.planner.tree.q[ap] < min_q && r.planner.tree.q[ap] > 0 
                worst_s = s
                min_q = r.planner.tree.q[ap]
            end
        end
    end
    if worst_s == 0
        return r.planner.tree.transitions[a][1][1]
    end
    return worst_s
end


function select_best_sequences(r::Robot)
    rng = MersenneTwister(rand(1:1000000))
    nb_robots = length(r.state.space_state.robots_plans) 
    sequences = Vector{Vector{Vector{action_robot}}}(undef, nb_robots)
    states = Vector{robot_state}(undef, nb_robots)
    for plan in r.plans
        if plan.state.id != r.id
            if !isempty(plan.best_sequences) && !isempty(plan.best_sequences[1])
                distribution = SparseCat([i for i in eachindex(plan.assigned_proba)], [q for q in plan.assigned_proba])
                seq_index = rand(rng, distribution)
                sequences[plan.state.id] = deepcopy([plan.best_sequences[seq_index]])
                states[plan.state.id] = deepcopy(plan.state)
            else
                sequences[plan.state.id] = Vector{Vector{action_robot}}(undef, 0)
                states[plan.state.id] = deepcopy(plan.state)
            end
        end
    end
    return sequences, states
end


function esperance_fr(r::Robot)
    depth = r.planner.solver.depth
    esp = 0
    s = deepcopy(r.state)
    sequences = Vector{Vector{Vector{action_robot}}}(undef, length(r.plans))
    proba = Vector{Vector{Float64}}(undef, length(r.plans))
    for i in eachindex(sequences)
        sequences[i] = r.plans[i].best_sequences
        proba[i] = r.plans[i].assigned_proba
    end

    all_seq = Iterators.product(sequences...)
    all_proba = collect(Iterators.product(proba...))
    for (i,seq) in enumerate(all_seq)
        prod_qr = 1
        for (i_r,seq_r) in enumerate(seq)
            s.space_state.robots_plans[i_r].best_sequences = [seq_r]
            prod_qr = prod_qr * all_proba[i][i_r]
        end
        esp += simulate_reward(s, r.planner, r.planner.mdp.discount, 1)*prod_qr
    end
    return esp
end


function esperance_fr_xr(r::Robot, xr::Vector{action_robot}, qr::Float64)
    esp = 0
    s = deepcopy(r.state)
    sequences = Vector{Vector{Vector{action_robot}}}(undef, length(r.plans))
    proba = Vector{Vector{Float64}}(undef, length(r.plans))
    for i in eachindex(sequences)
        if i==r.id
            sequences[i] = [xr]
            proba[i] = [qr]
        else
            sequences[i] = r.plans[i].best_sequences
            proba[i] = r.plans[i].assigned_proba
        end
    end

    all_seq = Iterators.product(sequences...)
    all_proba = collect(Iterators.product(proba...))
    for (i,seq) in enumerate(all_seq)
        prod_qr = 1
        for (i_r,seq_r) in enumerate(seq)
            s.space_state.robots_plans[i_r].best_sequences = [seq_r]
            prod_qr = prod_qr * all_proba[i][i_r]
        end
        esp += simulate_reward(s, r.planner, r.planner.mdp.discount, 1)*prod_qr
    end
    return esp
end
    

function shannon_entropy(q_nr::Vector{Float64})
    s = 0
    for q in q_nr
        s+=q*log(q)
    end
    return -s
end


function simulate_reward(s::mdp_state, planner::DPWPlanner, discount::Float64, compteur::Int64)
    id = s.space_state.id
    if length(s.space_state.robots_plans[id].best_sequences[1]) >= compteur
        action = s.space_state.robots_plans[id].best_sequences[1][compteur]
        sp, r = @gen(:sp, :r)(planner.mdp, s, action, planner.rng)
        return r + discount*simulate_reward(sp, planner, discount, compteur + 1)
    else
        return 0
    end
end



function update_distribution!(r::Robot, alpha::Float64, beta::Float64)
    updated_proba = Vector{Float64}(undef, length(r.plans[r.id].assigned_proba))
    for (i,xr) in enumerate(r.plans[r.id].best_sequences)
        qr = r.plans[r.id].assigned_proba[i]
        esp_fr = esperance_fr(r)
        esp_fr_xr = esperance_fr_xr(r,xr,qr)
        updated_proba[i] = qr - alpha*qr*((esp_fr - esp_fr_xr)/beta + shannon_entropy(r.plans[r.id].assigned_proba) + log(qr))
        if updated_proba[i] < 0 
            updated_proba[i] = 0
        elseif updated_proba[i] > 1
            updated_proba[i] = 1
        end
    end
    total_q = sum(updated_proba)
    for i in eachindex(r.plans[r.id].assigned_proba)
        if total_q > 0 
            r.plans[r.id].assigned_proba[i] = updated_proba[i]/total_q
        else 
            r.plans[r.id].assigned_proba[i] = 1/length(r.plans[r.id].assigned_proba)
        end
    end
end
        