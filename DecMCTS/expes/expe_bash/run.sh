#!/bin/bash

mkdir Resultats
mkdir Logs

# alpha_state=1.0
# k_state=10.0
# exploration_constant=1.0
# n_iterations=2000
# keep_tree=false
# discount=0.9
# nb_robots=2

# echo "Executing file for first compilation" 
# julia Execute.jl $alpha_state $k_state $exploration_constant $n_iterations $keep_tree $discount
# echo "Finished executing"

n=0
ni=1
nj=2
start=1

# chaque parametre est testé ni*nj fois et il y a nj simulations lancées en même temps
# lancer avec tmux puis fait ctrl+b puis d pour quitter le terminal tmux
# faire tmux attach pour recuperer la session

while IFS="," read -r col1 col2 col3
do
    for ((i=$start; i<=$ni; i++))
    do 
        for ((j=$start; j<=$nj; j++))
        do  
            julia -t 3 Execute.jl $col1 $col2 $col3 $((n+j)) &
        done
        echo "Creating $nj simu, n = $n" >> log_bash.txt
        wait 
        n=$((n+nj))
        echo "Finished $nj simu, n = $n" >> log_bash.txt
    done
done < <(tail -n +2 parameters_test.csv)

wait
echo "All done" >> log_bash.txt
