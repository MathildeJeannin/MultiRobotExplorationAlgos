#!/bin/bash

mkdir Resultats
mkdir Resultats/Cen
mkdir Resultats/Dec
mkdir Logs
mkdir Logs/Cen
mkdir Logs/Dec

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

while IFS="," read -r rec_column1 rec_column2 rec_column3 rec_column4 rec_column5 rec_column6 rec_column7 rec_column8 rec_column9 rec_column10 rec_column11 rec_column12
do
    for ((i=$start; i<=$ni; i++))
    do 
        for ((j=$start; j<=$nj; j++))
        do  
            julia ExecuteCen.jl $rec_column1 $rec_column2 $rec_column3 $rec_column4 $rec_column5 $rec_column6 $rec_column7 $rec_column8 $rec_column9 $rec_column10 $rec_column11 $rec_column12 $((n+j)) &
        done
        echo "Creating $nj simu, n = $n" >> log_bashCen.txt
        wait 
        n=$((n+nj))
        echo "Finished $nj simu, n = $n" >> log_bashCen.txt
    done
done < <(tail -n +2 parametersCen.csv)

wait 
echo "All done" >> log_bashCen.txt



n=0
ni=1
nj=2
start=1

while IFS="," read -r rec_column1 rec_column2 rec_column3 rec_column4 rec_column5 rec_column6 rec_column7 rec_column8 rec_column9 rec_column10 rec_column11 rec_column12 rec_column13 
do
    for ((i=$start; i<=$ni; i++))
    do 
        for ((j=$start; j<=$nj; j++))
        do  
            julia ExecuteDec.jl $rec_column1 $rec_column2 $rec_column3 $rec_column4 $rec_column5 $rec_column6 $rec_column7 $rec_column8 $rec_column9 $rec_column10 $rec_column11 $rec_column12 $rec_column13 $((n+j)) &
        done
        echo "Creating $nj simu, n = $n" >> log_bashDec.txt
        wait 
        n=$((n+nj))
        echo "Finished $nj simu, n = $n" >> log_bashDec.txt
    done
done < <(tail -n +2 parametersDec.csv)

wait
echo "All done" >> log_bashDec.txt
