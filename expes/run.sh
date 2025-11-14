
#!/bin/bash

# mkdir Resultats
mkdir ./expes/Logs



n=0
ni=1
nj=2
start=1

# mkdir Resultats/Dec
mkdir ./expes/Logs/Dec

while IFS="," read -r rec_column1 rec_column2 rec_column3 rec_column4 rec_column5 rec_column6 rec_column7 rec_column8 rec_column9 rec_column10 rec_column11 rec_column12 rec_column13 rec_column14 rec_column15 rec_column16 rec_column17 rec_column18 rec_column19 rec_column20 rec_column21 rec_column22 rec_column23 rec_column24
do
    for ((i=$start; i<=$ni; i++))
    do 
        for ((j=$start; j<=$nj; j++))
        do  
            julia -t 10 expes/ExecuteDec.jl $rec_column1 $rec_column2 $rec_column3 $rec_column4 $rec_column5 $rec_column6 $rec_column7 $rec_column8 $rec_column9 $rec_column10 $rec_column11 $rec_column12 $rec_column13 $rec_column14 $rec_column15 $rec_column16 $rec_column17 $rec_column18 $rec_column19 $rec_column20 $rec_column21 $rec_column22 $rec_column23 $rec_column24 $((n+j)) &
        done
        echo "Creating $nj simu, n = $n" >> log_bashDec.txt
        wait 
        n=$((n+nj))
        echo "Finished $nj simu, n = $n" >> log_bashDec.txt
    done
done < <(tail -n +2 expes/parametersDec.csv)

wait
echo "All done" >> log_bashDec.txt


