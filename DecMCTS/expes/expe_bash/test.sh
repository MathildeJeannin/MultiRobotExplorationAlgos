n=0
ni=5
nj=10
start=1

for ((i=$start; i<=$ni; i++))
do 
    for ((j=$start; j<=$nj; j++))
    do
        echo
    done
    n=$((n+nj))
    echo "n=$n" 
done
