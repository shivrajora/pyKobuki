mkdir tmp
x=1; for i in *depth.png; do counter=$(printf %03d $x); ln "$i" tmp/img"$counter".png; x=$(($x+1)); done

