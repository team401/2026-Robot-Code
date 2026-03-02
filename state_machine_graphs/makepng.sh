for l in *dot
do 
    dot -Tpng -o $(basename $l .dot).png $l
done
