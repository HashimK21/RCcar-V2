# TODO
-tune ngrid values to find acceptable steering configuration
cleaned up ackermann checks and split purge into three passes
this is bugged 5/12
code has been checked and doubled checked it is doing what is excpected, ngrid values have to be adjusted to find range

neaten and move items for more effiency -done

things need to be locked down, make the following constant in all cases
    - czComp
    - cxComp
    - h 

and remove s, cz becomes full z component of steering lever. it unnecessary to compute s.

this will allow large range of rack and tie rod configurations to be checked

an issue has been found, source of which is currently unknown :(

added transform to rotate frame of reference to car? (unsure if this works) 


removed transform and reformulated zr, i think it works

