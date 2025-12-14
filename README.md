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

it does work but needs tuning

write condition to check ackermann % on each angle and its corrisponding opposite angle, try not to hard code if possible

fine tuning required very higher anti ackermann being notice, change filters and fine tune to find middle ground

13/12
seems to be almost there, outer wheel behaves as expected, gradual increase towards peak angle at lock
inner wheel behaves not as expected and incorrectly, peaks mid sweep and gradually degreases towards lock, what is causing this is unknown, but changes to the direction and size of steering lever brings the most change, albiet small

varibale choices seem to be problematic, this will need to be adjusted and terms redfined to better conform to constraints, 

make t - tie rod length calcualted while fixing the other variables. this is the most logical next step.