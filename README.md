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

14/12
current behaviour, outer wheel behaves predictably in most cases, large jumps but could probably be tuned, inner wheel does not behave in predictable manner, it reaches a quick peak mid sweep then reduces towards end of sweep. problem unknown, any change to steering lever appears to do nothing. acts like none will work. lots of anti-ackermann

15/12

fixed most issues now back to early peak, this can maybe be solved with computing t for full servo lock angles and setting checking if configurations meet all 3 poses

potentially try zr, cz +



Suspension notes:
- front sus and rear sus codes produce expected results, bug seen in CarV1 is present, can be debugged but isnt damaging to results given. 
- potentially move calculations into for loop to make filtering more simple

estimated time to results, 2 - 3 days of work

22/12
perhaps slight bug in calculations

23/12 
merge conflict explained, accidentally deleted previous commit, caused some issues but those hae now been remedied, back to status quo. 

- rear values looking good, a little more filter to narrow down choice. moving on to front. need to calculate max travel.

24/12 

do not commit until checked steering2.m 
changed angle conversion using standard method of 90 - angle (pi - angle, for radians) this may yeild expected results, issues possibly because of inccorect translation in to vertical reference, if incorrect, other issues persit but the convention should be more robust, apparently i've just been silly for the past month in missing this maths error. - did not work

ride height increase to 20mm (hr) 

NOTE: front camber reported as -9.5 degrees this is due to king pin axis as it is used to determine camber gain. the gain across travel is acceptable

only need to print vertical wheel travel to csv and other useful information, like roll centre height and other key suspension details

front_data.csv, row 59
rear_data.csv, row 40