# CppMp
CS485 Motion Planning


#ExtendMyApproach:
We decided that for ExtendMyApproach we would tweak our existing ExtendRRT algorithm in order to get better results than ExtendEST and ExtendRandom. In our ExtendMyApproach, we wanted to have a different heuristic that would bias the tree growth towards promising areas in the workspace. Our approach was to have a higher probability that a random sample would be generated in the space between the goal and the current position of the robot. Because in some cases (like scene 5) this approach by itself would not work, our code allows that with a certain probability the random samples are generated from the entire workspace. The fundamental difference from RRT lies on the way we calculate the distance. 

