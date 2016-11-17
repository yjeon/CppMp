# CppMp
CS485 Motion Planning


#ExtendMyApproach:
We decided that for ExtendMyApproach we would tweak our existing ExtendRRT algorithm in order to get better results than ExtendEST and ExtendRandom. In our ExtendMyApproach, we wanted to have a different heuristic that would bias the tree growth towards promising areas in the workspace. Our approach was to have a higher probability that a random sample would be generated in the space between the goal and the current position of the robot. Because in some cases (like scene 5) this approach by itself would not work, our code allows that with a certain probability the random samples are generated from the entire workspace. The fundamental difference from RRT lies on the way we calculate the distance. 

#Pesudocode Algorithm 
Input: q_init , q_goal , 
Output: Path from q_init to q_goal 

p = randomNum(0,10);
if(p<4){
	q_rand = getRandomConfigurationNearGoal();
}else
	q_rand = getRandomConfiguration(); 
	
do from 1 to number of vertices in graph
	check the distance from each vertex to the goal.
	Save q_near the vertex with the smallest distance to goal.
  
extendTree(q_near)
