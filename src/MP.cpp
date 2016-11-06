#include "MP.hpp"
#include "PseudoRandom.hpp"
#include "MyTimer.hpp"
#include <cstring>
//mine
#include <iostream>

MotionPlanner::MotionPlanner(Simulator * const simulator)
{
    m_simulator = simulator;   

    Vertex *vinit = new Vertex();

    vinit->m_parent   = -1;   
    vinit->m_nchildren= 0;    
    vinit->m_state[0] = m_simulator->GetRobotCenterX();
    vinit->m_state[1] = m_simulator->GetRobotCenterY();

    AddVertex(vinit);
    m_vidAtGoal = -1;
    m_totalSolveTime = 0;
}

MotionPlanner::~MotionPlanner(void)
{
    //do not delete m_simulator  

    const int n = m_vertices.size();
    for(int i = 0; i < n; ++i)
	delete m_vertices[i];
}


void MotionPlanner::ExtendTree(const int    vid, 
			       const double sto[])
{
//your code
    //std::cout << "Hello"; 
    /*
    double vx = m_vertices[vid]->m_state[0];
    double vy = m_vertices[vid]->m_state[1];
    double step = m_simulator->GetDistOneStep();
    double dis = sqrt(pow(vx-sto[0],2)+pow(vy-sto[1],2));

    double dx = (sto[0]-vx)/dis;
    double dy = (sto[1]-vy)/dis;
    */
    
    double vertexX = m_vertices[vid]->m_state[0];
    double vertexY = m_vertices[vid]->m_state[1];   
    double stepSize = m_simulator->GetDistOneStep();
    double distance = sqrt(pow(vertexX - sto[0], 2) + pow(vertexY - sto[1], 2));
    bool inObstacle = false;
    
    double deltaX = (sto[0]-vertexX)/distance;
    double deltaY = (sto[1]-vertexY)/distance;
    // While we don't hit an obstacle and our distance is greater than 0
    // walk down this vertex path to see if there are any obstacles or the 
    // goal
    double nextX=vertexX;
    double nextY=vertexY;
    while (!inObstacle && distance >=stepSize){
        
        //This will first "normalize" the vector and will increase it by a single step
        nextX += (deltaX * stepSize);
        nextY += (deltaY * stepSize);
        
        // Set the robot location so we can determine if this is a valid state
        m_simulator->SetRobotCenter(nextX, nextY);
        
        if (m_simulator->IsValidState()){
            distance = sqrt(pow(nextX - sto[0], 2) + pow(nextY - sto[1], 2));
            if (m_simulator->HasRobotReachedGoal()){
                m_vidAtGoal = vid;
                break;
            }
        }
        else {
            inObstacle = true;
            // Reset the robot to be at the previous vertex
            m_simulator->SetRobotCenter(vertexX, vertexY);
        }
    }

    // If we are not in an obstacle then we have found a valid vertex
    // so add it to our list
    if (!inObstacle) {
        /*Vertex *vertex = new Vertex();
        vertex->m_state[0] = nextX;
        vertex->m_state[1] = nextY;
        vertex->m_parent = vid;
        AddVertex(vertex);
        m_simulator->SetRobotCenter(nextX, nextY);
        */
        Vertex *v = new Vertex();
        v->m_state[0] = nextX;
        v->m_state[1] = nextY;
        v->m_parent = vid;
        AddVertex(v);
        m_simulator->SetRobotCenter(nextX,nextY);
            
    }
    
}

void MotionPlanner::ExtendRandom(void)
{
    Clock clk;
    StartTime(&clk);
    //press 1
    //std::cout << "1"; 
    double sto[2];
    m_simulator->SampleState(sto);
    int vid = (int)PseudoRandomUniformReal(0,m_vertices.size()-1);
    if(m_simulator->HasRobotReachedGoal()){
        std::cout << "Goal";
    }
    else{
        ExtendTree(vid,sto);
    }
//your code
     

    m_totalSolveTime += ElapsedTime(&clk);
}

void MotionPlanner::ExtendRRT(void)
{
    Clock clk;
    StartTime(&clk);
    //press 2
    std::cout << "2"; 
    
//your code
    
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendEST(void)
{
    Clock clk;
    StartTime(&clk);
    //press 3
    std::cout << "3"; 
//your code    
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendMyApproach(void)
{
    Clock clk;
    StartTime(&clk);
    //press 4
    std::cout << "4"; 
    
//your code
    
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::AddVertex(Vertex * const v)
{
    if(v->m_type == Vertex::TYPE_GOAL)
	m_vidAtGoal = m_vertices.size();
    m_vertices.push_back(v); 
    if(v->m_parent >= 0)
	(++m_vertices[v->m_parent]->m_nchildren);
}

void MotionPlanner::GetPathFromInitToGoal(std::vector<int> *path) const
{
    std::vector<int> rpath;
    
    rpath.clear();
    
    int i = m_vidAtGoal;
    do
    {
	rpath.push_back(i);
	i = m_vertices[i]->m_parent;	
    } 
    while(i >= 0);
    
    path->clear();
    for(int i = rpath.size() - 1; i >= 0; --i)
	path->push_back(rpath[i]);
}
