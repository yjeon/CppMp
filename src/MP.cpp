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
    
    double vx = m_vertices[vid]->m_state[0];
    double vy = m_vertices[vid]->m_state[1];   
    
    
    bool obsFree = true;
    

    double distance = sqrt(pow(vx - sto[0], 2) + pow(vy - sto[1], 2));
    double dx = (sto[0]-vx)/distance;
    double dy = (sto[1]-vy)/distance;
    
    distance = sqrt(pow(dx,2)+pow(dy,2));
    double nx;// = vx;
    double ny;// = vy;
    double step = m_simulator->GetDistOneStep();

    while(distance <= step && obsFree){
        nx = vx+(dx*step);
        ny = vy+(dy*step);

        m_simulator->SetRobotCenter(nx,ny);

        if(m_simulator->IsValidState()){
            vx = nx;
            vy = ny;
            
            Vertex *v = new Vertex();
            v->m_state[0] = nx;
            v->m_state[1] = ny;
            v->m_parent = vid;
            AddVertex(v);

            if (m_simulator->HasRobotReachedGoal()){
                m_vidAtGoal = vid;
                break;
            }
            //distance = sqrt(pow(nx - sto[0], 2) + pow(ny - sto[1], 2));
        }
        else {
            m_simulator->SetRobotCenter(vx, vy);
            obsFree = false;
        }

    }

    
}


void MotionPlanner::ExtendRandom(void)
{
    Clock clk;
    StartTime(&clk);
    //your code
    double sto[2];
    //double distance = sqrt(pow(m_simulator->GetRobotCenterX() - m_simulator->GetGoalCenterX(), 2) + pow(m_simulator->GetRobotCenterY() - m_simulator->GetGoalCenterY(), 2));
    int prob = PseudoRandomUniformReal(0,10);
    //std::cout << prob;
    //std::cout << "\n";
    if (prob == 1){
        sto[0] = m_simulator->GetGoalCenterX();
        sto[1] = m_simulator->GetGoalCenterY();
    }
    else{
        m_simulator->SampleState(sto);
    }
    int vid = (int)PseudoRandomUniformReal(0, m_vertices.size()-1);
    
    ExtendTree(vid,sto);
     

    m_totalSolveTime += ElapsedTime(&clk);
}

void MotionPlanner::ExtendRRT(void)
{
    Clock clk;
    StartTime(&clk);
    double sto[2];
    int prob = PseudoRandomUniformReal(0,10);
    //std::cout << prob;
    //std::cout << "\n";
    if (prob == 1){
        sto[0] = m_simulator->GetGoalCenterX();
        sto[1] = m_simulator->GetGoalCenterY();
    } 
    else{
        m_simulator->SampleState(sto);
    }

    double min=10000;
    double min_index;
    double x;
    double y;
    double dis;
    for(int i = 0; i< m_vertices.size();i++){
        x = m_vertices[i]->m_state[0];
        y = m_vertices[i]->m_state[1];
        dis = sqrt(pow(x - sto[0], 2) + pow(y - sto[1], 2));
        //distance = sqrt(pow(x - m_simulator->GetGoalCenterX(), 2) + pow(y - m_simulator->GetGoalCenterY(), 2));
        if(dis<min){
            min_index = i;
            min = dis;
        }
    }

    ExtendTree(min_index, sto);

    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendEST(void)
{

    Clock clk;
    StartTime(&clk);
    //your code
    //std::cout << "JOSE!!!\n";
    // function should selet the random state sto as described
    //in ExtendRandom
    
    double sto[2];
    int prob = PseudoRandomUniformReal(0,10);
    //std::cout << prob;
    //std::cout << "\n";
    if (prob == 1){
        sto[0] = m_simulator->GetGoalCenterX();
        sto[1] = m_simulator->GetGoalCenterY();
    } 
    else{
        m_simulator->SampleState(sto);
    }

    //fucntion should select the vertex vid with probalbility
    //proportional to a weight defined in terms of the number 
    //of children coming out of vertex vid

    //w(q) is a running estimate on importance of selecting q 
    // as the tree configuration
    //from which to add a new tree branch
    //w(q) = 1/(1+ number of neighbors near q)
    //w(q) = 1/1+deg(q)
    
    double wTotal = 0;

    std::vector<double> partial_weight;  
    for(int i=0; i<m_vertices.size(); i++)
    {
        wTotal = 1.0/(1.0 + (m_vertices[i]->m_nchildren));  
        partial_weight.push_back(wTotal);
    }

    //double wRand = PseudoRandomUniformReal(0,partial_weight.size()-1);
    double w = PseudoRandomUniformReal()*wTotal;
    int vid = 0;
    for(int i=0; i<partial_weight.size(); i++)
    {
        

        if(w <= partial_weight[i])
        {
            vid = i;
            break;
        }
    }
    //int vid = partial_weight[wRand]/wTotal;

    ExtendTree(vid,sto);

    m_totalSolveTime += ElapsedTime(&clk);
    
}


void MotionPlanner::ExtendMyApproach(void)
{
    Clock clk;
    StartTime(&clk);
    //your code
    //press 4
    std::cout << "4"; 
    

    
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
