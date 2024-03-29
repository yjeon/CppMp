#ifndef MOTION_PLANNER_HPP_
#define MOTION_PLANNER_HPP_

#include "Simulator.hpp"

struct Vertex
{
    enum
	{
	    TYPE_NONE = 0,
	    TYPE_INIT = 1,
	    TYPE_GOAL = 2
	};
	
    int    m_parent;
    double m_state[Simulator::STATE_NR_DIMS];
    int    m_type;
    int    m_nchildren;
    
};

    

class MotionPlanner
{
public:
    MotionPlanner(Simulator * const simulator);
            
    ~MotionPlanner(void);

    void ExtendRandom(void);

    void ExtendRRT(void);

    void ExtendEST(void);

    void ExtendMyApproach(void);
    
        
protected:
    bool IsProblemSolved(void)
    {
	return m_vidAtGoal >= 0;
    }

    void GetPathFromInitToGoal(std::vector<int> *path) const;

    void AddVertex(Vertex * const v);

    //void AddVertexStart(Vertex * const v, std::vector<Vertex *> v_m);
    
    //void AddVertexGoal(Vertex * const v, std::vector<Vertex *> v_m);
    
    void ExtendTree(const int    vid,
		    const double sto[]);

    //void ExtendTreeFromGoal(const int    vid, const double sto[]);
    
    Simulator            *m_simulator;
    std::vector<Vertex *> m_vertices;

    //Simulator            *m_simulatorGoal;
    //std::vector<Vertex *> m_verticesGoal;

    int                   m_vidAtGoal;
    
    //int                   m_vidAtGoalStart;

    //int                   m_vidAtGoalGoal;

    double                m_totalSolveTime;

    
    friend class Graphics;    
};

#endif
