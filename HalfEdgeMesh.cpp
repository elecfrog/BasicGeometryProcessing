#include "HalfEdgeMesh.h"

#include <limits>
#include <map>
#include <algorithm>

// default constructor, actually, it could be deleted.
HalfEdgeMesh::HalfEdgeMesh()
{
    verts.resize(0);
    edges.resize(0);

    normals.resize(0);
    faces.resize(0);

    virutal_center = vec3();
    virutal_radius = .0f;
}

HalfEdgeMesh::HalfEdgeMesh(AttributedObject &attributedObject)
{
    //        if( &attributedObject != nullptr)
    {
        for (auto &p : attributedObject.vertices)
        {
            this->verts.emplace_back(HEVertex(p));
        }

        for (auto &n : attributedObject.colours)
        {
            this->normals.emplace_back(n);
        }

        for (auto &c : attributedObject.colours)
        {
            this->colors.emplace_back(c);
        }

        for (auto &f : attributedObject.faceVertices)
        {
            this->faces.emplace_back(f);
        }

        this->virutal_center = attributedObject.centreOfGravity;

        this->virutal_radius = attributedObject.objectSize;
    }
}


void HalfEdgeMesh::Pairing()
{

    // resize all edges at first -> clear data | prevent null pointer
    this->edges.resize(0);
    BuildHalfEdges();
    BuildOtherHalfs();
    BuildFirstDirectedEdges();

    // PrintEdges();
    // PrintFirstDirectedEdges();
    // CheckOtherHalf(0);
    // auto neibots = FindNeighbors(0);
    // for(auto& n: neibots)
    // {
    //     std::cout<<"neighbors" << n ;
    // }
    // std::cout<< std::endl;

    BuildCompelteGraph(FindShortestPath_Dijkstra(497,525));
}


void HalfEdgeMesh::BuildHalfEdges()
{
    for(auto f = 0; f < this->faces.size()/3;  ++f)
    {
        //          VERT_ID           FACEID    NEXT_ID         PREV_ID                      OPPO_ID(with default 0)
        HEEdge e0(faces[3 * f],        f,       3 * f + 1,      3 * f + 2,  0);
        HEEdge e1(faces[3 * f + 1],    f,       3 * f + 2,      3 * f    ,  0);
        HEEdge e2(faces[3 * f + 2],    f,       3 * f    ,      3 * f + 1,  0);
        edges.emplace_back(e0);
        edges.emplace_back(e1);
        edges.emplace_back(e2);
    }    
}


void HalfEdgeMesh::BuildOtherHalfs()
{
    for (unsigned int index = 0; index < edges.size() ; ++index)
    {
        // v_start
        auto& v_start = edges[edges[index].prev_edgeID].vert_ID;

        // v_point
        auto& v_point = edges[index].vert_ID;

        for (unsigned int inner = 0; inner < edges.size(); ++inner)
        {
            unsigned int i_start = edges[edges[inner].prev_edgeID].vert_ID;
            if (i_start == v_point && edges[inner].vert_ID == v_start)
            {
                edges[index].oppo_edgeID = inner;
                break;
            }
        }
    }
}

void HalfEdgeMesh::BuildFirstDirectedEdges()
{
    // Update FirstDirectedEdges
    for (auto vertID = 0;  vertID < verts.size(); ++vertID)
    {
        bool stop = false;
        for (auto edgeID = 0; edgeID < edges.size(); ++edgeID)
        {
            auto& start_vert = edges[edges[edgeID].prev_edgeID].vert_ID;
            if (start_vert == vertID && stop == false)
            {
                verts[vertID].fd_edgeID = edgeID;
                stop = true;
            }
        }
    }
}

std::deque<unsigned int> HalfEdgeMesh::FindShortestPath_Dijkstra(unsigned int start_vertID, unsigned int target_vertID)
{
    // initalize this table size as the vertices size of the model
    std::vector<DijkstraVert> d_verts;

    d_verts.resize(this->verts.size());

    // initalize the table, each vertex: disance_fromStart = inf,  tagged = false, prev_ID = start_vertID
    for(unsigned int v_d = 0; v_d< this->verts.size(); v_d++)
    {
        d_verts[v_d].tagged = false;
        d_verts[v_d].disance_fromStart = std::numeric_limits<float>::infinity();
        d_verts[v_d].prev_ID = start_vertID;
    }
    // Set the start vertex with tagged = true, disance_fromStart = 0;
    d_verts[start_vertID].tagged = true;
    d_verts[start_vertID].disance_fromStart = 0;

    auto curr = start_vertID;
    while(d_verts[target_vertID].tagged == false)
    {
        // find neigbors of the current vertex, 
        auto neigbors = FindNeighbors(curr);
        // and loop them to record the distance from the start, and choose the smallest one as the next vertex should be used
        for(auto& n: neigbors)
        {
            // if this neighbor is not tagged, it should compute the distance
            if(d_verts[n].tagged == false)
            {
                float distance = (verts[curr].position - verts[n].position).length();
                // if this vertex never updated, the prev_ID is start_vertID, that means d_verts[d_verts[n].prev_ID].disance_fromStart == 0
                // else if this vertex has updated, the prev_ID is the real previous vertex ID.
                d_verts[n].disance_fromStart = distance + d_verts[d_verts[n].prev_ID].disance_fromStart; 
                d_verts[n].prev_ID = curr;
            }
            // else ignore this neighbor, test next neighbor
            else continue;
        }
        // now, to find the smallest vertexID, exclude tagged vertices
        float smallest   = std::numeric_limits<float>::infinity();
        unsigned int smallestID = 0; 
        for(unsigned int v = 0 ; v< d_verts.size(); ++v)
        {
            if(d_verts[v].disance_fromStart < smallest && d_verts[v].tagged == false)
            {
                smallest = d_verts[v].disance_fromStart;
                smallestID = v;
            }
        }

        // tag this vertex as true, and set it as the next visit vertex
        d_verts[smallestID].tagged = true; 
        curr = smallestID;
    }

    // make an output
    std::deque<unsigned int> res_list;
    unsigned int output = target_vertID;
    res_list.push_front(output);
    do 
    {
        output = d_verts[output].prev_ID;
        res_list.push_front(output);
    }
    while(output!=start_vertID);
    
    // std::vector<unsigned int> res;
    // for(auto&r: res_list)
    // {
    //     res.emplace_back(r);
    // }
//    for(auto&r: res_list)
//        std::cout <<"v"<< r << std::endl;
    return res_list;
}

std::vector<HalfEdgeMesh::GraphNode> HalfEdgeMesh::BuildCompelteGraph(std::deque<unsigned int> _vertices)
{
    auto vertices = _vertices;
    std::vector<GraphNode> completeGraph;

    while( vertices.size() > 1)
    {
        vertices.front();
        for(unsigned int v = 1; v< vertices.size(); v++)
        {
            completeGraph.emplace_back(GraphNode(vertices.front(), vertices[v], (verts[vertices.front()].position - verts[vertices[v]].position).length()));
        }
        vertices.pop_front();
    }

    std::sort(completeGraph.begin(), completeGraph.end());

    for (const auto& c : completeGraph)
    {
        std::cout << c.from_ID << " -> " << c.goto_ID  << " length =" << c.length << "\n";
    }
    
    std::map<unsigned int, unsigned int> tree_list;

    for(auto&v : _vertices)
        tree_list.insert({v,v});

    for(auto&node : completeGraph)
    {
        auto& value_from = node.from_ID;
        auto& value_goto = node.goto_ID;
        // std::cout<<"key" << value_from << " value" <<  value_goto << std::endl;
        if(tree_list[node.from_ID] == tree_list[node.goto_ID])
        {
            continue;
        }
        else tree_list[value_goto] = value_from;
    }

    for(auto&node : tree_list)
    {
        std::cout<<"key" << node.first << " value" <<  node.second << std::endl;
    }

    return completeGraph;
}

std::vector<unsigned int> HalfEdgeMesh::FindNeighbors(unsigned int vertID)
{
    // return this as an array, the size is the degree of thie vertex
    std::vector<unsigned int> neighbors;
    
    HEVertex& curr_vert = verts[vertID];

    unsigned int des_edgeID = curr_vert.fd_edgeID;
    
    do
    {
        neighbors.push_back(edges[des_edgeID].vert_ID);

        des_edgeID = edges[des_edgeID].oppo_edgeID; //get the oppo edge ID, using oppo back home

        if (edges[des_edgeID].vert_ID == vertID) // check it if back home
            des_edgeID = edges[des_edgeID].next_edgeID; // get the next edge ID
        else break;

    } while (des_edgeID != curr_vert.fd_edgeID);

    return neighbors;
}

std::vector<unsigned int> HalfEdgeMesh::FindPinchVertices(unsigned int vertID)
{
    return std::vector<unsigned int>();
}

std::vector<HalfEdgeMesh::GraphNode> HalfEdgeMesh::BuildMST_Kruskal(std::vector<GraphNode> completeGraph)
{
    std::vector<GraphNode> res;
    for(auto&node : completeGraph)
    {
        std::vector<GraphNode> tmp = res;
        tmp.emplace_back(node);
        if(!isRing(tmp))
        {
            res.emplace_back(node);
        }
    }
    return std::vector<GraphNode>();
}

bool HalfEdgeMesh::isRing(std::vector<GraphNode> subGraph)
{
    return false;
}

void HalfEdgeMesh::PrintEdges()
{
    unsigned long eID = 0;
    for(auto&e:edges) std::cout 
    << "Edge " << eID++  
    << "\tvertID " << e.vert_ID 
    << "\tprevID " << e.prev_edgeID 
    << "\tnextID " << e.next_edgeID 
    << "\toppoID " << e.oppo_edgeID << std::endl;
}

void HalfEdgeMesh::CheckOtherHalf(unsigned int edgeID)
{

    std::cout 
    << "Edge" << edgeID  
    << "\tvertID " << edges[edgeID].vert_ID 
    << "\tprevID " << edges[edgeID].prev_edgeID 
    << "\tnextID " << edges[edgeID].next_edgeID 
    << "\toppoID " << edges[edgeID].oppo_edgeID << std::endl;
    
    auto& currOtherHalf = edges[edgeID].oppo_edgeID;

    std::cout 
    << "Edge " << currOtherHalf  
    << "\tvertID " << edges[currOtherHalf].vert_ID 
    << "\tprevID " << edges[currOtherHalf].prev_edgeID 
    << "\tnextID " << edges[currOtherHalf].next_edgeID 
    << "\toppoID " << edges[currOtherHalf].oppo_edgeID << std::endl;
}

void HalfEdgeMesh::PrintFirstDirectedEdges()
{
    auto vertID = 0;
    for(auto&v: this->verts)
    {
        std::cout 
        << "Vertex " << vertID++ 
        << "\t FirstDirectedEdgeID " << v.fd_edgeID << std::endl;        
    }
}
