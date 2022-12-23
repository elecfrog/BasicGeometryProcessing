#include "HalfEdgeMesh.h"

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
