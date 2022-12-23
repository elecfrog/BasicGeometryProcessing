#ifndef HALFEDGEMESH_H
#define HALFEDGEMESH_H

#include "Cartesian3.h"
#include "AttributedObject.h"
#include <vector>

// Half Edge Vertex Definition
struct HEVertex
{
    vec3 position;          // vertices positions
    unsigned int fd_edgeID; // first directed edgeIDs;

    HEVertex(vec3 _position, unsigned int _fd_edgeID)
    {
        position = _position;
        fd_edgeID = _fd_edgeID;
    }
    HEVertex(vec3 _position)
    {
        position = _position;
        fd_edgeID = -1;
    }
    HEVertex()
    {
        position = vec3();
        fd_edgeID = -1;
    }
};

// Half Edge Edge Definition
struct HEEdge
{
    unsigned int vert_ID;     // "Pointto" Vert ID;
    unsigned int face_ID;     // Adjcanit Face ID of this Edge;
    unsigned int next_edgeID; // Next Half Edge ID of this edge;
    unsigned int prev_edgeID; // Prev Edge ID of this edge;
    unsigned int oppo_edgeID; // Opposite Edge ID of this edge;

    bool hasJoint;        // flag of this edge takes a joint vertex
    unsigned int jointID; // actually is new vert ID

    HEEdge(unsigned int _vert, unsigned int _face, unsigned int _next, unsigned int _prev, unsigned int _oppo)
    {
        vert_ID = _vert;
        face_ID = _face;

        next_edgeID = _next;
        prev_edgeID = _prev;
        oppo_edgeID = _oppo;

        hasJoint = false;
        jointID = -1;
    }
    HEEdge()
    {
        vert_ID = -1;
        face_ID = -1;

        next_edgeID = -1;
        prev_edgeID = -1;
        oppo_edgeID = -1;

        hasJoint = false;
        jointID = -1;
    }
};

class HalfEdgeMesh
{
public:
    //  constructor of mesh data structure
    HalfEdgeMesh();
    //  constructor for getting data from attributedObject
    HalfEdgeMesh(AttributedObject &attributedObject);

private:
    // Mesh Data Members
    std::vector<HEVertex> verts; // vertices data

    std::vector<HEEdge> edges; // half-edges data

    std::vector<unsigned int> faces; // faces data

    std::vector<vec3> normals;

    std::vector<vec3> colors;

    vec3 virutal_center;  // center of the gravity

    float virutal_radius; // bounding sphere of the model


public:
    /***********************
     * Mesh Data Operations
     ************************/
    void Pairing();

    void BuildHalfEdges();
    void BuildOtherHalfs();
    void BuildFirstDirectedEdges();

    // get neighbors around a vertex. It should return its neighbors and the degree of this vertex
    std::vector<unsigned int> FindNeighbors(unsigned int vertID);

    // find non-mainfold vertices
    std::vector<unsigned int> FindPinchVertices(unsigned int vertID);

    /************************
     * Helper Functions
     ************************/
    void PrintEdges();
    void CheckOtherHalf(unsigned int edgeID);
    void PrintFirstDirectedEdges();

    /************************
     * Setters & Getters
     ************************/

    inline std::vector<HEVertex> GetHEVertices() { return this->verts; }
    inline vec3 GetVirutalCenter() { return this->virutal_center; }
    inline float GetVirutalRaidus() { return this->virutal_radius; }
};

#endif // HALFEDGEMESH_H
