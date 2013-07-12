//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2013, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1047 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "CGELMesh.h"
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Initialise deformable mesh.

    \fn       void cGELMesh::initialise()
*/
//===========================================================================
void cGELMesh::initialise()
{
    m_showSkeletonModel = false;
    m_showMassParticleModel = false;
    m_useSkeletonModel = false;
    m_useMassParticleModel = false;
}


//===========================================================================
/*!
    Clear forces.

    \fn       void cGELMesh::clearForces()
*/
//===========================================================================
void cGELMesh::clearForces()
{
    if (m_useSkeletonModel)
    {
        list<cGELSkeletonNode*>::iterator i;

        for(i = m_nodes.begin(); i != m_nodes.end(); ++i)
        {
            (*i)->clearForces();
        }
    }
    if (m_useMassParticleModel)
    {
        vector<cGELVertex>::iterator i;

        for(i = m_gelVertices.begin(); i != m_gelVertices.end(); ++i)
        {
            i->m_massParticle->clearForces();
        }
    }
}


//===========================================================================
/*!
    Clear external forces on nodes.

    \fn       void cGELMesh::clearExternalForces()
*/
//===========================================================================
void cGELMesh::clearExternalForces()
{
    if (m_useSkeletonModel)
    {
        list<cGELSkeletonNode*>::iterator i;

        for(i = m_nodes.begin(); i != m_nodes.end(); ++i)
        {
            (*i)->clearExternalForces();
        }
    }
    if (m_useMassParticleModel)
    {
        vector<cGELVertex>::iterator i;

        for(i = m_gelVertices.begin(); i != m_gelVertices.end(); ++i)
        {
            i->m_massParticle->clearExternalForces();
        }
    }
}


//===========================================================================
/*!
    Compute all internal forces.

    \fn       void cGELMesh::computeForces()
*/
//===========================================================================
void cGELMesh::computeForces()
{
    if (m_useSkeletonModel)
    {
        list<cGELSkeletonLink*>::iterator i;

        for(i = m_links.begin(); i != m_links.end(); ++i)
        {
            (*i)->computeForces();
        }
    }
    if (m_useMassParticleModel)
    {
        list<cGELLinearSpring*>::iterator i;

        for(i = m_linearSprings.begin(); i != m_linearSprings.end(); ++i)
        {
            (*i)->computeForces();
        }
    }
}

//===========================================================================
/*!
    Compute next pose of each node.

    \fn       void cGELMesh::computeNextPose(double a_timeInterval)
*/
//===========================================================================
void cGELMesh::computeNextPose(double a_timeInterval)
{
    if (m_useSkeletonModel)
    {
        list<cGELSkeletonNode*>::iterator i;

        for(i = m_nodes.begin(); i != m_nodes.end(); ++i)
        {
            (*i)->computeNextPose(a_timeInterval);
        }
    }
    if (m_useMassParticleModel)
    {
        vector<cGELVertex>::iterator i;

        for(i = m_gelVertices.begin(); i != m_gelVertices.end(); ++i)
        {
            i->m_massParticle->computeNextPose(a_timeInterval);
        }
    }
}


//===========================================================================
/*!
    Apply the next pose of each node.

    \fn       void cGELMesh::applyNextPose()
*/
//===========================================================================
void cGELMesh::applyNextPose()
{
    if (m_useSkeletonModel)
    {
        list<cGELSkeletonNode*>::iterator i;

        for(i = m_nodes.begin(); i != m_nodes.end(); ++i)
        {
            (*i)->applyNextPose();
        }
    }
    if (m_useMassParticleModel)
    {
        vector<cGELVertex>::iterator i;

        for(i = m_gelVertices.begin(); i != m_gelVertices.end(); ++i)
        {
            i->m_massParticle->applyNextPose();
        }
    }
}


//===========================================================================
/*!
    Render this deformable mesh in OpenGL.

    \fn       void cGELMesh::render(cRenderOptions& a_options)
    \param    a_options  Rendering options.
*/
//===========================================================================
void cGELMesh::render(cRenderOptions& a_options)
{
    // render mesh
    cMultiMesh::render(a_options);

	/////////////////////////////////////////////////////////////////////////
	// Render parts that are always opaque
	/////////////////////////////////////////////////////////////////////////
	if (SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options))
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		if (m_showSkeletonModel)
		{
			list<cGELSkeletonNode*>::iterator i;
			for(i = m_nodes.begin(); i != m_nodes.end(); ++i)
			{
				cGELSkeletonNode* nextItem = *i;
				nextItem->render();
			}

			list<cGELSkeletonLink*>::iterator j;
			for(j = m_links.begin(); j != m_links.end(); ++j)
			{
				cGELSkeletonLink* nextItem = *j;
				nextItem->render();
			}
		}

		if (m_showMassParticleModel)
		{
			glDisable(GL_LIGHTING);

			list<cGELLinearSpring*>::iterator j;
			for(j = m_linearSprings.begin(); j != m_linearSprings.end(); ++j)
			{
				(*j)->render();
			}

			vector<cGELVertex>::iterator i;
			for(i = m_gelVertices.begin(); i != m_gelVertices.end(); ++i)
			{
				i->m_massParticle->render();
			}

			glEnable(GL_LIGHTING);
		}
	}
}


//===========================================================================
/*!
    Build dynamic vertices for deformable mesh

    \fn       void cGELMesh::buildVertices();
*/
//===========================================================================
void cGELMesh::buildVertices()
{
    // clear all deformable vertices
    m_gelVertices.clear();

    // get number of vertices
    int numVertices = getNumVertices();

    // create a table of deformable vertices based on the vertices of image
    for (int i=0; i<numVertices; i++)
    {
        // create a new deformable vertex data
        cVertex* v = getVertex(i);
        cGELVertex newDefVertex(v);
        newDefVertex.m_link = NULL;
        newDefVertex.m_node = NULL;
        v->m_tag = i;

        // add vertex to list
        m_gelVertices.push_back(newDefVertex);
    }
}


//===========================================================================
/*!
    Connect each vertex to nearest node.

    \fn     void cGELMesh::connectVerticesToSkeleton(bool a_connectToNodesOnly)
    \param  a_connectToNodesOnly  if \b true, then skin is only connected to nodes.
                otherwise skin shall be connected to links too.
*/
//===========================================================================
void cGELMesh::connectVerticesToSkeleton(bool a_connectToNodesOnly)
{
    // get number of vertices
    int numVertices = (int)(m_gelVertices.size());

    // for each deformable vertex we search for the nearest sphere or link
    for (int i=0; i<numVertices; i++)
    {
        // get current deformable vertex
        cGELVertex* curVertex = &m_gelVertices[i];

        // get current vertex position
        cVector3d pos = curVertex->m_vertex->getLocalPos();

        // initialize constant
        double min_distance = 99999999999999999.0;
        cGELSkeletonNode* nearest_node = NULL;
        cGELSkeletonLink* nearest_link = NULL;

        // search for the nearest node
		list<cGELSkeletonNode*>::iterator itr;
        for(itr = m_nodes.begin(); itr != m_nodes.end(); ++itr)
        {
            cGELSkeletonNode* nextNode = *itr;
            double distance = cDistance(pos, nextNode->m_pos);
            if (distance < min_distance)
            {
                min_distance = distance;
                nearest_node = nextNode;
                nearest_link = NULL;
            }
        }

        // search for the nearest link if any
        if (!a_connectToNodesOnly)
        {
            list<cGELSkeletonLink*>::iterator j;
            for(j = m_links.begin(); j != m_links.end(); ++j)
            {
                cGELSkeletonLink* nextLink = *j;
                double angle0 = cAngle(nextLink->m_wLink01, cSub(pos, nextLink->m_node0->m_pos));
                double angle1 = cAngle(nextLink->m_wLink10, cSub(pos, nextLink->m_node1->m_pos));

                if ((angle0 < (C_PI / 2.0)) && (angle1 < (C_PI / 2.0)))
                {
                    cVector3d p = cProjectPointOnLine(pos,
                                                      nextLink->m_node0->m_pos,
                                                      nextLink->m_wLink01);

                    double distance = cDistance(pos, p);

                    if (distance < min_distance)
                    {
                        min_distance = distance;
                        nearest_node = NULL;
                        nearest_link = nextLink;
                    }
                }
            }
        }

        // attach vertex to nearest node if it exists
        if (nearest_node != NULL)
        {
            curVertex->m_node = nearest_node;
            curVertex->m_link = NULL;
            cVector3d posRel = cSub(pos, nearest_node->m_pos);
            curVertex->m_massParticle->m_pos = cMul(cTranspose(nearest_node->m_rot), posRel);
        }

        // attach vertex to nearest link if it exists
        else if (nearest_link != NULL)
        {
            curVertex->m_node = NULL;
            curVertex->m_link = nearest_link;

            cMatrix3d rot;
            rot.setCol( nearest_link->m_A0,
                        nearest_link->m_B0,
                        nearest_link->m_wLink01);
            cVector3d posRel = cSub(pos, nearest_link->m_node0->m_pos);
            curVertex->m_massParticle->m_pos = cMul(cInverse(rot), posRel);
        }
    }
}


//===========================================================================
/*!
    Update position of vertices connected to skeleton.

    \fn       void cGELMesh::updateVertexPosition()
*/
//===========================================================================
void cGELMesh::updateVertexPosition()
{
    if (m_useSkeletonModel)
    {
        // get number of vertices
        int numVertices = (int)(m_gelVertices.size());

        // for each deformable vertex, update its position
        for (int i=0; i<numVertices; i++)
        {
            // get current deformable vertex
            cGELVertex* curVertex = &m_gelVertices[i];

            // the vertex is attached to an node
            if (curVertex->m_node != NULL)
            {
                cVector3d newPos;
                curVertex->m_node->m_rot.mulr(curVertex->m_massParticle->m_pos, newPos);
                newPos.add(curVertex->m_node->m_pos);
                curVertex->m_vertex->setLocalPos(newPos);
            }

            else if (curVertex->m_link != NULL)
            {
                cVector3d newPos;
                curVertex->m_link->m_node0->m_pos.addr(curVertex->m_massParticle->m_pos.z() * curVertex->m_link->m_wLink01, newPos);
                newPos.add(curVertex->m_massParticle->m_pos.x() * curVertex->m_link->m_wA0);
                newPos.add(curVertex->m_massParticle->m_pos.y() * curVertex->m_link->m_wB0);


                //curVertex->node->m_rot.mulr(curVertex->pos, newPos);
                //newPos.add(curVertex->node->m_pos);
                curVertex->m_vertex->setLocalPos(newPos);
            }
        }
    }

    if (m_useMassParticleModel)
    {
        // get number of vertices
        int numVertices = (int)(m_gelVertices.size());

        // for each deformable vertex, update its position
        for (int i=0; i<numVertices; i++)
        {
            // get current deformable vertex
            cGELVertex* curVertex = &m_gelVertices[i];

            // the vertex is attached to an node
            if (curVertex->m_massParticle != NULL)
            {
                curVertex->m_vertex->setLocalPos(curVertex->m_massParticle->m_pos);
            }
        }
    }
}


