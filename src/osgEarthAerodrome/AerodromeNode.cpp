/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include "AerodromeNode"
#include <osgUtil/CullVisitor>
#include <osgEarth/Utils>

using namespace osgEarth::Aerodrome;


AerodromeNode::AerodromeNode(const std::string& icao)
  : _icao(icao), _rendered(false)
{
    //nop
}

void
AerodromeNode::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);

        // The following is a trick to force the Aerodrome to render in traversal order.

        osg::ref_ptr<osgUtil::RenderBin> newBin;

        osgUtil::RenderBin* bin = cv->getCurrentRenderBin();
        osgUtil::RenderBin* parentBin = bin->getParent();

        if ( parentBin )
        {
            // If a parent bin exists, "allocate" the first unused bin number from its list of children
            // and use that to force the creation of a new traversal-order bin:
            int newBinNumber = parentBin->getRenderBinList().rbegin()->first + 1;
            newBin = parentBin->find_or_insert(newBinNumber, "TraversalOrderBin");
        }
        else
        {
            // If there is no parent, fall back on the stage and allocate a new child bin there:
            int newBinNumber = bin->getStage()->getBinNum() + 1;
            newBin = bin->getStage()->find_or_insert(newBinNumber, "TraversalOrderBin");
        }

        // activate our new bin and traverse:
        cv->setCurrentRenderBin( newBin.get() );

        osg::Group::traverse(nv);

        // restore the previous bin:
        cv->setCurrentRenderBin( bin );
    }
    else
    {
        osg::Group::traverse( nv );
    }    
}
