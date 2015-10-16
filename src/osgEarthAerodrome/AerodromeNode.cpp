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

        osgUtil::RenderBin* bin = cv->getCurrentRenderBin();

        int newBinNumber = 0;
        if ( bin->getRenderBinList().size() > 0 )
            newBinNumber = bin->getRenderBinList().rbegin()->first + 1;

        cv->setCurrentRenderBin( bin->find_or_insert(newBinNumber, "TraversalOrderBin") );

        osg::Group::traverse( nv );

        cv->setCurrentRenderBin( bin );
    }
    else
    {
        osg::Group::traverse( nv );
    }    
}
