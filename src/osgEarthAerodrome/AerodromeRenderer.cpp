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

#include "AerodromeRenderer"
#include "Common"
#include "AerodromeNode"
#include "LightBeaconNode"
#include "LightIndicatorNode"
#include "LinearFeatureNode"
#include "PavementNode"
#include "RunwayNode"
#include "RunwayThresholdNode"
#include "StartupLocationNode"
#include "StopwayNode"
#include "TaxiwayNode"
#include "TerminalNode"
#include "WindsockNode"

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osgEarth/ECEF>
#include <osgEarth/ElevationQuery>
#include <osgEarth/Registry>
#include <osgEarthFeatures/Feature>
#include <osgEarthFeatures/GeometryCompiler>


using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Aerodrome;

#define LC "[AerodromeRenderer] "


namespace
{
    class BoundingBoxMaskSource : public MaskSource
    {
    public:
        BoundingBoxMaskSource(osgEarth::Bounds bounds, Map* map)
          : MaskSource(), _bounds(bounds), _map(map)
        {
        }

        osg::Vec3dArray* createBoundary(const SpatialReference* srs, ProgressCallback* progress =0L)
        {
            osg::Vec3dArray* boundary = new osg::Vec3dArray();

            if (_map.valid())
            {
                double elevation = 0.0;

                ElevationQuery eq(_map.get());
                eq.getElevation(GeoPoint(_map->getSRS(), _bounds.center().x(), _bounds.center().y()), elevation, 0.005);
            
                boundary->push_back(osg::Vec3d(_bounds.xMin(), _bounds.yMin(), elevation));
                boundary->push_back(osg::Vec3d(_bounds.xMax(), _bounds.yMin(), elevation));
                boundary->push_back(osg::Vec3d(_bounds.xMax(), _bounds.yMax(), elevation));
                boundary->push_back(osg::Vec3d(_bounds.xMin(), _bounds.yMax(), elevation));
            }

            return boundary;
        }

    private:
        osgEarth::Bounds _bounds;
        osg::ref_ptr<Map> _map;
    };
}


AerodromeRenderer::AerodromeRenderer(Map* map)
  : _map(map), osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
{
    //nop
}

void
AerodromeRenderer::apply(AerodromeNode& node)
{
    if (node.bounds().valid())
    {
        // use the center of the bounds as an anchor for localization
        GeoPoint center(_map->getSRS(), node.bounds().center().x(), node.bounds().center().y());
        createLocalizations(center);


        // create ground geometry beneath the aerodrome
        osg::Geometry* geometry = new osg::Geometry();
        geometry->setUseVertexBufferObjects( true );
        geometry->setUseDisplayList( false );

        std::vector<osg::Vec3d> boundary;
        boundary.push_back(osg::Vec3d(node.bounds().xMin(), node.bounds().yMin(), _elevation));
        boundary.push_back(osg::Vec3d(node.bounds().xMax(), node.bounds().yMin(), _elevation));
        boundary.push_back(osg::Vec3d(node.bounds().xMax(), node.bounds().yMax(), _elevation));
        boundary.push_back(osg::Vec3d(node.bounds().xMin(), node.bounds().yMax(), _elevation));

        //osg::Vec3Array* normals = new osg::Vec3Array();
        osg::Vec3Array* verts = new osg::Vec3Array();
        transformAndLocalize(boundary, _map->getSRS(), verts, 0L);

        geometry->setVertexArray( verts );

        osg::Vec4Array* colors = new osg::Vec4Array;
        colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
        geometry->setColorArray(colors);
        geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

        //osg::Vec2Array* tcoords = new osg::Vec2Array(4);
        //(*tcoords)[3].set(0.0f,0.0f);
        //(*tcoords)[2].set(1.0f,0.0f);
        //(*tcoords)[1].set(1.0f,1.0f);
        //(*tcoords)[0].set(0.0f,1.0f);
        //geometry->setTexCoordArray(0,tcoords);

        geometry->addPrimitiveSet( new osg::DrawArrays( GL_QUADS, 0, 4 ) );

        //geometry->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex, osg::StateAttribute::ON);
        geometry->setName(node.icao() + "_AERODROME_TERRAIN");

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable(geometry);

        Registry::shaderGenerator().run(geode.get(), "osgEarth.AerodromeRenderer");

        osg::MatrixTransform* mt = new osg::MatrixTransform();
        mt->setMatrix(_local2world);
        node.addChild(mt);

        // create mask layer based on accumulated bounds
        osgEarth::MaskLayer* mask = new osgEarth::MaskLayer(osgEarth::MaskLayerOptions(), new BoundingBoxMaskSource(node.bounds(), _map.get()));
        node.setMaskLayer(mask);
        _map->addTerrainMaskLayer(mask);
    }
    else
    {
        _elevation = 0.0;
    }
}

void
AerodromeRenderer::apply(LightBeaconNode& node)
{
    //TODO: create geometry for light beacon and add to node

    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get(), 4.0);
    if (geom)
        node.addChild(geom);

}

void
AerodromeRenderer::apply(LightIndicatorNode& node)
{
    //TODO: create geometry for light indicator and add to node

    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get(), 4.0);
    if (geom)
        node.addChild(geom);

}

void
AerodromeRenderer::apply(LinearFeatureNode& node)
{
    //TODO: create geometry for linear feature and add to node

    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get(), 1.5);
    if (geom)
        node.addChild(geom);

}

void
AerodromeRenderer::apply(PavementNode& node)
{
    //TODO: create geometry for pavement and add to node

    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get(), 0.0);
    if (geom)
        node.addChild(geom);

}

void
AerodromeRenderer::apply(RunwayNode& node)
{
    //TODO: create geometry for runway and add to node

    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get(), 1000.0);
    if (geom)
    {
        geom->setName(node.icao() + "_RUNWAY_" + osgEarth::toString(feature->getFID()));
        node.addChild(geom);
    }
}

void
AerodromeRenderer::apply(RunwayThresholdNode& node)
{
    //TODO: create geometry for runway threshold and add to node

    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get(),  3.0);
    if (geom)
        node.addChild(geom);

}

void
AerodromeRenderer::apply(StartupLocationNode& node)
{
    //TODO: create geometry for startup location and add to node

    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get(), 3.0);
    if (geom)
        node.addChild(geom);

}

void
AerodromeRenderer::apply(StopwayNode& node)
{
    //TODO: create geometry for stopway and add to node

    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get(), 3.0);
    if (geom)
        node.addChild(geom);

}

void
AerodromeRenderer::apply(TaxiwayNode& node)
{
    //TODO: create geometry for taxiway and add to node

    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get(), 1.0);
    if (geom)
        node.addChild(geom);

}

void
AerodromeRenderer::apply(TerminalNode& node)
{
    //TODO: create geometry for terminal and add to node

    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get(), 3.0);
    if (geom)
        node.addChild(geom);

}

void
AerodromeRenderer::apply(WindsockNode& node)
{
    //TODO: create geometry for windsock and add to node

    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get(), 4.0);
    if (geom)
        node.addChild(geom);

}

void
AerodromeRenderer::apply(LightBeaconGroup& group)
{
    //nop
}

void
AerodromeRenderer::apply(LightIndicatorGroup& group)
{
    //nop
}

void
AerodromeRenderer::apply(LinearFeatureGroup& group)
{
    //nop
}

void
AerodromeRenderer::apply(PavementGroup& group)
{
    //nop
}

void
AerodromeRenderer::apply(RunwayGroup& group)
{
    //nop
}

void
AerodromeRenderer::apply(RunwayThresholdGroup& group)
{
    //nop
}

void
AerodromeRenderer::apply(StartupLocationGroup& group)
{
    //nop
}

void
AerodromeRenderer::apply(StopwayGroup& group)
{
    //nop
}

void
AerodromeRenderer::apply(TaxiwayGroup& group)
{
    //nop
}

void
AerodromeRenderer::apply(TerminalGroup& group)
{
    //nop
}

void
AerodromeRenderer::apply(WindsockGroup& group)
{
    //nop
}

void
AerodromeRenderer::apply(osg::Group& node)
{
    if (dynamic_cast<AerodromeNode*>(&node))
        apply(static_cast<AerodromeNode&>(node));
    else if (dynamic_cast<LightBeaconNode*>(&node))
        apply(static_cast<LightBeaconNode&>(node));
    else if (dynamic_cast<LightIndicatorNode*>(&node))
        apply(static_cast<LightIndicatorNode&>(node));
    else if (dynamic_cast<LinearFeatureNode*>(&node))
        apply(static_cast<LinearFeatureNode&>(node));
    else if (dynamic_cast<PavementNode*>(&node))
        apply(static_cast<PavementNode&>(node));
    else if (dynamic_cast<RunwayNode*>(&node))
        apply(static_cast<RunwayNode&>(node));
    else if (dynamic_cast<RunwayThresholdNode*>(&node))
        apply(static_cast<RunwayThresholdNode&>(node));
    else if (dynamic_cast<StartupLocationNode*>(&node))
        apply(static_cast<StartupLocationNode&>(node));
    else if (dynamic_cast<StopwayNode*>(&node))
        apply(static_cast<StopwayNode&>(node));
    else if (dynamic_cast<TaxiwayNode*>(&node))
        apply(static_cast<TaxiwayNode&>(node));
    else if (dynamic_cast<TerminalNode*>(&node))
        apply(static_cast<TerminalNode&>(node));
    else if (dynamic_cast<WindsockNode*>(&node))
        apply(static_cast<WindsockNode&>(node));
    else if (dynamic_cast<LightBeaconGroup*>(&node))
        apply(static_cast<LightBeaconGroup&>(node));
    else if (dynamic_cast<LightIndicatorGroup*>(&node))
        apply(static_cast<LightIndicatorGroup&>(node));
    else if (dynamic_cast<LinearFeatureGroup*>(&node))
        apply(static_cast<LinearFeatureGroup&>(node));
    else if (dynamic_cast<PavementGroup*>(&node))
        apply(static_cast<PavementGroup&>(node));
    else if (dynamic_cast<RunwayGroup*>(&node))
        apply(static_cast<RunwayGroup&>(node));
    else if (dynamic_cast<RunwayThresholdGroup*>(&node))
        apply(static_cast<RunwayThresholdGroup&>(node));
    else if (dynamic_cast<StartupLocationGroup*>(&node))
        apply(static_cast<StartupLocationGroup&>(node));
    else if (dynamic_cast<StopwayGroup*>(&node))
        apply(static_cast<StopwayGroup&>(node));
    else if (dynamic_cast<TaxiwayGroup*>(&node))
        apply(static_cast<TaxiwayGroup&>(node));
    else if (dynamic_cast<TerminalGroup*>(&node))
        apply(static_cast<TerminalGroup&>(node));
    else if (dynamic_cast<WindsockGroup*>(&node))
        apply(static_cast<WindsockGroup&>(node));

    traverse(node);
}

osg::Node*
AerodromeRenderer::randomFeatureRenderer(osgEarth::Features::Feature* feature, double verticalOffset, float height)
{
    if (feature && _map.valid())
    {
        Session* session = new Session( _map.get() );
        GeoExtent extent(feature->getSRS(), feature->getGeometry()->getBounds());
        osg::ref_ptr<FeatureProfile> profile = new FeatureProfile( extent );
        FilterContext context(session, profile.get(), extent );

        //random color
        float r = (float)rand() / (float)RAND_MAX;
        float g = (float)rand() / (float)RAND_MAX;
        float b = (float)rand() / (float)RAND_MAX;
    
        Style style;

        if (feature->getGeometry()->getType() == osgEarth::Symbology::Geometry::TYPE_POLYGON)
        {
            style.getOrCreate<PolygonSymbol>()->fill()->color() = Color(r, g, b, 0.6);
            style.getOrCreate<LineSymbol>()->stroke()->color() = Color(r, g, b, 1.0);
        }
        else if (feature->getGeometry()->getType() == osgEarth::Symbology::Geometry::TYPE_POINTSET)
        {
            style.getOrCreate<PointSymbol>()->fill()->color() = Color(r, g, b, 1.0);
            style.getOrCreate<PointSymbol>()->size() = 5.0f;
        }
        else
        {
            style.getOrCreate<LineSymbol>()->stroke()->color() = Color(r, g, b, 1.0);
        }

        if (height > 0.0)
        {
            style.getOrCreate<ExtrusionSymbol>()->height() = height;
        }

        style.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_NONE;
        style.getOrCreate<AltitudeSymbol>()->verticalOffset() = _elevation;
        style.getOrCreate<RenderSymbol>()->depthOffset()->minBias() = verticalOffset;
        style.getOrCreate<RenderSymbol>()->depthOffset()->automatic() = false;

        GeometryCompiler compiler;
        osg::ref_ptr< Feature > clone = new Feature(*feature, osg::CopyOp::DEEP_COPY_ALL);
        return compiler.compile( clone, (clone->style().isSet() ? *clone->style() : style), context );
    }

    return 0L;
}

void
AerodromeRenderer::createLocalizations(const GeoPoint& anchor)
{
        // get a common elevation for the aerodrome
        ElevationQuery eq(_map.get());
        eq.getElevation(anchor, _elevation, 0.005);

        // create a w2l matrix for the aerodrome
        GeoPoint p(anchor.getSRS(), anchor.x(), anchor.y(), _elevation, ALTMODE_ABSOLUTE);
        p.createLocalToWorld(_local2world);
        _world2local.invert(_local2world);
}

void
AerodromeRenderer::transformAndLocalize(const std::vector<osg::Vec3d>& input,
                                        const SpatialReference*        inputSRS,
                                        osg::Vec3Array*                output_verts,
                                        osg::Vec3Array*                output_normals)
{
    output_verts->reserve( output_verts->size() + input.size() );

    if ( output_normals )
        output_normals->reserve( output_verts->size() );

    ECEF::transformAndLocalize(input, inputSRS, output_verts, output_normals, _map->getSRS(), _world2local);
}