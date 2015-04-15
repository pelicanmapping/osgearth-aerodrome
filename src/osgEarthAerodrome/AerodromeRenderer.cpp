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
#include "WindsockNode"

#include <osgEarthFeatures/Feature>
#include <osgEarthFeatures/GeometryCompiler>


using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Aerodrome;

#define LC "[AerodromeRenderer] "


AerodromeRenderer::AerodromeRenderer(const Map* map)
  : _map(map), osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
{
    //nop
}

void
AerodromeRenderer::apply(RunwayNode& node)
{
    //TODO: create geometry for runway and add to node
#if 0
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get());
    if (geom)
        node.addChild(geom);
#endif
}

void
AerodromeRenderer::apply(TaxiwayNode& node)
{
    //TODO: create geometry for taxiway and add to node
#if 0
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get());
    if (geom)
        node.addChild(geom);
#endif
}

void
AerodromeRenderer::apply(PavementNode& node)
{
    //TODO: create geometry for pavement and add to node
#if 0
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get());
    if (geom)
        node.addChild(geom);
#endif
}

void
AerodromeRenderer::apply(LinearFeatureNode& node)
{
    //TODO: create geometry for linear feature and add to node
#if 0
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get());
    if (geom)
        node.addChild(geom);
#endif
}

void
AerodromeRenderer::apply(LightBeaconNode& node)
{
    //TODO: create geometry for light beacon and add to node
#if 0
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get());
    if (geom)
        node.addChild(geom);
#endif
}

void
AerodromeRenderer::apply(LightIndicatorNode& node)
{
    //TODO: create geometry for light indicator and add to node
#if 0
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get());
    if (geom)
        node.addChild(geom);
#endif
}

void
AerodromeRenderer::apply(RunwayThresholdNode& node)
{
    //TODO: create geometry for runway threshold and add to node
#if 0
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get());
    if (geom)
        node.addChild(geom);
#endif
}

void
AerodromeRenderer::apply(StartupLocationNode& node)
{
    //TODO: create geometry for startup location and add to node
#if 0
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get());
    if (geom)
        node.addChild(geom);
#endif
}

void
AerodromeRenderer::apply(StopwayNode& node)
{
    //TODO: create geometry for stopway and add to node
#if 0
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get());
    if (geom)
        node.addChild(geom);
#endif
}

void
AerodromeRenderer::apply(WindsockNode& node)
{
    //TODO: create geometry for windsock and add to node
#if 0
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = randomFeatureRenderer(feature.get());
    if (geom)
        node.addChild(geom);
#endif
}

void
AerodromeRenderer::apply(osg::Group& node)
{
    //OE_WARN << LC << "apply(RunwayNode& node)" << std::endl;

    if (dynamic_cast<LightBeaconNode*>(&node))
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
    else if (dynamic_cast<WindsockNode*>(&node))
        apply(static_cast<WindsockNode&>(node));

    traverse(node);
}

osg::Node*
AerodromeRenderer::randomFeatureRenderer(osgEarth::Features::Feature* feature)
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

        style.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
        style.getOrCreate<AltitudeSymbol>()->verticalOffset() = 10.0f;
        style.getOrCreate<RenderSymbol>()->depthOffset()->minBias() = 3.6f;
        style.getOrCreate<RenderSymbol>()->depthOffset()->automatic() = false;

        GeometryCompiler compiler;
        osg::ref_ptr< Feature > clone = new Feature(*feature, osg::CopyOp::DEEP_COPY_ALL);
        return compiler.compile( clone, (clone->style().isSet() ? *clone->style() : style), context );
    }

    return 0L;
}