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
#include "AerodromeFeatureOptions"
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
#include <osg/Depth>
#include <osg/Texture2D>
#include <osgEarth/ECEF>
#include <osgEarth/ElevationQuery>
#include <osgEarth/Registry>
#include <osgEarth/Tessellator>
#include <osgEarthFeatures/Feature>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthFeatures/TransformFilter>


using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Aerodrome;

#define LC "[AerodromeRenderer] "


AerodromeRenderer::AerodromeRenderer(const Map* map, const osgDB::Options* options)
  : _map(map), osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
{
    _dbOptions = new osgDB::Options( *options );
    _dbOptions->setObjectCacheHint( (osgDB::Options::CacheHintOptions)(osgDB::Options::CACHE_IMAGES | osgDB::Options::CACHE_NODES) );
}

void
AerodromeRenderer::apply(AerodromeNode& node)
{
    OE_INFO << LC << "Rendering aerodrome: " << node.icao() << "..." << std::endl;

    // don't rerender if unnecessary 
    if (node.getRendered())
        return;

    node.setRendered(true);

    if (node.getBoundary() && node.getBoundary()->getFeature()->getGeometry())
    {
        osg::ref_ptr<BoundaryNode> boundary = node.getBoundary();
        osg::ref_ptr<Geometry> featureGeom = boundary->getFeature()->getGeometry();

        // create localizations for this aerodrome
        createLocalizations(featureGeom->getBounds(), boundary.get());
    }
    else
    {
        // node does not have a boundary or geometry so do not render
        return;
    }

    node.getOrCreateStateSet()->setAttributeAndModes( new osg::Depth(osg::Depth::LEQUAL, 0.0, 0.9999, false) );

    traverse(node);

    OE_INFO << LC << "...finished rendering aerodrome: " << node.icao() << std::endl;
}

void
AerodromeRenderer::apply(LightBeaconNode& node)
{
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();

    osg::Node* geom;

    if (node.getOptions().modelOptions().size() > 0)
        geom = featureModelRenderer(feature.get(), node.getOptions().modelOptions());
    else
        geom = defaultFeatureRenderer(feature.get(), Color::Yellow);

    if (geom)
        node.addChild(geom);
}

void
AerodromeRenderer::apply(LightIndicatorNode& node)
{
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();

    osg::Node* geom;

    if (node.getOptions().modelOptions().size() > 0)
        geom = featureModelRenderer(feature.get(), node.getOptions().modelOptions());
    else
        geom = defaultFeatureRenderer(feature.get(), Color::Red);

    if (geom)
    {
        Registry::shaderGenerator().run(geom, "osgEarth.AerodromeRenderer");
        node.addChild(geom);
    }
}

void
AerodromeRenderer::apply(LinearFeatureNode& node)
{
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    osg::Node* geom = defaultFeatureRenderer(feature.get(), Color(0.75f, 0.65f, 0.15f, 0.8f));
    if (geom)
        node.addChild(geom);
}

void
AerodromeRenderer::apply(PavementNode& node)
{
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();

    osg::ref_ptr<osg::Node> geom;

    if (node.getOptions().textureOptions().isSet() && node.getOptions().textureOptions()->url().isSet())
    {
        geom = featureSingleTextureRenderer(feature.get(), node.getOptions().textureOptions()->url().value(), node.getOptions().textureOptions()->length().isSet() ? node.getOptions().textureOptions()->length().value() : 10.0);
    }
    else
    {
        geom = defaultFeatureRenderer(feature.get(), Color(0.6, 0.6, 0.6, 1.0));
    }

    if (geom.valid())
    {
        geom->setName(node.icao() + "_PAVEMENT");
        node.addChild(geom);
    }
}

void
AerodromeRenderer::apply(RunwayNode& node)
{
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();

    osg::ref_ptr<osg::Node> geom;

    osg::ref_ptr<osg::Vec3dArray> geomPoints = feature->getGeometry()->toVec3dArray();
    if (geomPoints.valid() && geomPoints->size() == 4)
    {
        std::vector<osg::Vec3d> featurePoints;
        for (int i=0; i < geomPoints->size(); i++)
        {
            featurePoints.push_back(osg::Vec3d((*geomPoints)[i].x(), (*geomPoints)[i].y(), _elevation));
        }
     
        //osg::Vec3Array* normals = new osg::Vec3Array();
        osg::Vec3Array* verts = new osg::Vec3Array();
        transformAndLocalize(featurePoints, _map->getSRS(), verts, 0L);

        osg::Geometry* geometry = new osg::Geometry();
        geometry->setVertexArray( verts );

        osg::Vec3Array* normals = new osg::Vec3Array();
        normals->push_back( osg::Vec3(0.0f, 0.0f, 1.0f) );
        geometry->setNormalArray( normals );
        geometry->setNormalBinding( osg::Geometry::BIND_OVERALL );

        osg::Vec4Array* colors = new osg::Vec4Array;

        if (node.getOptions().textureOptions().isSet() && node.getOptions().textureOptions()->url().isSet())
        {
            osg::Image* tex = node.getOptions().textureOptions()->url()->getImage(_dbOptions);
            if (tex)
            {
                osg::Texture2D* _texture = new osg::Texture2D(tex);
                _texture->setWrap(_texture->WRAP_S, _texture->CLAMP_TO_EDGE);
                _texture->setWrap(_texture->WRAP_T, _texture->REPEAT);
                //_texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST); 
                //_texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
                _texture->setResizeNonPowerOfTwoHint(false);
                geometry->getOrCreateStateSet()->setTextureAttributeAndModes(0, _texture, osg::StateAttribute::ON);

                osg::Vec2Array* tcoords = new osg::Vec2Array(4);

                float side1 = ((*verts)[1] - (*verts)[0]).length();
                float side2 = ((*verts)[2] - (*verts)[1]).length();

                float width = osg::minimum(side1, side2);
                float scale = width / tex->getPixelAspectRatio();

                float length = osg::maximum(side1, side2);
                float repeat = length / scale;

                if (side1 > side2)
                {
                    (*tcoords)[3].set(0.0f,0.0f);
                    (*tcoords)[0].set(1.0f,0.0f);
                    (*tcoords)[1].set(1.0f,repeat);
                    (*tcoords)[2].set(0.0f,repeat);
                }
                else
                {
                    (*tcoords)[0].set(0.0f,0.0f);
                    (*tcoords)[1].set(1.0f,0.0f);
                    (*tcoords)[2].set(1.0f,repeat);
                    (*tcoords)[3].set(0.0f,repeat);
                }
            
                geometry->setTexCoordArray(0,tcoords);
            }
            else
            {
                OE_WARN << LC << "Error reading texture file: " << node.getOptions().textureOptions()->url()->full() << std::endl;
            }



            colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
        }
        else
        {
            colors->push_back(osg::Vec4(0.4f,0.4f,0.4f,1.0f));
        }

        geometry->setColorArray(colors);
        geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

        geometry->addPrimitiveSet( new osg::DrawArrays( GL_POLYGON, 0, verts->size() ) );

        geometry->setName(node.icao() + "_RUNWAY");

        //osgEarth::Tessellator tess;
        //tess.tessellateGeometry(*geometry);

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable(geometry);

        Registry::shaderGenerator().run(geode.get(), "osgEarth.AerodromeRenderer");

        osg::MatrixTransform* mt = new osg::MatrixTransform();
        mt->setMatrix(_local2world);
        mt->addChild(geode);

        geom = mt;
    }
    else
    {
        geom = defaultFeatureRenderer(feature.get(), Color(0.4, 0.4, 0.4, 1.0));
    }

    if (geom.valid())
    {
        node.addChild(geom);
    }
}

void
AerodromeRenderer::apply(RunwayThresholdNode& node)
{
    // We don't necessarily want to render statup locations (metadata)
    //osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    //osg::Node* geom = defaultFeatureRenderer(feature.get(), Color::White);
    //if (geom)
    //    node.addChild(geom);
}

void
AerodromeRenderer::apply(StartupLocationNode& node)
{
    // We don't necessarily want to render statup locations (metadata)
    //osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    //osg::Node* geom = defaultFeatureRenderer(feature.get(), Color::White);
    //if (geom)
    //    node.addChild(geom);
}

void
AerodromeRenderer::apply(StopwayNode& node)
{
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();

    osg::ref_ptr<osg::Node> geom;

    osg::ref_ptr<osg::Vec3dArray> geomPoints = feature->getGeometry()->toVec3dArray();
    if (geomPoints.valid() && geomPoints->size() == 4)
    {
        std::vector<osg::Vec3d> featurePoints;
        for (int i=0; i < geomPoints->size(); i++)
        {
            featurePoints.push_back(osg::Vec3d((*geomPoints)[i].x(), (*geomPoints)[i].y(), _elevation));
        }
     
        //osg::Vec3Array* normals = new osg::Vec3Array();
        osg::Vec3Array* verts = new osg::Vec3Array();
        transformAndLocalize(featurePoints, _map->getSRS(), verts, 0L);

        osg::Geometry* geometry = new osg::Geometry();
        geometry->setVertexArray( verts );

        osg::Vec3Array* normals = new osg::Vec3Array();
        normals->push_back( osg::Vec3(0.0f, 0.0f, 1.0f) );
        geometry->setNormalArray( normals );
        geometry->setNormalBinding( osg::Geometry::BIND_OVERALL );

        osg::Vec4Array* colors = new osg::Vec4Array;

        if (node.getOptions().textureOptions().isSet() && node.getOptions().textureOptions()->url().isSet())
        {
            osg::Image* tex = node.getOptions().textureOptions()->url()->getImage(_dbOptions);
            if (tex)
            {
                osg::Texture2D* _texture = new osg::Texture2D(tex);
                _texture->setWrap(_texture->WRAP_S, _texture->CLAMP_TO_EDGE);
                _texture->setWrap(_texture->WRAP_T, _texture->REPEAT);
                //_texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST); 
                //_texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
                _texture->setResizeNonPowerOfTwoHint(false);
                geometry->getOrCreateStateSet()->setTextureAttributeAndModes(0, _texture, osg::StateAttribute::ON);

                osg::Vec2Array* tcoords = new osg::Vec2Array(4);

                osg::Vec3d refPoint = transformAndLocalize(node.getReferencePoint(), _map->getSRS());

                float side1 = (refPoint - (((*verts)[1] + (*verts)[0]) / 2.0)).length();
                float side2 = (refPoint - (((*verts)[2] + (*verts)[1]) / 2.0)).length();
                float side3 = (refPoint - (((*verts)[3] + (*verts)[2]) / 2.0)).length();
                float side4 = (refPoint - (((*verts)[0] + (*verts)[3]) / 2.0)).length();

                if (side1 < side2 && side1 < side3 && side1 < side4)
                {
                    float width = ((*verts)[1] - (*verts)[0]).length();
                    float scale = width / tex->getPixelAspectRatio();
                    float length = ((*verts)[0] - (*verts)[3]).length();
                    float repeat = length / scale;

                    (*tcoords)[2].set(0.0f,0.0f);
                    (*tcoords)[3].set(1.0f,0.0f);
                    (*tcoords)[0].set(1.0f,repeat);
                    (*tcoords)[1].set(0.0f,repeat);
                }
                else if (side2 < side3 && side2 < side4)
                {
                    float width = ((*verts)[2] - (*verts)[1]).length();
                    float scale = width / tex->getPixelAspectRatio();
                    float length = ((*verts)[1] - (*verts)[0]).length();
                    float repeat = length / scale;

                    (*tcoords)[3].set(0.0f,0.0f);
                    (*tcoords)[0].set(1.0f,0.0f);
                    (*tcoords)[1].set(1.0f,repeat);
                    (*tcoords)[2].set(0.0f,repeat);
                }
                else if (side3 < side4)
                {
                    float width = ((*verts)[3] - (*verts)[2]).length();
                    float scale = width / tex->getPixelAspectRatio();
                    float length = ((*verts)[2] - (*verts)[1]).length();
                    float repeat = length / scale;

                    (*tcoords)[0].set(0.0f,0.0f);
                    (*tcoords)[1].set(1.0f,0.0f);
                    (*tcoords)[2].set(1.0f,repeat);
                    (*tcoords)[3].set(0.0f,repeat);
                }
                else
                {
                    float width = ((*verts)[0] - (*verts)[3]).length();
                    float scale = width / tex->getPixelAspectRatio();
                    float length = ((*verts)[3] - (*verts)[2]).length();
                    float repeat = length / scale;

                    (*tcoords)[1].set(0.0f,0.0f);
                    (*tcoords)[2].set(1.0f,0.0f);
                    (*tcoords)[3].set(1.0f,repeat);
                    (*tcoords)[0].set(0.0f,repeat);
                }

                geometry->setTexCoordArray(0,tcoords);
            }
            else
            {
                OE_WARN << LC << "Error reading texture file: " << node.getOptions().textureOptions()->url()->full() << std::endl;
            }



            colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
        }
        else
        {
            colors->push_back(osg::Vec4(0.4f,0.4f,0.4f,1.0f));
        }

        geometry->setColorArray(colors);
        geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

        geometry->addPrimitiveSet( new osg::DrawArrays( GL_POLYGON, 0, verts->size() ) );

        geometry->setName(node.icao() + "_STOPWAY");

        //osgEarth::Tessellator tess;
        //tess.tessellateGeometry(*geometry);

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable(geometry);

        Registry::shaderGenerator().run(geode.get(), "osgEarth.AerodromeRenderer");

        osg::MatrixTransform* mt = new osg::MatrixTransform();
        mt->setMatrix(_local2world);
        mt->addChild(geode);

        geom = mt;
    }
    else
    {
        geom = defaultFeatureRenderer(feature.get(), Color::White);
    }

    if (geom)
        node.addChild(geom);
}

void
AerodromeRenderer::apply(TaxiwayNode& node)
{
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();

    osg::ref_ptr<osg::Node> geom;

    if (node.getOptions().textureOptions().isSet() && node.getOptions().textureOptions()->url().isSet())
    {
        geom = featureSingleTextureRenderer(feature.get(), node.getOptions().textureOptions()->url().value(), node.getOptions().textureOptions()->length().isSet() ? node.getOptions().textureOptions()->length().value() : 10.0);
    }
    else
    {
        geom = defaultFeatureRenderer(feature.get(), Color(0.6, 0.6, 0.6, 1.0));
    }

    if (geom.valid())
    {
        geom->setName(node.icao() + "_TAXIWAY");
        node.addChild(geom);
    }
}

void
AerodromeRenderer::apply(TerminalNode& node)
{
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();

    osg::Node* geom;
    
    if (node.getOptions().skinsUrl().isSet())
    {
        ResourceLibrary* reslib = new ResourceLibrary( "terminal_resources", node.getOptions().skinsUrl().value() );

        // a style for the building data:
        Style buildingStyle;
        buildingStyle.setName( "buildings" );

        // Extrude the shapes into 3D buildings.
        ExtrusionSymbol* extrusion = buildingStyle.getOrCreate<ExtrusionSymbol>();
        extrusion->height() = 25.0;
        extrusion->flatten() = true;
        extrusion->wallStyleName() = "building-wall";
        extrusion->roofStyleName() = "building-roof";

        PolygonSymbol* poly = buildingStyle.getOrCreate<PolygonSymbol>();
        poly->fill()->color() = Color::White;

        // Clamp the buildings to the terrain.
        AltitudeSymbol* alt = buildingStyle.getOrCreate<AltitudeSymbol>();
        alt->clamping() = AltitudeSymbol::CLAMP_NONE;
        alt->verticalOffset() = _elevation;

        // a style for the wall textures:
        Style wallStyle;
        wallStyle.setName( "building-wall" );
        SkinSymbol* wallSkin = wallStyle.getOrCreate<SkinSymbol>();
        wallSkin->library() = "terminal_resources";

        for (std::vector<std::string>::const_iterator it = node.getOptions().wallTags().begin(); it != node.getOptions().wallTags().end(); ++it)
            wallSkin->addTag( *it );

        if (wallSkin->tags().size() == 0)
          wallSkin->addTag("building");

        //wallSkin->randomSeed() = 1;

        // a style for the rooftop textures:
        Style roofStyle;
        roofStyle.setName( "building-roof" );
        SkinSymbol* roofSkin = roofStyle.getOrCreate<SkinSymbol>();
        roofSkin->library() = "terminal_resources";

        for (std::vector<std::string>::const_iterator it = node.getOptions().roofTags().begin(); it != node.getOptions().roofTags().end(); ++it)
            roofSkin->addTag( *it );

        if (roofSkin->tags().size() == 0)
          roofSkin->addTag("rooftop");

        //roofSkin->randomSeed() = 1;
        roofSkin->isTiled() = true;

        // assemble a stylesheet and add our styles to it:
        osg::ref_ptr<StyleSheet> styleSheet = new StyleSheet();
        styleSheet->addResourceLibrary(reslib);
        styleSheet->addStyle( wallStyle );
        styleSheet->addStyle( roofStyle );

        geom = defaultFeatureRenderer(feature.get(), buildingStyle, styleSheet.get());
    }
    else
    {
        geom = defaultFeatureRenderer(feature.get(), Color::Red, 25.0);
    }

    if (geom)
    {
        geom->getOrCreateStateSet()->setAttributeAndModes( new osg::Depth(osg::Depth::LEQUAL, 0.0, 1.0, true) );
        node.addChild(geom);
    }

}

void
AerodromeRenderer::apply(WindsockNode& node)
{
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();

    osg::Node* geom;

    if (node.getOptions().modelOptions().size() > 0)
        geom = featureModelRenderer(feature.get(), node.getOptions().modelOptions());
    else
        geom = defaultFeatureRenderer(feature.get(), Color::Orange);

    if (geom)
        node.addChild(geom);
}

void
AerodromeRenderer::apply(LightBeaconGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(LightIndicatorGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(LinearFeatureGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(PavementGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(RunwayGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(RunwayThresholdGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(StartupLocationGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(StopwayGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(TaxiwayGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(TerminalGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(WindsockGroup& group)
{
    traverse(group);
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
    else
        traverse(node);
}

template <typename T, typename Y>
osgEarth::Symbology::Geometry* AerodromeRenderer::combineGeometries(T& group)
{
    osg::ref_ptr<Geometry> combGeom = 0L;
    for (int c=0; c < group.getNumChildren(); c++)
    {
        Y* childNode = dynamic_cast<Y*>(group.getChild(c));
        if (childNode)
        {
            osg::ref_ptr<osgEarth::Features::Feature> feature = childNode->getFeature();
            Polygon* featurePoly = dynamic_cast<Polygon*>(feature->getGeometry());
            if (featurePoly)
            {
                if (!combGeom.valid())
                {
                    combGeom = new Polygon(*featurePoly);
                }
                else
                {
                    osg::ref_ptr<Geometry> copy = combGeom.get();
                    combGeom = 0L;

                    if (!featurePoly->geounion(copy, combGeom))
                    {
                        
                        OE_WARN << LC << "GEOS union failed. (" << childNode->icao() << ")" << std::endl;
                    }
                }
            }
        }
    }

    return combGeom.release();
}

osg::Node*
AerodromeRenderer::featureSingleTextureRenderer(osgEarth::Features::Feature* feature, const osgEarth::URI& uri, float length)
{
    osg::ref_ptr<osg::Node> geom;
    osg::ref_ptr<osg::Vec3dArray> geomPoints = feature->getGeometry()->toVec3dArray();
    if (geomPoints.valid() && geomPoints->size() > 2)
    {
        std::vector<osg::Vec3d> featurePoints;
        for (int i=0; i < geomPoints->size(); i++)
        {
            featurePoints.push_back(osg::Vec3d((*geomPoints)[i].x(), (*geomPoints)[i].y(), _elevation));
        }

        osg::Vec3Array* verts = new osg::Vec3Array();
        transformAndLocalize(featurePoints, _map->getSRS(), verts, 0L);

        osg::Geometry* geometry = new osg::Geometry();
        geometry->setVertexArray( verts );

        osg::Vec3Array* normals = new osg::Vec3Array();
        normals->push_back( osg::Vec3(0.0f, 0.0f, 1.0f) );
        geometry->setNormalArray( normals );
        geometry->setNormalBinding( osg::Geometry::BIND_OVERALL );

        osg::Vec4Array* colors = new osg::Vec4Array;

        osg::Image* tex = uri.getImage(_dbOptions);
        if (tex)
        {
            osg::Texture2D* _texture = new osg::Texture2D(tex);     
            _texture->setWrap(_texture->WRAP_S, _texture->REPEAT);
            _texture->setWrap(_texture->WRAP_T, _texture->REPEAT);
            _texture->setResizeNonPowerOfTwoHint(false);
            geometry->getOrCreateStateSet()->setTextureAttributeAndModes(0, _texture, osg::StateAttribute::ON);

            osg::Vec2Array* tcoords = new osg::Vec2Array(verts->size());

            for (int i=0; i < verts->size(); i++)
            {
                float tcX = ((*verts)[i].x() - _localMin.x()) / length;
                float tcY = ((*verts)[i].y() - _localMin.y()) / length;

                (*tcoords)[i].set(tcX, tcY);
            }
            
            geometry->setTexCoordArray(0,tcoords);
        }
        else
        {
            OE_WARN << LC << "Error reading texture file: " << uri.full() << std::endl;
        }

        colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));

        geometry->setColorArray(colors);
        geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

        geometry->addPrimitiveSet( new osg::DrawArrays( GL_POLYGON, 0, verts->size() ) );

        osgEarth::Tessellator tess;
        tess.tessellateGeometry(*geometry);

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable(geometry);

        Registry::shaderGenerator().run(geode.get(), "osgEarth.AerodromeRenderer");

        osg::MatrixTransform* mt = new osg::MatrixTransform();
        mt->setMatrix(_local2world);
        mt->addChild(geode);

        geom = mt;
    }

    return geom.release();
}

osg::Node*
AerodromeRenderer::featureModelRenderer(osgEarth::Features::Feature* feature, const ModelOptionsSet& modelOptions)
{
    Style style;

    ModelSymbol* model = style.getOrCreate<ModelSymbol>();
    model->scale()->setLiteral( 1.0 );
    model->placement() = model->PLACEMENT_VERTEX;

    for (ModelOptionsSet::const_iterator i = modelOptions.begin(); i != modelOptions.end(); i++)
    {
        if (!i->selector().isSet() ||
            (i->selector()->attr().isSet() && i->selector()->value().isSet() && feature->getString(i->selector()->attr().value()) == i->selector()->value().value()))
        {
            // construct url string w/ scale in necessary
            float scale = i->scale().isSet() ? i->scale().get() : 1.0f;
            std::string url = i->url().isSet() ? i->url().value().full() : "";
            std::string modelUrl = url + (scale != 1.0f ? "." + toString(scale) + ".scale" : "");
            model->url()->setLiteral(modelUrl);
        }
    }

    style.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_NONE;
    style.getOrCreate<AltitudeSymbol>()->verticalOffset() = _elevation;

    return defaultFeatureRenderer(feature, style);
}

osg::Node*
AerodromeRenderer::defaultFeatureRenderer(osgEarth::Features::Feature* feature, float height)
{
    //random color
    float r = (float)rand() / (float)RAND_MAX;
    float g = (float)rand() / (float)RAND_MAX;
    float b = (float)rand() / (float)RAND_MAX;

    return defaultFeatureRenderer(feature, Color(r, g, b, 1.0), height);
}

osg::Node*
AerodromeRenderer::defaultFeatureRenderer(osgEarth::Features::Feature* feature, const Color& color, float height)
{
    Style style;

    if (feature->getGeometry()->getType() == osgEarth::Symbology::Geometry::TYPE_POLYGON)
    {
        style.getOrCreate<PolygonSymbol>()->fill()->color() = color;
    }
    else if (feature->getGeometry()->getType() == osgEarth::Symbology::Geometry::TYPE_POINTSET)
    {
        style.getOrCreate<PointSymbol>()->fill()->color() = color;
        style.getOrCreate<PointSymbol>()->size() = 2.0f;
    }
    else
    {
        style.getOrCreate<LineSymbol>()->stroke()->color() = color;
        style.getOrCreate<LineSymbol>()->stroke()->width() = 1.0f;
        style.getOrCreate<LineSymbol>()->stroke()->widthUnits() = Units::METERS;
    }

    if (height > 0.0)
    {
        style.getOrCreate<ExtrusionSymbol>()->height() = height;
    }

    style.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_NONE;
    style.getOrCreate<AltitudeSymbol>()->verticalOffset() = _elevation;

    return defaultFeatureRenderer(feature, style);
}

osg::Node*
AerodromeRenderer::defaultFeatureRenderer(osgEarth::Features::Feature* feature, const Style& style, StyleSheet* styleSheet)
{
    if (feature && _map.valid())
    {
        Session* session = new Session( _map.get() );
        if (styleSheet)
            session->setStyles(styleSheet);

        GeoExtent extent(feature->getSRS(), feature->getGeometry()->getBounds());
        osg::ref_ptr<FeatureProfile> profile = new FeatureProfile( extent );
        FilterContext context(session, profile.get(), extent );

        GeometryCompiler compiler;
        osg::ref_ptr< Feature > clone = new Feature(*feature, osg::CopyOp::DEEP_COPY_ALL);
        return compiler.compile( clone, (clone->style().isSet() ? *clone->style() : style), context );
    }

    return 0L;
}

void
AerodromeRenderer::createLocalizations(const osgEarth::Bounds& bounds, BoundaryNode* boundary)
{
    // use the center of the bounds as an anchor for localization
    GeoPoint anchor(_map->getSRS(), bounds.center().x(), bounds.center().y());

    // get a common elevation for the aerodrome
    if (boundary && boundary->hasElevation())
    {
        _elevation = boundary->elevation();
    }
    else
    {
        ElevationQuery eq(_map.get());
        eq.getElevation(anchor, _elevation, 0.005);
    }

    // create a w2l matrix for the aerodrome
    GeoPoint p(anchor.getSRS(), anchor.x(), anchor.y(), _elevation, ALTMODE_ABSOLUTE);
    p.createLocalToWorld(_local2world);
    _world2local.invert(_local2world);

    // find the local min point (lower-left), used for calculating site-wide texture coords
    GeoPoint vert(_map->getSRS(), osg::Vec3d(bounds.xMin(), bounds.yMin(), _elevation), osgEarth::ALTMODE_ABSOLUTE);

    osg::Vec3d world;
    vert.toWorld(world);
    _localMin = world * _world2local;
    _localMin.z() = 0.0;
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

    for (int i=0; i < input.size(); i++)
    {
        GeoPoint vert(inputSRS, input[i], osgEarth::ALTMODE_ABSOLUTE);

        osg::Vec3d world;
        vert.toWorld(world);
        osg::Vec3d local = world * _world2local;
        local.z() = 0.0;
        output_verts->push_back(local);
    }
}

osg::Vec3d
AerodromeRenderer::transformAndLocalize(const osg::Vec3d& input, const SpatialReference* inputSRS)
{
    GeoPoint vert(inputSRS, input, osgEarth::ALTMODE_ABSOLUTE);

    osg::Vec3d world;
    vert.toWorld(world);
    osg::Vec3d local = world * _world2local;
    local.z() = 0.0;

    return local;
}
