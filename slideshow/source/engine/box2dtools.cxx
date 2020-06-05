/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/*
 * This file is part of the LibreOffice project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * This file incorporates work covered by the following license notice:
 *
 *   Licensed to the Apache Software Foundation (ASF) under one or more
 *   contributor license agreements. See the NOTICE file distributed
 *   with this work for additional information regarding copyright
 *   ownership. The ASF licenses this file to you under the Apache
 *   License, Version 2.0 (the "License"); you may not use this file
 *   except in compliance with the License. You may obtain a copy of
 *   the License at http://www.apache.org/licenses/LICENSE-2.0 .
 */

#include <box2dtools.hxx>
#include <Box2D/Box2D.h>
#include <shapemanager.hxx>

#define BOX2D_SLIDE_SIZE_IN_METERS 100.00f

namespace box2d::utils
{
enum box2DNonsimulatedShapeUpdateType
{
    BOX2D_UPDATE_POSITION,
    BOX2D_UPDATE_ANGLE,
    BOX2D_UPDATE_SIZE,
    BOX2D_UPDATE_VISIBILITY_APPEAR,
    BOX2D_UPDATE_VISIBILITY_DISAPPEAR
};

struct Box2DShapeUpdateInformation
{
    css::uno::Reference<css::drawing::XShape> mxShape;
    union {
        ::basegfx::B2DPoint maPosition;
        double mfAngle;
    };
    box2DNonsimulatedShapeUpdateType meUpdateType;
};

double calculateScaleFactor(const ::basegfx::B2DVector& rSlideSize)
{
    double fWidth = rSlideSize.getX();
    double fHeight = rSlideSize.getY();

    if (fWidth > fHeight)
        return BOX2D_SLIDE_SIZE_IN_METERS / fWidth;
    else
        return BOX2D_SLIDE_SIZE_IN_METERS / fHeight;
}

b2BodyType getBox2DInternalBodyType(box2DBodyType eType)
{
    switch (eType)
    {
        case BOX2D_STATIC_BODY:
            return b2_staticBody;
        case BOX2D_KINEMATIC_BODY:
            return b2_kinematicBody;
        case BOX2D_DYNAMIC_BODY:
            return b2_dynamicBody;
    }
}

box2DBodyType getBox2DLOBodyType(b2BodyType eType)
{
    switch (eType)
    {
        case b2_staticBody:
            return BOX2D_STATIC_BODY;
        case b2_kinematicBody:
            return BOX2D_KINEMATIC_BODY;
        case b2_dynamicBody:
            return BOX2D_DYNAMIC_BODY;
    }
}

box2DWorld::box2DWorld(const ::basegfx::B2DVector& rSlideSize)
    : mpBox2DWorld()
    , mfScaleFactor(calculateScaleFactor(rSlideSize))
    , mbShapesInitialized(false)
    , mbHasWorldStepper(false)
    , mnDynamicShapeCount(0)
    , mpXShapeToBodyMap()
    , maShapeUpdateQueue()
{
}

box2DWorld::~box2DWorld() = default;

bool box2DWorld::initiateWorld(const ::basegfx::B2DVector& rSlideSize)
{
    if (!mpBox2DWorld)
    {
        mpBox2DWorld = std::make_unique<b2World>(b2Vec2(0.0f, -30.0f));
        createStaticFrameAroundSlide(rSlideSize);
        return false;
    }
    else
    {
        return true;
    }
}

b2Body* box2DWorld::createStaticFrameAroundSlide(const ::basegfx::B2DVector& rSlideSize)
{
    assert(mpBox2DWorld);

    float fWidth = static_cast<float>(rSlideSize.getX() * mfScaleFactor);
    float fHeight = static_cast<float>(rSlideSize.getY() * mfScaleFactor);

    // static body for creating the frame around the slide
    b2BodyDef aBodyDef;
    aBodyDef.type = b2_staticBody;
    aBodyDef.position.Set(0, 0);
    b2Body* pStaticBody = mpBox2DWorld->CreateBody(&aBodyDef);

    b2Vec2 aEdgePoints[4];
    aEdgePoints[0].Set(0, 0);
    aEdgePoints[1].Set(0, -fHeight);
    aEdgePoints[2].Set(fWidth, -fHeight);
    aEdgePoints[3].Set(fWidth, 0);

    b2ChainShape aEdgesChainShape;
    aEdgesChainShape.CreateLoop(aEdgePoints, 4);

    b2FixtureDef aFixtureDef;
    aFixtureDef.shape = &aEdgesChainShape;
    pStaticBody->CreateFixture(&aFixtureDef);

    return pStaticBody;
}

void box2DWorld::setShapePositionByLinearVelocity(
    const css::uno::Reference<com::sun::star::drawing::XShape> xShape,
    const basegfx::B2DPoint& rOutPos, const double fPassedTime)
{
    assert(mpBox2DWorld);
    if (fPassedTime > 0) // this only makes sense if there was an advance in time
    {
        Box2DBodySharedPtr pBox2DBody = mpXShapeToBodyMap.find(xShape)->second;

        pBox2DBody->setPositionByLinearVelocity(rOutPos, mfScaleFactor, fPassedTime);
    }
}

void box2DWorld::processUpdateQueue(const double fPassedTime)
{
    while (!maShapeUpdateQueue.empty())
    {
        auto aQueueElement = maShapeUpdateQueue.front();
        maShapeUpdateQueue.pop();

        switch (aQueueElement.meUpdateType)
        {
            case BOX2D_UPDATE_POSITION:
                setShapePositionByLinearVelocity(aQueueElement.mxShape, aQueueElement.maPosition,
                                                 fPassedTime);
                break;
            case BOX2D_UPDATE_ANGLE:
            case BOX2D_UPDATE_SIZE:
            case BOX2D_UPDATE_VISIBILITY_APPEAR:
            case BOX2D_UPDATE_VISIBILITY_DISAPPEAR:
                break;
        }
    }
}

void box2DWorld::initateAllShapesAsStaticBodies(
    const slideshow::internal::ShapeManagerSharedPtr pShapeManager)
{
    assert(mpBox2DWorld);

    mbShapesInitialized = true;
    auto aXShapeToShapeMap = pShapeManager->getXShapeToShapeMap();

    // iterate over shapes in the current slide
    for (auto aIt = aXShapeToShapeMap.begin(); aIt != aXShapeToShapeMap.end(); aIt++)
    {
        slideshow::internal::ShapeSharedPtr pShape = aIt->second;
        if (pShape->isVisible() && pShape->isForeground())
        {
            Box2DBodySharedPtr pBox2DBody = createStaticBodyFromBoundingBox(pShape);
            mpXShapeToBodyMap.insert(std::make_pair(pShape->getXShape(), pBox2DBody));
        }
    }
}

bool box2DWorld::hasWorldStepper() { return mbHasWorldStepper; }

void box2DWorld::setHasWorldStepper(bool bHasWorldStepper) { mbHasWorldStepper = bHasWorldStepper; }

void box2DWorld::queryPositionUpdate(css::uno::Reference<com::sun::star::drawing::XShape> xShape,
                                     basegfx::B2DPoint rOutPos)
{
    maShapeUpdateQueue.push({
        xShape, // mxShape
        { .maPosition = rOutPos },
        BOX2D_UPDATE_POSITION // meUpdateType
    });
}

void box2DWorld::step(const float fTimeStep, const int nVelocityIterations,
                      const int nPositionIterations)
{
    assert(mpBox2DWorld);
    mpBox2DWorld->Step(fTimeStep, nVelocityIterations, nPositionIterations);
}

double box2DWorld::stepAmount(double fPassedTime, const float fTimeStep,
                              const int nVelocityIterations, const int nPositionIterations)
{
    assert(mpBox2DWorld);

    unsigned int nStepAmount = static_cast<unsigned int>(std::round(fPassedTime / fTimeStep));
    double fTimeSteppedThrough = fTimeStep * nStepAmount;

    processUpdateQueue(fTimeSteppedThrough);

    for (unsigned int nStepCounter = 0; nStepCounter < nStepAmount; nStepCounter++)
    {
        step(fTimeStep, nVelocityIterations, nPositionIterations);
    }

    return fTimeSteppedThrough;
}

bool box2DWorld::getShapesInitialized() { return mbShapesInitialized; }

bool box2DWorld::getWorldInitialized()
{
    if (mpBox2DWorld)
        return true;
    else
        return false;
}

Box2DBodySharedPtr box2DWorld::makeShapeDynamic(const slideshow::internal::ShapeSharedPtr pShape)
{
    assert(mpBox2DWorld);
    Box2DBodySharedPtr pBox2DBody = mpXShapeToBodyMap.find(pShape->getXShape())->second;
    if (pBox2DBody->getType() != BOX2D_DYNAMIC_BODY)
    {
        pBox2DBody->setType(BOX2D_DYNAMIC_BODY);
        mnDynamicShapeCount++;
    }
    return pBox2DBody;
}

Box2DBodySharedPtr box2DWorld::makeBodyDynamic(const Box2DBodySharedPtr pBox2DBody)
{
    assert(mpBox2DWorld);
    if (pBox2DBody->getType() != BOX2D_DYNAMIC_BODY)
    {
        pBox2DBody->setType(BOX2D_DYNAMIC_BODY);
        mnDynamicShapeCount++;
    }
    return pBox2DBody;
}

Box2DBodySharedPtr box2DWorld::makeShapeStatic(const slideshow::internal::ShapeSharedPtr pShape)
{
    assert(mpBox2DWorld);
    Box2DBodySharedPtr pBox2DBody = mpXShapeToBodyMap.find(pShape->getXShape())->second;
    if (pBox2DBody->getType() != BOX2D_STATIC_BODY)
    {
        pBox2DBody->setType(BOX2D_STATIC_BODY);
        mnDynamicShapeCount--;
    }
    return pBox2DBody;
}

Box2DBodySharedPtr box2DWorld::makeBodyStatic(const Box2DBodySharedPtr pBox2DBody)
{
    assert(mpBox2DWorld);
    if (pBox2DBody->getType() != BOX2D_STATIC_BODY)
    {
        pBox2DBody->setType(BOX2D_STATIC_BODY);
        mnDynamicShapeCount--;
    }
    return pBox2DBody;
}

Box2DBodySharedPtr box2DWorld::createDynamicBodyFromBoundingBox(
    const slideshow::internal::ShapeSharedPtr& rShape,
    const slideshow::internal::ShapeAttributeLayerSharedPtr& rAttrLayer, const float fDensity,
    const float fFriction)
{
    assert(mpBox2DWorld);
    ::basegfx::B2DRectangle aShapeBounds = rShape->getBounds();
    double fShapeWidth = aShapeBounds.getWidth() * mfScaleFactor;
    double fShapeHeight = aShapeBounds.getHeight() * mfScaleFactor;

    double fRotationAngle = ::basegfx::deg2rad(-rAttrLayer->getRotationAngle());

    ::basegfx::B2DPoint aShapePosition = aShapeBounds.getCenter();
    float fBodyPosX = aShapePosition.getX() * mfScaleFactor;
    float fBodyPosY = aShapePosition.getY() * -mfScaleFactor;

    b2BodyDef aBodyDef;
    aBodyDef.type = b2_dynamicBody;
    aBodyDef.position.Set(fBodyPosX, fBodyPosY);
    aBodyDef.angle = static_cast<float>(fRotationAngle);

    b2Body* pBody = mpBox2DWorld->CreateBody(&aBodyDef);

    b2PolygonShape aDynamicBox;
    aDynamicBox.SetAsBox(static_cast<float>(fShapeWidth / 2), static_cast<float>(fShapeHeight / 2));

    b2FixtureDef aFixtureDef;
    aFixtureDef.shape = &aDynamicBox;
    aFixtureDef.density = fDensity;
    aFixtureDef.friction = fFriction;

    pBody->CreateFixture(&aFixtureDef);
    return std::make_shared<box2DBody>(pBody, mfScaleFactor);
}

Box2DBodySharedPtr
box2DWorld::createStaticBodyFromBoundingBox(const slideshow::internal::ShapeSharedPtr& rShape,
                                            const float fDensity, const float fFriction)
{
    assert(mpBox2DWorld);
    ::basegfx::B2DRectangle aShapeBounds = rShape->getBounds();
    double fShapeWidth = aShapeBounds.getWidth() * mfScaleFactor;
    double fShapeHeight = aShapeBounds.getHeight() * mfScaleFactor;

    ::basegfx::B2DPoint aShapePosition = aShapeBounds.getCenter();
    float fBodyPosX = aShapePosition.getX() * mfScaleFactor;
    float fBodyPosY = aShapePosition.getY() * -mfScaleFactor;

    b2BodyDef aBodyDef;
    aBodyDef.type = b2_staticBody;
    aBodyDef.position.Set(fBodyPosX, fBodyPosY);

    b2Body* pBody = mpBox2DWorld->CreateBody(&aBodyDef);

    b2PolygonShape aDynamicBox;
    aDynamicBox.SetAsBox(static_cast<float>(fShapeWidth / 2), static_cast<float>(fShapeHeight / 2));

    b2FixtureDef aFixtureDef;
    aFixtureDef.shape = &aDynamicBox;
    aFixtureDef.density = fDensity;
    aFixtureDef.friction = fFriction;
    aFixtureDef.restitution = 0.1f;

    pBody->CreateFixture(&aFixtureDef);
    return std::make_shared<box2DBody>(pBody, mfScaleFactor);
}

box2DBody::box2DBody(b2Body* pBox2DBody, double fScaleFactor)
    : mpBox2DBody(pBox2DBody)
    , mfScaleFactor(fScaleFactor)
{
}

box2DBody::~box2DBody() { mpBox2DBody->GetWorld()->DestroyBody(mpBox2DBody); }

::basegfx::B2DPoint box2DBody::getPosition(const ::basegfx::B2DVector& rSlideSize)
{
    double fScaleFactor = calculateScaleFactor(rSlideSize);
    b2Vec2 aPosition = mpBox2DBody->GetPosition();
    double fX = static_cast<double>(aPosition.x) / fScaleFactor;
    double fY = static_cast<double>(aPosition.y) / -fScaleFactor;
    return ::basegfx::B2DPoint(fX, fY);
}

void box2DBody::setPositionByLinearVelocity(const basegfx::B2DPoint& rOutPos,
                                            const float fScaleFactor, const double fPassedTime)
{
    if (mpBox2DBody->GetType() != b2_kinematicBody)
        mpBox2DBody->SetType(b2_kinematicBody);
    float fDesiredX = static_cast<float>(rOutPos.getX() * fScaleFactor);
    float fDesiredY = static_cast<float>(rOutPos.getY() * -fScaleFactor);
    b2Vec2 aDesiredPos = { fDesiredX, fDesiredY };

    b2Vec2 aCurrentPos = mpBox2DBody->GetPosition();

    b2Vec2 aVelocity = (1 / fPassedTime) * (aDesiredPos - aCurrentPos);

    mpBox2DBody->SetLinearVelocity(aVelocity);
}

double box2DBody::getAngle()
{
    double fAngle = static_cast<double>(mpBox2DBody->GetAngle());
    return ::basegfx::rad2deg(-fAngle);
}

void box2DBody::setType(box2DBodyType eType)
{
    mpBox2DBody->SetType(getBox2DInternalBodyType(eType));
}

box2DBodyType box2DBody::getType() { return getBox2DLOBodyType(mpBox2DBody->GetType()); }
}

/* vim:set shiftwidth=4 softtabstop=4 expandtab: */
