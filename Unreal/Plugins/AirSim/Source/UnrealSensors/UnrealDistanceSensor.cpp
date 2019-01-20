// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "UnrealDistanceSensor.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "NedTransform.h"

UnrealDistanceSensor::UnrealDistanceSensor(const AirSimSettings::DistanceSetting& setting,
    AActor* actor, const NedTransform* ned_transform)
    : DistanceSimple(setting), actor_(actor), ned_transform_(ned_transform)
{
    this->ignore_pawn_collision_ = setting.ignore_pawn_collision;
    this->draw_debug_points_ = setting.draw_debug_points;
}

msr::airlib::real_T UnrealDistanceSensor::getRayLength(const msr::airlib::Pose& pose)
{
    msr::airlib::DistanceSimpleParams params = getParams();
    if (this->ned_transform_ != nullptr)
    {
        Vector3r start = pose.position;
        Vector3r end = start + VectorMath::rotateVector(VectorMath::front(), pose.orientation, true) * params.max_distance;

        FHitResult dist_hit = FHitResult(ForceInit);
        bool is_hit = UAirBlueprintLib::GetObstacle(actor_, ned_transform_->fromLocalNed(start), ned_transform_->fromLocalNed(end), dist_hit);
        float distance = is_hit ? dist_hit.Distance / 100.0f : params.max_distance;

        return distance;
    }
    else
    {
        //FVector actorLocation = this->actor_->GetActorLocation();
        Vector3r actorLocation = this->getGroundTruth().kinematics->pose.position;
        Vector3r offset = params.relative_pose.position;
        offset = VectorMath::rotateVector(offset, this->getGroundTruth().kinematics->pose.orientation, true);
        
        actorLocation += offset;

        FVector actorLocationVec = FVector(actorLocation.x(), actorLocation.y(), actorLocation.z());
        if (this->draw_debug_points_)
        {
            DrawDebugPoint(
                actor_->GetWorld(),
                actorLocationVec,
                5,                       //size
                FColor::Blue,
                true,                    //persistent (never goes away)
                0.1                      //point leaves a trail on moving object
            );
        }

        FRotator poseRotator = FQuat(params.relative_pose.orientation.x(), params.relative_pose.orientation.y(), params.relative_pose.orientation.z(), params.relative_pose.orientation.w()).Rotator();
        FRotator actorRotator = this->actor_->GetActorRotation();

        FVector endVec(1, 0, 0);
        endVec = poseRotator.RotateVector(endVec);
        endVec = actorRotator.RotateVector(endVec);
        endVec = endVec * params.max_distance + actorLocationVec;

        FHitResult dist_hit = FHitResult(ForceInit);
        bool isHit = UAirBlueprintLib::GetObstacle(actor_, actorLocationVec, endVec, dist_hit, nullptr, ECC_Visibility, this->ignore_pawn_collision_);

        if (isHit)
        {
            if (this->draw_debug_points_)
            {
                DrawDebugPoint(
                    actor_->GetWorld(),
                    dist_hit.ImpactPoint,
                    5,                       //size
                    FColor::Red,
                    true,                    //persistent (never goes away)
                    0.1                      //point leaves a trail on moving object
                );
            }

            return FMath::Max(params.min_distance, FVector::Distance(dist_hit.ImpactPoint, actorLocationVec));
        }
        else
        {
            if (this->draw_debug_points_)
            {
                DrawDebugPoint(
                    actor_->GetWorld(),
                    endVec,
                    5,                       //size
                    FColor::Green,
                    true,                    //persistent (never goes away)
                    0.1                      //point leaves a trail on moving object
                );
            }

            return params.max_distance;
        }
    }
   
}
