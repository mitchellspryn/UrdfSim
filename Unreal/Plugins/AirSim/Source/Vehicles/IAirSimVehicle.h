#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

class IAirSimVehicle
{
    public:
        virtual ~IAirSimVehicle() {};

        virtual USceneComponent* GetComponent(FString componentName) = 0;
        virtual void GetComponentReferenceTransform(FString componentName, FVector& translation, FRotator& rotation) = 0;
        virtual APawn* GetPawn() = 0;
        virtual bool PawnUsesNedCoords() = 0;
};