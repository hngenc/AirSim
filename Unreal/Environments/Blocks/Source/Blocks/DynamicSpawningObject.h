// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DynamicSpawningObject.generated.h"

UCLASS()
class BLOCKS_API ADynamicSpawningObject : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ADynamicSpawningObject();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;


	float generateX(float Deltaseconds); //generate a new X

	float currentTime; //keep alive time
	float xCoord; //c coordinatei
	FTransform oldSpawnLocation;
	FVector curLoc;			//spawn location
	FRotator ourRotation; //rotation of object
	TSubclassOf<class AActorItem> itemSpawning;

	
};
