// Fill out your copyright notice in the Description page of Project Settings.

#include "DynamicSpawningObject.h"
#include "ActorItem.h"

// Sets default values
ADynamicSpawningObject::ADynamicSpawningObject()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	currentTime = 0.f;
	xCoord = 0.f;
	oldSpawnLocation;
	curLoc.X = 0.f;			//spawn location
	curLoc.Y = 100.f;
	curLoc.Z = 900.f;
	ourRotation.ZeroRotator; //rotation of object

}

// Called when the game starts or when spawned
void ADynamicSpawningObject::BeginPlay()
{
	Super::BeginPlay();
}
int i = 0;
// Called every frame
void ADynamicSpawningObject::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	FVector NewLocation = GetActorLocation() + FVector(FMath::RandRange(-300, 300), FMath::RandRange(-300, 300), FMath::RandRange(-300, 300));
	UWorld* World = GetWorld();
	if (i<=30)
	{
		World->SpawnActor<AActorItem>(GetClass(), NewLocation, FRotator::ZeroRotator);
	}
	i++;
}

float ADynamicSpawningObject::generateX(float DeltaSeconds) {
	float passBack = 0.f;
	passBack = FMath::RandRange(-300, 300);

	passBack = passBack + DeltaSeconds;

	return passBack;
}

