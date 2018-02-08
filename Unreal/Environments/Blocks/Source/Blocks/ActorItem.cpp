// Fill out your copyright notice in the Description page of Project Settings.

#include "ActorItem.h"
#include "Runtime/CoreUObject/Public/UObject/ConstructorHelpers.h"
#include <math.h>
#include <cmath>   
#include "Runtime/Engine/Classes/Engine/SkeletalMesh.h"
//#include "../../Plugins/AirSim/Source/AirLib/include/controllers/Settings.h"

// Sets default values
AActorItem::AActorItem()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	PlayerMesh = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("PlayerMesh"));
	/*static ConstructorHelpers::FObjectFinder <UStaticMesh>StaticMesh(TEXT("StaticMesh'/Engine/BasicShapes/Cube.Cube'"));
	Mesh->SetStaticMesh(StaticMesh.Object);*/
	static ConstructorHelpers::FObjectFinder<USkeletalMesh> MeshContainer(TEXT("SkeletalMesh'/Block/Content/GR_MALE_HERO_02/Character/Mesh/SK_Mannequin.SK_Mannequin'"));
	if (MeshContainer.Succeeded())
	{
		//PlayerMesh = GetMesh();
		if (AlternateMeshAsset)
		{
			PlayerMesh->SetSkeletalMesh(AlternateMeshAsset);
		}
	}
	currentTime = 0;
	movementType = 0;
}

// Called when the game starts or when spawned
void AActorItem::BeginPlay()
{
	Super::BeginPlay();
	curLoc = GetActorLocation();
	origLoc = GetActorLocation();
	newVector = GetActorLocation();
	lowBound = curLoc.X - 300;
	highBound = curLoc.X + 300;
}


void AActorItem::move(float x, float y, float z) {
	
	if (movementType == 0) {
		if (curLoc.X > newVector.X)
		{
			curLoc.X -= x;
		}
		else
			curLoc.X += x;

		if (curLoc.Y > newVector.Y)
		{
			curLoc.Y -= y;
		}
		else
			curLoc.Y += y;

		if (curLoc.Z > newVector.Z)
		{
			curLoc.Z -= z;
		}
		else
			curLoc.Z += z;
	}
	else {
		curLoc.X += x;
		curLoc.Y += y;
		curLoc.Z += z;
	}
}

void AActorItem::moveRand() {
	curLoc = GetActorLocation();
	FVector low = newVector - FVector(2, 2, 2);
	FVector hi = newVector + FVector(2, 2, 2);
	if ((curLoc.X < hi.X) && (curLoc.Y < hi.Y) && (curLoc.Z < hi.Z) &&
		(curLoc.X > low.X) && (curLoc.Y > low.Y) && (curLoc.Z > low.Z)) {
		newVector = curLoc + FVector(FMath::RandRange(-300, 300), FMath::RandRange(-300, 300), FMath::RandRange(-300, 300));
		move(1,1,1);
		SetActorLocation(curLoc, false);
	}
	else {
		move(1,1,1);
		SetActorLocation(curLoc, false);
	}
}

void AActorItem::moveSin(float deltaTime) {
	curLoc = GetActorLocation();
	currentTime = currentTime + 1;

	float moveDisX = 1;
	float moveDisY = 10*sin((currentTime)*PI / 180);
	move(0, moveDisX, moveDisY);
	if ( currentTime == 360*5)
	{
		SetActorLocation(origLoc, false);
		currentTime = 0;
	}
	else
		SetActorLocation(curLoc, false);

}


// Called every frame
void AActorItem::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	switch (movementType)
	{
		case 0: moveRand(); break;
		case 1: moveSin(DeltaTime); break;
		default:
			break;
	}
	

}