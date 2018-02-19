// Fill out your copyright notice in the Description page of Project Settings.

#include "ActorItem.h"
#include "Runtime/CoreUObject/Public/UObject/ConstructorHelpers.h"
#include <math.h>
#include <cmath>   
#include "Runtime/Engine/Classes/Engine/SkeletalMesh.h"
#include "Runtime/Json/Public/Serialization/JsonSerializer.h"
#include "Runtime/Json/Public/Dom/JsonObject.h"
#include "FileHelper.h"

//#include "../../Plugins/AirSim/Source/AirLib/include/controllers/Settings.h"

// Sets default values
AActorItem::AActorItem()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	//Mesh = CreateDefaultSubobject<UClass>(TEXT("Mesh"));
	static ConstructorHelpers::FObjectFinder <UBlueprint>ItemBlueprint(TEXT("Blueprint'/Game/Cube.Cube'"));
	//Mesh->GeneratedClass(UBlueprint.Object);
	if (ItemBlueprint.Object) {
		Mesh = (UClass*)ItemBlueprint.Object->GeneratedClass;
	}
	//static ConstructorHelpers::FObjectFinder<USkeletalMesh> MeshContainer(TEXT("SkeletalMesh'/Block/Content/GR_MALE_HERO_02/Character/Mesh/SK_Mannequin.SK_Mannequin'"));
	//if (MeshContainer.Succeeded())
	//{
	//	//PlayerMesh = GetMesh();
	//	if (AlternateMeshAsset)
	//	{
	//		Mesh->SetSkeletalMesh(AlternateMeshAsset);
	//	}
	//}
	UE_LOG(LogTemp, Warning, TEXT("%s"), *path);
	FFileHelper::LoadFileToString(JsonString, *path);
	currentTime = 0;
	movementType = 0;
}

// Called when the game starts or when spawned
void AActorItem::BeginPlay()
{
	path = FPaths::GameSourceDir() + "Blocks/setting/DataGenerationSetting.json";
	FFileHelper::LoadFileToString(JsonString, *path);
	TSharedPtr<FJsonObject> JsonObject;
	TSharedRef< TJsonReader<TCHAR> > JsonReader = TJsonReaderFactory<TCHAR>::Create(JsonString);
	if (FJsonSerializer::Deserialize(JsonReader, JsonObject) &&
		JsonObject.IsValid())
	{
		TSharedPtr<FJsonObject> jsonObj = JsonObject->GetObjectField("obstacle");
		speed = jsonObj->GetNumberField("MaxSpeed");

		TSharedPtr<FJsonObject> jsonMov = jsonObj->GetObjectField("Movement");
		movementType = jsonMov->GetNumberField("Mode");

		TSharedPtr<FJsonObject> jsonArea = jsonObj->GetObjectField("Range");
		Rx = jsonArea->GetNumberField("X");
		Ry = jsonArea->GetNumberField("Y");
		Rz = jsonArea->GetNumberField("Z");
	}
	Super::BeginPlay();
	curLoc = GetActorLocation();
	lowBoundary = GetActorLocation() + FVector(-Rx,- Ry, -Rz);
	highBoundary = GetActorLocation() + FVector(Rx, Ry, Rz);
	newVector = GetActorLocation();
	
}




void AActorItem::moveRand(float deltaTime, int Rx, int Ry, int Rz) {
	curLoc = GetActorLocation();
	newVector = GetActorLocation();
	/*FVector low = newVector - FVector(Rx,Ry, Rz);
	FVector hi = newVector + FVector(Rx, Ry, Rz);
	if ((curLoc.X < hi.X) || (curLoc.Y < hi.Y) || (curLoc.Z < hi.Z) ||
		(curLoc.X > low.X) || (curLoc.Y > low.Y) || (curLoc.Z > low.Z)) {
		newVector = curLoc + FVector(FMath::RandRange(-speed, speed), FMath::RandRange(-speed, speed), FMath::RandRange(-speed, speed));
		move(x,y,z);
		SetActorLocation(curLoc, false);
	}
	else {
		move(x,y,z);
		SetActorLocation(curLoc, false);
	}*/
	
	int x = FMath::RandRange(-100, 100);
	int y = FMath::RandRange(-100, 100);
	int z = FMath::RandRange(-100, 100);
	
	if (x > 0) {
		if ((newVector.X + speed*deltaTime) < highBoundary.X)
			newVector.X += speed*deltaTime;
		else
			newVector.X -= speed*deltaTime;
	}
	else if(x<0) {
		if ((newVector.X - speed*deltaTime) > lowBoundary.X)
			newVector.X -= speed*deltaTime;
		else
			newVector.X += speed*deltaTime;
	}

	if (y >0) {
		if ((newVector.Y + speed*deltaTime) < highBoundary.Y)
			newVector.Y += speed*deltaTime;
		else
			newVector.Y -= speed*deltaTime;
	}
	else if(y <0){
		if ((newVector.Y - speed*deltaTime) > lowBoundary.Y)
			newVector.Y -= speed*deltaTime;
		else
			newVector.Y += speed*deltaTime;
	}

	if (z > 0) {
		if ((newVector.Z + speed*deltaTime) < highBoundary.Z)
			newVector.Z+= speed*deltaTime;
		else
			newVector.Z -= speed*deltaTime;
	}
	else if( z<0) {
		if ((newVector.Z- speed*deltaTime) > lowBoundary.Z)
			newVector.Z -= speed*deltaTime;
		else
			newVector.Z += speed*deltaTime;
	}

	SetActorLocation(newVector, false);

}

void AActorItem::moveSin(float deltaTime) {
	curLoc = GetActorLocation();
	currentTime = currentTime + speed;

	float moveDisX = 1;
	float moveDisY = speed*sin((currentTime)*PI / 180);
	if ( currentTime == 360*5)
	{
		speed = -speed;
	}
	else
		SetActorLocation(curLoc+FVector(0,moveDisX, moveDisY), false);

}


// Called every frame
void AActorItem::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	switch (movementType)
	{
		case 0: moveRand(DeltaTime, Rx, Ry, Rz); break;
		case 1: moveSin(DeltaTime); break;
		default:
			break;
	}
	

}