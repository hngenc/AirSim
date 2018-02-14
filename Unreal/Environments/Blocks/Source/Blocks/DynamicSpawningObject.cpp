// Fill out your copyright notice in the Description page of Project Settings.

#include "DynamicSpawningObject.h"
#include "ActorItem.h"
#include "Runtime/CoreUObject/Public/UObject/ConstructorHelpers.h"
#include "Runtime/Core/Public/Misc/Paths.h"
#include "Runtime/Json/Public/Serialization/JsonSerializer.h"
#include "Runtime/Json/Public/Dom/JsonObject.h"
#include "Runtime/Core/Public/Misc/Paths.h"
#include "FileHelper.h"
#include <string>


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
	path = FPaths::GameSourceDir() + "Blocks/setting/SpawningSetting.json";
	FFileHelper::LoadFileToString(JsonString, *path);
	UE_LOG(LogTemp, Warning, TEXT("%s"), *path);
	FFileHelper::LoadFileToString(JsonString, *path);
	static ConstructorHelpers::FObjectFinder<UBlueprint> ItemBlueprint(TEXT("Blueprint'/Game/MyActorItem.MyActorItem'"));
	if (ItemBlueprint.Object) {
		itemSpawning = (UClass*)ItemBlueprint.Object->GeneratedClass;
	}
}

// Called when the game starts or when spawned
void ADynamicSpawningObject::BeginPlay()
{

	i = 0;
	Super::BeginPlay();
}
// Called every frame
void ADynamicSpawningObject::Tick(float DeltaTime)
{
	float density = 0;
	float z = 0;
	float y = 0;
	float x = 0;
	TSharedPtr<FJsonObject> JsonParsed;
	TSharedRef< TJsonReader<TCHAR> > JsonReader = TJsonReaderFactory<TCHAR>::Create(JsonString);
	if (FJsonSerializer::Deserialize(JsonReader, JsonParsed))
	{
		density = JsonParsed->GetNumberField("Density");
		x = JsonParsed->GetNumberField("X");
		y = JsonParsed->GetNumberField("Y");
		z = JsonParsed->GetNumberField("Z");
		
	}
	
	Super::Tick(DeltaTime);
	FVector NewLocation = GetActorLocation() + FVector(FMath::RandRange(-x, x), FMath::RandRange(-y, y), FMath::RandRange(-z, z));
	UWorld* World = GetWorld();
	if (i<density)
	{
		GetWorld()->SpawnActor<AActorItem>(itemSpawning, NewLocation, FRotator::ZeroRotator);

	}
	i++;
}

float ADynamicSpawningObject::generateX(float DeltaSeconds) {
	float passBack = 0.f;
	passBack = FMath::RandRange(-300, 300);

	passBack = passBack + DeltaSeconds;

	return passBack;
}

