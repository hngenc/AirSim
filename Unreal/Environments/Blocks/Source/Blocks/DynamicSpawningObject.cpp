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
#include <codecvt>
#include <fstream>
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
	UE_LOG(LogTemp, Warning, TEXT("%s"), *path);
	FFileHelper::LoadFileToString(JsonString, *path);
	static ConstructorHelpers::FObjectFinder<UBlueprint> ItemBlueprint(TEXT("Blueprint'/Game/Obstacle.Obstacle'"));
	if (ItemBlueprint.Object) {
		itemSpawning = (UClass*)ItemBlueprint.Object->GeneratedClass;
	}
}




// Called when the game starts or when spawned
void ADynamicSpawningObject::BeginPlay()
{
	std::wstring userProfile = _wgetenv(L"USERPROFILE");
	std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
	//return converter.to_bytes(userProfile);
	std::string path2 = (std::string) converter.to_bytes(userProfile) + "\\Documents\\DataGenerationSetting.json";
	
	//path = FPaths::GameSourceDir() + "Blocks/setting/DataGenerationSetting.json";
	
	path = FString(path2.c_str());
	FFileHelper::LoadFileToString(JsonString, *path);
	i = 0;
	TSharedPtr<FJsonObject> JsonObject;
	TSharedRef< TJsonReader<TCHAR> > JsonReader = TJsonReaderFactory<TCHAR>::Create(JsonString);
	if (FJsonSerializer::Deserialize(JsonReader, JsonObject) &&
		JsonObject.IsValid())
	{
		TSharedPtr<FJsonObject> jsonObj = JsonObject->GetObjectField("obstacle_spawning");
		density = jsonObj->GetNumberField("Density");

		TSharedPtr<FJsonObject> jsonArea = jsonObj->GetObjectField("Area");
		x = jsonArea->GetNumberField("X");
		y = jsonArea->GetNumberField("Y");
		z = jsonArea->GetNumberField("Z");
	}
	Super::BeginPlay();
}
// Called every frame
void ADynamicSpawningObject::Tick(float DeltaTime)
{
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

