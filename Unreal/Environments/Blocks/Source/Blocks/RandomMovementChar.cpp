// Fill out your copyright notice in the Description page of Project Settings.

#include "RandomMovementChar.h"
#include "Runtime/Json/Public/Serialization/JsonSerializer.h"
#include "Runtime/Json/Public/Dom/JsonObject.h"
#include "Runtime/Core/Public/Misc/Paths.h"
#include "Runtime/Engine/Classes/GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "FileHelper.h"
// Sets default values
ARandomMovementChar::ARandomMovementChar()
{
 	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	counter = 0;
	
	UE_LOG(LogTemp, Warning, TEXT("%s"), *path);
	FFileHelper::LoadFileToString(JsonString, *path);

}

// Called when the game starts or when spawned
void ARandomMovementChar::BeginPlay()
{
	path = FPaths::GameSourceDir() + "Blocks/setting/DataGenerationSetting.json";
	FFileHelper::LoadFileToString(JsonString, *path);
	Super::BeginPlay();
	
}

// Called every frame
void ARandomMovementChar::Tick(float DeltaTime)
{
	float speed =1;
	float x = 1;
	float y = 1;
	TSharedPtr<FJsonObject> JsonParsed;
	TSharedRef< TJsonReader<TCHAR> > JsonReader = TJsonReaderFactory<TCHAR>::Create(JsonString);
	if (FJsonSerializer::Deserialize(JsonReader, JsonParsed))
	{
		speed = JsonParsed->GetNumberField("RandMovSpeed");
		x = JsonParsed->GetNumberField("X");
		y = JsonParsed->GetNumberField("Y");
	}

	/*FString JsonRaw = "{ \"exampleString\": \".1\" }";
	TSharedPtr<FJsonObject> JsonParsed;
	TSharedRef<TJsonReader<TCHAR>> JsonReader = TJsonReaderFactory<TCHAR>::Create(JsonRaw);
	if (FJsonSerializer::Deserialize(JsonReader, JsonParsed))
	{
		speed = JsonParsed->GetNumberField("exampleString");
	}*/
	GetCharacterMovement()->MaxWalkSpeed = speed;
	Super::Tick(DeltaTime);
	counter += 1 * DeltaTime;
	if (counter == 100) {
		choice = FMath::RandRange((int)(0), (int)(3));
		counter = 0;
	}
	else
		counter++;
	switch (choice)
	{
		case 0: AddMovementInput(GetActorForwardVector(), 1*x); break;
		case 1: AddMovementInput(GetActorForwardVector(), -1*x); break;
		case 2: AddMovementInput(GetActorRightVector(), 1*y); break;
		case 3: AddMovementInput(GetActorRightVector(), -1*y); break;

		default:
			break;
	}
}

// Called to bind functionality to input
void ARandomMovementChar::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

}

