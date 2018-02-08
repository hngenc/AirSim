// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "RandomMovementChar.generated.h"

UCLASS()
class BLOCKS_API ARandomMovementChar : public ACharacter
{
	GENERATED_BODY()

public:
	// Sets default values for this character's properties
	ARandomMovementChar();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

	FString JsonString;
	FString path;
	
	
	int counter = 0;
	int choice;
	int speedScale;
};
