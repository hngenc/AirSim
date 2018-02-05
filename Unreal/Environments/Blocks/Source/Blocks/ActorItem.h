// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ActorItem.generated.h"

UCLASS()
class BLOCKS_API AActorItem : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AActorItem();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	void move(float x, float y, float z);
	FVector curLoc;
	FVector origLoc;
	FVector newVector;
	float currentTime;
	float lowBound;
	float highBound;
	int movementType;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	void moveRand();
	void moveSin(float deltaTime);


	UPROPERTY(EditAnywhere)
	UStaticMeshComponent* Mesh;
	
	UPROPERTY(EditAnywhere)
	class USkeletalMeshComponent* PlayerMesh;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assets")
	USkeletalMesh* AlternateMeshAsset;
	
};
