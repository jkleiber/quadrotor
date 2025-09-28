// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.Collections.Generic;

public class QuadrotorSimTarget : TargetRules
{
	public QuadrotorSimTarget( TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		DefaultBuildSettings = BuildSettingsVersion.V5;
		IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_6;
		ExtraModuleNames.AddRange( new string[] { "QuadrotorSim" } );
		// TEMPO_TOOLCHAIN_BLOCK BEGIN - Added by UseTempoToolChain.sh script
		if (Platform == UnrealTargetPlatform.Win64)
		{
			ToolChainName = "TempoVCToolChain";
		}
		else if (Platform == UnrealTargetPlatform.Mac)
		{
			ToolChainName = "TempoMacToolChain";
		}
		else if (Platform == UnrealTargetPlatform.Linux)
		{
			ToolChainName = "TempoLinuxToolChain";
		}
		// TEMPO_TOOLCHAIN_BLOCK END
	}
}
