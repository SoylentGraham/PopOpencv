#pragma once

#include <array.hpp>
#include <SoyMath.h>

namespace Soy
{
	class TCamera;
};


namespace Opencv
{
	class TCalibrateCameraParams;

	//	view points should be normalised
	bool	CalibrateCamera(Soy::TCamera& Camera,TCalibrateCameraParams Params,const ArrayBridge<vec3f>&& WorldPoints,const ArrayBridge<vec2f>&& ViewPoints);
};



class Soy::TCamera
{
public:
	TCamera() :
		mCalibrationError	( 0.f )
	{
	}
	
public:
	float		mCalibrationError;
	float4x4	mMatrix;	//	extrinsic matrix
};


class Opencv::TCalibrateCameraParams
{
public:
	TCalibrateCameraParams() :
		mCameraImageSize		( 100, 100 ),
		mCalculateIntrinsic		( true ),
		mForceImageAspectRatio	( true ),
		mCalculateExtrinsic		( true )
	{
	}
	
	//	Soy::TCamera	mKnownCamera;
	bool	mForceImageAspectRatio;
	bool	mCalculateIntrinsic;
	bool	mCalculateExtrinsic;
	vec2f	mCameraImageSize;
};

