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
	class TGetHomographyParams;

	//	view points should be normalised
	bool	CalibrateCamera(Soy::TCamera& Camera,TCalibrateCameraParams Params,const ArrayBridge<vec3f>&& WorldPoints,const ArrayBridge<vec2f>&& ViewPoints);
	bool	GetHomography(Soy::Matrix3x3& Homography,TGetHomographyParams Params,const ArrayBridge<vec2f>&& Points2D,const ArrayBridge<vec2f>&& PointsUv);
};



class Soy::TCamera
{
public:
	TCamera() :
		mCalibrationError		( 0.f ),
		mFocalLength			( 1.f ),
		mDistortionK5			( 0 ),
		mFov					( 40.f, 40.f ),
		mAspectRatio			( 1.f )
	{
	}
	
public:
	float		mCalibrationError;
	float4x4	mMatrix;			//	extrinsic matrix. Modelview?
	float4x4	mIntrinsicMatrix;	//	projection?
	vec3f		mCameraWorldPosition;
	vec3f		mCameraRotationEularDeg;
	
	//	in view space we go from -1 (against the lense/near clip) to 0 (focal length where it crosses) to 1 (far visibility/far clip)
	//	mFocalLength is the world-space distance of 0^^
	//	http://www.cambridgeincolour.com/tutorials/camera-lenses.htm
	float	mFocalLength;	//	mm, relative to apature?
	vec2f	mRadialDistortion;
	vec2f	mTangentialDistortion;
	float	mDistortionK5;

	//	gr: normalised
	vec2f	mFocalSize;		//	"fx/fy" camera matrix focal points x/y; http://stackoverflow.com/questions/16329867/why-does-the-focal-length-in-the-camera-intrinsics-matrix-have-two-dimensions
	vec2f	mLensOffset;	//	"cx/cy" principle point?
	vec2f	mPrinciplePoint;

	//	gr: do we ever use horz & vert AND aspect ratio? doesn't one set calculate the other?
	vec2f	mFov;
	float	mAspectRatio;
	
	std::tuple<vec3f,vec3f>	ScreenToWorldRay(vec2f Screen);
	vec3f					ScreenToWorld(vec2f Screen,float ViewDepth);
	vec3f					ScreenToWorldY(vec2f Screen,float ViewDepth);
};


class Opencv::TCalibrateCameraParams
{
public:
	TCalibrateCameraParams() :
		mCameraImageSize		( 100, 100 ),
		mCalculateIntrinsic		( true ),
		mForceImageAspectRatio	( true ),
		mCalculateExtrinsic		( true ),
		mZeroRadialDistortion	( false )
	{
	}
	
	//	Soy::TCamera	mKnownCamera;
	bool	mForceImageAspectRatio;
	bool	mCalculateIntrinsic;
	bool	mCalculateExtrinsic;
	vec2f	mCameraImageSize;
	bool	mZeroRadialDistortion;
};



class Opencv::TGetHomographyParams
{
public:
	TGetHomographyParams() :
		mCameraImageSize		( 100, 100 )
	{
	}
	
	vec2f	mCameraImageSize;
};


