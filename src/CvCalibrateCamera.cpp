#include "CvCalibrateCamera.h"
#include <opencv2/opencv.hpp>
#include <SoyAssert.h>
#include <SoyDebug.h>

const Soy::Matrix4x4 gWorldToCalibration(
								   1, 0, 0, 0,
								   0, 0, 1, 0,
								   0, 1, 0, 0,
								   0, 0, 0, 1
								   );

cv::Point3f MatrixToPoint(const Soy::Matrix3x1& m)
{
	return cv::Point3f( m.x(), m.y(), m.z() );
}

Soy::Matrix3x1 PointToMatrix(const cv::Point3f& p)
{
	return Soy::Matrix3x1( p.x, p.y, p.z );
}

cv::Point2f MatrixToPoint(const Soy::Matrix2x1& m)
{
	return cv::Point2f( m.x(), m.y() );
}


cv::Point3f WorldToCalibration(const vec3f& Position)
{
	//	re-order and convert to openCV vector type
	Soy::Matrix3x1 CalibPos = gWorldToCalibration * Soy::VectorToMatrix(Position);
	return MatrixToPoint( CalibPos );
}

vec3f CalibrationToWorld(const cv::Point3f& Position)
{
	auto gCalibrationToWorld = gWorldToCalibration.Inverse();
	auto World = gCalibrationToWorld * PointToMatrix( Position );
	return Soy::MatrixToVector( World );
//	vec3f CalibPos( Position.x, Position.y, Position.z );
//	ofMatrix4x4 gCalibrationToWorld = gWorldToCalibration.getInverse();
//	vec3f WorldPos = gCalibrationToWorld * CalibPos;
//	return WorldPos;
}



bool GetCalibrationVectors(std::vector<std::vector<cv::Point3f> >& WorldPointsArray,std::vector<std::vector<cv::Point2f> >& ViewPointsArray,const ArrayBridge<vec3f>& WorldPoints,const ArrayBridge<vec2f>& ViewPoints,const vec2f& ImageScalar)
{
	if ( WorldPoints.GetSize() != ViewPoints.GetSize() || WorldPoints.IsEmpty() )
		return false;
	
	//	move into vectors
	WorldPointsArray.resize( 1 );
	ViewPointsArray.resize( WorldPointsArray.size() );
	
	for ( int p=0;	p<WorldPoints.GetSize();	p++ )
	{
		auto& World3 = WorldPoints[p];
		auto& View2 = ViewPoints[p];
		cv::Point3f Calib3 = WorldToCalibration( World3 );
		
		auto& VecWorldPoints = WorldPointsArray[0];
		auto& VecViewPoints = ViewPointsArray[0];
		VecWorldPoints.push_back( Calib3 );
		VecViewPoints.push_back( cv::Point2f( View2.x * ImageScalar.x, View2.y * ImageScalar.y ) );
	}
	
	assert( WorldPointsArray.size() == ViewPointsArray.size() );
	for ( int i=0;	i<WorldPointsArray.size();	i++ )
	{
		assert( WorldPointsArray[i].size() == ViewPointsArray[i].size() );
	}
	
	return true;
}


bool Opencv::CalibrateCamera(Soy::TCamera& Camera,TCalibrateCameraParams Params,const ArrayBridge<vec3f>&& WorldPoints,const ArrayBridge<vec2f>&& ViewPoints)
{
	auto ImageScalar = Params.mCameraImageSize;
	if ( ImageScalar.x < 1 || ImageScalar.y < 1 )
	{
		Soy::Assert(false, "Camera image size too small");
		return false;
	}
	
	//	http://docs.opencv.org/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
	
	int Flags = 0;
	
	//	The functions considers only fy as a free parameter.
	//	The ratio fx/fy stays the same as in the input cameraMatrix .
	//	When CV_CALIB_USE_INTRINSIC_GUESS is not set, the actual input values of fx and fy are ignored,
	//	only their ratio is computed and used further.
	//	gr: aspect ratio always has a fixed X;
	//			results are always better, and it affects FOV x/y and we only use Y
	Flags |= CV_CALIB_FIX_ASPECT_RATIO;
	
	//	fix for
	//	error: (-5) For non-planar calibration rigs the initial intrinsic matrix must be specified
	//	although I'm sure it is a planar calibration rig... this flag means "we already have intrinsics"
	if ( !Params.mCalculateIntrinsic )
		Flags |= CV_CALIB_USE_INTRINSIC_GUESS;
	
	//Flags |= CV_CALIB_FIX_K4|CV_CALIB_FIX_K5;
	
	//	"corners of chessboard"
	//	gr: define the plane of an object (z=0)... so I guess this is the 3D image (complete with units)
	//	worldpoints is objectPoints in the examples. "in the calibration pattern coordinate space" so, world space
	std::vector<std::vector<cv::Point3f> > WorldPointsArray;
	std::vector<std::vector<cv::Point2f> > ViewPointsArray;
	
	if( !GetCalibrationVectors( WorldPointsArray, ViewPointsArray, WorldPoints, ViewPoints, ImageScalar ) )
		return false;
	
	//	matrix we're calculating. 3x3 matrix, 64bit floats (doubles)
	//	1000 0100 0010 0001
	cv::Mat cameraMatrix = cv::Mat::eye( 4, 3, CV_64F );
	
	if ( Params.mForceImageAspectRatio )
	{
		float InitialRatio = ImageScalar.x / ImageScalar.y;
		cameraMatrix.at<double>(0,0) = InitialRatio;
		cameraMatrix.at<double>(1,1) = InitialRatio;
	}
	
	//	distortion coefficients;
	//	tangent h/v, fov etc... work out these!
	//	gr: (VT)refine to get better versions of these?
	//	8 params when CV_CALIB_RATIONAL_MODEL
	cv::Mat distortionCoeffs = cv::Mat::zeros( 5, 1, CV_64F );
	
	//	view-space points already normalised so image is [0,0] .. [1,1]
	cv::Size ImageSize( ImageScalar.x, ImageScalar.y );
	
	std::vector<cv::Mat> ObjectRotations;
	std::vector<cv::Mat> ObjectTranslations;	//	note; will need converting; CalibrationToWorld
	
	float AverageError = 0.f;
	
	try
	{
		double AverageErrord = cv::calibrateCamera( WorldPointsArray, ViewPointsArray, ImageSize, cameraMatrix, distortionCoeffs, ObjectRotations, ObjectTranslations, Flags );
		Soy::Assert( Soy::IsValidFloat(Soy::IsValidFloat(AverageErrord)), "Calibration gave invalid error value" );
		AverageError = static_cast<float>( AverageErrord );
	}
	catch ( cv::Exception& Exception )
	{
		//	re throw as Soy exception
		std::stringstream Error;
		Error << "Calibration exception: " << Exception.what();
		throw Soy::AssertException( Error.str() );
	}
	catch ( Soy::AssertException& e )
	{
		throw e;
	}
	catch ( std::exception& e )
	{
		throw Soy::AssertException( e.what() );
	}
	catch (...)
	{
		throw Soy::AssertException("unknown exception during calibrate camera");
	}
	

	//	save error
	Camera.mCalibrationError = AverageError;
	/*
	//	pull out lens info
	Soy::TCamera NewCamera = Camera.GetCamera();
	if ( Params.mCalculateIntrinsic )
	{
		auto& Projection = NewCamera;
		{
			//	http://stackoverflow.com/questions/16329867/why-does-the-focal-length-in-the-camera-intrinsics-matrix-have-two-dimensions
			static float ApertureWidth = 0.1f;
			static float ApertureHeight = 0.1f;
			static float FocalLengthMultiplier = 1.f;
			vec2f ApertureSize( ApertureWidth, ApertureHeight );	//	physical width of sensor, ie, the film. metres?
			double fovx,fovy,aspectRatio;
			double focalLength;	//	mm, relative to apature?
			cv::Point2d principalPoint;	//	in pixels...
			calibrationMatrixValues( cameraMatrix, ImageSize, ApertureSize.x, ApertureSize.y, fovx, fovy, focalLength, principalPoint, aspectRatio);
			
			if ( Params.mForceImageAspectRatio )
			{
				aspectRatio = ImageScalar.x / ImageScalar.y;
			}
			
			ofMatrix3x3 CameraMatrixOf( cameraMatrix.at<double>(0,0), cameraMatrix.at<double>(1,0), cameraMatrix.at<double>(2,0),
									   cameraMatrix.at<double>(0,1), cameraMatrix.at<double>(1,1), cameraMatrix.at<double>(2,1),
									   cameraMatrix.at<double>(0,2), cameraMatrix.at<double>(1,2), cameraMatrix.at<double>(2,2) );
			
			vec2f CameraCenter( cameraMatrix.at<double>(0,2), cameraMatrix.at<double>(1,2) );
			Projection.mFocal = vec2f( cameraMatrix.at<double>(0,0), cameraMatrix.at<double>(1,1) );
			
			//	horz fov not used in openframeworks stuff
			Projection.setFov( fovy );
			Projection.setLensOffset( CameraCenter );	//	principalPoint?
			Projection.setAspectRatio( aspectRatio );
			Projection.mFocalLength = focalLength * FocalLengthMultiplier;
			
			//	near clip is the distance from the sensor (film) to the edge of the lens in world space
			Projection.setNearClip( Camera.GetCamera().getNearClip() );
			Projection.setFarClip( Camera.GetCamera().getFarClip() );
			Projection.setLensOffset( vec2f( principalPoint.x, principalPoint.y ) );
			
			//	get the distortion values
			//	k1,k2,p1,p2,k3,k4,k5,k6
			BufferArray<float,5> DistortionParams(5);
			if ( Params.mZeroRadialDistortion )
			{
				DistortionParams.SetAll( 0.f );
			}
			else
			{
				DistortionParams[0] = distortionCoeffs.at<double>(0,0);
				DistortionParams[1] = distortionCoeffs.at<double>(1,0);
				DistortionParams[2] = distortionCoeffs.at<double>(2,0);
				DistortionParams[3] = distortionCoeffs.at<double>(3,0);
				DistortionParams[4] = distortionCoeffs.at<double>(4,0);
			}
			
			Projection.mRadialDistortion.x = fabsf(DistortionParams[0]);	//	k1
			Projection.mRadialDistortion.y = fabsf(DistortionParams[1]);	//	k2
			Projection.mTangentialDistortion.x = fabsf(DistortionParams[2]);	//	p1
			Projection.mTangentialDistortion.y = fabsf(DistortionParams[3]);	//	p1
			Projection.mDistortionK5 = DistortionParams[4];
		}
		
		Array<vec3f> ObjectPositions;
		for ( auto it=ObjectTranslations.begin();	it!=ObjectTranslations.end();	it++ )
		{
			cv::Mat& Trans = *it;
			vec3f WorldPos = CalibrationToWorld( cv::Point3f( Trans.at<double>(0), Trans.at<double>(1), Trans.at<double>(2) ) );
			ObjectPositions.PushBack( WorldPos );
		}
	}
	*/
	
	
	//	verify matrixes by re-projecting
	{
		//	get intrinsic matrix
		auto& WorldPoints = WorldPointsArray[0];
		auto& ViewPoints = ViewPointsArray[0];
		
		
		//	convert rotation to matrix
		cv::Mat expandedRotationVector;
		cv::Mat& TranslationVector = ObjectTranslations[0];
		cv::Rodrigues(ObjectRotations[0], expandedRotationVector);
		
		//	merge translation and rotation into a model-view matrix
		static bool identityext = false;
		cv::Mat ExtrinsicMtx = cv::Mat::eye(4, 4, CV_64FC1);
		if ( !identityext )
		{
			for (int y = 0; y < 3; y++)
				for (int x = 0; x < 3; x++)
					ExtrinsicMtx.at<double>(y, x) = expandedRotationVector.at<double>(y, x);
			ExtrinsicMtx.at<double>(0, 3) = TranslationVector.at<double>(0, 0);
			ExtrinsicMtx.at<double>(1, 3) = TranslationVector.at<double>(1, 0);
			ExtrinsicMtx.at<double>(2, 3) = TranslationVector.at<double>(2, 0);
			ExtrinsicMtx.at<double>(3, 3) = 1.0;
		}
		
		vec3f TranslationVector3( TranslationVector.at<double>(0, 0), TranslationVector.at<double>(1, 0), TranslationVector.at<double>(2, 0) );
		
		cv::Mat IntrinsicMtx = cv::Mat::zeros(4, 4, CV_64FC1);
		for (int y = 0; y < 3; y++)
			for (int x = 0; x < 3; x++)
				IntrinsicMtx.at<double>(y, x) = cameraMatrix.at<double>(y, x);
		IntrinsicMtx.at<double>(0, 3) = 0;
		IntrinsicMtx.at<double>(1, 3) = 0;
		IntrinsicMtx.at<double>(2, 3) = 0;
		IntrinsicMtx.at<double>(3, 3) = 1.0;
		
		
		for ( int i=0;	i<WorldPoints.size();	i++ )
		{
			auto& WorldPoint = WorldPoints[0];
			
			cv::Mat ViewPos( 4, 1, CV_64FC1 );
			ViewPos.at<double>(0,0) = ViewPoints[i].x / ImageScalar.x;
			ViewPos.at<double>(1,0) = ViewPoints[i].y / ImageScalar.y;
			ViewPos.at<double>(2,0) = 0;
			ViewPos.at<double>(3,0) = 1;
			
			cv::Mat Multiplied;
			Multiplied = IntrinsicMtx * ExtrinsicMtx * ViewPos;
			
			Soy::Matrix4x1 ReProjected4( Multiplied.at<double>(0), Multiplied.at<double>(1), Multiplied.at<double>(2), Multiplied.at<double>(3) );
			Soy::Matrix3x1 ReProjected3( ReProjected4.x(), ReProjected4.y(), ReProjected4.z() );
			
			Soy::Matrix3x1 WorldPos( WorldPoint.x, WorldPoint.y, WorldPoint.z );
			Soy::Matrix3x1 Delta( ReProjected3 - WorldPos );
			
			std::Debug << "CameraCalibration " << i << " difference: ";
			//std::Debug << "CameraCalibration " << i << " difference: " << Delta << Soy::lf;
		}
	}
	
	
	//	rot and trans output...
	if ( Params.mCalculateExtrinsic )
	{
		cv::Mat& RotationVector = ObjectRotations[0];
		cv::Mat& TranslationVector = ObjectTranslations[0];
		
		//	for debug-peeking
		vec3f tran3( TranslationVector.at<double>(0), TranslationVector.at<double>(1), TranslationVector.at<double>(2) );
		vec3f rot3( Soy::RadToDeg(RotationVector.at<double>(0)), Soy::RadToDeg(RotationVector.at<double>(1)), Soy::RadToDeg(RotationVector.at<double>(2)) );
		
		//	convert rotation to matrix
		cv::Mat expandedRotationVector;
		cv::Rodrigues(RotationVector, expandedRotationVector);
		
		//	merge translation and rotation into a model-view matrix
		cv::Mat Rt = cv::Mat::zeros(4, 4, CV_64FC1);
		for (int y = 0; y < 3; y++)
			for (int x = 0; x < 3; x++)
				Rt.at<double>(y, x) = expandedRotationVector.at<double>(y, x);
		Rt.at<double>(0, 3) = TranslationVector.at<double>(0, 0);
		Rt.at<double>(1, 3) = TranslationVector.at<double>(1, 0);
		Rt.at<double>(2, 3) = TranslationVector.at<double>(2, 0);
		Rt.at<double>(3, 3) = 1.0;
		
		//	convert to openframeworks matrix AND transpose at the same time
		Soy::Matrix4x4 ModelView;
		for ( int r=0;	r<4;	r++ )
			for ( int c=0;	c<4;	c++ )
				ModelView(r,c) = Rt.at<double>( c, r );
		
		//	swap y & z planes so y is up
		auto gCalibrationToWorld = gWorldToCalibration.Inverse();
		ModelView *= gCalibrationToWorld;
		
		//	invert y and z planes for -/+ differences between opencv and opengl
		static Soy::Matrix4x4 InvertHandednessMatrix(
												  1,  0,  0, 0,
												  0,  -1, 0, 0,
												  0,  0,  -1, 0,
												  0,  0,  0,  1
												  );
		ModelView *= InvertHandednessMatrix;
		
		//	invert to turn matrix from object-relative-to-camera to camera-relative-to-object(0,0,0)
		static bool doinverse = true;
		if ( doinverse )
			ModelView = ModelView.Inverse();
		
		/*
		 //	http://stackoverflow.com/a/1264880/355753
		 //	http://stackoverflow.com/questions/1263072
		 ofSwap( ModelView._mat[0].y, ModelView._mat[0].z );
		 ofSwap( ModelView._mat[1].y, ModelView._mat[1].z );
		 ofSwap( ModelView._mat[2].y, ModelView._mat[2].z );
		 ofSwap( ModelView._mat[3].y, ModelView._mat[3].z );
		 */
		
		Camera.mMatrix = Soy::MatrixToVector(ModelView);
		
		/*
		static bool JustLookAt = true;
		if ( JustLookAt )
			NewCamera.lookAt( WORLD_UP );
		 */
	}
		
	return true;
}

