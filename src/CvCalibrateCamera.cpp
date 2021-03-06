#include "CvCalibrateCamera.h"
#include <opencv2/opencv.hpp>
#include <SoyAssert.h>
#include <SoyDebug.h>
#include <HeapArray.hpp>


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

cv::Point2f VectorToPoint(const vec2f& v)
{
	return cv::Point2f( v.x, v.y );
}

Soy::Matrix3x1 PointToMatrix(const cv::Point3f& p)
{
	return Soy::Matrix3x1( p.x, p.y, p.z );
}

Soy::Matrix2x1 PointToMatrix(const cv::Point2f& p)
{
	return Soy::Matrix2x1( p.x, p.y );
}

vec3f PointToVector(const cv::Point3f& p)
{
	auto m = PointToMatrix( p );
	return Soy::MatrixToVector( m );
}

vec2f PointToVector(const cv::Point2f& p)
{
	auto m = PointToMatrix( p );
	return Soy::MatrixToVector( m );
}

cv::Point2f MatrixToPoint(const Soy::Matrix2x1& m)
{
	return cv::Point2f( m.x(), m.y() );
}

Soy::Matrix4x1 MatToMatrix4x1(const cv::Mat& Mat)
{
	return Soy::Matrix4x1( Mat.at<double>(0), Mat.at<double>(1), Mat.at<double>(2), Mat.at<double>(3) );
}

Soy::Matrix4x4 MatToMatrix4x4(const cv::Mat& Mat)
{
	return Soy::Matrix4x4( Mat.at<double>(0,0), Mat.at<double>(1,0), Mat.at<double>(2,0), Mat.at<double>(3,0),
						  Mat.at<double>(0,1), Mat.at<double>(1,1), Mat.at<double>(2,1), Mat.at<double>(3,1),
						  Mat.at<double>(0,2), Mat.at<double>(1,2), Mat.at<double>(2,2), Mat.at<double>(3,2),
						   Mat.at<double>(0,3), Mat.at<double>(1,3), Mat.at<double>(2,3), Mat.at<double>(3,3)
						  );
}

Soy::Matrix3x3 MatToMatrix3x3(const cv::Mat& Mat)
{
	return Soy::Matrix3x3( Mat.at<double>(0,0), Mat.at<double>(1,0), Mat.at<double>(2,0),
						  Mat.at<double>(0,1), Mat.at<double>(1,1), Mat.at<double>(2,1),
						  Mat.at<double>(0,2), Mat.at<double>(1,2), Mat.at<double>(2,2) );
}

cv::Point3f WorldToCalibration(const vec3f& Position)
{
	//	re-order and convert to openCV vector type
	Soy::Matrix3x1 CalibPos = gWorldToCalibration * Soy::VectorToMatrix(Position);
	return MatrixToPoint( CalibPos );
}

const Soy::Matrix4x4& GetWorldToCalibrationMtx()
{
	return gWorldToCalibration;
}

Soy::Matrix4x4 GetCalibrationToWorldMtx()
{
	return GetWorldToCalibrationMtx().Inverse();
}

vec3f CalibrationToWorld(const cv::Point3f& Position)
{
	auto gCalibrationToWorld = GetCalibrationToWorldMtx();
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


std::tuple<vec3f,vec3f> Soy::TCamera::ScreenToWorldRay(vec2f Screen)
{
	//	gr: old code started at -1 for near!?
	auto Near = ScreenToWorld( Screen, 0 );
	auto Far = ScreenToWorld( Screen, 1 );
	return std::make_tuple( Near, Far );
}

vec3f Soy::TCamera::ScreenToWorld(vec2f Screen,float ViewDepth)
{
	return vec3f(0,0,0);
	/*
	auto ProjectionMtx = Soy::VectorToMatrix( Camera.mIntrinsicMatrix );
	auto ModelViewMtx = Soy::VectorToMatrix( Camera.mMatrix );
	auto Mvp = ModelViewMtx * ProjectionMtx;
	auto Pvm = Mvp.Inverse();
	//Soy::Matrix4x1 View4( ViewPoints[i].x / ImageScalar.x, ViewPoints[i].y / ImageScalar.y, 0, 1 );
	//Soy::Matrix4x1 Screen4( ViewPoints[i].x / ImageScalar.x, ViewPoints[i].y / ImageScalar.y, 0, 1 );
	
	float ViewDepth = 0;
	Soy::Matrix4x1 Screen4( ViewPoints[i].x, ViewPoints[i].y, 0, 1 );
	auto ReprojWorld4 = Screen4 * Pvm;
	*/
	
}

vec3f Soy::TCamera::ScreenToWorldY(vec2f Screen,float ViewDepth)
{
	return vec3f(0,0,0);
	/*
	auto Ray = ScreenToWorldRay(<#vec2f Screen#>)
	std::tuple<vec3f,vec3f> Soy::TCamera::ScreenToWorldRay(vec2f Screen)
*/
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
	
	//	pull out lens info
	if ( Params.mCalculateIntrinsic )
	{
		auto& Projection = Camera;
		{
			//	http://stackoverflow.com/questions/16329867/why-does-the-focal-length-in-the-camera-intrinsics-matrix-have-two-dimensions
			static float ApertureWidth = 0.1f;
			static float ApertureHeight = 0.1f;
			static float FocalLengthMultiplier = 1.f;
			vec2f ApertureSize( ApertureWidth, ApertureHeight );	//	physical width of sensor, ie, the film. metres?
			double fovx,fovy,aspectRatio;
			double focalLength;
			cv::Point2d principalPoint;	//	in pixels...
			calibrationMatrixValues( cameraMatrix, ImageSize, ApertureSize.x, ApertureSize.y, fovx, fovy, focalLength, principalPoint, aspectRatio);
			
			if ( Params.mForceImageAspectRatio )
			{
				aspectRatio = ImageScalar.x / ImageScalar.y;
			}
			
			Soy::Matrix3x3 CameraMatrixOf = MatToMatrix3x3(cameraMatrix);

			//	gr: normalise lens offset
			auto fx = cameraMatrix.at<double>(0,0) / ImageScalar.x;
			auto fy = cameraMatrix.at<double>(1,1) / ImageScalar.y;
			auto cx = cameraMatrix.at<double>(0,2) / ImageScalar.x;
			auto cy = cameraMatrix.at<double>(1,2) / ImageScalar.y;
			Projection.mLensOffset = vec2f( 0.5f-cx, 0.5f-cy );
			Projection.mFocalSize = vec2f( fx / ImageScalar.x, fy / ImageScalar.y );
			
			Projection.mFov = vec2f( fovx, fovy );
			Projection.mAspectRatio = aspectRatio;
			Projection.mFocalLength = focalLength * FocalLengthMultiplier;
			
			//	near clip is the distance from the sensor (film) to the edge of the lens in world space
			Projection.mPrinciplePoint = vec2f( principalPoint.x, principalPoint.y );
			
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
	
	/*
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
		
		//	gr: copy, not transposed!
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
			
			Soy::Matrix4x1 View4( )
			
			Soy::Matrix4x1 ReProjected4 = MatToMatrix4x1( Multiplied );
			Soy::Matrix3x1 ReProjected3( ReProjected4.x(), ReProjected4.y(), ReProjected4.z() );
			
			Soy::Matrix3x1 WorldPos( WorldPoint.x, WorldPoint.y, WorldPoint.z );
			Soy::Matrix3x1 Delta( ReProjected3 - WorldPos );
			
			std::Debug << "CameraCalibration " << i << " difference: " << Delta << Soy::lf;
		}
	}
	*/
	
	
	//	rot and trans output...
	if ( Params.mCalculateExtrinsic )
	{
		cv::Mat& RotationVector = ObjectRotations[0];
		cv::Mat& TranslationVector = ObjectTranslations[0];
		
		//	for debug-peeking
		vec3f tran3( TranslationVector.at<double>(0), TranslationVector.at<double>(1), TranslationVector.at<double>(2) );
		vec3f rot3( Soy::RadToDeg(RotationVector.at<double>(0)), Soy::RadToDeg(RotationVector.at<double>(1)), Soy::RadToDeg(RotationVector.at<double>(2)) );
		
		Camera.mCameraWorldPosition = tran3;
		Camera.mCameraRotationEularDeg = rot3;
		
		//	convert rotation to matrix
		cv::Mat expandedRotationVector;
		cv::Rodrigues(RotationVector, expandedRotationVector);
		
		//	merge translation and rotation into a model-view matrix
		cv::Mat ExtrinsicMtx = cv::Mat::zeros(4, 4, CV_64FC1);
		for (int y = 0; y < 3; y++)
			for (int x = 0; x < 3; x++)
				ExtrinsicMtx.at<double>(y, x) = expandedRotationVector.at<double>(y, x);
		ExtrinsicMtx.at<double>(0, 3) = TranslationVector.at<double>(0, 0);
		ExtrinsicMtx.at<double>(1, 3) = TranslationVector.at<double>(1, 0);
		ExtrinsicMtx.at<double>(2, 3) = TranslationVector.at<double>(2, 0);
		ExtrinsicMtx.at<double>(3, 3) = 1.0;
	
		//	convert to our matrix AND transpose(row major to col major) at the same time
		Soy::Matrix4x4 ModelView;
		for ( int r=0;	r<4;	r++ )
			for ( int c=0;	c<4;	c++ )
				ModelView(r,c) = ExtrinsicMtx.at<double>( c, r );

		//	swap y & z planes so y is up
		ModelView *= GetCalibrationToWorldMtx();
	
		/*
		//	invert y and z planes for -/+ differences between opencv and opengl
		static Soy::Matrix4x4 InvertHandednessMatrix(
												  1,  0,  0, 0,
												  0,  -1, 0, 0,
												  0,  0,  -1, 0,
												  0,  0,  0,  1
												  );
		ModelView *= InvertHandednessMatrix;
		*/
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
		
		//	calc intrinsic mtx
		//	gr: projection matrix?
		cv::Mat IntrinsicMtx = cv::Mat::zeros(4, 4, CV_64FC1);
		for (int y = 0; y < 3; y++)
			for (int x = 0; x < 3; x++)
				IntrinsicMtx.at<double>(y, x) = cameraMatrix.at<double>(y, x);
		IntrinsicMtx.at<double>(0, 3) = 0;
		IntrinsicMtx.at<double>(1, 3) = 0;
		IntrinsicMtx.at<double>(2, 3) = 0;
		IntrinsicMtx.at<double>(3, 3) = 1.0;
		
		Camera.mIntrinsicMatrix = Soy::MatrixToVector( MatToMatrix4x4(IntrinsicMtx) );
	}
	
	static bool TestReprojection = true;
	if ( TestReprojection )
	{
		auto& WorldPoints = WorldPointsArray[0];
		auto& ViewPoints = ViewPointsArray[0];
		
		//	test reprojection error
		for ( int i=0;	i<WorldPoints.size();	i++ )
		{
			auto& WorldPoint = WorldPoints[0];
			
			/*
			cv::Mat ViewPos( 4, 1, CV_64FC1 );
			ViewPos.at<double>(0,0) = ViewPoints[i].x / ImageScalar.x;
			ViewPos.at<double>(1,0) = ViewPoints[i].y / ImageScalar.y;
			ViewPos.at<double>(2,0) = 0;
			ViewPos.at<double>(3,0) = 1;
			
			cv::Mat Multiplied;
			Multiplied = IntrinsicMtx * ExtrinsicMtx * ViewPos;
			
			Soy::Matrix4x1 ReProjected4( Multiplied.at<double>(0), Multiplied.at<double>(1), Multiplied.at<double>(2), Multiplied.at<double>(3) );
			Soy::Matrix3x1 ReProjected3( ReProjected4.x(), ReProjected4.y(), ReProjected4.z() );
			*/
			
			auto ReprojectedWorldPos = Camera.ScreenToWorldY( PointToVector(ViewPoints[i]), 0.f );
			/*
			Soy::Matrix3x1 WorldPos( WorldPoint.x, WorldPoint.y, WorldPoint.z );
			Soy::Matrix3x1 Delta( ReprojectedWorldPos.xyz() - WorldPos );
			
			std::Debug << "CameraCalibration point " << i << " reprojection difference: " << Delta << Soy::lf;
			*/
		}
	}
	
	return true;
}




bool Opencv::GetHomography(Soy::Matrix3x3& HomographyMtx, Opencv::TGetHomographyParams Params, const ArrayBridge<vec2f> &&Points2D, const ArrayBridge<vec2f> &&PointsUv)
{
	auto ImageScalar = Params.mCameraImageSize;
	if ( ImageScalar.x < 1 || ImageScalar.y < 1 )
	{
		Soy::Assert(false, "Camera image size too small");
		return false;
	}
	
	if ( !Soy::Assert( Points2D.GetSize() > 0, "No points 2D" ) )
		return false;
	if ( !Soy::Assert( PointsUv.GetSize() > 0, "No points uv" ) )
		return false;
	if ( !Soy::Assert( PointsUv.GetSize() == Points2D.GetSize(), "point count mis match" ) )
		return false;
	
	std::vector<cv::Point2f> SrcPoints;
	Points2D.ForEach( [&SrcPoints,&ImageScalar](const vec2f& p)	{ SrcPoints.push_back(VectorToPoint(p*ImageScalar));	return true;	} );
	std::vector<cv::Point2f> DestinationPoints;
	PointsUv.ForEach( [&DestinationPoints,&ImageScalar](const vec2f& p)	{ DestinationPoints.push_back(VectorToPoint(p*ImageScalar));	return true;	} );
	
	cv::Mat Homography = cv::findHomography( SrcPoints, DestinationPoints );

	HomographyMtx = MatToMatrix3x3( Homography );
	
	return true;
}
