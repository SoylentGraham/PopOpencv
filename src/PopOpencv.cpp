#include "PopOpencv.h"
#include <TParameters.h>
#include <SoyDebug.h>
#include <TProtocolCli.h>
#include <TProtocolHttp.h>
#include <SoyApp.h>
#include <PopMain.h>
#include <TJobRelay.h>
#include <SoyPixels.h>
#include <SoyString.h>
#include <TFeatureBinRing.h>
#include <SortArray.h>
#include <TChannelFile.h>
#include "CvCalibrateCamera.h"




TPopOpencv::TPopOpencv() :
	TJobHandler		( static_cast<TChannelManager&>(*this) ),
	TPopJobHandler	( static_cast<TJobHandler&>(*this) )
{
	AddJobHandler("exit", TParameterTraits(), *this, &TPopOpencv::OnExit );
	
	AddJobHandler("newframe", TParameterTraits(), *this, &TPopOpencv::OnNewFrame );
	AddJobHandler("re:getframe", TParameterTraits(), *this, &TPopOpencv::OnNewFrame );
	
	TParameterTraits GetFeatureTraits;
	GetFeatureTraits.mAssumedKeys.PushBack("x");
	GetFeatureTraits.mAssumedKeys.PushBack("y");
	GetFeatureTraits.mRequiredKeys.PushBack("image");
	AddJobHandler("getfeature", GetFeatureTraits, *this, &TPopOpencv::OnGetFeature );

	TParameterTraits FindFeatureTraits;
	FindFeatureTraits.mAssumedKeys.PushBack("feature");
	FindFeatureTraits.mRequiredKeys.PushBack("image");
	AddJobHandler("findfeature", FindFeatureTraits, *this, &TPopOpencv::OnFindFeature );
	
	TParameterTraits TrackFeaturesTraits;
	TrackFeaturesTraits.mAssumedKeys.PushBack("sourcefeatures");
	//TrackFeaturesTraits.mRequiredKeys.PushBack("image");
	AddJobHandler("trackfeatures", TrackFeaturesTraits, *this, &TPopOpencv::OnTrackFeatures );
	
	TParameterTraits FindInterestingFeaturesTraits;
	//FindInterestingFeaturesTraits.mRequiredKeys.PushBack("image");
	AddJobHandler("findinterestingfeatures", FindInterestingFeaturesTraits, *this, &TPopOpencv::OnFindInterestingFeatures );

	
	TParameterTraits CalibrateCameraTraits;
	CalibrateCameraTraits.mRequiredKeys.PushBack("points2D");
	CalibrateCameraTraits.mRequiredKeys.PushBack("points3D");
	AddJobHandler("calibratecamera", CalibrateCameraTraits, *this, &TPopOpencv::OnCalibrateCamera );
	
	TParameterTraits GetHomographyTraits;
	GetHomographyTraits.mRequiredKeys.PushBack("points2D");
	GetHomographyTraits.mRequiredKeys.PushBack("pointsuv");
	AddJobHandler("gethomography", GetHomographyTraits, *this, &TPopOpencv::OnGetHomography );
}

bool TPopOpencv::AddChannel(std::shared_ptr<TChannel> Channel)
{
	if ( !TChannelManager::AddChannel( Channel ) )
		return false;
	TJobHandler::BindToChannel( *Channel );
	return true;
}


void TPopOpencv::OnExit(TJobAndChannel& JobAndChannel)
{
	mConsoleApp.Exit();
	
	//	should probably still send a reply
	TJobReply Reply( JobAndChannel );
	Reply.mParams.AddDefaultParam(std::string("exiting..."));
	TChannel& Channel = JobAndChannel;
	Channel.OnJobCompleted( Reply );
}


void TPopOpencv::OnGetFeature(TJobAndChannel& JobAndChannel)
{
	auto& Job = JobAndChannel.GetJob();
	
	//	pull image
	auto ImageParam = Job.mParams.GetParam("image");

	/*
	//	assume format is now set right, decode the pixels out of it
	SoyPixels Pixels;
	if ( !ImageParam.Decode(Pixels) )
	{
		std::stringstream Error;
		Error << "Failed to decode " << Job.mParams.GetParamAs<std::string>("image") << " to an image";
		TJobReply Reply( JobAndChannel );
		Reply.mParams.AddErrorParam( Error.str() );
		
		TChannel& Channel = JobAndChannel;
		Channel.OnJobCompleted( Reply );
		return;
	}
	*/
	
	SoyPixels Image;
	if ( !Job.mParams.GetParamAs("image",Image) )
	{
		std::stringstream Error;
		Error << "Failed to decode image param";
		TJobReply Reply( JobAndChannel );
		Reply.mParams.AddErrorParam( Error.str() );
		
		TChannel& Channel = JobAndChannel;
		Channel.OnJobCompleted( Reply );
		return;
	}
	
	auto PixelxParam = Job.mParams.GetParam("x");
	auto PixelyParam = Job.mParams.GetParam("y");
	int x = PixelxParam.Decode<int>();
	int y = PixelyParam.Decode<int>();
	


	//	return descriptor and stuff
	std::stringstream Error;
	TFeatureBinRingParams Params( Job.mParams );
	TFeatureBinRing Feature;
	TFeatureExtractor::GetFeature( Feature, Image, x, y, Params, Error );
	
	TJobReply Reply( JobAndChannel );
	
	SoyData_Impl<TFeatureBinRing> FeatureData( Feature );
	std::shared_ptr<SoyData> FeatureEncoded( new SoyData_Stack<std::string>() );
	static_cast<SoyData_Stack<std::string>&>(*FeatureEncoded).Encode( FeatureData );
	
	Reply.mParams.AddDefaultParam( FeatureEncoded );
	Reply.mParams.AddParam( PixelxParam );
	Reply.mParams.AddParam( PixelyParam );
	
	TChannel& Channel = JobAndChannel;
	Channel.OnJobCompleted( Reply );
}


void ScoreInterestingFeatures(ArrayBridge<TFeatureMatch>&& Features,float MinScore)
{
	std::map<std::string,int> FeatureHistogram;		//	build a histogram to work out how unique the features are
	int HistogramMaxima = 0;

	for ( int f=0;	f<Features.GetSize();	f++ )
	{
		auto& Feature = Features[f];
		std::stringstream FeatureString;
		FeatureString << Feature.mFeature;
		auto& FeatureCount = FeatureHistogram[FeatureString.str()];
		FeatureCount++;
		HistogramMaxima = std::max( HistogramMaxima, FeatureCount );
	}
	
	//	now re-apply the feature's score based on their uniqueness in the histogram
	for ( ssize_t f=Features.GetSize()-1;	f>=0;	f-- )
	{
		auto& Feature = Features[f];
		auto& Score = Feature.mScore;
		std::stringstream FeatureString;
		FeatureString << Feature.mFeature;
		auto Occurrance = FeatureHistogram[FeatureString.str()];
		Score = 1.f - (Occurrance / static_cast<float>(HistogramMaxima));
		
		//	cull if score is too low
		if ( Score >= MinScore )
			continue;
		
		Features.RemoveBlock(f,1);
	}
}

void TPopOpencv::OnFindInterestingFeatures(TJobAndChannel& JobAndChannel)
{
	auto& Job = JobAndChannel.GetJob();
	
	//	decode image now into a param so we can send back the one we used
	std::shared_ptr<SoyData_Stack<SoyPixels>> ImageData( new SoyData_Stack<SoyPixels>() );
	auto& Image = ImageData->mValue;
	auto ImageParam = Job.mParams.GetParam( TJobParam::Param_Default );
	if ( !ImageParam.Decode( *ImageData ) )
	{
		std::stringstream Error;
		Error << "Failed to decode image param (" << Image.GetFormat() << ")";
		TJobReply Reply( JobAndChannel );
		Reply.mParams.AddErrorParam( Error.str() );
		
		TChannel& Channel = JobAndChannel;
		Channel.OnJobCompleted( Reply );
		return;
	}
	
	//	resize for speed
	//	gr: put this in params!
//	static int NewWidth = 400;
//	static int NewHeight = 200;
//	Image.ResizeFastSample(NewWidth,NewHeight);

	//	grab a feature at each point on a grid on the image
	TFeatureBinRingParams Params( Job.mParams );
	Array<TFeatureMatch> FeatureMatches;
	FeatureMatches.Reserve( (Image.GetHeight()/Params.mMatchStepY) * (Image.GetWidth()/Params.mMatchStepX) );
	std::stringstream Error;
	for ( int y=0;	y<Image.GetHeight();	y+=Params.mMatchStepY )
	{
		for ( int x=0;	x<Image.GetWidth();	x+=Params.mMatchStepX )
		{
			TFeatureBinRing Feature;
			TFeatureExtractor::GetFeature( Feature, Image, x, y, Params, Error );
			if ( !Error.str().empty() )
				break;
			auto& Match = FeatureMatches.PushBack();
			Match.mSourceCoord = vec2x<int>(-1,-1);
			Match.mCoord.x = x;
			Match.mCoord.y = y;
			Match.mFeature = Feature;
			Match.mScore = 0.f;	//	make interesting score
		}
	}
	
	//	do initial scoring to remove low-interest features
	ScoreInterestingFeatures( GetArrayBridge( FeatureMatches ), Params.mMinInterestingScore );

	//	re-score to normalise the score. (could probably do this faster, but this is simpler)
	ScoreInterestingFeatures( GetArrayBridge( FeatureMatches ), 0.f );
	
	//	some some params back with the reply
	TJobReply Reply( JobAndChannel );
	
	//	gr: repalce with desired format/container
	bool AsJson = Job.mParams.GetParamAsWithDefault("asjson", false );
	bool AsBinary = Job.mParams.GetParamAsWithDefault("asbinary", false );
	
	if ( AsJson )
	{
		//	gr: the internal SoyData system doesn't know this type, so won't auto encode :/ need to work on this!
		std::shared_ptr<SoyData_Impl<json::Object>> FeatureMatchesJsonData( new SoyData_Stack<json::Object>() );
		if ( FeatureMatchesJsonData->EncodeRaw( FeatureMatches ) )
		{
			std::shared_ptr<SoyData> FeatureMatchesJsonDataGen( FeatureMatchesJsonData );
			Reply.mParams.AddDefaultParam( FeatureMatchesJsonDataGen );
		}
	}
	
	
	if ( AsBinary )
	{
		static bool JoinFeaturesAndImage = true;
		
		if ( JoinFeaturesAndImage )
		{
			TFeatureMatchesAndImage MaI;
			SoyData_Impl<TFeatureMatchesAndImage> MaIData( MaI );
			MaI.mFeatureMatches = FeatureMatches;
			MaI.mImage = Image;
			
			//	gr: the internal SoyData system doesn't know this type, so won't auto encode :/ need to work on this!
			std::shared_ptr<SoyData_Impl<Array<char>>> MaiBinary( new SoyData_Stack<Array<char>>() );
			if ( MaiBinary->EncodeRaw( MaI ) )
			{
				Reply.mParams.AddDefaultParam( MaiBinary );
			}
		}
		else
		{
			//	gr: the internal SoyData system doesn't know this type, so won't auto encode :/ need to work on this!
			std::shared_ptr<SoyData_Impl<Array<char>>> FeatureMatchesJsonData( new SoyData_Stack<Array<char>>() );
			if ( FeatureMatchesJsonData->EncodeRaw( FeatureMatches ) )
			{
				Reply.mParams.AddDefaultParam( FeatureMatchesJsonData );
			}
		}
	}
	
	//	add as generic
	if ( !Reply.mParams.HasDefaultParam() )
		Reply.mParams.AddDefaultParam( FeatureMatches );
	
	if ( !Error.str().empty() )
		Reply.mParams.AddErrorParam( Error.str() );
	
	//	gr: need to work out a good way to automatically send back token/meta params (all the ones we didn't read?)
	auto SerialParam = Job.mParams.GetParam("serial");
	Reply.mParams.AddParam( SerialParam );

	//	gr: this sends a big payload... and a MASSIVE image as a string param!, but maybe need it at some point
	static bool SendBackImageImage = false;
	if ( SendBackImageImage )
	{
		//	gr: send back a decoded image, not the original param (which might be a now-different memfile)
		Reply.mParams.AddParam("image",ImageData);
	}
	
	TChannel& Channel = JobAndChannel;
	Channel.OnJobCompleted( Reply );
}

void TPopOpencv::OnFindFeature(TJobAndChannel& JobAndChannel)
{
	auto& Job = JobAndChannel.GetJob();
	
	//	pull image
	auto Image = Job.mParams.GetParamAs<SoyPixels>("image");
	auto Feature = Job.mParams.GetParamAs<TFeatureBinRing>("Feature");
	
	if ( !Image.IsValid() )
	{
		std::stringstream Error;
		Error << "Failed to decode image param";
		TJobReply Reply( JobAndChannel );
		Reply.mParams.AddErrorParam( Error.str() );
		
		TChannel& Channel = JobAndChannel;
		Channel.OnJobCompleted( Reply );
		return;
	}

	//	run a search
	TFeatureBinRingParams Params( Job.mParams );
	Array<TFeatureMatch> FeatureMatches;
	std::stringstream Error;
	TFeatureExtractor::FindFeatureMatches( GetArrayBridge(FeatureMatches), Image, Feature, Params, Error );
	
	//	some some params back with the reply
	TJobReply Reply( JobAndChannel );
	Reply.mParams.AddParam( Job.mParams.GetParam("Feature") );
	
	
	//	gr: repalce with desired format/container
	bool AsJson = Job.mParams.GetParamAsWithDefault("asjson", false );
	bool AsBinary = Job.mParams.GetParamAsWithDefault("asbinary", false );
	
	if ( AsJson )
	{
		//	gr: the internal SoyData system doesn't know this type, so won't auto encode :/ need to work on this!
		std::shared_ptr<SoyData_Impl<json::Object>> FeatureMatchesJsonData( new SoyData_Stack<json::Object>() );
		if ( FeatureMatchesJsonData->EncodeRaw( FeatureMatches ) )
		{
			std::shared_ptr<SoyData> FeatureMatchesJsonDataGen( FeatureMatchesJsonData );
			Reply.mParams.AddDefaultParam( FeatureMatchesJsonDataGen );
		}
	}
	
	if ( AsBinary )
	{
		//	gr: the internal SoyData system doesn't know this type, so won't auto encode :/ need to work on this!
		std::shared_ptr<SoyData_Impl<Array<char>>> FeatureMatchesJsonData( new SoyData_Stack<Array<char>>() );
		if ( FeatureMatchesJsonData->EncodeRaw( FeatureMatches ) )
		{
			std::shared_ptr<SoyData> FeatureMatchesJsonDataGen( FeatureMatchesJsonData );
			Reply.mParams.AddDefaultParam( FeatureMatchesJsonDataGen );
		}
	}
	
	//	add as generic
	if ( !Reply.mParams.HasDefaultParam() )
		Reply.mParams.AddDefaultParam( FeatureMatches );
	
	if ( !Error.str().empty() )
		Reply.mParams.AddErrorParam( Error.str() );
	
	TChannel& Channel = JobAndChannel;
	Channel.OnJobCompleted( Reply );
}


void TPopOpencv::OnTrackFeatures(TJobAndChannel& JobAndChannel)
{
	//	just grab interesting ones for now
	OnFindInterestingFeatures( JobAndChannel );
/*
	auto& Job = JobAndChannel.GetJob();
	
	//	pull image
	auto Image = Job.mParams.GetParamAs<SoyPixels>("image");
	auto SourceFeatures = Job.mParams.GetParamAs<Array<TFeatureMatch>>("sourcefeatures");
	
	if ( !Image.IsValid() )
	{
		std::stringstream Error;
		Error << "Failed to decode image param";
		TJobReply Reply( JobAndChannel );
		Reply.mParams.AddErrorParam( Error.str() );
		
		TChannel& Channel = JobAndChannel;
		Channel.OnJobCompleted( Reply );
		return;
	}
	
	//	find nearest neighbour best match...
	
	//	run a search
	TFeatureBinRingParams Params( Job.mParams );
	Array<TFeatureMatch> FeatureMatches;
	std::stringstream Error;
	//TFeatureExtractor::FindFeatureMatches( GetArrayBridge(FeatureMatches), Image, Feature, Params, Error );
	for ( int f=0;	f<SourceFeatures.GetSize();	f++ )
	{
		auto& SourceFeature = SourceFeatures[f];
		auto& Match = FeatureMatches.PushBack();
		Match.mSourceCoord = SourceFeature.mCoord;
		Match.mSourceFeature = SourceFeature.mFeature;
		
		Match.mCoord = SourceFeature.mCoord;
		Match.mCoord.x += 10;
		Match.mCoord.y += 10;
		Match.mFeature = SourceFeature.mFeature;
	}

	//	some some params back with the reply
	TJobReply Reply( JobAndChannel );
	Reply.mParams.AddParam( Job.mParams.GetParam("serial") );
	
	
	//	gr: repalce with desired format/container
	bool AsJson = Job.mParams.GetParamAsWithDefault("asjson", false );
	bool AsBinary = Job.mParams.GetParamAsWithDefault("asbinary", false );
	
	if ( AsJson )
	{
		//	gr: the internal SoyData system doesn't know this type, so won't auto encode :/ need to work on this!
		std::shared_ptr<SoyData_Impl<json::Object>> FeatureMatchesJsonData( new SoyData_Stack<json::Object>() );
		if ( FeatureMatchesJsonData->EncodeRaw( FeatureMatches ) )
		{
			std::shared_ptr<SoyData> FeatureMatchesJsonDataGen( FeatureMatchesJsonData );
			Reply.mParams.AddDefaultParam( FeatureMatchesJsonDataGen );
		}
	}
	
	if ( AsBinary )
	{
		//	gr: the internal SoyData system doesn't know this type, so won't auto encode :/ need to work on this!
		std::shared_ptr<SoyData_Impl<Array<char>>> FeatureMatchesJsonData( new SoyData_Stack<Array<char>>() );
		if ( FeatureMatchesJsonData->EncodeRaw( FeatureMatches ) )
		{
			std::shared_ptr<SoyData> FeatureMatchesJsonDataGen( FeatureMatchesJsonData );
			Reply.mParams.AddDefaultParam( FeatureMatchesJsonDataGen );
		}
	}
	
	//	add as generic
	if ( !Reply.mParams.HasDefaultParam() )
		Reply.mParams.AddDefaultParam( FeatureMatches );
	
	if ( !Error.str().empty() )
		Reply.mParams.AddErrorParam( Error.str() );
	
	TChannel& Channel = JobAndChannel;
	Channel.OnJobCompleted( Reply );
 */
}



void TPopOpencv::OnNewFrame(TJobAndChannel& JobAndChannel)
{
	auto& Job = JobAndChannel.GetJob();
	
	//	pull image
	auto ImageParam = Job.mParams.GetDefaultParam();
	SoyPixels Image;
	std::Debug << "Getting image from " << ImageParam.GetFormat() << std::endl;
	if ( !ImageParam.Decode( Image ) )
	{
		std::Debug << "Failed to decode image" << std::endl;
		return;
	}
	std::Debug << "Decoded image " << Image.GetWidth() << "x" << Image.GetHeight() << " " << Image.GetFormat() << std::endl;
}


void TPopOpencv::OnCalibrateCamera(TJobAndChannel& JobAndChannel)
{
	auto& Job = JobAndChannel.GetJob();

	//	read points
	auto Point2strings = Job.mParams.GetParamAs<std::string>("points2D");
	auto Point3strings = Job.mParams.GetParamAs<std::string>("points3D");
	
	std::stringstream Error;

	//	extract floats
	Array<vec2f> Point2s;
	{
		std::stringstream ParseError;
		auto AppendPoints2 = [&ParseError,&Point2s](const std::string& Point2str)
		{
			vec2f Point2f;
			if ( !Soy::StringToType( Point2f, Point2str ) )
			{
				ParseError << "Failed to parse \"" << Point2str << "\" to NxM";
				return false;
			}
			Point2s.PushBack( Point2f );
			return true;
		};
		if ( !Soy::StringSplitByMatches( AppendPoints2, Point2strings, ",", false ) )
		{
			Error << "failed to parse 2d points; " << ParseError.str() << Soy::lf;
		}
	}
	
	Array<vec3f> Point3s;
	{
		std::stringstream ParseError;
		auto AppendPoints3 = [&ParseError,&Point3s](const std::string& Point3str)
		{
			vec3f Point3f;
			if ( !Soy::StringToType( Point3f, Point3str ) )
			{
				ParseError << "Failed to parse \"" << Point3str << "\" to NxMxO";
				return false;
			}
			Point3s.PushBack( Point3f );
			return true;
		};
		if ( !Soy::StringSplitByMatches( AppendPoints3, Point3strings, ",", false ) )
		{
			Error << "failed to parse 3d points; " << ParseError.str() << Soy::lf;
		}
	}
	
	//	need matching amounts
	if ( Point2s.IsEmpty() )
		Error << "no 2d points" << Soy::lf;
	if ( Point3s.IsEmpty() )
		Error << "no 3d points" << Soy::lf;
	if ( Point2s.GetSize() != Point3s.GetSize() )
		Error << "Number of points mis matched (" << Point2s.GetSize() << " vs " << Point3s.GetSize() << ")" << Soy::lf;
	
	TJobReply Reply( JobAndChannel );

	if ( !Error.str().empty() )
	{
		Reply.mParams.AddErrorParam( Error.str() );
		JobAndChannel.GetChannel().SendJobReply( Reply );
		return;
	}

	Soy::TCamera Camera;
	Opencv::TCalibrateCameraParams Params;
	static float imgw = 3000;
	static float imgh = 2250;
	Params.mCameraImageSize = vec2f( imgw, imgh );
	try
	{
		if ( !Opencv::CalibrateCamera( Camera, Params, GetArrayBridge(Point3s), GetArrayBridge(Point2s) ) )
			Error << "Failed to calibrate camera";
	}
	catch ( const Soy::AssertException& e )
	{
		Error << e.what();
	}
	catch ( ... )
	{
		Error << "Unknown exception calibrating camera";
	}

	if ( !Error.str().empty() )
	{
		Reply.mParams.AddErrorParam( Error.str() );
	}
	else
	{
		Reply.mParams.AddParam("CalibrationError", Camera.mCalibrationError );
	
		std::stringstream CameraOutput;

		CameraOutput << "cameramtx:";
		CameraOutput << Camera.mMatrix.rows[0] << ',';
		CameraOutput << Camera.mMatrix.rows[1] << ',';
		CameraOutput << Camera.mMatrix.rows[2] << ',';
		CameraOutput << Camera.mMatrix.rows[3] << Soy::lf;
		
		CameraOutput << "cameraprojectionmtx:";
		CameraOutput << Camera.mIntrinsicMatrix.rows[0] << ',';
		CameraOutput << Camera.mIntrinsicMatrix.rows[1] << ',';
		CameraOutput << Camera.mIntrinsicMatrix.rows[2] << ',';
		CameraOutput << Camera.mIntrinsicMatrix.rows[3] << Soy::lf;
		
		CameraOutput << "calibrationerror:" << Camera.mCalibrationError << Soy::lf;
		CameraOutput << "fov:" << Camera.mFov << Soy::lf;
		CameraOutput << "lensoffset:" << Camera.mLensOffset << Soy::lf;
		CameraOutput << "cameraworldpos:" << Camera.mCameraWorldPosition << Soy::lf;
		CameraOutput << "camerarotationeulardegrees:" << Camera.mCameraRotationEularDeg << Soy::lf;
		
		CameraOutput << "distortionradtank5:";
		CameraOutput << Camera.mRadialDistortion << ',';
		CameraOutput << Camera.mTangentialDistortion << ',';
		CameraOutput << Camera.mDistortionK5 << Soy::lf;
		
		Reply.mParams.AddDefaultParam( CameraOutput.str() );
	}
	
	JobAndChannel.GetChannel().SendJobReply( Reply );

}


void TPopOpencv::OnGetHomography(TJobAndChannel& JobAndChannel)
{
	auto& Job = JobAndChannel.GetJob();
	
	//	read points
	auto Point2strings = Job.mParams.GetParamAs<std::string>("points2D");
	auto Pointuvstrings = Job.mParams.GetParamAs<std::string>("pointsuv");
	
	std::stringstream Error;
	
	//	extract floats
	Array<vec2f> Point2s;
	{
		std::stringstream ParseError;
		auto AppendPoints2 = [&ParseError,&Point2s](const std::string& Point2str)
		{
			vec2f Point2f;
			if ( !Soy::StringToType( Point2f, Point2str ) )
			{
				ParseError << "Failed to parse \"" << Point2str << "\" to NxM";
				return false;
			}
			Point2s.PushBack( Point2f );
			return true;
		};
		if ( !Soy::StringSplitByMatches( AppendPoints2, Point2strings, ",", false ) )
		{
			Error << "failed to parse 2d points; " << ParseError.str() << Soy::lf;
		}
	}
	
	Array<vec2f> Pointuvs;
	{
		std::stringstream ParseError;
		auto AppendPoints3 = [&ParseError,&Pointuvs](const std::string& Point3str)
		{
			vec2f Pointuv;
			if ( !Soy::StringToType( Pointuv, Point3str ) )
			{
				ParseError << "Failed to parse \"" << Point3str << "\" to NxMxO";
				return false;
			}
			Pointuvs.PushBack( Pointuv );
			return true;
		};
		if ( !Soy::StringSplitByMatches( AppendPoints3, Pointuvstrings, ",", false ) )
		{
			Error << "failed to parse uv points; " << ParseError.str() << Soy::lf;
		}
	}
	
	//	need matching amounts
	if ( Point2s.IsEmpty() )
		Error << "no 2d points" << Soy::lf;
	if ( Pointuvs.IsEmpty() )
		Error << "no uv points" << Soy::lf;
	if ( Point2s.GetSize() != Pointuvs.GetSize() )
		Error << "Number of points mis matched (" << Point2s.GetSize() << " vs " << Pointuvs.GetSize() << ")" << Soy::lf;
	
	TJobReply Reply( JobAndChannel );
	
	if ( !Error.str().empty() )
	{
		Reply.mParams.AddErrorParam( Error.str() );
		JobAndChannel.GetChannel().SendJobReply( Reply );
		return;
	}
	
	Soy::Matrix3x3 Homography;
	Opencv::TGetHomographyParams Params;
	static float imgw = 3000;
	static float imgh = 2250;
	Params.mCameraImageSize = vec2f( imgw, imgh );
	try
	{
		if ( !Opencv::GetHomography( Homography, Params, GetArrayBridge(Pointuvs), GetArrayBridge(Point2s) ) )
		Error << "Failed to get homography";
	}
	catch ( const Soy::AssertException& e )
	{
		Error << e.what();
	}
	catch ( ... )
	{
		Error << "Unknown exception getting homography";
	}
	
	if ( !Error.str().empty() )
	{
		Reply.mParams.AddErrorParam( Error.str() );
	}
	else
	{
		std::stringstream CameraOutput;
		
		CameraOutput << "homography:";
		CameraOutput << Homography << Soy::lf;
		
		Reply.mParams.AddDefaultParam( CameraOutput.str() );
	}
	
	JobAndChannel.GetChannel().SendJobReply( Reply );
	
}


TPopAppError::Type PopMain(TJobParams& Params)
{
	TPopOpencv App;

	
	//	create stdio channel for commandline output
	auto StdioChannel = CreateChannelFromInputString("std:", SoyRef("stdio") );
	auto HttpChannel = CreateChannelFromInputString("http:8080-8090", SoyRef("http") );
	
	App.AddChannel( StdioChannel );
	App.AddChannel( HttpChannel );
	
	
	//	bootup commands via a channel
	std::shared_ptr<TChannel> BootupChannel( new TChan<TChannelFileRead,TProtocolCli>( SoyRef("Bootup"), "bootup.txt" ) );
	
	//	display reply to stdout
	//	when the commandline SENDs a command (a reply), send it to stdout
	auto RelayFunc = [](TJobAndChannel& JobAndChannel)
	{
		std::Debug << JobAndChannel.GetJob().mParams << std::endl;
	};
	//BootupChannel->mOnJobRecieved.AddListener( RelayFunc );
	BootupChannel->mOnJobSent.AddListener( RelayFunc );
	BootupChannel->mOnJobLost.AddListener( RelayFunc );
	
	App.AddChannel( BootupChannel );
	
	
	

	
	//	run
	App.mConsoleApp.WaitForExit();

	return TPopAppError::Success;
}




