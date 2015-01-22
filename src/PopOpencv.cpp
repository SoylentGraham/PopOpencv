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



TPopOpencv::TPopOpencv()
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
}

void TPopOpencv::AddChannel(std::shared_ptr<TChannel> Channel)
{
	TChannelManager::AddChannel( Channel );
	if ( !Channel )
		return;
	TJobHandler::BindToChannel( *Channel );
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
	for ( int f=Features.GetSize()-1;	f>=0;	f-- )
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



class TChannelLiteral : public TChannel
{
public:
	TChannelLiteral(SoyRef ChannelRef) :
	TChannel	( ChannelRef )
	{
	}
	
	virtual void				GetClients(ArrayBridge<SoyRef>&& Clients)
	{

	}
	
	bool				FixParamFormat(TJobParam& Param,std::stringstream& Error) override
	{
		return true;
	}
	void		Execute(std::string Command)
	{
		TJobParams Params;
		Execute( Command, Params );
	}
	void		Execute(std::string Command,const TJobParams& Params)
	{
		auto& Channel = *this;
		TJob Job;
		Job.mParams = Params;
		Job.mParams.mCommand = Command;
		Job.mChannelMeta.mChannelRef = Channel.GetChannelRef();
		Job.mChannelMeta.mClientRef = SoyRef("x");
		
		//	send job to handler
		Channel.OnJobRecieved( Job );
	}
	
	//	we don't do anything, but to enable relay, we say it's "done"
	virtual bool				SendJobReply(const TJobReply& Job) override
	{
		OnJobSent( Job );
		return true;
	}
	virtual bool				SendCommandImpl(const TJob& Job) override
	{
		OnJobSent( Job );
		return true;
	}
};



//	horrible global for lambda
std::shared_ptr<TChannel> gStdioChannel;
std::shared_ptr<TChannel> gCaptureChannel;



TPopAppError::Type PopMain(TJobParams& Params)
{
	std::cout << Params << std::endl;
	
	TPopOpencv App;

	auto CommandLineChannel = std::shared_ptr<TChan<TChannelLiteral,TProtocolCli>>( new TChan<TChannelLiteral,TProtocolCli>( SoyRef("cmdline") ) );
	
	//	create stdio channel for commandline output
	gStdioChannel = CreateChannelFromInputString("std:", SoyRef("stdio") );
	auto HttpChannel = CreateChannelFromInputString("http:8080-8090", SoyRef("http") );
	auto WebSocketChannel = CreateChannelFromInputString("ws:json:9090-9099", SoyRef("websock") );
//	auto WebSocketChannel = CreateChannelFromInputString("ws:cli:9090-9099", SoyRef("websock") );
	auto SocksChannel = CreateChannelFromInputString("cli:7090-7099", SoyRef("socks") );
	
	
	App.AddChannel( CommandLineChannel );
	App.AddChannel( gStdioChannel );
	App.AddChannel( HttpChannel );
	App.AddChannel( WebSocketChannel );
	App.AddChannel( SocksChannel );

	//	when the commandline SENDs a command (a reply), send it to stdout
	auto RelayFunc = [](TJobAndChannel& JobAndChannel)
	{
		if ( !gStdioChannel )
			return;
		TJob Job = JobAndChannel;
		Job.mChannelMeta.mChannelRef = gStdioChannel->GetChannelRef();
		Job.mChannelMeta.mClientRef = SoyRef();
		gStdioChannel->SendCommand( Job );
	};
	CommandLineChannel->mOnJobSent.AddListener( RelayFunc );
	
	//	connect to another app, and subscribe to frames
	bool CreateCaptureChannel = false;
	if ( CreateCaptureChannel )
	{
		auto CaptureChannel = CreateChannelFromInputString("cli://localhost:7070", SoyRef("capture") );
		gCaptureChannel = CaptureChannel;
		CaptureChannel->mOnJobRecieved.AddListener( RelayFunc );
		App.AddChannel( CaptureChannel );
		
		//	send commands from stdio to new channel
		auto SendToCaptureFunc = [](TJobAndChannel& JobAndChannel)
		{
			TJob Job = JobAndChannel;
			Job.mChannelMeta.mChannelRef = gStdioChannel->GetChannelRef();
			Job.mChannelMeta.mClientRef = SoyRef();
			gCaptureChannel->SendCommand( Job );
		};
		gStdioChannel->mOnJobRecieved.AddListener( SendToCaptureFunc );
		
		auto StartSubscription = [](TChannel& Channel)
		{
			TJob GetFrameJob;
			GetFrameJob.mChannelMeta.mChannelRef = Channel.GetChannelRef();
			//GetFrameJob.mParams.mCommand = "subscribenewframe";
			//GetFrameJob.mParams.AddParam("serial", "isight" );
			GetFrameJob.mParams.mCommand = "getframe";
			GetFrameJob.mParams.AddParam("serial", "isight" );
			GetFrameJob.mParams.AddParam("memfile", "1" );
			Channel.SendCommand( GetFrameJob );
		};
		
		CaptureChannel->mOnConnected.AddListener( StartSubscription );
	}
	
	
	/*
	std::string TestFilename = "/users/grahamr/Desktop/ringo.png";
	
	//	gr: bootup commands
	auto BootupGet = [TestFilename](TChannel& Channel)
	{
		TJob GetFrameJob;
		GetFrameJob.mChannelMeta.mChannelRef = Channel.GetChannelRef();
		GetFrameJob.mParams.mCommand = "getfeature";
		GetFrameJob.mParams.AddParam("x", 120 );
		GetFrameJob.mParams.AddParam("y", 120 );
		GetFrameJob.mParams.AddParam("image", TestFilename, TJobFormat("text/file/png") );
		Channel.OnJobRecieved( GetFrameJob );
	};
	
	auto BootupMatch = [TestFilename](TChannel& Channel)
	{
		TJob GetFrameJob;
		GetFrameJob.mChannelMeta.mChannelRef = Channel.GetChannelRef();
		GetFrameJob.mParams.mCommand = "findfeature";
		GetFrameJob.mParams.AddParam("feature", "01011000000000001100100100000000" );
		GetFrameJob.mParams.AddParam("image", TestFilename, TJobFormat("text/file/png") );
		Channel.OnJobRecieved( GetFrameJob );
	};
	

	//	auto BootupFunc = BootupMatch;
	//auto BootupFunc = BootupGet;
	auto BootupFunc = BootupMatch;
	if ( CommandLineChannel->IsConnected() )
		BootupFunc( *CommandLineChannel );
	else
		CommandLineChannel->mOnConnected.AddListener( BootupFunc );
*/
	
	
	//	run
	App.mConsoleApp.WaitForExit();

	gStdioChannel.reset();
	return TPopAppError::Success;
}




